#include "odometry.h"
#include "robot_config.h"
#include "utils.h"
#include "main.h"
#include "motion_config.h"

// Global position (cm) and heading (degrees, continuous VEX Coordinates)
// 0° = North, positive clockwise
double globalX = 0.0;
double globalY = 0.0;
double globalRotation = 0.0;

// Encoder velocity computed from position deltas — use instead of get_velocity() which is broken in PROS
double globalLeftEncoderRPM  = 0.0;
double globalRightEncoderRPM = 0.0;

// Previous encoder readings for delta calculations
double prevLeftEncoder = 0.0;
double prevRightEncoder = 0.0;
double prevXEncoder = 0.0;
double prevRotation = 0.0;

// Flag to enable/disable lateral tracking wheel
bool xEncoderEnabled = true;  

// ── GPS reset handoff ─────────────────────────────────────────────────────────
// The GPS reset task runs on a separate PROS task and must not write globalX/Y
// directly — that would race with the odometry task's globalX += deltaXPos tick.
// Instead, the GPS task calls applyGpsReset() which sets a pending flag.
// updateOdometry() checks the flag at the top of each tick (between accumulations)
// and applies the correction safely — no tearing possible.
std::atomic<bool> pendingGpsReset{false};
static double pendingGpsX = 0.0;
static double pendingGpsY = 0.0;

void applyGpsReset(double newX_cm, double newY_cm) {
    pendingGpsX = newX_cm;
    pendingGpsY = newY_cm;
    pendingGpsReset.store(true);
}

// Motion states for context-aware encoder interpretation
enum RobotState { STATIONARY, TURNING, STRAIGHT };
RobotState currentState = STATIONARY;

// ======================================================================
// setStartPosition — Initialize field position and heading
//
// Sets globalX/globalY to the robot's starting field coordinates (cm).
// startHeading is in VEX Coordinates (North = 0°, CW positive).
// Snapshots the IMU reading and resets encoders so all subsequent odometry
// updates are delta-based from this reference point.
// ======================================================================
void setStartPosition(double startX, double startY, double startHeading) {
    // 1. Set global field coordinates (X = East/West, Y = North/South, in cm)
    globalX = startX;  
    globalY = startY;

    // 2. Store starting heading for display and continuous heading calculation
    robotStartingHeading = startHeading; 
    
    // 3. Store for heading math reference (VEX Coordinates, North = 0°, CW+)
    robotStartingHeadingStandard = startHeading;
    
    // 4. Reset Sensors
    // Snapshot current IMU rotation — odometry treats this as the zero reference
    gyroReadingAtStart = InertialSensor.get_rotation();

    // 5. Reset encoders — set_position(0) zeroes the accumulated position counter.
    // reset() only resets the angle within one revolution, not get_position().
    passiveEncoderLeft.set_position(0);
    passiveEncoderRight.set_position(0);
    passiveEncoderX.set_position(0);
    pros::delay(50);  // allow set_position to propagate before odometry task reads

    // Initialize previous readings to 0
    prevLeftEncoder = 0;
    prevRightEncoder = 0;
    prevXEncoder = 0;
    
    // Initialize rotation history
    // IMPORTANT: prevRotation must match the current standard heading, 
    // or the first odometry update will think the robot jumped.
    prevRotation = robotStartingHeadingStandard;
}

// ======================================================================
// CORE ODOMETRY LOOP
// ======================================================================
/**
 * Updates globalX and globalY based on sensor changes since the last call.
 * Uses "Dead Reckoning" math to integrate small movements into a global position.
 */
void updateOdometry() {
    // Apply any pending GPS reset between ticks — safe because this runs
    // before the delta accumulation, so globalX/Y are in a stable state.
    if (pendingGpsReset.load()) {
        globalX = pendingGpsX;
        globalY = pendingGpsY;
        pendingGpsReset.store(false);
    }

    // 1. Read Current Sensor Values
    // PROS Rotation returns centidegrees; divide by 100 to get degrees
    double leftEncoder  = passiveEncoderLeft.get_position()  / 100.0;
    double rightEncoder = passiveEncoderRight.get_position() / 100.0;
    // Use the stored previous value if the X encoder is disabled
    double xEncoder     = xEncoderEnabled ? passiveEncoderX.get_position() / 100.0 : prevXEncoder;
    double currentRotation = getContinuousStandardHeading();

    // 2. Calculate Deltas (Change since last loop)
    double deltaLeft     = leftEncoder - prevLeftEncoder;
    double deltaRight    = rightEncoder - prevRightEncoder;
    double deltaX        = xEncoder - prevXEncoder;
    double deltaRotation = currentRotation - prevRotation;

    // 3. Update History for next loop
    prevLeftEncoder = leftEncoder;
    prevRightEncoder = rightEncoder;
    prevXEncoder    = xEncoder;
    prevRotation    = currentRotation;

    // Compute encoder velocity from position deltas — get_velocity() is unreliable in PROS
    // deltaLeft in degrees; x100 = cdeg; /6 = RPM (cdeg x 60000 / (36000 x 10ms))
    globalLeftEncoderRPM  = (deltaLeft  * 100.0) / 6.0;
    globalRightEncoderRPM = (deltaRight * 100.0) / 6.0;

    // 4. Convert Sensor Data to Physical Distances (cm)
    // Calculate the scaling factor once to ensure consistency across all wheels
    double cmPerDegree = encoderWheelCircumferenceCM / 360.0;

    // Forward movement is the average of the left and right tracking wheels
    double forwardDistance = ((deltaLeft + deltaRight) / 2.0) * cmPerDegree;

    // 5. Calculate Local Coordinate Changes
    double deltaXPos = 0.0;
    double deltaYPos = 0.0;
    
    // Use average heading during the tick (half-angle Euler integration).
    // Using currentRotation alone biases the position update toward the end-of-tick
    // angle, causing systematic drift when the robot is turning.
    double averageRotation = prevRotation + (deltaRotation / 2.0);
    double headingRad = averageRotation * (M_PI / 180.0);

    // Only update position if the robot is in a valid state
    if (currentState == STRAIGHT || currentState == STATIONARY || currentState == TURNING) {
        
        // --- ARC CORRECTION ALGORITHM ---
        // Raw Strafe: How much the X-wheel spun
        double lateralRaw = - (deltaX * cmPerDegree);

        // Correction: When the robot turns, the X-wheel traces an arc even if the robot
        // didn't slide sideways. We calculate this "ghost movement" and subtract it.
        // Ghost Movement = AngleChange(radians) * OffsetFromCenter(cm)
        double deltaRotationRad = deltaRotation * (M_PI / 180.0);
        double lateralMovement = lateralRaw - (deltaRotationRad * ENCODER_OFFSET_X); 
        
        // 6. Rotate Local Movements to Global Coordinate System
        // Standard rotation matrix to convert local forward/strafe to global X/Y
        deltaXPos = forwardDistance * sin(headingRad) + lateralMovement * cos(headingRad);
        deltaYPos = forwardDistance * cos(headingRad) - lateralMovement * sin(headingRad);
    }

    // 7. Accumulate Global Position
    globalX += deltaXPos;
    globalY += deltaYPos;
    globalRotation = currentRotation;

    // Debug: Print position to brain screen — commented out, use main.cpp PosDisplay task
    // pros::screen::print(pros::E_TEXT_MEDIUM, 1, "X: %.2f, Y: %.2f, H: %.2f",
    //                     globalX, globalY, getContinuousStandardHeading());
}

// ======================================================================
// calculatePathToTarget — Straight-line distance and heading to a field point
// Heading is returned in VEX Coordinates (North = 0°, CW+) from atan2
// ======================================================================
void calculatePathToTarget(double currentX, double currentY, double targetX, double targetY,
                           double &distance, double &heading) {
    double deltaX = targetX - currentX;
    double deltaY = targetY - currentY;
    distance = sqrt(deltaX * deltaX + deltaY * deltaY);
    heading = atan2(deltaX, deltaY) * 180.0 / M_PI;
}

// ── All point-to-point navigation functions (turnToPoint, forwardToPoint, etc.)
// are implemented in navigation.cpp.

// ======================================================================
// Background task for continuous odometry updates
// ======================================================================
OdometryTaskParams odometryParams = {false};

// PROS task functions must return void — called by pros::Task internally
void odometryTask(void *params) {
    OdometryTaskParams *p = static_cast<OdometryTaskParams *>(params);
    while (p->isRunning) {
        updateOdometry();
        pros::delay(10);  // ~100Hz update rate
    }
}

void startOdometryTask() {
    if (!odometryParams.isRunning) {
        odometryParams.isRunning = true;
        pros::Task(odometryTask, &odometryParams, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "OdometryTask");
    }
}

void stopOdometryTask() {
    odometryParams.isRunning = false;
}