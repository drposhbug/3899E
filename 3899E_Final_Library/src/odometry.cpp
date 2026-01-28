#include "odometry.h"
#include "navigation.h"
#include "robot_config.h"
#include "utils.h"
#include "vex.h"

using namespace vex;

// Global position (cm) and heading (degrees, continuous Standard Cartesian)
// 0° = East, positive counterclockwise
double globalX = 0.0;
double globalY = 0.0;
double globalRotation = 0.0;

// Previous encoder readings for delta calculations
double prevLeftEncoder = 0.0;
double prevRightEncoder = 0.0;
double prevXEncoder = 0.0;
double prevRotation = 0.0;

// Flag to enable/disable lateral tracking wheel
bool xEncoderEnabled = true;

// Motion states for context-aware encoder interpretation
enum RobotState { STATIONARY, TURNING, STRAIGHT };
RobotState currentState = STATIONARY;

// ======================================================================
// Set starting position and heading
// Input heading is Modified Cartesian (North = 0°)
// Converted once to Standard Cartesian for internal calculations
// Tares gyro and resets encoders
// ======================================================================
void setStartPosition(double startX, double startY, double startHeading_Modified) {
    // 1. Set Global Coordinates
    globalX = startY;  // User's Y (East/West) → Standard X
    globalY = startX;  // User's X (North/South) → Standard Y

    // 2. Save Display Heading (The "North" heading for the screen)
    // FIX: Don't do math with uninitialized variables. Just save the input.
    robotStartingHeading = startHeading_Modified; 
    
    // 3. Set Internal Math Heading (The "East" heading for Odometry)
    // FIX: This calls the function (adds 90). We do NOT add extra offsets here.
    robotStartingHeadingStandard = modifiedToStandardCartesian(startHeading_Modified);
    
    // 4. Reset Sensors
    // This tells the gyro "0 change has happened since we started"
    gyroReadingAtStart = InertialSensor.rotation(degrees);

    // 5. Reset Encoders to 0 (Clean Slate)
    // It is much safer to track "Change from Start" than "Absolute Wheel Rotations"
    passiveEncoderLeft.resetPosition();
    passiveEncoderRight.resetPosition();
    passiveEncoderX.resetPosition();
    
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
    // 1. Read Current Sensor Values
    double leftEncoder  = passiveEncoderLeft.position(vex::rotationUnits::deg);
    double rightEncoder = passiveEncoderRight.position(vex::rotationUnits::deg);
    // Use the stored previous value if the X encoder is disabled
    double xEncoder     = xEncoderEnabled ? passiveEncoderX.position(vex::rotationUnits::deg) : prevXEncoder;
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

    // 4. Convert Sensor Data to Physical Distances (cm)
    // Calculate the scaling factor once to ensure consistency across all wheels
    double cmPerDegree = encoderWheelCircumferenceCM / 360.0;

    // Forward movement is the average of the left and right tracking wheels
    double forwardDistance = ((deltaLeft + deltaRight) / 2.0) * cmPerDegree;

    // 5. Calculate Local Coordinate Changes
    double deltaXPos = 0.0;
    double deltaYPos = 0.0;
    
    // Convert current heading to radians for trigonometry
    double headingRad = currentRotation * (M_PI / 180.0);

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
        deltaXPos = forwardDistance * cos(headingRad) - lateralMovement * sin(headingRad);
        deltaYPos = forwardDistance * sin(headingRad) + lateralMovement * cos(headingRad);
    }

    // 7. Accumulate Global Position
    globalX += deltaXPos;
    globalY += deltaYPos;
    globalRotation = currentRotation;

    // Debug: Print position to screen (converted back to North-Up for readability)
    Brain.Screen.printAt(10, 20, "X: %.2f, Y: %.2f, H: %.2f",
                         globalX, globalY, getNormalizedModifiedHeading());
}

// ======================================================================
// Compute straight-line distance and heading to target point
// Heading is in Standard Cartesian from atan2
// ======================================================================
void calculatePathToTarget(double currentX, double currentY, double targetX, double targetY,
                           double &distance, double &heading) {
    double deltaX = targetX - currentX;
    double deltaY = targetY - currentY;
    distance = sqrt(deltaX * deltaX + deltaY * deltaY);
    heading = atan2(deltaY, deltaX) * 180.0 / M_PI;
}

// ======================================================================
// Turn to face a target field point using shortest path
// Uses continuous heading for accurate error calculation
// ======================================================================
void turnToPoint(double targetX, double targetY, double breakDistanceInDegrees,
                 double minSpeed, double maxSpeed) {
    currentState = TURNING;
    updateOdometry();
    
    double deltaX = targetX - globalX;
    double deltaY = targetY - globalY;
    
    double targetStandardHeading = atan2(deltaY, deltaX) * 180.0 / M_PI;
    double currentStandardHeading = getContinuousStandardHeading();
    
    double headingError = targetStandardHeading - currentStandardHeading;
    headingError = fmod(headingError + 540.0, 360.0) - 180.0;
    
    double finalTargetHeading = currentStandardHeading + headingError;
    
    turnOdometry(finalTargetHeading, breakDistanceInDegrees, minSpeed, maxSpeed);
    updateOdometry();
    currentState = STATIONARY;
}

void turnLeftToPoint(double targetX, double targetY, double breakDistanceInDegrees,
                     double minSpeed, double maxSpeed, double exitTolerance) {
    currentState = TURNING;
    updateOdometry();
    
    double deltaX = targetX - globalX;
    double deltaY = targetY - globalY;
    
    double targetAbsoluteHeading = atan2(deltaY, deltaX) * 180.0 / M_PI;
    double currentHeading = getContinuousStandardHeading();
    
    double targetHeading = targetAbsoluteHeading;
    while (targetHeading <= currentHeading - 0.5) targetHeading += 360.0;
    
    turnOdometry(targetHeading, breakDistanceInDegrees, minSpeed, maxSpeed, exitTolerance);
    updateOdometry();
    currentState = STATIONARY;
}

void turnRightToPoint(double targetX, double targetY, double breakDistanceInDegrees,
                      double minSpeed, double maxSpeed, double exitTolerance) {
    currentState = TURNING;
    updateOdometry();
    
    double deltaX = targetX - globalX;
    double deltaY = targetY - globalY;
    
    double targetStandardHeading = atan2(deltaY, deltaX) * 180.0 / M_PI;
    double currentStandardHeading = getContinuousStandardHeading();
    
    double targetHeading = targetStandardHeading;
    while (targetHeading >= currentStandardHeading + 0.5) targetHeading -= 360.0;
    
    turnOdometry(targetHeading, breakDistanceInDegrees, minSpeed, maxSpeed, exitTolerance);
    updateOdometry();
    currentState = STATIONARY;
}

// ======================================================================
// Move forward to target point using computed distance and heading
// ======================================================================
void forwardToPoint(double targetX, double targetY, double breakDistance,
                    double minSpeed, double distanceTolerance, double kp_heading, double ki_heading, double kd_heading,
                    double accelHeadingScaling, double decelHeadingScaling,
                    double approachHeadingScaling, double maxSpeed) {
    updateOdometry();
    double distanceToTarget, targetHeading;
    calculatePathToTarget(globalX, globalY, targetX, targetY, distanceToTarget, targetHeading);
    straightOdometryV3(distanceToTarget, breakDistance, targetHeading,
                     minSpeed, distanceTolerance, kp_heading, ki_heading, kd_heading,
                     accelHeadingScaling, decelHeadingScaling,
                     approachHeadingScaling, maxSpeed);
    updateOdometry();
}

// ======================================================================
// Move backward to target point (reverses heading and distance)
// ======================================================================
void backwardToPoint(double targetX, double targetY, double breakDistance,
                     double minSpeed, double distanceTolerance,
                     double kp_heading, double ki_heading, double kd_heading,
                     double accelHeadingScaling, double decelHeadingScaling,
                     double approachHeadingScaling, double maxSpeed) {
    currentState = STRAIGHT;
    if (maxSpeed > 0) maxSpeed = -fabs(maxSpeed);
    updateOdometry();
    double distanceToTarget, targetHeading;
    calculatePathToTarget(globalX, globalY, targetX, targetY, distanceToTarget, targetHeading);
    targetHeading += 180.0;
    distanceToTarget = -fabs(distanceToTarget);
    straightOdometryV3(distanceToTarget, breakDistance, targetHeading,
                     minSpeed, distanceTolerance, kp_heading, ki_heading, kd_heading,
                     accelHeadingScaling, decelHeadingScaling,
                     approachHeadingScaling, maxSpeed);
    updateOdometry();
    currentState = STATIONARY;
}

// ======================================================================
// Background task for continuous odometry updates
// ======================================================================
OdometryTaskParams odometryParams = {false};

int odometryTask(void *params) {
    OdometryTaskParams *p = static_cast<OdometryTaskParams *>(params);
    while (p->isRunning) {
        updateOdometry();
        wait(10, msec);
    }
    return 0;
}

void startOdometryTask() {
    if (!odometryParams.isRunning) {
        odometryParams.isRunning = true;
        task odomTask(odometryTask, &odometryParams);
    }
}

void stopOdometryTask() {
    odometryParams.isRunning = false;
}