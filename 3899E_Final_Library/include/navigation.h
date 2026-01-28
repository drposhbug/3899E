#include "vex.h"  // Make sure this is included to use vex:: types
#include "utils.h"  // Added: Defines Color enum

#ifndef PID_TASKS_H // Include guard to prevent multiple inclusions
#define PID_TASKS_H

void move(double distanceCM, double maxSpeed, vex::directionType dir = vex::forward);
void smartMove(double distanceCM, double maxSpeed, vex::directionType dir = vex::forward, double wallStalledTimeMs = -1);
void pidStraight(double targetHeading, double targetDistanceCM, double speed, double kp_heading = 0.6, double ki_heading = 0, double kd_heading = 0, double distanceOffset = 5.0, vex::brakeType brakeMode = vex::brakeType::brake);
void turn(double targetHeading, 
        double breakDistanceInDegrees, 
        double minSpeed = 17, 
        double maxSpeed = 100);

void straight(double targetDistance, 
            double breakDistance, 
            double minSpeed = 17, 
            double targetHeading = 0, 
            double kp_heading = 0.2, 
            double ki_heading = 0.0, 
            double kd_heading = 0.0, 
            double accelHeadingScaling = 0.25, 
            double decelHeadingScaling = 0.25, 
            double approachHeadingScaling = 0.25, 
            double maxSpeed = 50);

void straightOdometry(double targetDistance, 
    double breakDistance, 
    double targetHeading = 0, 
    double minSpeed = 16, 
    double kp_heading = 0.4, 
    double ki_heading = 0.01, 
    double kd_heading = 0.05, 
    double accelHeadingScaling = 0.2, 
    double decelHeadingScaling = 0.2, 
    double approachHeadingScaling = 0.2, 
    double maxSpeed = 100);   

    void straightOdometryV2(double targetDistance, 
    double breakDistance, 
    double targetHeading = 0, 
    double minSpeed = 16, 
    double distanceTolerance = 6.0,
    double kp_heading = 0.4, 
    double ki_heading = 0.01, 
    double kd_heading = 0.05, 
    double accelHeadingScaling = 0.2, 
    double decelHeadingScaling = 0.2, 
    double approachHeadingScaling = 0.2, 
    double maxSpeed = 100);

 /**
 * straightOdometryV3 - Motion profiling with configurable stopping tolerance
 * Now uses standard Cartesian heading throughout
 */
void straightOdometryV3(double targetDistance, 
                        double breakDistance, 
                        double targetHeading = 0, 
                        double minSpeed = 16, 
                        double distanceTolerance = 6.0,
                        double kp_heading = 0.4, 
                        double ki_heading = 0.01, 
                        double kd_heading = 0.05, 
                        double accelHeadingScaling = 0.2, 
                        double decelHeadingScaling = 0.2, 
                        double approachHeadingScaling = 0.2, 
                        double maxSpeed = 100);
    
void smartStraight(double targetDistance, 
    double breakDistance, 
    double targetHeading = 0, 
    double minSpeed = 16,
    double wallStalledTimeMs = 100,  // Wall detect: -1 = disabled, >0 = exit if stalled for this many ms
    double kp_heading = 0.4, 
    double ki_heading = 0.01, 
    double kd_heading = 0.05, 
    double accelHeadingScaling = 0.2, 
    double decelHeadingScaling = 0.2, 
    double approachHeadingScaling = 0.2, 
    double maxSpeed = 100);
            
void backward(double targetDistance, 
            double breakDistance, 
            double minSpeed = 17, 
            double targetHeading = 0, 
            double kp_heading = 1.7, 
            double ki_heading = 0.0007, 
            double kd_heading = 0.0025, 
            double accelHeadingScaling = 0.275, 
            double decelHeadingScaling = 0.2, 
            double approachHeadingScaling = 0.2, 
            double maxSpeed = 100);     

void arcTurn(double targetDistance, 
             double breakDistance,
             double minSpeed,
             double maxSpeed,
             double turnRadius,    // Radius of turn in cm
             bool turnLeft);



//double targetDistance, double maxSpeed = 100, double targetHeading = 0, double breakDistance = 90, double kp_heading = 0.2, double ki_heading = 0.0, double kd_heading = 0.0, double accelHeadingScaling = 0.4, double decelHeadingScaling = 0.25, double approachHeadingScaling = 0.25, double minSpeed = 15 Pretty good for backwards
void turnOdometry(double targetHeading, 
            double breakDistanceInDegrees, 
            double minSpeed = 25, 
            double maxSpeed = 100,
            double exitTolerance = 16.0);


double launchControl(double targetDriverSpeed, vex::motor& motor, vex::rotation& encoder);

// Define the LaunchControl class
class LaunchControl {
public:
    LaunchControl(vex::motor& motor, vex::rotation& encoder, double slipThresholdValue = 1.1); // Default value changed to 0.1

    double adjustSpeed(double targetPower);

private:
    vex::motor& motor;
    vex::rotation& encoder;

    const double slipThreshold; // Threshold for slip detection
    double motorRPM;      // Motor RPM value
    double encoderRPMScaled;    // Encoder RPM value
};


// Define the LaunchControl class
class tractionControl {
public:
    tractionControl(double minSpeedVoltage, double maxSpeedVoltage, double slipThreshold);
    double tractionControlSpeed(double tractionMotorVoltage, double motorSpeed, double robotSpeed, double accelFactor);

private:
    double minSpeedVoltage;
    double maxSpeedVoltage;
    double slipThreshold;
    //static constexpr double slipThreshold = 1.1; // Class-wide constant
    //static constexpr double accelFactor = 1.15;

}; 

// Adaptive ABS for deceleration phase - prevents wheel lockup while guaranteeing minimum braking
class adaptiveABS {
private:
    double lockThreshold;
    double decelStepVoltage;
    double lastAttemptedVoltage;
    bool wasLockedLastCycle;
    vex::brakeType currentBrakeMode;

public:
    adaptiveABS(double decelStepPercent, double lockThreshold);
    void initialize(double startingVoltage);
    double decelControlSpeed(double wheelSpeed, double robotSpeed);
    vex::brakeType getBrakeMode() { return currentBrakeMode; }
};
// Forward/backward wrappers
void forwardMP(double targetDistance,
            double breakDistance = 35, 
            double targetHeading = 0,
            double minSpeed = 16,
            double kp_heading = 0.615, 
            double ki_heading = 0,
            double kd_heading = 0, 
            double accelHeadingScaling = .10,
            double decelHeadingScaling = 0.05, 
            double approachHeadingScaling = 0.05,
            double maxSpeed = 100);

void backwardMP(double targetDistance,
             double breakDistance = 35, 
             double targetHeading = 0,
             double minSpeed = 16,
             double kp_heading = 0.615, 
             double ki_heading = 0,
             double kd_heading = 0, 
             double accelHeadingScaling = .10,
             double decelHeadingScaling = 0.05, 
             double approachHeadingScaling = 0.05,
             double maxSpeed = 100);

// Turn wrappers
void leftMP(double turnAmount, 
    double breakDistance = 35, 
    double minSpeed = 17, 
    double maxSpeed = 100);

void rightMP(double turnAmount,
             double breakDistance = 35,
             double minSpeed = 17,
             double maxSpeed = 100);

void pivotTurnOdometry(double targetHeading,
             double breakDistanceInDegrees,
             double minSpeed, double maxSpeed);

// ───────────────────────────────────────────────
// V2: Modernized pivot turn with motion profiling, continuous headings,
//     target snapping, and true pivot behavior
//     (recommended replacement - uses brake mode at end)
// ───────────────────────────────────────────────
void pivotTurnOdometryV2(double targetHeading,
                         double breakDistanceInDegrees,
                         double minSpeed,
                         double maxSpeed,
                         double exitTolerance = 2.0);           

void pivotLeftMP(double turnAmount, double breakDistance, double minSpeed, double maxSpeed);

void pivotRightMP(double turnAmount, double breakDistance, double minSpeed, double maxSpeed);

void driveForward(double targetDistance,
             double breakDistance = 10, 
             double targetHeading = 0,
             double minSpeed = 24,
             double kp_heading = 1.1, 
             double ki_heading = 0.0,
             double kd_heading = 0, 
             double accelHeadingScaling = 0.1,
             double decelHeadingScaling = 0.2, 
             double approachHeadingScaling = 0.3,
             double maxSpeed = 100);

void driveBackward(double targetDistance,
              double breakDistance = 10, 
              double targetHeading = 0,
              double minSpeed = 24,
              double kp_heading = 1.1, 
              double ki_heading = 0.0,
              double kd_heading = 0, 
              double accelHeadingScaling = 0.1,
              double decelHeadingScaling = 0.2, 
              double approachHeadingScaling = 0.3,
              double maxSpeed = 100);

void turnRight(double absoluteTargetHeading, 
               double breakDistance, 
               double minSpeed = 25,
               double maxSpeed = 100,   
               double exitTolerance =2);

void turnLeft(double absoluteTargtHeading, 
              double breakDistance, 
              double minSpeed = 25, 
              double maxSpeed = 100,
              double exitTolerance =2);
              
void pidlessForward(double timeMs, double speedPct);

void driveForwardV2(double targetDistance,
             double breakDistance = 10, 
             double targetHeading = 0,
             double minSpeed = 24,
             double distanceTolerance = 5,
             double kp_heading = 1.1, 
             double ki_heading = 0.005,
             double kd_heading = 0, 
             double accelHeadingScaling = 0.1,
             double decelHeadingScaling = 1, 
             double approachHeadingScaling = 0.3,
             double maxSpeed = 100);

void driveBackwardV2(double targetDistance,
             double breakDistance = 10, 
             double targetHeading = 0,
             double minSpeed = 24,
             double distanceTolerance = 5,
             double kp_heading = 1.1, 
             double ki_heading = 0.005,
             double kd_heading = 0, 
             double accelHeadingScaling = 0.1,
             double decelHeadingScaling = 1, 
             double approachHeadingScaling = 0.3,
             double maxSpeed = 100);

void driveForwardV3(double targetDistance,
             double breakDistance = 10, 
             double targetHeading = 0,
             double minSpeed = 24,
             double distanceTolerance = 5,
             double kp_heading = 1.1, 
             double ki_heading = 0,
             double kd_heading = 0, 
             double accelHeadingScaling = 0.1,
             double decelHeadingScaling = 0.1, 
             double approachHeadingScaling = 0.3,
             double maxSpeed = 100);

void driveBackwardV3(double targetDistance,
             double breakDistance = 10, 
             double targetHeading = 0,
             double minSpeed = 24,
             double distanceTolerance = 5,
             double kp_heading = 1.1, 
             double ki_heading = 0,
             double kd_heading = 0, 
             double accelHeadingScaling = 0.1,
             double decelHeadingScaling = 0.1, 
             double approachHeadingScaling = 0.3,
             double maxSpeed = 100);



/**
 * Drives the robot toward a game object using AI Vision for precise final approach.
 *
 * This function uses a color signature to detect and track an object (e.g. ring, mobile goal).
 * It centers the object laterally using heading PID and modulates forward speed using distance PID on pixel width.
 *
 * Primary stop condition: object width reaches or exceeds targetPixelWidth for consecutiveRequired frames.
 * Safety stop: traveled distance exceeds timeoutDistanceCM (odometry-based).
 *
 * Behavior on vision loss:
 *   - If object was previously seen: continues using last known turn correction and pixel width.
 *   - If never seen: falls back to IMU heading hold toward targetHeading and drives forward at up to maxSpeedPct
 *     (PID sees large error since width=0, so full speed until timeout or reacquisition).
 *
 * Designed for short-to-medium final approaches (30–150 cm) after coarse odometry positioning.
 * Includes noise rejection (4-frame rolling median on width) and turn authority limiting.
 *
 * @param targetSignature      Pre-configured color signature from Vision Utility (e.g. AIVision20__redRing)
 * @param targetPixelWidth     Desired minimum object width (pixels) to consider reached (e.g. 140–180)
 * @param timeoutDistanceCM    Maximum distance (cm) to travel before safety abort (odometry)
 * @param targetHeading        Fallback absolute heading (degrees) when vision is unavailable (VEX: 0° usually North/downfield)
 * @param minSpeedPct          Minimum speed floor (% of 12V) to prevent stalling near target
 * @param maxSpeedPct          Maximum allowed speed (%) – caps blind search and approach speed
 * @param brakeMode            Brake type applied on exit (coast for smooth stop, hold for position lock)
 * @param kp_head              Proportional gain for lateral (X-error) heading correction
 * @param ki_head              Integral gain for heading correction (usually 0)
 * @param kd_head              Derivative gain for heading damping
 * @param kp_dist              Proportional gain for distance (pixel width) control
 * @param ki_dist              Integral gain for distance control (small value to reduce steady-state error)
 * @param kd_dist              Derivative gain for smooth deceleration as width increases
 * @param minX                 Left edge of valid detection region (pixels, 0–319)
 * @param maxX                 Right edge of valid detection region
 * @param minY                 Top edge of valid detection region (pixels, 0–239)
 * @param maxY                 Bottom edge of valid detection region
 * @param maxObjectsToCheck    Maximum number of detected objects to evaluate (performance optimization)
 * @param consecutiveRequired  Number of consecutive frames width must be >= targetPixelWidth to stop
 */
void visionDrive(
    vex::aivision::colordesc targetSignature,
    int    targetPixelWidth,
    double timeoutDistanceCM,
    double targetHeading        = 0.0,
    double minSpeedPct          = 20.0,
    double maxSpeedPct          = 85.0,
    vex::brakeType brakeMode    = vex::brakeType::coast,
    double kp_head              = 0.10,
    double ki_head              = 0.00,
    double kd_head              = 0.10,
    double kp_dist              = 1.30,
    double ki_dist              = 0.06,
    double kd_dist              = 0.14,
    int    minX                 = 0,
    int    maxX                 = 320,
    int    minY                 = 0,
    int    maxY                 = 240,
    int    maxObjectsToCheck    = 5,
    int    consecutiveRequired  = 3
);

void visionDriveMinimal(
    vex::aivision::colordesc targetSignature,
    int    targetPixelWidth,
    double targetHeading        = 0.0,
    double minSpeedPct          = 20.0,
    double maxSpeedPct          = 85.0,
    vex::brakeType brakeMode    = vex::brakeType::coast,
    double kp_head              = 0.20,
    double ki_head              = 0.00,
    double kd_head              = 0.00,
    double kp_distToHeadScaling = 0.015,
    double kp_dist              = 1.30,
    double ki_dist              = 0.00,
    double kd_dist              = 0.00
);


#endif // PID_TASKS_H;

// ======================================================================
// CLOSED-LOOP ODOMETRY FUNCTION DECLARATIONS
// Add these to your odometry.h or navigation.h header file
// ======================================================================

/**
 * Closed-loop point-to-point movement function
 * Continuously recalculates distance and heading to target during movement
 * Combines V3 structure with original launch control and adaptive ABS
 * 
 * @param targetX              Target X position in cm
 * @param targetY              Target Y position in cm
 * @param breakDistance        Distance before target to begin deceleration (cm)
 * @param minSpeed             Minimum speed during approach phase (0-100%)
 * @param distanceTolerance    Distance tolerance to exit movement (cm)
 * @param kp_heading           Proportional gain for heading correction
 * @param ki_heading           Integral gain for heading correction
 * @param kd_heading           Derivative gain for heading correction
 * @param finalBrakeMode       Brake mode at final stop (brake/coast/hold)
 * @param accelHeadingScaling  Heading correction scaling during acceleration
 * @param decelHeadingScaling  Heading correction scaling during deceleration
 * @param approachHeadingScaling Heading correction scaling during approach
 * @param maxSpeed             Maximum speed (0-100%)
 */
void moveOdometry(double targetX,
                  double targetY, 
                  double breakDistance, 
                  double minSpeed,
                  double distanceTolerance,
                  double kp_heading, 
                  double ki_heading, 
                  double kd_heading,
                  vex::brakeType finalBrakeMode,
                  double accelHeadingScaling, 
                  double decelHeadingScaling,
                  double approachHeadingScaling, 
                  double maxSpeed);
