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

void pivotLeftMP(double turnAmount, double breakDistance, double minSpeed, double maxSpeed);

void pivotRightMP(double turnAmount, double breakDistance, double minSpeed, double maxSpeed);

void driveForward(double targetDistance,
             double breakDistance = 10, 
             double targetHeading = 0,
             double minSpeed = 24,
             double kp_heading = 1.1, 
             double ki_heading = 0.005,
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
              double ki_heading = 0.005,
              double kd_heading = 0, 
              double accelHeadingScaling = 0.1,
              double decelHeadingScaling = 1, 
              double approachHeadingScaling = 0.3,
              double maxSpeed = 100);

void turnRight(double absoluteTargetHeading, 
               double breakDistance, 
               double minSpeed = 25,
               double maxSpeed = 100,   
               double exitTolerance =16);

void turnLeft(double absoluteTargtHeading, 
              double breakDistance, 
              double minSpeed = 25, 
              double maxSpeed = 100,
              double exitTolerance =16);
              
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



/**
 * Drives the robot toward a game object using AI Vision color signatures for precise final approach corrections.
 * 
 * Pass one of your configured color signatures from the Vision Utility as the first argument:
 *   - AIVision20__blueCube
 *   - AIVision20__orangeGoal
 *   - AIVision20__redCube
 * 
 * The function uses the provided color signature to detect the object.
 * When detected in the bounding box, centers it in the camera frame using heading PID.
 * If no valid object is found, falls back to IMU correction toward the targetHeading.
 * 
 * Designed for short final approaches (<50 cm) after fast odometry moves.
 * Prioritizes speed with vision correcting lateral/heading errors.
 * 
 * @param targetSignature      The configured color signature (e.g., AIVision20__redCube)
 * @param targetDistanceCM     Distance to travel (positive = forward, negative = reverse)
 * @param targetHeading        Absolute target heading in degrees (VEX field: 0° = North)
 *                             Used as fallback when vision tracking is lost
 * @param kp_head              Proportional gain for heading correction (X-error)
 * @param ki_head              Integral gain for heading correction
 * @param kd_head              Derivative gain for heading correction
 * @param kp_dist              Proportional gain for distance control
 * @param ki_dist              Integral gain for distance control
 * @param kd_dist              Derivative gain for distance control
 * @param brakeMode            Brake type to apply at the end (coast, brake, hold)
 * @param distanceTolerance    Acceptable error in cm to consider distance goal achieved
 * @param minSpeed             Minimum speed floor to prevent stalling near target (%)
 * @param visionTimeout        Maximum time (ms) before aborting due to no progress
 * @param minX/maxX/minY/maxY  Bounding box in pixels (0-319 x, 0-239 y) for valid detections
 */
void visionDrive(
    vex::aivision::colordesc targetSignature,
    double targetDistanceCM,
    double targetHeading = 0.0,
    double kp_head = 0.45,
    double ki_head = 0.01,
    double kd_head = 0.05,
    double kp_dist = 1.2,
    double ki_dist = 0.02,
    double kd_dist = 0.08,
    vex::brakeType brakeMode = vex::brakeType::brake,
    double distanceTolerance = 2.0,
    double minSpeed = 15.0,
    int visionTimeout = 5000,
    int minX = 50,
    int maxX = 270,
    int minY = 100,
    int maxY = 212
);

#endif // PID_TASKS_H;
