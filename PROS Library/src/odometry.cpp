#include "odometry.hpp"
#include "navigation.hpp"
#include "robot_config.hpp"
#include "utils.hpp"
#include "main.h"
#include <cmath>

// Global Odometry Variables
double globalX = 0.0;       // Tracks the robot's X-coordinate on the field
double globalY = 0.0;       // Tracks the robot's Y-coordinate on the field
double globalRotation = 0.0; // Tracks the robot's cumulative rotation (in degrees)

// Previous Encoder Readings (used to calculate delta changes)
double prevLeftEncoder = 0.0;  // Stores the last reading from the left encoder
double prevRightEncoder = 0.0; // Stores the last reading from the right encoder
double prevXEncoder = 0.0;     // Stores the last reading from the X encoder (if enabled)
double prevRotation = 0.0;     // Stores the last rotation from the inertial sensor

// Encoder State Management
bool xEncoderEnabled = true; // Determines whether the X encoder is active (disabled during spot turns)

enum RobotState
{
    STATIONARY,
    TURNING,
    STRAIGHT
};

RobotState currentState = STATIONARY; // Initialize robot state as stationary

// Function to set custom starting position and heading
void setStartPosition(double startX, double startY, double startHeading)
{
    // Set global position variables
    globalX = startX;
    globalY = startY;

    // Calculate offset between current inertial reading and desired heading
    headingOffset = startHeading - inertialSensor.get_rotation();

    // Convert cm to degrees for encoders
    double startX_deg = (startX / encoderWheelCircumferenceCM) * 360.0;
    double startY_deg = (startY / encoderWheelCircumferenceCM) * 360.0;

    // Initialize encoders with start position
    passiveEncoderLeft.set_position(startY_deg * 100);  // PROS uses centidegrees
    passiveEncoderRight.set_position(startY_deg * 100);
    passiveEncoderX.set_position(startX_deg * 100);
}

// Function to Update Odometry Readings
void updateOdometry()
{
    // Step 1: Read Encoder and Inertial Sensor Values
    double leftEncoder = passiveEncoderLeft.get_position() / 100.0;    // Convert from centidegrees
    double rightEncoder = passiveEncoderRight.get_position() / 100.0;
    double xEncoder = xEncoderEnabled ? (passiveEncoderX.get_position() / 100.0) : prevXEncoder;
    double currentRotation = getAdjustedRotation();

    // Step 2: Calculate Delta Changes
    double deltaLeft = leftEncoder - prevLeftEncoder;
    double deltaRight = rightEncoder - prevRightEncoder;
    double deltaX = xEncoder - prevXEncoder;
    double deltaRotation = currentRotation - prevRotation;

    // Step 3: Update Previous Values for the Next Cycle
    prevLeftEncoder = leftEncoder;
    prevRightEncoder = rightEncoder;
    prevXEncoder = xEncoder;
    prevRotation = currentRotation;

    // Step 4: Calculate Average Distance Traveled
    double avgDeltaDistance = ((deltaLeft + deltaRight) / 2.0) * (encoderWheelCircumferenceCM / 360.0);

    // Step 5: Calculate Movement Components (ΔX and ΔY)
    double deltaXPos = 0.0, deltaYPos = 0.0;
    double headingRad = globalRotation * (M_PI / 180.0);

    if (currentState == TURNING)
    {
        double deltaRotationRad = deltaRotation * (M_PI / 180.0);
        if (fabs(deltaRotationRad) > 0.001)
        {
            // Calculate individual turning radii for Y wheels
            double leftRadius = (deltaLeft * (encoderWheelCircumferenceCM / 360.0)) / deltaRotationRad;
            double rightRadius = (deltaRight * (encoderWheelCircumferenceCM / 360.0)) / deltaRotationRad;
            double avgRadius = (leftRadius + rightRadius) / 2.0;

            // Use actual Y encoder readings for Y position change  
            deltaYPos = (deltaX * (encoderWheelCircumferenceCM / 360.0));

            // X change uses the average of parallel wheels
            deltaXPos = avgRadius * (sin(headingRad + deltaRotationRad) - sin(headingRad));
        }
    }
    else if (currentState == STRAIGHT)
    {
        // Calculate lateral movement from X encoder (sideways drift)
        double lateralMovement = (deltaX * (encoderWheelCircumferenceCM / 360.0));
        
        // Combine forward movement with lateral drift compensation
        deltaYPos = avgDeltaDistance * cos(headingRad) + lateralMovement * (-sin(headingRad));
        deltaXPos = avgDeltaDistance * sin(headingRad) + lateralMovement * cos(headingRad);
    }

    // Step 6: Update Global Position
    globalX += deltaXPos;
    globalY += deltaYPos;
    globalRotation = fmod(getAdjustedRotation() + 360.0, 360.0);

    // Step 7: Debugging Information (Displayed on Brain Screen)
    // Note: PROS uses printf to terminal instead of Brain.Screen
    // Uncomment if needed for debugging
    // printf("X: %.2f, Y: %.2f, Rotation: %.2f\n", globalX, globalY, globalRotation);
}

// Function to calculate the distance and heading needed to reach a target point
void calculatePathToTarget(double currentX, double currentY,
                           double targetX, double targetY,
                           double &distance, double &heading)
{
    double deltaX = targetX - currentX;
    double deltaY = targetY - currentY;

    distance = sqrt(deltaX * deltaX + deltaY * deltaY);

    // User coordinate system: 0° = +Y (forward), 90° = +X (right)
    double userHeading = atan2(deltaX, deltaY) * 180.0 / M_PI;
    
    // Convert to VEX internal coordinates (CW positive, matches inertial sensor)
    heading = -userHeading + headingOffset;
}

// Function to turn robot to face a specific (x,y) coordinate
void turnToPoint(double targetX, double targetY,
                 double breakDistanceInDegrees,
                 double minSpeed, double maxSpeed)
{
    // Set state to TURNING
    currentState = TURNING;

    // Start timer for timeout safety
    uint32_t startTime = pros::millis();
    const uint32_t TIMEOUT = 3000; // 3 seconds maximum for turn

    // Update odometry to get fresh position
    updateOdometry();

    // Calculate turn parameters
    double deltaX = targetX - globalX;
    double deltaY = targetY - globalY;

    double targetAbsoluteHeading = atan2(deltaX, deltaY) * 180.0 / M_PI;
    double currentRobotHeading = getAdjustedRotation();
    double desiredRobotHeading = targetAbsoluteHeading + headingOffset;

    // Calculate shortest turn direction (-180° … +180°)
    double headingError = desiredRobotHeading - currentRobotHeading;
    headingError = fmod(headingError + 540.0, 360.0) - 180.0;

    // Final target heading that turnOdometry expects (unwrapped)
    double finalTargetHeading = currentRobotHeading + headingError;

    // Call the turn function
    turnOdometry(finalTargetHeading, breakDistanceInDegrees, minSpeed, maxSpeed);

    // Check timeout
    if ((pros::millis() - startTime) > TIMEOUT)
    {
        printf("Turn timeout\n");
    }

    // Update odometry after completing turn
    updateOdometry();
    currentState = STATIONARY;
}

// Function to force LEFT turn to face a specific (x,y) coordinate  
void turnLeftToPoint(double targetX, double targetY,
                    double breakDistanceInDegrees,
                    double minSpeed, double maxSpeed)
{
    currentState = TURNING;
    uint32_t startTime = pros::millis();
    const uint32_t TIMEOUT = 3000;

    updateOdometry();

    double deltaX = targetX - globalX;
    double deltaY = targetY - globalY;
    double targetAbsoluteHeading = atan2(deltaY, deltaX) * 180.0 / M_PI;
    
    double currentHeading = inertialSensor.get_rotation() + headingOffset;
    double targetHeading = -targetAbsoluteHeading + headingOffset;
    
    // FORCE counter-clockwise
    while (targetHeading >= currentHeading) {
        targetHeading -= 360.0;
    }
    
    turnOdometry(targetHeading, breakDistanceInDegrees, minSpeed, maxSpeed);

    if ((pros::millis() - startTime) > TIMEOUT)
    {
        printf("Turn timeout\n");
    }

    updateOdometry();
    currentState = STATIONARY;
}

// Function to force RIGHT turn to face a specific (x,y) coordinate
void turnRightToPoint(double targetX, double targetY,
                     double breakDistanceInDegrees,
                     double minSpeed, double maxSpeed)
{
    currentState = TURNING;
    uint32_t startTime = pros::millis();
    const uint32_t TIMEOUT = 3000;

    updateOdometry();

    double deltaX = targetX - globalX;
    double deltaY = targetY - globalY;
    double targetAbsoluteHeading = atan2(deltaY, deltaX) * 180.0 / M_PI;
    
    double currentHeading = inertialSensor.get_rotation() + headingOffset;
    double targetHeading = -targetAbsoluteHeading + headingOffset;
    
    // FORCE clockwise
    while (targetHeading <= currentHeading) {
        targetHeading += 360.0;
    }
    
    turnOdometry(targetHeading, breakDistanceInDegrees, minSpeed, maxSpeed);

    if ((pros::millis() - startTime) > TIMEOUT)
    {
        printf("Turn timeout\n");
    }

    updateOdometry();
    currentState = STATIONARY;
}

// Function to move robot in a straight line to a specific (x,y) coordinate
void forwardToPoint(double targetX, double targetY, double breakDistance, 
                   double minSpeed, double kp_heading, double ki_heading, 
                   double kd_heading, double accelHeadingScaling, 
                   double decelHeadingScaling, double approachHeadingScaling, 
                   double maxSpeed)
{
    updateOdometry();
    
    double distanceToTarget, targetHeading;
    calculatePathToTarget(globalX, globalY, targetX, targetY, distanceToTarget, targetHeading);

    targetHeading = -targetHeading;
    
    straightOdometry(distanceToTarget, breakDistance, targetHeading, minSpeed, 
              kp_heading, ki_heading, kd_heading, accelHeadingScaling, 
              decelHeadingScaling, approachHeadingScaling, maxSpeed);
    
    updateOdometry();
}

void backwardToPoint(double targetX, double targetY, 
                     double minSpeed,
                     double breakDistance,
                     double kp_heading, double ki_heading,
                     double kd_heading, double accelHeadingScaling,
                     double decelHeadingScaling, double approachHeadingScaling,
                     double maxSpeed)
{
    currentState = STRAIGHT;

    if (maxSpeed > 0)
    {
        maxSpeed = -fabs(maxSpeed);
    }

    uint32_t startTime = pros::millis();
    const uint32_t TIMEOUT = 5000;

    updateOdometry();

    double distanceToTarget, targetHeading;
    calculatePathToTarget(globalX, globalY, targetX, targetY, distanceToTarget, targetHeading);
    targetHeading = targetHeading + 180.0;
    distanceToTarget = -fabs(distanceToTarget);

    straightOdometry(distanceToTarget, breakDistance, minSpeed, targetHeading,
                    kp_heading, ki_heading, kd_heading, accelHeadingScaling,
                    decelHeadingScaling, approachHeadingScaling, maxSpeed);

    if ((pros::millis() - startTime) > TIMEOUT)
    {
        printf("Straight move timeout\n");
    }

    updateOdometry();
    currentState = STATIONARY;
}

// Initialize odometry task parameters
OdometryTaskParams odometryParams = {false};

// Odometry task function - PROS version
void odometryTask(void* params)
{
    OdometryTaskParams* p = static_cast<OdometryTaskParams*>(params);
    
    while (p->isRunning)
    {
        updateOdometry();
        pros::delay(10);  // PROS uses delay instead of task::sleep
    }
}

// Function to start the odometry tracking task
void startOdometryTask()
{
    if (!odometryParams.isRunning)
    {
        odometryParams.isRunning = true;
        
        // Create and start the task - PROS version
        pros::Task odomTask(odometryTask, &odometryParams, "Odometry Task");
    }
}

// Function to stop the odometry tracking task
void stopOdometryTask()
{
    odometryParams.isRunning = false;
}
