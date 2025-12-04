#include "odometry.h"
#include "navigation.h"
#include "robot_config.h"
#include "utils.h"
#include "vex.h"

using namespace vex;

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
    headingOffset = startHeading - InertialSensor.rotation(degrees);

    // Convert cm to degrees for encoders
    double startX_deg = (startX / encoderWheelCircumferenceCM) * 360.0;
    double startY_deg = (startY / encoderWheelCircumferenceCM) * 360.0;

    // Initialize encoders with start position
    passiveEncoderLeft.setPosition(startY_deg, vex::rotationUnits::deg);
    passiveEncoderRight.setPosition(startY_deg, vex::rotationUnits::deg);
    passiveEncoderX.setPosition(startX_deg, vex::rotationUnits::deg);
}

// Function to Update Odometry Readings
void updateOdometry()
{
    // Step 1: Read Encoder and Inertial Sensor Values
    double leftEncoder = passiveEncoderLeft.position(vex::rotationUnits::deg);                            // Current left encoder value in degrees
    double rightEncoder = passiveEncoderRight.position(vex::rotationUnits::deg);                          // Current right encoder value in degrees
    double xEncoder = xEncoderEnabled ? passiveEncoderX.position(vex::rotationUnits::deg) : prevXEncoder; // Read from X encoder if enabled
    double currentRotation = getAdjustedRotation();                                                        // Current heading from the inertial sensor in degrees

    // Step 2: Calculate Delta Changes
    double deltaLeft = leftEncoder - prevLeftEncoder;                // Change in left encoder value
    double deltaRight = rightEncoder - prevRightEncoder;             // Change in right encoder value
    double deltaX = xEncoder - prevXEncoder;                         // Change in X encoder value (if enabled)
    double deltaRotation = currentRotation - prevRotation;

    // Step 3: Update Previous Values for the Next Cycle
    prevLeftEncoder = leftEncoder;   // Store current left encoder value for the next update
    prevRightEncoder = rightEncoder; // Store current right encoder value for the next update
    prevXEncoder = xEncoder;         // Store current X encoder value for the next update
    prevRotation = currentRotation;     // Store current rotation for the next update

    // Step 4: Calculate Average Distance Traveled
    // Average distance moved by the robot based on both left and right encoders
    double avgDeltaDistance = ((deltaLeft + deltaRight) / 2.0) * (encoderWheelCircumferenceCM / 360.0);

    // Step 5: Calculate Movement Components (ΔX and ΔY)
    // Cache trigonometric calculations for better performance
    double deltaXPos = 0.0, deltaYPos = 0.0;
    double headingRad = globalRotation * (M_PI / 180.0);    // Convert current heading to radians

    if (currentState == TURNING)
    {
        double deltaRotationRad = deltaRotation * (M_PI / 180.0); // Change in heading in radians
        if (fabs(deltaRotationRad) > 0.001)
        {
            // Calculate individual turning radii for Y wheels
            double leftRadius = (deltaLeft * (encoderWheelCircumferenceCM / 360.0)) / deltaRotationRad;
            double rightRadius = (deltaRight * (encoderWheelCircumferenceCM / 360.0)) / deltaRotationRad;
            double avgRadius = (leftRadius + rightRadius) / 2.0;

            // Use actual X encoder readings for X position change
            deltaXPos = (deltaX * (encoderWheelCircumferenceCM / 360.0));

            // Y change uses the average of parallel wheels
            deltaYPos = avgRadius * (sin(headingRad + deltaRotationRad) - sin(headingRad));
        }
    }
    else if (currentState == STRAIGHT)
    {
        // Calculate lateral movement from X encoder (sideways drift)
        double lateralMovement = (deltaX * (encoderWheelCircumferenceCM / 360.0));
        
        // Combine forward movement with lateral drift compensation
        // Forward component + lateral component rotated into global frame
        deltaXPos = avgDeltaDistance * cos(headingRad) + lateralMovement * (-sin(headingRad));
        deltaYPos = avgDeltaDistance * sin(headingRad) + lateralMovement * cos(headingRad);
    }

    // Brain.Screen.printAt(10, 80, "xEnabled: %d", xEncoderEnabled);
    //  Brain.Screen.printAt(10, 100, "deltaX: %.2f deltaY: %.2f", deltaXPos, deltaYPos);

    // Step 6: Update Global Position
    globalX += deltaXPos;                        // Update global X position
    globalY += deltaYPos;                        // Update global Y position
    globalRotation = currentRotation; // Update global heading

    // Step 7: Debugging Information (Displayed on Brain Screen)
    Brain.Screen.printAt(10, 20, "X: %.2f, Y: %.2f, Rotation: %.2f", globalX, globalY, globalRotation);
    // Brain.Screen.printAt(10, 40, "DeltaLeft: %.2f, DeltaRight: %.2f", deltaLeft, deltaRight);
    // Brain.Screen.printAt(10, 60, "DeltaHeading: %.2f", deltaHeading);
}

// Function to calculate the distance and heading needed to reach a target point
// Parameters:
//   currentX, currentY: Robot's current position (in cm)
//   targetX, targetY: Target point to reach (in cm)
//   distance: Reference parameter - will be set to distance to target (in cm)
//   heading: Reference parameter - will be set to required heading (in degrees)
void calculatePathToTarget(double currentX, double currentY,
                           double targetX, double targetY,
                           double &distance, double &heading)
{
    double deltaX = targetX - currentX;
    double deltaY = targetY - currentY;

    distance = sqrt(deltaX * deltaX + deltaY * deltaY);

    // Just return the absolute target heading without subtracting current heading
    heading = atan2(deltaY, deltaX) * 180.0 / M_PI;

    // Debug output
    // Brain.Screen.clearScreen();
    // Brain.Screen.printAt(10, 200, "AbsTgt=%.2f AbsCur=%.2f",
    //                    heading, globalHeading);
}

// Function to turn robot to face a specific (x,y) coordinate
// Parameters:
//   targetX, targetY: The point we want to face (in cm)
//   turnSpeed: Maximum speed for the turn (0-100)
//   minSpeed: Minimum speed to maintain during turn
//   breakDistance: Break distance in degrees for motion profiling
void turnToPoint(double targetX, double targetY,
                 double breakDistanceInDegrees,
                 double minSpeed, double maxSpeed)
{
    // Set state to TURNING
    currentState = TURNING;

    // Start timer for timeout safety
    double startTime = Brain.Timer.time(msec);
    const double TIMEOUT = 3000; // 3 seconds maximum for turn

    // Update odometry to get fresh position
    updateOdometry();

    double deltaX = targetX - globalX;
    double deltaY = targetY - globalY;
    double turnAmount = atan2(deltaY, deltaX) * 180.0 / M_PI;
    turnOdometry(turnAmount, breakDistanceInDegrees, minSpeed, maxSpeed);

    // Brain.Screen.clearScreen();  // Clear previous prints
    // Brain.Screen.printAt(10,100, "deltaX: %.2f deltaY: %.2f", deltaX, deltaY);
    // Brain.Screen.printAt(10, 120, "turnAmount calculated: %.2f", turnAmount);

    // Check if we've exceeded timeout
    if ((Brain.Timer.time(msec) - startTime) > TIMEOUT)
    {
        Brain.Screen.printAt(10, 40, "Turn timeout");
    }

    // Re-enable X-encoder tracking
    // xEncoderEnabled = previousXEncoderState;

    // Update odometry after completing turn
    updateOdometry();
    // Set state back to STATIONARY
    currentState = STATIONARY;
    // Add right before final }
    // Brain.Screen.clearScreen();
    // Brain.Screen.printAt(10, 20, "TURN COMPLETE");
    // Brain.Screen.printAt(10, 40, "X: %.2f, Y: %.2f, H: %.2f", globalX, globalY, globalHeading);
    // wait(2000, msec);  // Small delay to ensure we can read the values
}


// Function to force LEFT turn to face a specific (x,y) coordinate  
void turnLeftToPoint(double targetX, double targetY,
                    double breakDistanceInDegrees,
                    double minSpeed, double maxSpeed)
{
    // Set state to TURNING
    currentState = TURNING;

    // Start timer for timeout safety
    double startTime = Brain.Timer.time(msec);
    const double TIMEOUT = 3000; // 3 seconds maximum for turn

    // Update odometry to get fresh position
    updateOdometry();

    double deltaX = targetX - globalX;
    double deltaY = targetY - globalY;
    double targetAbsoluteHeading = atan2(deltaY, deltaX) * 180.0 / M_PI;
    
    // Get current heading in robot coordinate system
    double currentHeading = InertialSensor.rotation(degrees) + headingOffset;
    
    // Start with the target in robot coordinate system
    double targetHeading = -targetAbsoluteHeading + headingOffset;
    
    // FORCE counter-clockwise by making target lower than current (negative error)
    // Keep subtracting 360° until target < current (this forces CCW motion)
    while (targetHeading >= currentHeading) {
        targetHeading -= 360.0;
    }
    
    turnOdometry(targetHeading, breakDistanceInDegrees, minSpeed, maxSpeed);

    // Check if we've exceeded timeout
    if ((Brain.Timer.time(msec) - startTime) > TIMEOUT)
    {
        Brain.Screen.printAt(10, 40, "Turn timeout");
    }

    // Update odometry after completing turn
    updateOdometry();
    // Set state back to STATIONARY
    currentState = STATIONARY;
}

// Function to force RIGHT turn to face a specific (x,y) coordinate
void turnRightToPoint(double targetX, double targetY,
                     double breakDistanceInDegrees,
                     double minSpeed, double maxSpeed)
{
    // Set state to TURNING
    currentState = TURNING;

    // Start timer for timeout safety
    double startTime = Brain.Timer.time(msec);
    const double TIMEOUT = 3000; // 3 seconds maximum for turn

    // Update odometry to get fresh position
    updateOdometry();

    double deltaX = targetX - globalX;
    double deltaY = targetY - globalY;
    double targetAbsoluteHeading = atan2(deltaY, deltaX) * 180.0 / M_PI;
    
    // Get current heading in robot coordinate system
    double currentHeading = InertialSensor.rotation(degrees) + headingOffset;
    
    // Start with the target in robot coordinate system
    double targetHeading = -targetAbsoluteHeading + headingOffset;
    
    // FORCE clockwise by making target higher than current (positive error)
    // Keep adding 360° until target > current (this forces CW motion)
    while (targetHeading <= currentHeading) {
        targetHeading += 360.0;
    }
    
    turnOdometry(targetHeading, breakDistanceInDegrees, minSpeed, maxSpeed);

    // Check if we've exceeded timeout
    if ((Brain.Timer.time(msec) - startTime) > TIMEOUT)
    {
        Brain.Screen.printAt(10, 40, "Turn timeout");
    }

    // Update odometry after completing turn
    updateOdometry();
    // Set state back to STATIONARY
    currentState = STATIONARY;
}

// Function to turn robot to face a specific (x,y) coordinate
// Parameters:
//   targetX, targetY: The point we want to face (in cm)
//   turnSpeed: Maximum speed for the turn (0-100)
//   minSpeed: Minimum speed to maintain during turn
//   breakDistance: Break distance in degrees for motion profiling

// Function to move robot in a straight line to a specific (x,y) coordinate
// Parameters:
//   targetX, targetY: The point to move to (in cm)
//   straightSpeed: Maximum forward speed (0-100)
//   breakDistance: Break distance for motion profiling (in cm)
//   kp_heading: Proportional gain for heading correction
//   ki_heading: Integral gain for heading correction
//   kd_heading: Derivative gain for heading correction
//   accelHeadingScaling: Heading correction scaling during acceleration
//   decelHeadingScaling: Heading correction scaling during deceleration
//   approachHeadingScaling: Heading correction scaling during final approach
//   minSpeed: Minimum speed to maintain during movement
void forwardToPoint(double targetX, double targetY,
                    double breakDistance, double minSpeed,
                    double kp_heading, double ki_heading,
                    double kd_heading, double accelHeadingScaling,
                    double decelHeadingScaling, double approachHeadingScaling,
                    double maxSpeed)
{
    // Set state to STRAIGHT
    currentState = STRAIGHT;

    // Start timer for timeout safety
    double startTime = Brain.Timer.time(msec);
    const double TIMEOUT = 5000; // 5 seconds maximum for straight movement

    // Update odometry to get fresh position
    updateOdometry();

    // Brain.Screen.printAt(10, 140, "Calc heading: %.2f", targetHeading);
    // Brain.Screen.printAt(10, 160, "Current pos: %.2f, %.2f", globalX, globalY);
    // Brain.Screen.printAt(10, 180, "Target pos: %.2f, %.2f", targetX, targetY);
    // wait(1000, msec);  // Give us time to see the values

    // targetHeading = 0;

    double distanceToTarget, targetHeading;
    calculatePathToTarget(globalX, globalY, targetX, targetY, distanceToTarget, targetHeading);
    
    // Move straight with PID heading correction
    straightOdometry(distanceToTarget, breakDistance, targetHeading, minSpeed,
                    kp_heading, ki_heading, kd_heading, accelHeadingScaling,
                    decelHeadingScaling, approachHeadingScaling, maxSpeed);

    // Check if we've exceeded timeout
    if ((Brain.Timer.time(msec) - startTime) > TIMEOUT)
    {
        //   Brain.Screen.printAt(10, 60, "Straight move timeout");
    }

    // Update odometry after completing movement
    updateOdometry();
    // Set state back to STATIONARY
    currentState = STATIONARY;
    //  Brain.Screen.clearScreen();
    // Add right before final }
    // Brain.Screen.clearScreen();
    // Brain.Screen.printAt(10, 20, "STRAIGHT COMPLETE");
    // Brain.Screen.printAt(10, 40, "X: %.2f, Y: %.2f, H: %.2f", globalX, globalY, globalHeading);
    // wait(2000, msec);  // Small delay to ensure we can read the values
}

void backwardToPoint(double targetX, double targetY,
                     double breakDistance, double minSpeed,
                     double kp_heading, double ki_heading,
                     double kd_heading, double accelHeadingScaling,
                     double decelHeadingScaling, double approachHeadingScaling,
                     double maxSpeed)
{
    // Set state to STRAIGHT
    currentState = STRAIGHT;

    if (maxSpeed > 0)
    {
        maxSpeed = -fabs(maxSpeed);
    }

    // Start timer for timeout safety
    double startTime = Brain.Timer.time(msec);
    const double TIMEOUT = 5000; // 5 seconds maximum for straight movement

    // Update odometry to get fresh position
    updateOdometry();

    // Calculate initial path to target
    double distanceToTarget, targetHeading;
    calculatePathToTarget(globalX, globalY, targetX, targetY, distanceToTarget, targetHeading);
    targetHeading = targetHeading + 180.0;  // Just add 180 for backward
    distanceToTarget = -fabs(distanceToTarget);

    // Brain.Screen.printAt(10, 140, "Calc heading: %.2f", targetHeading);
    // Brain.Screen.printAt(10, 160, "Current pos: %.2f, %.2f", globalX, globalY);
    // Brain.Screen.printAt(10, 180, "Target pos: %.2f, %.2f", targetX, targetY);
    // wait(1000, msec);  // Give us time to see the values

    // targetHeading = 0;

    // Move straight with PID heading correction
    straightOdometry(distanceToTarget, breakDistance, minSpeed, targetHeading,
                    kp_heading, ki_heading, kd_heading, accelHeadingScaling,
                    decelHeadingScaling, approachHeadingScaling, maxSpeed);

    // Check if we've exceeded timeout
    if ((Brain.Timer.time(msec) - startTime) > TIMEOUT)
    {
        //   Brain.Screen.printAt(10, 60, "Straight move timeout");
    }

    // Update odometry after completing movement
    updateOdometry();
    // Set state back to STATIONARY
    currentState = STATIONARY;
    //  Brain.Screen.clearScreen();
    // Add right before final }
    // Brain.Screen.clearScreen();
    // Brain.Screen.printAt(10, 20, "STRAIGHT COMPLETE");
    // Brain.Screen.printAt(10, 40, "X: %.2f, Y: %.2f, H: %.2f", globalX, globalY, globalHeading);
    // wait(2000, msec);  // Small delay to ensure we can read the values
}


// Initialize odometry task parameters
OdometryTaskParams odometryParams = {false};

// Odometry task function
int odometryTask(void *params)
{
    OdometryTaskParams *p = static_cast<OdometryTaskParams *>(params);
    
    while (p->isRunning)
    {
        updateOdometry();
        
        // Small delay to prevent CPU overload
        wait(10, msec);
    }
    
    return 0;
}

// Function to start the odometry tracking task
void startOdometryTask()
{
    if (!odometryParams.isRunning)
    {
        odometryParams.isRunning = true;
        
        // Create and start the task
        task odomTask(odometryTask, &odometryParams);
    }
}

// Function to stop the odometry tracking task
void stopOdometryTask()
{
    odometryParams.isRunning = false;
}