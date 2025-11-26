#include "vex.h"
#include "robot_config.h" // Move this before utils.h since utils.h needs ArmPosition
#include "utils.h"
#include <cmath>
#include <algorithm>

using namespace vex;

// Minimum threshold for division operations to prevent divide by zero errors
const double DIV_BY_ZERO_THRESHOLD = 0.001;

double normalizeHeading(double heading) {
    // Add 180 to the heading and take the modulus with 360 to wrap the angle.
    // This ensures the result is between [0, 360) degrees.
    heading = fmod(heading + 180.0, 360.0);

    // If the result is negative (e.g., due to fmod returning a negative value),
    // add 360 to shift it into the [0, 360) range.
    if (heading < 0)
        heading += 360.0;

    // Subtract 180 to convert the range from [0, 360) to [-180, 180).
    heading -= 180.0;
    
    // Ensure 180 stays as 180, and -180 stays as -180.
    if (heading == -180.0)
        heading = 180.0;

    // Return the normalized heading value.
    return heading;
}

double normalizeHeading180(double heading) {
    heading = fmod(heading, 360.0);
    if (heading > 180) {
        heading -= 360;
    } else if (heading < -180) {
        heading += 360;
    }
    return heading;
}

double normHeading(double heading)
{
    // Add 180 to the heading and take the modulus with 360 to wrap the angle.
    // This ensures the result is between [0, 360) degrees.
    double normHeading = fmod(heading + 180.0, 360.0);

    // If the result is negative (e.g., due to fmod returning a negative value),
    // add 360 to shift it into the [0, 360) range.
    if (normHeading < 0)
    {
        normHeading += 360.0;
    }

    // Subtract 180 to convert the range from [0, 360) to [-180, 180).
    normHeading -= 180.0;

    if ((heading + 180.0) > 0 && normHeading == -180)
    {
        // Ensure 180 stays as 180, and -180 stays as -180.
        normHeading = 180.0;
    }

    // Return the normalized heading value.
    return normHeading;
}

double normHeading360(double heading)
{
    // Take the modulus with 360 to wrap the angle.
    // This ensures the result is between [0, 360) degrees.
    double normHeading = fmod(heading, 360.0);

    // If the result is negative (e.g., due to fmod returning a negative value),
    // add 360 to shift it into the [0, 360) range.
    if (normHeading < 0)
    {
        normHeading += 360.0;
    }

    // Return the normalized heading value in [0, 360) range
    return normHeading;
}

/**
 * Calculate the shortest path error between target and current heading.
 * Ensures the error is always in the range of -180 to +180 degrees for smooth PID control.
 *
 * This ensures:
 * 1. PID gets continuous error values with no discontinuities at 0/360
 * 2. Robot always takes the shortest path to target heading
 * 3. Error magnitude never exceeds 180 degrees
 */
double getHeadingError360(double targetHeading, double currentHeading)
{
    // First normalize both headings to 0-360 range
    double error = normHeading360(targetHeading) - normHeading360(currentHeading);

    // Convert error to -180 to +180 range for shortest path
    if (error > 180)
    {
        error -= 360; // If error > 180, shorter to turn CCW
    }
    else if (error < -180)
    {
        error += 360; // If error < -180, shorter to turn CW
    }

    return error; // Returns error in range -180 to +180 degrees
}

double getHeadingError(double targetHeading, double currentHeading)
{
    double error = targetHeading - currentHeading;

    // Convert error to -180 to +180 range for shortest path
    if (error > 180)
    {
        error -= 360;
    }
    else if (error < -180)
    {
        error += 360;
    }

    return error;
}

// Implement slip detection logic
bool isSlipping(double motorSpeed, double encoderSpeed)
{
    const double slipThreshold = 0.1;
    return (motorSpeed > encoderSpeed * (1 + slipThreshold));
}

// Implement lock-up detection logic
bool isLocking(double motorSpeed, double encoderSpeed)
{
    const double lockThreshold = .85; // 10% threshold
    return (motorSpeed < encoderSpeed * (1.0 - lockThreshold));
}

// Color Detection Constants for utils.cpp
// const double RED_HUE_MIN_1 = 340.0;  // First red range (340°-360°)
// const double RED_HUE_MAX_1 = 360.0;
const double RED_HUE_MIN_1 = 335.0; // First red range (340°-360°)
const double RED_HUE_MAX_1 = 365.0;
const double RED_HUE_MIN_2 = 0.0; // Second red range (0°-15°)
const double RED_HUE_MAX_2 = 15.0;
// const double BLUE_HUE_MIN = 215.0;   // Blue range
// const double BLUE_HUE_MAX = 225.0;
const double BLUE_HUE_MIN = 210.0; // Blue range
const double BLUE_HUE_MAX = 230.0;
const double MIN_BRIGHTNESS = 15.0; // Minimum brightness threshold

// Function to initialize the Optical Sensor
void initializeOpticalSensor()
{
    opticalSensor.setLightPower(100, percent); // Turn on the sensor light at 100% power
    opticalSensor.setLight(ledState::on);      // Ensure the light is on
}

// Track consecutive detections to prevent false positives
static int consecutiveDetections = 0;
static bool lastDetectedColor = false; // false = no color, true = color detected

bool detectColor()
{
    double hue = opticalSensor.hue();
    double brightness = opticalSensor.brightness();
    bool colorDetected = false;

    // Check brightness threshold
    if (brightness < MIN_BRIGHTNESS)
    {
        consecutiveDetections = 0;
        lastDetectedColor = false;
        return false;
    }

    // Check for red or blue
    if (((hue >= RED_HUE_MIN_1 && hue <= RED_HUE_MAX_1) ||
         (hue >= RED_HUE_MIN_2 && hue <= RED_HUE_MAX_2)) ||
        (hue >= BLUE_HUE_MIN && hue <= BLUE_HUE_MAX))
    {
        colorDetected = true;
    }

    // Handle consecutive detections
    if (colorDetected == lastDetectedColor && colorDetected)
    {
        consecutiveDetections++;
    }
    else
    {
        consecutiveDetections = 1;
    }

    lastDetectedColor = colorDetected;

    // Return true if we have enough consecutive detections
    return (consecutiveDetections >= 3); // Require 3 consecutive detections
}

// Reset detection state if needed
void resetColorDetection()
{
    consecutiveDetections = 0;
    lastDetectedColor = false;
}

// Handle the ejection process
void ringEjection()
{
    // Spin forward by 720 degrees (2 rotations) at 100% velocity
    intakeMotor.spinFor(forward, 720, rotationUnits::deg, 100, velocityUnits::pct);

    // Spin in reverse by 180 degrees to eject the ring
    intakeMotor.spinFor(reverse, 180, rotationUnits::deg, 100, velocityUnits::pct);

    // Resume forward intake at 12 volts
    intakeMotor.spin(forward, 12, voltageUnits::volt);
}

// Generic function to control any motor
// Function to control any motor
void MotorControl(motor &targetMotor, int DelayStart, int OnTime, directionType dir)
{
    task::sleep(DelayStart);                       // Wait before starting
    targetMotor.spin(dir, 12, voltageUnits::volt); // Spin motor
    task::sleep(OnTime);                           // Keep spinning
    targetMotor.stop();                            // Stop motor
}

// Wrapper function matching the expected thread signature
int MotorControlThread(void *params)
{
    MotorControlParams *mcParams = static_cast<MotorControlParams *>(params);
    MotorControl(*mcParams->targetMotor, mcParams->DelayStart, mcParams->OnTime, mcParams->dir);
    return 0; // Return value as required by thread signature
}

bool isAccelerating(double targetDriverSpeed, double currentSpeed)
{
    // If both speeds are in the same direction
    if ((targetDriverSpeed * currentSpeed) > 0)
    {
        // Check if the target speed is greater than the current speed
        return fabs(targetDriverSpeed) > fabs(currentSpeed);
    }
    // If the speeds are in opposite directions
    else if ((targetDriverSpeed * currentSpeed) < 0)
    {

        // Moving from positive to negative or vice versa is still a sign of acceleration
        return true;
    }

    // If both are zero, or no acceleration
    return false;
}

// Function to calculate motor speed in cm per second using a constant circumference
double getMotorSpeed(vex::motor &motor)
{
    // Get motor velocity in RPM and convert to cm/s using the constant circumference
    return motor.velocity(vex::velocityUnits::rpm) / gearRatio * wheelCircumferenceCM / 60.0;
}

// Function to calculate motor encoder speed in cm per second
double getEncoderSpeed(vex::rotation &encoder)
{
    return encoder.velocity(vex::velocityUnits::rpm) * encoderWheelCircumferenceCM / 60.0;
}

double convertHeading(double currentHeading, double offset)
{
    return currentHeading - offset;
}

void PIDVoltageCapCorrection(double &leftVoltage, double &rightVoltage, double absoluteMaxVoltage)
{
    double pidCorrectionDiff = fabs(leftVoltage - rightVoltage);

    if (std::abs(leftVoltage) > absoluteMaxVoltage)
    {
        leftVoltage = std::copysign(absoluteMaxVoltage, leftVoltage);
        rightVoltage = std::copysign((absoluteMaxVoltage - pidCorrectionDiff), rightVoltage);
    }
    else if (std::abs(rightVoltage) > absoluteMaxVoltage)
    {
        leftVoltage = std::copysign((absoluteMaxVoltage - pidCorrectionDiff), leftVoltage);
        rightVoltage = std::copysign(absoluteMaxVoltage, rightVoltage);
    }
}

/**
 * Calculates slip ratio between wheel and robot speeds
 * @param wheelSpeed Individual wheel speed in RPM
 * @param robotSpeed Robot's ground speed in RPM
 * @return Slip ratio from 0 to 1: 0 = no slip, 1 = full slip
 */
double calculateSlipRatio(double wheelSpeed, double robotSpeed)
{
    double maxSpeed = std::max(std::fabs(wheelSpeed), std::fabs(robotSpeed));
    if (maxSpeed < DIV_BY_ZERO_THRESHOLD)
    {
        return 0.0;
    }
    return std::fabs((wheelSpeed - robotSpeed) / maxSpeed);
}

/**
 * Calculates rolling average of a value over N samples
 * @param newValue Latest measurement to include in average
 * @param currentAverage Previous rolling average value
 * @param n Number of samples to average over (typical: 5-10 for 50-100ms window at 10ms rate)
 * @return Updated rolling average
 */
float rollingAverage(float newValue, float currentAverage, int n)
{
    return currentAverage * (n - 1) / n + newValue / n;
}

int armTask(void *params)
{
    ArmTaskParams *p = static_cast<ArmTaskParams *>(params);

    while (p->isRunning)
    {
        if (p->moveRequested && p->delayMs > 0)
        {
            wait(p->delayMs, msec); // Wait for specified delay
            armMotor1.spinToPosition(p->targetPosition, rotationUnits::deg, 100, velocityUnits::pct, false);
            armMotor2.spinToPosition(p->targetPosition, rotationUnits::deg, 100, velocityUnits::pct, false);
            p->moveRequested = false; // Reset the move request
        }
        wait(10, msec); // Small delay to prevent CPU overload
    }
    return 0;
}

int colorDetectionTask(void *params)
{
    ColorTaskParams *p = static_cast<ColorTaskParams *>(params);

    while (p->isRunning)
    {
        double hue = opticalSensor.hue();
        Brain.Screen.clearLine(1);    // Clear line 1 before printing
        Brain.Screen.setCursor(1, 1); // Set cursor to beginning of line 1

        if (((hue >= RED_HUE_MIN_1 && hue <= RED_HUE_MAX_1) ||
             (hue >= RED_HUE_MIN_2 && hue <= RED_HUE_MAX_2)) &&
            p->targetColor == Color::RED)
        {
            Brain.Screen.print("RED");
            wait(p->delayMs, msec);
            intakeMotor.stop(); // Stop the motor
            wait(50, msec);
            intakeMotor.spin(reverse, 100, velocityUnits::pct); // Spins continuously until stopped
        }
        else if ((hue >= BLUE_HUE_MIN && hue <= BLUE_HUE_MAX) && p->targetColor == Color::BLUE)
        {
            Brain.Screen.print("BLUE");
            wait(p->delayMs, msec);
            intakeMotor.stop(); // Stop the motor
            wait(50, msec);
            intakeMotor.spin(reverse, 100, velocityUnits::pct); // Spins continuously until stopped
        }

        wait(10, msec); // Small delay to prevent CPU overload
    }
    return 0;
}

// Gets heading in counterclockwise degrees
// Converts raw clockwise sensor reading & applies calibration offset
double getAdjustedHeading()
{
    // Convert clockwise sensor to counterclockwise & apply offset
    return normHeading(InertialSensor.heading() + headingOffset);
}

double convertToVEXHeading(double euclideanHeading)
{
    // Convert counterclockwise to clockwise
    double vexHeading = fmod(360.0 - euclideanHeading, 360.0);
    if (vexHeading < 0)
    {
        vexHeading += 360.0;
    }
    return vexHeading;
}

// Convert from Euclidean/CCW heading to VEX CW heading
double convertEuclideanToVEX(double euclideanHeading)
{
    double vexHeading = fmod(360.0 - euclideanHeading, 360.0);
    if (vexHeading < 0)
    {
        vexHeading += 360.0;
    }
    return vexHeading;
}


void waitForButtonPress() {
    // Display a message on the Brain's screen
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Press R1 to continue");
    
    // Wait until the R1 button is not pressed (in case it's already pressed)
    while(Controller.ButtonR1.pressing()) {
      wait(20, msec);
    }
    
    // Wait until the R1 button is pressed
    while(!Controller.ButtonR1.pressing()) {
      wait(20, msec);
    }
    
    // Wait until the R1 button is released
    while(Controller.ButtonR1.pressing()) {
      wait(20, msec);
    }
    
    // Clear the message
    Brain.Screen.clearScreen();
  }

  int armResetTask(void *params)
{
    ArmResetTaskParams *p = static_cast<ArmResetTaskParams *>(params);
    
    bool wasBumperPressed = false;
    bool isMovingDown = false;
    
    // Start the arm moving down
    armMotor1.setBrake(brakeType::coast);
    armMotor2.setBrake(brakeType::coast);
    armMotor1.spin(reverse, 100, velocityUnits::pct);
    armMotor2.spin(reverse, 100, velocityUnits::pct);
    isMovingDown = true;
    p->isResetComplete = false;
    
    while (p->isRunning)
    {
        // Check if bumper is pressed and arm has stopped moving
        if (armBumper.value() == 1 && 
            fabs(armMotor1.velocity(velocityUnits::rpm)) < 5 && 
            fabs(armMotor2.velocity(velocityUnits::rpm)) < 5)
        {
            if (!wasBumperPressed && isMovingDown)
            {
                wasBumperPressed = true;
                armMotor1.stop(brakeType::coast);
                armMotor2.stop(brakeType::coast);
                armMotor1.resetPosition();
                armMotor2.resetPosition();
                armstat = ArmPosition::Starting;
                isMovingDown = false;
                p->isResetComplete = true;
                
            }
        }
        else
        {
            wasBumperPressed = false;
        }
        
        wait(20, msec); // Small delay to prevent CPU overload
    }
    return 0;
}

void runIntakeToStall() {
    // Start the intake
    intakeMotor.spin(reverse, 100, velocityUnits::pct);
    
    // Wait for intake to stall
    waitUntil(intakeMotor.velocity(percentUnits::pct) < 1.0 || 
              intakeMotor.current(currentUnits::amp) > 11.0);
    
    // Small delay to ensure it's actually stalled
    wait(50, msec);
    
    // Stop the intake and set to coast mode
    intakeMotor.stop(brakeType::coast);
    
    // Back up slightly to release pressure
  // Briefly reverse the intake to relieve pressure
  intakeMotor.spin(forward, 30, velocityUnits::pct);
  wait(150, msec);
  
  // Stop the intake and set to coast mode
  intakeMotor.stop(brakeType::coast);
}

// Add this near your other global variables at the top of the file
IntakeStallTaskParams intakeStallParams;

// Add this near your other task functions like colorDetectionTask
// Task function for monitoring intake stalls
int intakeStallTask(void *params) {
    IntakeStallTaskParams *p = static_cast<IntakeStallTaskParams *>(params);
    
    // Counter for consecutive stall detections
    int stallCounter = 0;
    const int REQUIRED_CONSECUTIVE_STALLS = 3;
    
    while (p->isRunning) {
        // Get current velocity
        double currentVelocity = fabs(intakeMotor.velocity(percentUnits::pct));
        
        // Check for stall condition
        if (currentVelocity < p->stallThreshold) {
            stallCounter++;
            
            // If we have enough consecutive stalls
            if (stallCounter >= REQUIRED_CONSECUTIVE_STALLS) {
                // Briefly reverse the intake by specified rotation
                intakeMotor.spinFor(forward, p->reverseRotation, rotationUnits::deg, 
                                    p->reverseSpeed, velocityUnits::pct, false);
                
                // Wait for reversal to complete
                waitUntil(!intakeMotor.isSpinning());
                
                // Stop the intake and set to coast mode
                intakeMotor.stop(brakeType::coast);
                
                // Task accomplished, exit
                p->isRunning = false;
                break;
            }
        } else {
            // Reset counter if not stalled
            stallCounter = 0;
        }
        
        // Check every 20ms as requested
        wait(20, msec);
    }
    
    return 0;
}

// Add this near your other utility functions
// Function to start monitoring intake for stalls
void startIntakeStallDetection() {
    // Set up parameters
    intakeStallParams.isRunning = true;
    intakeStallParams.stallThreshold = 5.0;     // 5% velocity threshold
    intakeStallParams.reverseRotation = 110;     // 90 degrees of reversal
    intakeStallParams.reverseSpeed = 30;        // 30% speed for reversal
    
    // Start the intake motor
    intakeMotor.spin(reverse, 100, velocityUnits::pct);
    
    // Start the stall detection task
    vex::task stall_task(intakeStallTask, &intakeStallParams);
}

// Add this to your utils.cpp file

void waitForButton() {
    // Display message on the brain screen
    Brain.Screen.setCursor(12, 1); // Position near bottom of screen
    Brain.Screen.setPenColor(vex::color::yellow);
    Brain.Screen.print("Press auton button to continue...");
    
    // Wait for button press
    while (!autonBumper.pressing()) {
        // Add a small delay to prevent CPU hogging
        wait(20, msec);
    }
    
    // Wait for button release to prevent multiple triggers
    while (autonBumper.pressing()) {
        wait(20, msec);
    }
    
    // Clear the message
    Brain.Screen.setCursor(12, 1);
    Brain.Screen.clearLine();
    
    // Add a small delay to debounce
    wait(300, msec);
}

SimpleArmTaskParams simpleArmParams;

/**
 * Task to move arm to a position after a delay
 * @param params Pointer to SimpleArmTaskParams
 * @return 0 when task completes
 */
int simpleArmTask(void *params) {
    SimpleArmTaskParams *p = static_cast<SimpleArmTaskParams *>(params);
    
    p->isComplete = false;
    
    // Wait for the specified delay
    if (p->delayMs > 0) {
        wait(p->delayMs, msec);
    }
    
    // Calculate position with adjustment
    double targetPosition = static_cast<double>(p->position) + p->adjustment;
    
    // Move arm to position
    armMotor1.spinToPosition(targetPosition, rotationUnits::deg, 100, velocityUnits::pct, false);
    armMotor2.spinToPosition(targetPosition, rotationUnits::deg, 100, velocityUnits::pct, false);
    
    p->isComplete = true;
    p->isRunning = false;
    
    return 0;
}

/**
 * Move arm to a position with optional adjustment and delay
 * @param position Position enum value
 * @param adjustment Adjustment to add to position value
 * @param delayMs Delay before moving in milliseconds
 */
void moveArm(ArmPosition position, int adjustment, int delayMs) {
    // Setup parameters
    simpleArmParams.isRunning = true;
    simpleArmParams.position = position;
    simpleArmParams.adjustment = adjustment;
    simpleArmParams.delayMs = delayMs;
    simpleArmParams.isComplete = false;
    
    // Start the task
    vex::task arm_task(simpleArmTask, &simpleArmParams);
}