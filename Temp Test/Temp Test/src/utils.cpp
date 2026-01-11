#include "robot_config.hpp"
#include "utils.hpp"
#include <cmath>
#include <algorithm>

// Minimum threshold for division operations to prevent divide by zero errors
const double DIV_BY_ZERO_THRESHOLD = 0.001;

/**
 * Calculate the shortest path error between target and current heading.
 * Ensures the error is always in the range of -180 to +180 degrees for smooth PID control.
 */

// Implement slip detection logic
bool isSlipping(double motorSpeed, double encoderSpeed)
{
    const double slipThreshold = 0.1;
    return (motorSpeed > encoderSpeed * (1 + slipThreshold));
}

// Implement lock-up detection logic
bool isLocking(double motorSpeed, double encoderSpeed)
{
    const double lockThreshold = .85;
    return (motorSpeed < encoderSpeed * (1.0 - lockThreshold));
}

// Color Detection Constants for utils.cpp
const double RED_HUE_MIN_1 = 335.0;
const double RED_HUE_MAX_1 = 365.0;
const double RED_HUE_MIN_2 = 0.0;
const double RED_HUE_MAX_2 = 15.0;
const double BLUE_HUE_MIN = 210.0;
const double BLUE_HUE_MAX = 230.0;
const double MIN_BRIGHTNESS = 15.0;

// Function to initialize the Optical Sensor
void initializeOpticalSensor()
{
    opticalSensor.set_led_pwm(100); // Turn on the sensor light at 100% power
}

// Track consecutive detections to prevent false positives
static int consecutiveDetections = 0;
static bool lastDetectedColor = false;

/**
 * Detect if a specific color is present
 * @param targetColor The color to detect (Color::RED or Color::BLUE)
 * @return true if the target color is detected, false otherwise
 */
bool detectColor(Color targetColor)
{
    double hue = opticalSensor.get_hue();
    double brightness = opticalSensor.get_brightness();
    bool colorDetected = false;

    // Check brightness threshold
    if (brightness < MIN_BRIGHTNESS)
    {
        consecutiveDetections = 0;
        lastDetectedColor = false;
        return false;
    }

    // Check for the target color only
    if (targetColor == Color::RED)
    {
        // Check for red hue ranges
        if ((hue >= RED_HUE_MIN_1 && hue <= RED_HUE_MAX_1) ||
            (hue >= RED_HUE_MIN_2 && hue <= RED_HUE_MAX_2))
        {
            colorDetected = true;
        }
    }
    else if (targetColor == Color::BLUE)
    {
        // Check for blue hue range
        if (hue >= BLUE_HUE_MIN && hue <= BLUE_HUE_MAX)
        {
            colorDetected = true;
        }
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
    return (consecutiveDetections >= 3);
}

/**
 * Detect if red color is present
 * @return true if red is detected, false otherwise
 */
bool detectRed()
{
    return detectColor(Color::RED);
}

/**
 * Detect if blue color is present
 * @return true if blue is detected, false otherwise
 */
bool detectBlue()
{
    return detectColor(Color::BLUE);
}

// Reset detection state if needed
void resetColorDetection()
{
    consecutiveDetections = 0;
    lastDetectedColor = false;
}

// Generic function to control any motor
void MotorControl(pros::Motor& targetMotor, int DelayStart, int OnTime, int dir)
{
    pros::delay(DelayStart);
    targetMotor.move_voltage(dir * 12000); // 12V = 12000 millivolts
    pros::delay(OnTime);
    targetMotor.move_voltage(0);
}

// Wrapper function matching the expected thread signature
void MotorControlThread(void* params)
{
    MotorControlParams* mcParams = static_cast<MotorControlParams*>(params);
    MotorControl(*mcParams->targetMotor, mcParams->DelayStart, mcParams->OnTime, mcParams->dir);
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
double getMotorSpeed(pros::Motor& motor)
{
    // Get motor velocity in RPM and convert to cm/s using the constant circumference
    return motor.get_actual_velocity() / gearRatio * wheelCircumferenceCM / 60.0;
}

// Function to calculate motor encoder speed in cm per second
double getEncoderSpeed(pros::Rotation& encoder)
{
    return encoder.get_velocity() * encoderWheelCircumferenceCM / 360.0; // PROS returns degrees/sec
}

double getRotation() {
    return inertialSensor.get_rotation() + headingOffset;
}

void PIDVoltageCapCorrection(double& leftVoltage, double& rightVoltage, double absoluteMaxVoltage)
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
    // If robot isn't moving
    if (std::fabs(robotSpeed) < DIV_BY_ZERO_THRESHOLD)
    {
        // Both stopped = no slip, wheels spinning = full slip
        return (std::fabs(wheelSpeed) < DIV_BY_ZERO_THRESHOLD) ? 0.0 : 1.0;
    }

    // Unified formula with absolute value
    return std::fabs((wheelSpeed - robotSpeed) / robotSpeed);
}

// Calculate wheel lockup ratio for ABS braking
double calculateLockupRatio(double wheelSpeed, double robotSpeed)
{
    if (std::fabs(robotSpeed) < DIV_BY_ZERO_THRESHOLD)
    {
        return (std::fabs(wheelSpeed) < DIV_BY_ZERO_THRESHOLD) ? 0.0 : 1.0;
    }

    return std::fabs((robotSpeed - wheelSpeed) / robotSpeed);
}

/**
 * Calculates rolling average of a value over N samples
 */
float rollingAverage(float newValue, float currentAverage, int n)
{
    return currentAverage * (n - 1) / n + newValue / n;
}

void colorDetectionTask(void* params)
{
    ColorTaskParams* p = static_cast<ColorTaskParams*>(params);

    while (p->isRunning)
    {
        double hue = opticalSensor.get_hue();
        pros::lcd::clear_line(0);
        pros::lcd::print(0, "Hue: %.2f", hue);

        if (((hue >= RED_HUE_MIN_1 && hue <= RED_HUE_MAX_1) ||
             (hue >= RED_HUE_MIN_2 && hue <= RED_HUE_MAX_2)) &&
            p->targetColor == Color::RED)
        {
            pros::lcd::print(0, "RED");
            pros::delay(p->delayMs);
            intakeMotor1.move_voltage(0);
            pros::delay(50);
            intakeMotor1.move_voltage(-12000); // Reverse
        }
        else if ((hue >= BLUE_HUE_MIN && hue <= BLUE_HUE_MAX) && p->targetColor == Color::BLUE)
        {
            pros::lcd::print(0, "BLUE");
            pros::delay(p->delayMs);
            intakeMotor1.move_voltage(0);
            pros::delay(50);
            intakeMotor1.move_voltage(-12000); // Reverse
        }

        pros::delay(10);
    }
}

double getAdjustedRotation() {
    return inertialSensor.get_rotation() + headingOffset;
}

void waitForButtonPress() {
    // Display a message on the Brain's screen
    pros::lcd::print(0, "Press R1 to continue");

    // Wait until the R1 button is not pressed (in case it's already pressed)
    while(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        pros::delay(20);
    }

    // Wait until the R1 button is pressed
    while(!controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        pros::delay(20);
    }

    // Wait until the R1 button is released
    while(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        pros::delay(20);
    }

    // Clear the message
    pros::lcd::clear();
}

void runIntakeToStall() {
    // Start the intake
    intakeMotor1.move_voltage(-12000); // 100% reverse

    // Wait for intake to stall
    while(!(fabs(intakeMotor1.get_actual_velocity()) < 1.0 &&
            intakeMotor1.get_current_draw() > 7000)) { // 7000 mA = 7 A
        pros::delay(10);
    }

    // Stop the intake and set to coast mode
    intakeMotor1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    intakeMotor1.move_voltage(0);

    // Briefly reverse the intake to relieve pressure
    intakeMotor1.move_voltage(3600); // 30% forward = 3.6V
    pros::delay(150);

    // Stop the intake and set to coast mode
    intakeMotor1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    intakeMotor1.move_voltage(0);
}

// Global task parameters
IntakeStallTaskParams intakeStallParams;

// Task function for monitoring intake stalls
void intakeStallTask(void* params) {
    IntakeStallTaskParams* p = static_cast<IntakeStallTaskParams*>(params);

    // Counter for consecutive stall detections
    int stallCounter = 0;
    const int REQUIRED_CONSECUTIVE_STALLS = 10;

    while (p->isRunning) {
        // Get current velocity
        double currentVelocity = fabs(intakeMotor1.get_actual_velocity() / 6.0); // Convert to percentage

        // Check for stall condition
        if (currentVelocity < p->stallThreshold) {
            stallCounter++;

            // If we have enough consecutive stalls
            if (stallCounter >= REQUIRED_CONSECUTIVE_STALLS) {
                // Briefly reverse the intake by specified rotation
                intakeMotor1.move_relative(p->reverseRotation, p->reverseSpeed * 6); // RPM = pct * 6 for 600 RPM motor

                // Wait for reversal to complete
                while(fabs(intakeMotor1.get_actual_velocity()) > 5) {
                    pros::delay(10);
                }

                // Stop the intake and set to coast mode
                intakeMotor1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
                intakeMotor1.move_voltage(0);

                // Task accomplished, exit
                p->isRunning = false;
                break;
            }
        } else {
            // Reset counter if not stalled
            stallCounter = 0;
        }

        // Check every 20ms as requested
        pros::delay(20);
    }
}

// Function to start monitoring intake for stalls
void startIntakeStallDetection() {
    // Set up parameters
    intakeStallParams.isRunning = true;
    intakeStallParams.stallThreshold = 1.0;
    intakeStallParams.reverseRotation = 210;
    intakeStallParams.reverseSpeed = 60;

    // Start the intake motor
    intakeMotor1.move_voltage(-12000); // 100% reverse

    // Start the stall detection task
    pros::Task stall_task(intakeStallTask, &intakeStallParams);
}

void waitForButton() {
    // Display message on the brain screen
    pros::lcd::print(7, "Press auton button to continue...");

    // Wait for button press
    while (!autonBumper.get_value()) {
        pros::delay(20);
    }

    // Wait for button release to prevent multiple triggers
    while (autonBumper.get_value()) {
        pros::delay(20);
    }

    // Clear the message
    pros::lcd::clear_line(7);

    // Add a small delay to debounce
    pros::delay(300);
}

SimpleArmTaskParams simpleArmParams;

/**
 * Task to move arm to a position after a delay
 */
void simpleArmTask(void* params) {
    SimpleArmTaskParams* p = static_cast<SimpleArmTaskParams*>(params);

    p->isComplete = false;

    // Wait for the specified delay
    if (p->delayMs > 0) {
        pros::delay(p->delayMs);
    }

    // Calculate position with adjustment
    double targetPosition = static_cast<double>(p->position) + p->adjustment;

    // Move arm to position (commented out as arm motors not defined in this conversion)
    // armMotor1.move_absolute(targetPosition, 100);
    // armMotor2.move_absolute(targetPosition, 100);

    p->isComplete = true;
    p->isRunning = false;
}

/**
 * Move arm to a position with optional adjustment and delay
 */
void moveArm(ArmPosition position, int adjustment, int delayMs) {
    // Setup parameters
    simpleArmParams.isRunning = true;
    simpleArmParams.position = position;
    simpleArmParams.adjustment = adjustment;
    simpleArmParams.delayMs = delayMs;
    simpleArmParams.isComplete = false;

    // Start the task
    pros::Task arm_task(simpleArmTask, &simpleArmParams);
}
