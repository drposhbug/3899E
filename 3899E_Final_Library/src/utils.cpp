/**
 * UTILS.CPP - Utility Functions for VEX Robot Control
 * 
 * This file contains helper functions that support the robot's main operations:
 * - Motor speed detection & control algorithms (how fast wheels are spinning)
 * - Color detection for ring sorting (identifying red vs blue rings)
 * - Traction control & ABS systems (preventing wheel slip and lockup)
 * - Task management for autonomous operations (background processes)
 * - Mathematical utilities for navigation (calculations for movement)
 * 
 * Programming concepts used:
 * - Functions: Reusable blocks of code that perform specific tasks
 * - Classes: Blueprints for objects that contain data and functions
 * - Static variables: Variables that remember their value between function calls
 * - Ternary operator: Shorthand if/else statements (condition ? true_value : false_value)
 */

#include "vex.h"          // VEX robotics library - gives us motor, sensor commands
#include "robot_config.h" // Our robot's specific motor and sensor definitions
#include "utils.h"        // Header file with function declarations
#include <cmath>          // Math functions like fabs() for absolute value
#include <algorithm>      // Utility functions like std::min, std::max

using namespace vex;  // Lets us write "motor" instead of "vex::motor"

// ======================== CONSTANTS =========================================
// These are fixed values used throughout the program

const double DIV_BY_ZERO_THRESHOLD = 0.001;  // Very small number to prevent division by zero
                                              // Division by zero crashes programs, so we check if
                                              // numbers are smaller than this before dividing

// Color detection ranges - hue is measured in degrees (0-360°) like a color wheel
// Red is tricky because it wraps around the color wheel (near 0° and near 360°)
const double RED_HUE_MIN_1 = 335.0, RED_HUE_MAX_1 = 365.0;  // Red range 1 (wraps past 360°)
const double RED_HUE_MIN_2 = 0.0,   RED_HUE_MAX_2 = 15.0;   // Red range 2 (starts at 0°)
const double BLUE_HUE_MIN = 210.0,  BLUE_HUE_MAX = 230.0;   // Blue is simpler - one range
const double MIN_BRIGHTNESS = 15.0;  // Minimum light level to trust color readings

// ======================== MOTOR DIAGNOSTICS =================================
// These functions help us understand what's happening with the robot's wheels

/**
 * Detects if a wheel is slipping (spinning faster than the robot is actually moving)
 * 
 * Think of a car spinning its wheels in mud - the wheels spin fast but the car doesn't move.
 * This happens when there's not enough grip between wheel and floor.
 * 
 * @param motorSpeed How fast the motor thinks it's going (what we commanded)
 * @param encoderSpeed How fast we're actually moving (measured by sensors)
 * @return true if the wheel is slipping, false if it has good grip
 */
bool isSlipping(double motorSpeed, double encoderSpeed) {
    const double slipThreshold = 0.1;  // Allow 10% difference before calling it "slip"
    
    // If motor speed is more than 110% of actual speed, we're slipping
    // The > symbol means "greater than"
    return (motorSpeed > encoderSpeed * (1 + slipThreshold));
}

/**
 * Detects if a wheel is locking up (not spinning when it should be)
 * 
 * This is like when you slam on car brakes and wheels stop rotating but 
 * the car keeps sliding. The wheel isn't rolling anymore.
 * 
 * @param motorSpeed How fast the motor is supposed to go
 * @param encoderSpeed How fast the wheel is actually spinning
 * @return true if wheel is locked, false if it's rolling normally
 */
bool isLocking(double motorSpeed, double encoderSpeed) {
    const double lockThreshold = 0.85;  // If wheel is 85% slower than expected, it's locked
    
    // If motor speed is less than 85% of what's expected, wheel is probably locked
    return (motorSpeed < encoderSpeed * (1.0 - lockThreshold));
}

/**
 * Checks if we're trying to speed up (accelerate) the robot
 * 
 * Acceleration happens when:
 * 1. We want to go faster in the same direction
 * 2. We want to change direction (even if going slower, changing direction requires acceleration)
 * 
 * @param targetDriverSpeed Speed we want to achieve
 * @param currentSpeed Speed we're currently at
 * @return true if we're accelerating, false if we're maintaining speed or slowing down
 */
bool isAccelerating(double targetDriverSpeed, double currentSpeed) {
    // Check if both speeds have the same sign (both positive or both negative)
    // If they do, we're going in the same direction
    if ((targetDriverSpeed * currentSpeed) > 0) {
        // fabs() means "absolute value" - it removes the negative sign if there is one
        // So we're comparing the magnitudes (how big the numbers are)
        return fabs(targetDriverSpeed) > fabs(currentSpeed);
    }
    // If signs are different, we're changing direction - that's always acceleration
    else if ((targetDriverSpeed * currentSpeed) < 0) {
        return true;  // Direction change always counts as acceleration
    }
    // If we get here, both speeds are zero - no acceleration
    return false;
}

/**
 * Convert motor RPM (Rotations Per Minute) to linear speed in cm/s
 * 
 * Motors tell us how fast they're spinning, but we want to know how fast
 * the robot is moving across the ground.
 * 
 * @param motor The motor object we want to measure
 * @return Speed in centimeters per second
 */
double getMotorSpeed(vex::motor &motor) {
    // Step by step conversion:
    // 1. Get RPM from motor
    // 2. Divide by gear ratio (gears slow down the wheel but increase torque)
    // 3. Multiply by wheel circumference (how far we go per rotation)
    // 4. Divide by 60 to convert from "per minute" to "per second"
    return motor.velocity(vex::velocityUnits::rpm) / gearRatio * wheelCircumferenceCM / 60.0;
}

/**
 * Convert encoder RPM to linear speed in cm/s
 * 
 * Encoders are separate sensors that measure actual wheel rotation.
 * They're more accurate than motor readings because they measure what
 * actually happened, not what we tried to make happen.
 * 
 * @param encoder The encoder sensor we want to read
 * @return Actual speed in centimeters per second
 */
double getEncoderSpeed(vex::rotation &encoder) {
    // Simpler conversion because encoders directly measure wheel rotation
    return encoder.velocity(vex::velocityUnits::rpm) * encoderWheelCircumferenceCM / 60.0;
}

/**
 * Calculate slip ratio between wheel speed and actual robot speed
 * 
 * Slip ratio tells us how much the wheel is slipping:
 * - 0 = perfect grip, no slip
 * - 1 = complete slip (wheel spinning but robot not moving)
 * - Values between 0 and 1 = partial slip
 * 
 * @param wheelSpeed How fast the wheel is spinning
 * @param robotSpeed How fast the robot is actually moving
 * @return Slip ratio from 0.0 (no slip) to 1.0 (full slip)
 */
double calculateSlipRatio(double wheelSpeed, double robotSpeed) {
    // Special case: if robot isn't moving (robotSpeed near zero)
    if (std::fabs(robotSpeed) < DIV_BY_ZERO_THRESHOLD) {
        // This is a ternary operator: condition ? value_if_true : value_if_false
        // If wheel speed is also near zero, no slip (both stopped)
        // If wheel is spinning but robot isn't moving, full slip
        return (std::fabs(wheelSpeed) < DIV_BY_ZERO_THRESHOLD) ? 0.0 : 1.0;
    }
    
    // Normal case: calculate the difference as a percentage
    // std::fabs gives us the absolute value (always positive)
    return std::fabs((wheelSpeed - robotSpeed) / robotSpeed);
}

/**
 * Update a rolling average (smoothed value) over multiple samples
 * 
 * Rolling averages help reduce noise in sensor readings. Instead of using
 * just the latest reading, we blend it with previous readings for stability.
 * 
 * @param newValue The latest measurement
 * @param currentAverage The current average value
 * @param n Number of samples to average over (higher = smoother but slower to respond)
 * @return Updated rolling average
 */
float rollingAverage(float newValue, float currentAverage, int n) {
    // Mathematical formula for rolling average:
    // New average = (old average * (n-1) + new value) / n
    // This gives more weight to the old average, creating smooth transitions
    return currentAverage * (n - 1) / n + newValue / n;
}

/**
 * Cap motor voltages to prevent exceeding limits while preserving PID correction
 * 
 * Sometimes our PID controller wants to send more voltage to motors than they can handle.
 * This function limits the voltage but tries to keep the difference between left and
 * right motors (needed for steering corrections).
 * 
 * @param leftVoltage Reference to left motor voltage (modified by this function)
 * @param rightVoltage Reference to right motor voltage (modified by this function)  
 * @param absoluteMaxVoltage Maximum allowed voltage
 */
void PIDVoltageCapCorrection(double &leftVoltage, double &rightVoltage, double absoluteMaxVoltage) {
    // Calculate how much difference there is between left and right
    // This difference is what makes the robot steer
    double pidCorrectionDiff = fabs(leftVoltage - rightVoltage);

    // If left voltage is too high
    if (std::abs(leftVoltage) > absoluteMaxVoltage) {
        // Set left to maximum, but keep the same sign (+ or -)
        leftVoltage = std::copysign(absoluteMaxVoltage, leftVoltage);
        // Reduce right voltage to maintain the steering difference
        rightVoltage = std::copysign((absoluteMaxVoltage - pidCorrectionDiff), rightVoltage);
    }
    // If right voltage is too high
    else if (std::abs(rightVoltage) > absoluteMaxVoltage) {
        // Same logic but for right motor
        leftVoltage = std::copysign((absoluteMaxVoltage - pidCorrectionDiff), leftVoltage);
        rightVoltage = std::copysign(absoluteMaxVoltage, rightVoltage);
    }
}

// ======================== COLOR DETECTION ===================================
// Functions for identifying colored rings and controlling ring intake/ejection

/**
 * Set up the optical sensor for color detection
 * 
 * The optical sensor needs its LED light turned on to illuminate objects
 * so it can accurately read their colors.
 */
void initializeOpticalSensor() {
    opticalSensor.setLightPower(100, percent);  // Turn LED to full brightness
    opticalSensor.setLight(ledState::on);       // Make sure the light is on
}

// These variables remember their values between function calls (static variables)
// We need to track multiple readings to avoid false color detections
static int consecutiveDetections = 0;    // How many times in a row we've seen the same result
static bool lastDetectedColor = false;   // What we detected last time (true = color, false = no color)

/**
 * Check if we're detecting a valid colored ring
 * 
 * This function is careful about color detection because sensor readings can be noisy.
 * It requires seeing the same color for several readings in a row before deciding
 * that there's really a colored ring present.
 * 
 * @return true if we're confident there's a colored ring, false otherwise
 */
bool detectColor() {
    // Read current color and brightness from the sensor
    double hue = opticalSensor.hue();           // Color (0-360 degrees on color wheel)
    double brightness = opticalSensor.brightness(); // How bright the object is
    bool colorDetected = false;                 // Assume no color until we find one

    // First check: is it bright enough to trust the color reading?
    if (brightness < MIN_BRIGHTNESS) {
        // Too dark - can't trust the color reading
        consecutiveDetections = 0;      // Reset our counter
        lastDetectedColor = false;      // Remember we found nothing
        return false;                   // Report no color detected
    }

    // Second check: is the hue (color) in our target ranges?
    // Red is tricky because it wraps around the color wheel near 0/360 degrees
    if (((hue >= RED_HUE_MIN_1 && hue <= RED_HUE_MAX_1) ||  // Red range 1 (335-365°)
         (hue >= RED_HUE_MIN_2 && hue <= RED_HUE_MAX_2)) || // Red range 2 (0-15°)
        (hue >= BLUE_HUE_MIN && hue <= BLUE_HUE_MAX)) {     // Blue range (210-230°)
        colorDetected = true;  // Found a color we care about
    }

    // Update our consecutive detection counter
    // We only count it if we got the same result as last time AND we detected a color
    if (colorDetected == lastDetectedColor && colorDetected) {
        consecutiveDetections++;  // Add to our streak
    } else {
        consecutiveDetections = 1;  // Reset streak (this reading starts a new potential streak)
    }

    lastDetectedColor = colorDetected;  // Remember this result for next time

    // Only report success if we've seen the color consistently for 3 readings
    // This prevents false positives from sensor noise
    return (consecutiveDetections >= 3);
}

/**
 * Reset the color detection system
 * 
 * Call this when you want to start fresh with color detection,
 * clearing any previous readings from memory.
 */
void resetColorDetection() {
    consecutiveDetections = 0;     // Clear the streak counter
    lastDetectedColor = false;     // Clear the last reading
}

// ======================== MOTOR CONTROL TASKS ==============================
// These functions control motors with specific timing - useful for automated sequences

/**
 * CONTROL ANY MOTOR WITH TIMING
 * 
 * This function runs a motor for a specific amount of time after a delay.
 * Useful for things like "wait 2 seconds, then run intake for 5 seconds"
 * 
 * @param targetMotor Which motor to control (passed by reference with &)
 * @param DelayStart How long to wait before starting (milliseconds)
 * @param OnTime How long to run the motor (milliseconds)
 * @param dir Direction to spin (forward or reverse)
 */
void MotorControl(motor &targetMotor, int DelayStart, int OnTime, directionType dir) {
    task::sleep(DelayStart);                       // Wait for the delay period
    targetMotor.spin(dir, 12, voltageUnits::volt); // Start the motor at 12 volts
    task::sleep(OnTime);                           // Keep running for OnTime
    targetMotor.stop();                            // Stop the motor
}

/**
 * THREAD WRAPPER FOR MOTOR CONTROL
 * 
 * VEX tasks expect a specific function format. This wrapper converts our
 * MotorControl function into the format that tasks need.
 * 
 * @param params Pointer to MotorControlParams structure with motor info
 * @return 0 (required by VEX task system)
 */
int MotorControlThread(void *params) {
    // Cast the void pointer back to the correct type
    MotorControlParams *mcParams = static_cast<MotorControlParams *>(params);
    
    // Call our motor control function with the parameters
    MotorControl(*mcParams->targetMotor, mcParams->DelayStart, mcParams->OnTime, mcParams->dir);
    
    return 0;  // Tasks must return 0 to indicate successful completion
}

// ======================== COLOR SORTING TASK ================================
// Background task for automatically rejecting wrong-colored rings

/**
 * BACKGROUND COLOR DETECTION AND RING EJECTION
 * 
 * This task runs continuously, watching for colored rings and automatically
 * ejecting ones we don't want. It runs in parallel with the main program.
 * 
 * @param params Pointer to ColorTaskParams with settings
 * @return 0 when task ends
 */
int colorDetectionTask(void *params) {
    // Convert the generic pointer to our specific parameter type
    ColorTaskParams *p = static_cast<ColorTaskParams *>(params);

    // Keep running until told to stop
    while (p->isRunning) {
        double hue = opticalSensor.hue();  // Get current color reading
        
        // Update display (clear old text first)
        Brain.Screen.clearLine(1);
        Brain.Screen.setCursor(1, 1);

        // Check for red ring detection
        if (((hue >= RED_HUE_MIN_1 && hue <= RED_HUE_MAX_1) ||  // Red range 1
             (hue >= RED_HUE_MIN_2 && hue <= RED_HUE_MAX_2)) && // Red range 2
            p->targetColor == Color::RED) {                     // And we want red rings
            
            Brain.Screen.print("RED");           // Show what we found
            wait(p->delayMs, msec);             // Wait (gives time to see the ring)
            intakeMotor1.stop();                // Stop intake
            wait(50, msec);                     // Brief pause
            intakeMotor1.spin(reverse, 100, velocityUnits::pct); // Eject the ring
        }
        // Check for blue ring detection
        else if ((hue >= BLUE_HUE_MIN && hue <= BLUE_HUE_MAX) && // Blue range
                 p->targetColor == Color::BLUE) {                // And we want blue rings
            
            Brain.Screen.print("BLUE");          // Show what we found
            wait(p->delayMs, msec);             // Wait
            intakeMotor1.stop();                // Stop intake  
            wait(50, msec);                     // Brief pause
            intakeMotor1.spin(reverse, 100, velocityUnits::pct); // Eject the ring
        }

        wait(10, msec);  // Small delay before checking again (don't overload the processor)
    }
    return 0;
}

// ======================== INTAKE STALL DETECTION ========================
// Automatic jam detection and clearing for the intake system

// Global variable to control the stall detection task
IntakeStallTaskParams intakeStallParams;

/**
 * BACKGROUND INTAKE STALL MONITORING
 * 
 * This task watches the intake motor and automatically clears jams.
 * If the intake gets stuck (stalled) for too long, it reverses briefly
 * to clear whatever is jamming it.
 * 
 * @param params Pointer to IntakeStallTaskParams with settings
 * @return 0 when task ends  
 */
int intakeStallTask(void *params) {
    IntakeStallTaskParams *p = static_cast<IntakeStallTaskParams *>(params);
    
    int stallCounter = 0;                        // Count how long we've been stalled
    const int REQUIRED_CONSECUTIVE_STALLS = 10;  // Stall for 200ms (10 * 20ms) before acting
    
    while (p->isRunning) {
        // Check how fast the intake is currently moving
        double currentVelocity = fabs(intakeMotor1.velocity(percentUnits::pct));
        
        // Is the motor moving slower than our stall threshold?
        if (currentVelocity < p->stallThreshold) {
            stallCounter++;  // Add to our stall counter
            
            // Have we been stalled long enough to be sure it's jammed?
            if (stallCounter >= REQUIRED_CONSECUTIVE_STALLS) {
                // Yes - reverse the motor to clear the jam
                intakeMotor1.spinFor(forward, p->reverseRotation, rotationUnits::deg, 
                                    p->reverseSpeed, velocityUnits::pct, false);
                
                // Wait for the reversal to complete
                waitUntil(!intakeMotor1.isSpinning());
                
                // Stop the motor and let it coast
                intakeMotor1.stop(brakeType::coast);
                
                // Our job is done - stop this task
                p->isRunning = false;
                break;
            }
        } else {
            // Motor is moving - reset the stall counter
            stallCounter = 0;
        }
        
        // Check again in 20 milliseconds
        wait(20, msec);
    }
    
    return 0;
}

/**
 * START INTAKE WITH AUTOMATIC STALL DETECTION
 * 
 * This function starts the intake motor and launches a background task
 * to monitor for jams and automatically clear them.
 */
void startIntakeStallDetection() {
    // Configure the stall detection parameters
    intakeStallParams.isRunning = true;          // Enable the monitoring task
    intakeStallParams.stallThreshold = 1.0;      // 1% velocity = considered stalled
    intakeStallParams.reverseRotation = 210;     // Reverse 210 degrees to clear jam
    intakeStallParams.reverseSpeed = 60;         // At 60% speed
    
    // Start the intake motor
    intakeMotor1.spin(reverse, 100, velocityUnits::pct);
    
    // Launch the background monitoring task
    vex::task stall_task(intakeStallTask, &intakeStallParams);
}

// ======================== USER INPUT UTILITIES ===========================
// Functions for waiting for button presses from user or autonomous selector

/**
 * WAIT FOR CONTROLLER BUTTON PRESS
 * 
 * Pauses the program until the user presses R1 on the controller.
 * Handles button debouncing to prevent accidental double-presses.
 */
void waitForButtonPress() {
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Press R1 to continue");
    
    // Wait for button to be released (in case it's already pressed)
    while(Controller.ButtonR1.pressing()) {
        wait(20, msec);
    }
    
    // Wait for the button to be pressed
    while(!Controller.ButtonR1.pressing()) {
        wait(20, msec);
    }
    
    // Wait for the button to be released (completes the press)
    while(Controller.ButtonR1.pressing()) {
        wait(20, msec);
    }
    
    Brain.Screen.clearScreen();  // Clean up the display
}

/**
 * WAIT FOR AUTONOMOUS SELECTION BUTTON
 * 
 * Pauses the program until the autonomous selector button is pressed.
 * Used in autonomous routines to let the user choose when to start.
 */
void waitForButton() {
    // Display message on brain screen
    Brain.Screen.setCursor(12, 1);                          // Bottom of screen
    Brain.Screen.setPenColor(vex::color::yellow);           // Yellow text
    Brain.Screen.print("Press auton button to continue...");
    
    // Wait for button press
    while (!autonBumper.pressing()) {
        wait(20, msec);
    }
    
    // Wait for button release (debouncing)
    while (autonBumper.pressing()) {
        wait(20, msec);
    }
    
    // Clear the message
    Brain.Screen.setCursor(12, 1);
    Brain.Screen.clearLine();
    
    wait(300, msec);  // Extra delay to prevent accidental double-presses
}

// ======================== ARM CONTROL (LEGACY) ==============================
// These functions control the robot's arm mechanism (if present)

SimpleArmTaskParams simpleArmParams;  // Global parameters for arm control task

/**
 * BACKGROUND ARM POSITIONING TASK
 * 
 * Moves the arm to a specific position after an optional delay.
 * Runs in background so main program can continue.
 * 
 * @param params Pointer to SimpleArmTaskParams
 * @return 0 when movement is complete
 */
int simpleArmTask(void *params) {
    SimpleArmTaskParams *p = static_cast<SimpleArmTaskParams *>(params);
    
    p->isComplete = false;  // Mark as not finished yet
    
    // Wait for specified delay if any
    if (p->delayMs > 0) {
        wait(p->delayMs, msec);
    }
    
    // Calculate final position (base position + any adjustment)
    double targetPosition = static_cast<double>(p->position) + p->adjustment;
    
    // Note: Actual arm movement code would go here
    // It's commented out because this robot may not have an arm
    
    p->isComplete = true;   // Mark as finished
    p->isRunning = false;   // Stop the task
    return 0;
}

/**
 * MOVE ARM TO POSITION
 * 
 * Public function to start arm movement. Launches background task
 * so the main program doesn't have to wait.
 * 
 * @param position Predefined arm position (from ArmPosition enum)
 * @param adjustment Fine adjustment in degrees (+ or -)
 * @param delayMs Delay before starting movement
 */
void moveArm(ArmPosition position, int adjustment, int delayMs) {
    // Set up the parameters
    simpleArmParams.isRunning = true;
    simpleArmParams.position = position;
    simpleArmParams.adjustment = adjustment;
    simpleArmParams.delayMs = delayMs;
    simpleArmParams.isComplete = false;
    
    // Start the background task
    vex::task arm_task(simpleArmTask, &simpleArmParams);
}

// ======================== TRACTION CONTROL ==================================
// Advanced wheel slip management system - prevents wheels from spinning uselessly

/**
 * TRACTION CONTROL CLASS CONSTRUCTOR
 * 
 * Sets up a traction control system for one wheel. Think of this like the
 * traction control in a car - it prevents wheels from spinning when there's
 * not enough grip.
 * 
 * @param minSpeedVoltage Minimum power we'll ever send to the motor
 * @param maxSpeedVoltage Maximum power we'll ever send to the motor  
 * @param slipThreshold How much slip to allow (0.0 = no slip, 1.0 = full slip allowed)
 */
tractionControl::tractionControl(double minSpeedVoltage, double maxSpeedVoltage, double slipThreshold)
    : minSpeedVoltage(minSpeedVoltage), maxSpeedVoltage(maxSpeedVoltage), slipThreshold(slipThreshold) {
    // The : syntax is called an "initializer list" - it sets the class member variables
    // This is more efficient than setting them inside the function body
}

/**
 * ADJUST MOTOR POWER BASED ON WHEEL SLIP
 * 
 * This is the main traction control function. It monitors how much the wheel
 * is slipping and adjusts power accordingly:
 * - If slipping too much: reduce power to regain grip
 * - If gripping well: increase power to go faster
 * 
 * @param motorVoltage Current power being sent to the motor
 * @param wheelSpeed How fast this individual wheel is spinning  
 * @param robotSpeed How fast the robot is actually moving
 * @param accelFactor How aggressively to adjust power (higher = more aggressive)
 * @return New voltage to send to the motor
 */
double tractionControl::tractionControlSpeed(double motorVoltage, double wheelSpeed, 
                                            double robotSpeed, double accelFactor) {
    
    // Step 1: Calculate how much this wheel is slipping
    double slipRatio = calculateSlipRatio(wheelSpeed, robotSpeed);

    // Step 2: Decide whether to increase or decrease power
    if (slipRatio > slipThreshold) {
        // Wheel is slipping too much - reduce power to regain grip
        // Dividing by accelFactor makes the voltage smaller
        motorVoltage = motorVoltage / accelFactor;
    } else {
        // Wheel has good grip - increase power to go faster
        // Multiplying by accelFactor makes the voltage bigger
        motorVoltage = motorVoltage * accelFactor;
    }

    // Step 3: Make sure we don't exceed safe voltage limits
    // std::copysign() keeps the direction (+ or -) while changing the magnitude
    // std::max() picks the larger of two values
    // std::min() picks the smaller of two values
    // std::fabs() gets the absolute value (removes + or - sign)
    
    // This complex line does several things:
    // 1. Make sure voltage is at least minSpeedVoltage (std::max with minSpeedVoltage)
    // 2. Make sure voltage is at most maxSpeedVoltage (std::min with maxSpeedVoltage) 
    // 3. Keep the original direction/sign (std::copysign)
    motorVoltage = std::copysign(
        std::max(std::fabs(minSpeedVoltage), 
                std::min(std::fabs(motorVoltage), std::fabs(maxSpeedVoltage))),
        motorVoltage);

    return motorVoltage;  // Send back the adjusted voltage
}

// ======================== ABS CONTROLLER ====================================
// Anti-lock Braking System - prevents wheels from locking up during braking

/**
 * ABS CONTROLLER CLASS CONSTRUCTOR
 * 
 * Sets up an anti-lock braking system for one wheel. Like ABS in cars,
 * this prevents wheels from locking up (stopping rotation) during hard braking.
 * When wheels lock, you lose steering control and slide farther.
 * 
 * @param lockThreshold How much slip indicates a locked wheel (0.0 = any slip, 1.0 = only full lock)
 */
ABSController::ABSController(double lockThreshold) : ABSLockThreshold(lockThreshold) {
    // Store the threshold value in the class member variable
    // The : syntax is an initializer list - it sets member variables during construction
}

/**
 * DETERMINE BRAKING MODE BASED ON WHEEL LOCK STATUS
 * 
 * This function decides how to brake each wheel:
 * - If wheel is locking: let it coast (roll freely) to unlock
 * - If wheel is rolling normally: apply normal brakes
 * 
 * @param wheelSpeed How fast this individual wheel is rotating
 * @param robotSpeed How fast the robot is actually moving
 * @return vex::coast (let wheel roll) or vex::brake (apply brakes)
 */
vex::brakeType ABSController::ABSSpeedReduction(double wheelSpeed, double robotSpeed) {
    // Calculate how much this wheel is slipping/locking
    double slipRatio = calculateSlipRatio(wheelSpeed, robotSpeed);

    // Decision logic:
    if (slipRatio > ABSLockThreshold) {
        // Wheel is locking up - let it roll freely to regain rotation
        return vex::coast;  // Coast mode = motor doesn't resist, wheel can spin freely
    } else {
        // Wheel is rolling normally - safe to apply brakes
        return vex::brake;  // Brake mode = motor actively resists rotation
    }
}