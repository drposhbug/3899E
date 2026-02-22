#include "robot_config.h"
#include "vex.h"
#include "utils.h"
#include "pid.h" 
#include <cmath>
#include <atomic>
#include "odometry.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace vex;

const double VISION_CENTER_X = 160;  
  
enum MotionPhase { READY, LAUNCH, CRUISE, DECELERATE, APPROACH, STOP };

double getCurrentEncoderDistanceCM() {
    double avgDeg = (passiveEncoderLeft.position(degrees) + passiveEncoderRight.position(degrees)) / 2.0;
    return (avgDeg / 360.0) * encoderWheelCircumferenceCM;
}


// ======================================================================
// Basic move for fixed distance
// Used for simple forward/backward commands
// ======================================================================
void move(double distanceCM, double maxSpeed, vex::directionType dir) {
    double targetRotations = distanceCM / wheelCircumferenceCM;

    for (int i = 0; i < 3; i++) {
        leftMotor[i].setBrake(brakeType::coast);
        rightMotor[i].setBrake(brakeType::coast);
    }

    for (int i = 0; i < 3; i++) {
        leftMotor[i].spinFor(dir, targetRotations, rotationUnits::rev, maxSpeed, velocityUnits::pct, false);
        rightMotor[i].spinFor(dir, targetRotations, rotationUnits::rev, maxSpeed, velocityUnits::pct, false);
    }
}

void smartMove(double distanceCM, double maxSpeed, vex::directionType dir, double wallStalledTimeMs) {
    const double WALL_STOP_THRESHOLD_RPM = 5.0;
    vex::timer wallStallTimer;
    bool wallDetected = false, wallDetectEnabled = (wallStalledTimeMs > 0), isCurrentlyStalled = false;

    for (int i = 0; i < 3; i++) {
        leftMotor[i].setBrake(brakeType::coast);
        rightMotor[i].setBrake(brakeType::coast);
    }

    double voltage = maxSpeed * 0.01 * absoluteMaxVoltage;
    if (dir == vex::reverse) voltage = -voltage;

    double startDist = getCurrentEncoderDistanceCM();
    double distanceTravelled = 0;

    while (fabs(distanceTravelled) < fabs(distanceCM) && !wallDetected) {
        distanceTravelled = getCurrentEncoderDistanceCM() - startDist;
        double avgEncoderSpeed = (fabs(passiveEncoderLeft.velocity(velocityUnits::rpm)) + fabs(passiveEncoderRight.velocity(velocityUnits::rpm))) / 2.0;

        if (wallDetectEnabled) {
            if (avgEncoderSpeed < WALL_STOP_THRESHOLD_RPM) {
                if (!isCurrentlyStalled) { wallStallTimer.reset(); isCurrentlyStalled = true; }
                else if (wallStallTimer.time(msec) >= wallStalledTimeMs) wallDetected = true;
            } else isCurrentlyStalled = false;
        }

        for (int i = 0; i < 3; i++) {
            leftMotor[i].spin(forward, voltage, voltageUnits::volt);
            rightMotor[i].spin(forward, voltage, voltageUnits::volt);
        }
        vex::task::sleep(10);
    }
    for (int i = 0; i < 3; i++) {
        leftMotor[i].stop(brake);
        rightMotor[i].stop(brake);
    }
}

void turnOdometry(double targetHeading, double breakDistanceInDegrees, double minSpeed, double maxSpeed, double exitTolerance) {
    bool decelCompleted = false;
    bool accelCompleted = false;
    bool decel = false;

    // 1. Just get the current heading
    double currentHeading = getContinuousStandardHeading();
    
    // 2. THE FIX: REMOVE THE "SMART" ROTATION MATH
    // We assume the caller (turnRight/turnLeft) has already handled the 360 logic.
    // BEFORE: 
    // int completeRotations = (int)(currentHeading / 360.0);
    // double targetRotationHeading = targetHeading + (completeRotations * 360.0);
    
    // AFTER: dumb and obedient
    double targetRotationHeading = targetHeading; 

    // Calculate error based on the exact numbers provided
    double headingError = targetRotationHeading - currentHeading;

    // ... (The rest of the function stays exactly the same) ... 
    
    Brain.Screen.printAt(10, 40, "Target Head: %.2f", targetHeading);
    Brain.Screen.printAt(10, 100, "Curr Rotation: %.2f", currentHeading);

   // CORRECTED - Uses positive headingError so the loop logic works
    double maxSpeedVoltage       = std::copysign(maxSpeed * 0.01 * absoluteMaxVoltage, headingError);
    double minSpeedVoltage       = std::copysign(minSpeed * 0.01 * absoluteMaxVoltage, headingError);
    double launchVoltage         = std::copysign(4, headingError);
    double minLaunchSpeedVoltage = std::copysign(std::min(fabs(maxSpeedVoltage), fabs(launchVoltage)), headingError);
    double minDriveMotorRPM      = (minSpeed * .01) * absoluteMaxRPM;

    const double TURN_ACCEL_FACTOR_LAUNCH = 1.5;
    const double SLIP_THRESHOLD_TRACTION = 10; 
    const double DECEL_STEP_PERCENT = 20;     
    const double LOCK_THRESHOLD_DECEL = 10;

    double averageMotorVoltage = 0;
    double motorVoltageLeft[3] = {minLaunchSpeedVoltage, minLaunchSpeedVoltage, minLaunchSpeedVoltage};  
    double motorVoltageRight[3] = {minLaunchSpeedVoltage, minLaunchSpeedVoltage, minLaunchSpeedVoltage}; 

    double leftEncoderRollingAverage = 0;
    double rightEncoderRollingAverage = 0;
    double voltageRollingAverage = 0;

    adaptiveABS adaptiveABSLeft(DECEL_STEP_PERCENT, LOCK_THRESHOLD_DECEL);
    adaptiveABS adaptiveABSRight(DECEL_STEP_PERCENT, LOCK_THRESHOLD_DECEL);

    tractionControl tractionControlLeft(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION);
    tractionControl tractionControlRight(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION);

    // Loop to continuously adjust motor power
    while ((maxSpeedVoltage > 0 && currentHeading <= targetRotationHeading - exitTolerance) ||
           (maxSpeedVoltage < 0 && currentHeading >= targetRotationHeading + exitTolerance))
    {
        currentHeading = getContinuousStandardHeading();
        headingError = targetRotationHeading - currentHeading;

        Brain.Screen.printAt(10, 100, "Curr Rotation: %.2f", currentHeading);
        Brain.Screen.printAt(10, 140, "Target: %.2f", targetHeading);

        double leftMotorRPM = fabs(leftMotor[1].velocity(vex::velocityUnits::rpm)) * DRIVE_MOTOR_RPM_ADJ;
        double rightMotorRPM = fabs(rightMotor[1].velocity(vex::velocityUnits::rpm)) * DRIVE_MOTOR_RPM_ADJ;

        double leftEncoderRPM = fabs(passiveEncoderLeft.velocity(velocityUnits::rpm)) *
                                (encoderWheelCircumferenceCM / wheelCircumferenceCM);
        double rightEncoderRPM = fabs(passiveEncoderRight.velocity(velocityUnits::rpm)) *
                                 (encoderWheelCircumferenceCM / wheelCircumferenceCM);

        double averageEncoderRPM = (leftEncoderRPM + rightEncoderRPM) / 2.0;

        // Launch Phase
        if ((std::fabs(headingError) > fabs(breakDistanceInDegrees)) && !accelCompleted && !decel)
        {
            double leftTractionVoltage = tractionControlLeft.tractionControlSpeed(motorVoltageLeft[1], leftMotorRPM, averageEncoderRPM, TURN_ACCEL_FACTOR_LAUNCH);
            double rightTractionVoltage = tractionControlRight.tractionControlSpeed(motorVoltageRight[1], rightMotorRPM, averageEncoderRPM, TURN_ACCEL_FACTOR_LAUNCH);

            double syncedMotorVoltage = std::min(fabs(leftTractionVoltage), fabs(rightTractionVoltage));

            for (int i = 0; i < 3; i++)
            {
                motorVoltageLeft[i] = std::copysign(syncedMotorVoltage, motorVoltageLeft[i]);
                motorVoltageRight[i] = std::copysign(syncedMotorVoltage, motorVoltageRight[i]);
            }

            averageMotorVoltage = (fabs(motorVoltageLeft[0]) + fabs(motorVoltageRight[0]) + fabs(motorVoltageLeft[1]) + fabs(motorVoltageRight[1]) + fabs(motorVoltageLeft[2]) + fabs(motorVoltageRight[2])) / numberDriveMotor;
            voltageRollingAverage = rollingAverage(averageMotorVoltage, voltageRollingAverage, 5);

            if (fabs(voltageRollingAverage) >= (fabs(maxSpeedVoltage) - VOLTAGE_TOLERANCE))
            {
                accelCompleted = true;
            }
        }
        // Cruise Phase
        else if ((std::abs(headingError) > fabs(breakDistanceInDegrees)) && accelCompleted)
        {
            for (int i = 0; i < 3; i++)
            {
                motorVoltageLeft[i] = maxSpeedVoltage;
                motorVoltageRight[i] = maxSpeedVoltage;
            }
        }
        // Decel Phase
        else if ((std::abs(headingError) <= fabs(breakDistanceInDegrees)) && decelCompleted == false)
        {
            if (!decel)
            {
                adaptiveABSLeft.initialize(motorVoltageLeft[1]);
                adaptiveABSRight.initialize(motorVoltageRight[1]);
            }
            decel = true;

            double leftMotorRPMDecel = fabs(leftMotor[1].velocity(vex::velocityUnits::rpm)) * DRIVE_MOTOR_RPM_ADJ;
            double rightMotorRPMDecel = fabs(rightMotor[1].velocity(vex::velocityUnits::rpm)) * DRIVE_MOTOR_RPM_ADJ;
            double leftEncoderRPMDecel = fabs(passiveEncoderLeft.velocity(velocityUnits::rpm)) * (encoderWheelCircumferenceCM / wheelCircumferenceCM);
            double rightEncoderRPMDecel = fabs(passiveEncoderRight.velocity(velocityUnits::rpm)) * (encoderWheelCircumferenceCM / wheelCircumferenceCM);

            double leftDecelVoltage = adaptiveABSLeft.decelControlSpeed(leftMotorRPMDecel, leftEncoderRPMDecel);
            double rightDecelVoltage = adaptiveABSRight.decelControlSpeed(rightMotorRPMDecel, rightEncoderRPMDecel);

            vex::brakeType leftBrakeMode = adaptiveABSLeft.getBrakeMode();
            vex::brakeType rightBrakeMode = adaptiveABSRight.getBrakeMode();

            vex::brakeType syncedBrakeMode;
            if (leftBrakeMode == vex::coast || rightBrakeMode == vex::coast) {
                syncedBrakeMode = vex::coast;
            } else {
                syncedBrakeMode = vex::brake;
            }

            double syncedDecelVoltage = std::max(fabs(leftDecelVoltage), fabs(rightDecelVoltage));

            for (int i = 0; i < 3; i++)
            {
                leftMotor[i].setBrake(syncedBrakeMode);
                rightMotor[i].setBrake(syncedBrakeMode);

                if (syncedBrakeMode == vex::brake && syncedDecelVoltage > 0.0)
                {
                    motorVoltageLeft[i] = std::copysign(syncedDecelVoltage, motorVoltageLeft[i]);
                    motorVoltageRight[i] = std::copysign(syncedDecelVoltage, motorVoltageRight[i]);
                }
                else
                {
                    motorVoltageLeft[i] = 0.0;
                    motorVoltageRight[i] = 0.0;
                }
            }

            leftEncoderRollingAverage = rollingAverage(leftEncoderRPMDecel, leftEncoderRollingAverage, 3);
            rightEncoderRollingAverage = rollingAverage(rightEncoderRPMDecel, rightEncoderRollingAverage, 3);

            if (fabs(leftEncoderRollingAverage) <= fabs(minDriveMotorRPM) &&
                fabs(rightEncoderRollingAverage) <= fabs(minDriveMotorRPM))
            {
                decelCompleted = true;
            }
        }
        // Final Approach
        else if (decelCompleted == true)
        {
            for (int i = 0; i < 3; i++)
            {
                motorVoltageLeft[i] = minSpeedVoltage;
                motorVoltageRight[i] = minSpeedVoltage;
            }
        }

        // Power Drive Motors
        for (int i = 0; i < 3; i++)
        {
            leftMotor[i].spin(forward, -motorVoltageLeft[i], voltageUnits::volt);
            rightMotor[i].spin(forward, motorVoltageRight[i], voltageUnits::volt);
        }

        vex::task::sleep(10);
    }

    for (int i = 0; i < 3; i++)
    {
        leftMotor[i].setBrake(brake);
        rightMotor[i].setBrake(brake);
        leftMotor[i].stop();
        rightMotor[i].stop();
    }
}

tractionControl::tractionControl(double minSpeedVoltage, double maxSpeedVoltage, double slipThreshold) : minSpeedVoltage(minSpeedVoltage), maxSpeedVoltage(maxSpeedVoltage), slipThreshold(slipThreshold) {}
double tractionControl::tractionControlSpeed(double motorVoltage, double wheelSpeed, double robotSpeed, double accelFactor) {
    double slipRatio = calculateSlipRatio(wheelSpeed, robotSpeed); 
    motorVoltage = (slipRatio > slipThreshold) ? motorVoltage / accelFactor : motorVoltage * accelFactor;
    return std::copysign(std::max(std::fabs(minSpeedVoltage), std::min(std::fabs(motorVoltage), std::fabs(maxSpeedVoltage))), motorVoltage);
}

adaptiveABS::adaptiveABS(double decelStepPercent, double lockThreshold) : lockThreshold(lockThreshold), lastAttemptedVoltage(0.0), wasLockedLastCycle(false), currentBrakeMode(vex::brake) {
    decelStepVoltage = absoluteMaxVoltage * (decelStepPercent / 100.0);
}
void adaptiveABS::initialize(double startingVoltage) { lastAttemptedVoltage = startingVoltage; wasLockedLastCycle = false; currentBrakeMode = vex::brake; }
double adaptiveABS::decelControlSpeed(double wheelSpeed, double robotSpeed) {
    double lockupRatio = calculateLockupRatio(wheelSpeed, robotSpeed);
    if (lockupRatio > lockThreshold) { lastAttemptedVoltage = 0.0; currentBrakeMode = vex::coast; wasLockedLastCycle = true; }
    else if (wasLockedLastCycle) { currentBrakeMode = vex::brake; wasLockedLastCycle = false; }
    else { lastAttemptedVoltage = std::copysign(std::max(0.0, std::fabs(lastAttemptedVoltage) - decelStepVoltage), lastAttemptedVoltage); currentBrakeMode = vex::brake; }
    return lastAttemptedVoltage;
}

void arcTurn(double targetDistance, double breakDistance, double minSpeed, double maxSpeed, double turnRadius, bool turnLeft) { 
    double innerRatio = (turnRadius - (TRACK_WIDTH / 2)) / turnRadius, outerRatio = (turnRadius + (TRACK_WIDTH / 2)) / turnRadius;
    
    double startDist = getCurrentEncoderDistanceCM();
    double distanceTravelled = 0;
    double maxV = maxSpeed * 0.12, minV = minSpeed * 0.12;
    double innerV = minV * innerRatio, outerV = minV * outerRatio;

    while (std::fabs(distanceTravelled) <= fabs(targetDistance)) {
        distanceTravelled = getCurrentEncoderDistanceCM() - startDist;
        if (fabs(distanceTravelled) < (fabs(targetDistance) - breakDistance)) { innerV = std::min(maxV * innerRatio, innerV + 0.1); outerV = std::min(maxV * outerRatio, outerV + 0.1); }
        else { innerV = std::max(minV * innerRatio, innerV - 0.1); outerV = std::max(minV * outerRatio, outerV - 0.1); }
        for (int i = 0; i < 3; i++) { leftMotor[i].spin(forward, turnLeft ? innerV : outerV, volt); rightMotor[i].spin(forward, turnLeft ? outerV : innerV, volt); }
        vex::task::sleep(20);
    }
    for (int i = 0; i < 3; i++) { leftMotor[i].stop(coast); rightMotor[i].stop(coast); }
}

void straightOdometry(double targetDistance,
                      double breakDistance,
                      double targetHeading,
                      double minSpeed,
                      double kp_heading,
                      double ki_heading,
                      double kd_heading,
                      double accelHeadingScaling,
                      double decelHeadingScaling,
                      double approachHeadingScaling,
                      double maxSpeed)
{

    // ========================================
    // CONFIGURATION CONSTANTS
    // ========================================
    // Adaptive Launch Control Configuration
    const double LAUNCH_VOLTAGE = 12;         // Starting voltage, must be higher than 0
    const double ACCEL_FACTOR_LAUNCH = 1.25; // Acceleration rate MUST be > 1.0
    // slipThreshold: 0-1 range (0 = no slip allowed, 1 = full slip allowed, .15-.25 = optimal slip)
    const double SLIP_THRESHOLD_TRACTION = 0.25; // Slip threshold 1 is always power, 0 is no power
    // Adaptive ABS configuration
    const double DECEL_STEP_PERCENT = 20;    // Voltage step as % of 12V (range: 1-10)
    const double LOCK_THRESHOLD_DECEL = 0.25; // Lockup sensitivity (range: 0.15-0.40)
    // ========================================

    // Add timer for acceleration phase
    vex::timer accelTimer;

    // Reset the encoders to start counting from zero
    passiveEncoderLeft.resetPosition();
    passiveEncoderRight.resetPosition();

    // Initialize PID controllers
    PID headingPID(kp_heading, ki_heading, kd_heading);
    headingPID.pidReset(); // Remove extra blank line after this

    // Motion Parameters
    double currentDistance = 0;
    // double headingDirection = (targetDistance > 0) ? 1.0 : -1.0;

    // Target Speeds & Voltages
    double maxSpeedVoltage = std::copysign(maxSpeed * 0.01 * absoluteMaxVoltage, targetDistance);
    double minSpeedVoltage = std::copysign(minSpeed * 0.01 * absoluteMaxVoltage, targetDistance);
    double launchVoltage = std::copysign(LAUNCH_VOLTAGE, targetDistance);
    double minLaunchSpeedVoltage = std::copysign(std::min(fabs(maxSpeedVoltage), fabs(LAUNCH_VOLTAGE)), targetDistance);

    // RPM Parameters
    double minDriveMotorRPM = (minSpeed * .01) * absoluteMaxRPM; // Adjusted for frictional losses
    double maxEncoderRPM = 0;
    // Motor Arrays
    double motorVoltageLeft[3] = {minLaunchSpeedVoltage, minLaunchSpeedVoltage, minLaunchSpeedVoltage};
    double motorVoltageRight[3] = {minLaunchSpeedVoltage, minLaunchSpeedVoltage, minLaunchSpeedVoltage};
    double leftMotorRPM = 0;
    double rightMotorRPM = 0;

    // PID and Heading Control

    // double normTargetHeading = normHeading(targetHeading);
    double avgMotorVoltage = 0; // Used for phase transition checking
    double leftEncoderRollingAverage = 0;
    double rightEncoderRollingAverage = 0;
    double voltageRollingAverage = 0;

    bool decel = false;
    bool decelCompleted = false;
    bool accelCompleted = false;
    int consecutiveAtTarget = 0;
    const int REQUIRED_CONSECUTIVE = 3;

    // TEMPORARY - Voltage tracking for debugging traction control
    double maxLeftVoltageReached = 0;
    double maxRightVoltageReached = 0;
    double maxAvgLeftVoltage = 0;
    double maxAvgRightVoltage = 0;
    double maxLeftMotor[3] = {0, 0, 0};  // â† ADD THIS
    double maxRightMotor[3] = {0, 0, 0}; // â† ADD THIS

    // Declare arrays of Slip Control instances for each wheel
    tractionControl tractionControlLeft(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION);
    tractionControl tractionControlRight(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION);

    // Create ABS instances for left and right sides
    adaptiveABS adaptiveABSLeft(DECEL_STEP_PERCENT, LOCK_THRESHOLD_DECEL);
    adaptiveABS adaptiveABSRight(DECEL_STEP_PERCENT, LOCK_THRESHOLD_DECEL);

    while (std::fabs(currentDistance) <= fabs(targetDistance) - 6)
    {

        currentDistance = ((passiveEncoderLeft.position(degrees) + passiveEncoderRight.position(degrees)) / 2.0 / 360.0) * encoderWheelCircumferenceCM;
        avgMotorVoltage = (motorVoltageLeft[0] + motorVoltageLeft[1] + motorVoltageLeft[2] + motorVoltageRight[0] + motorVoltageRight[1] + motorVoltageRight[2]) / numberDriveMotor;

        // Display current encoder distance
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Current: %.2f cm", std::fabs(currentDistance));
        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("Target: %.2f cm", std::fabs(targetDistance));

        /*
        // Distance calculation debug
        Brain.Screen.clearScreen();  // Clear previous prints
        Brain.Screen.setCursor(1,1);
        Brain.Screen.print("L/R deg: %.1f/%.1f",
            passiveEncoderLeft.position(degrees),
            passiveEncoderRight.position(degrees));
        Brain.Screen.setCursor(2,1);
        Brain.Screen.print("Dist/Target: %.1f/%.1f", currentDistance, targetDistance);

        Brain.Screen.setCursor(4,1);
        Brain.Screen.print("Launch/Cruise/Decel: %d/%d/%d",
            (std::fabs(currentDistance) < (std::fabs(targetDistance) - breakDistance) && !accelCompleted && !decel),
            (std::fabs(currentDistance) < (std::fabs(targetDistance) - breakDistance) && accelCompleted == true),
            (std::fabs(currentDistance) >= std::fabs(targetDistance) - breakDistance));
        */

        // Calculate the heading correction using the PID controller
        // double headingError = getHeadingError360(targetHeading, InertialSensor.heading());
        // Calculate the heading correction using the PID controller with normalization
        // Calculate the heading correction using normalized error
        // Use continuous rotation instead of heading()
        double currentHeading = getAdjustedRotation();
        double headingCorrection = headingPID.calculate(targetHeading, currentHeading);

        // Get encoder speeds FIRST
        double leftEncoderRPM = passiveEncoderLeft.velocity(vex::velocityUnits::rpm) * (encoderWheelCircumferenceCM / wheelCircumferenceCM);
        double rightEncoderRPM = passiveEncoderRight.velocity(vex::velocityUnits::rpm) * (encoderWheelCircumferenceCM / wheelCircumferenceCM);
        double avgEncoderRPM = (leftEncoderRPM + rightEncoderRPM) / 2;

        // Get motor RPMs
        // Get middle motor RPM only (index 1 - traction wheel)
        double leftMotorRPM = leftMotor[1].velocity(vex::velocityUnits::rpm) * DRIVE_MOTOR_RPM_ADJ;
        double rightMotorRPM = rightMotor[1].velocity(vex::velocityUnits::rpm) * DRIVE_MOTOR_RPM_ADJ;

        // Launch Phase
        if (fabs(currentDistance) < (fabs(targetDistance) - breakDistance) && !accelCompleted && !decel)
        { // will keep going until acceleration is complete and not in decel
          //  Brain.Screen.printAt(10, 20, "Launch Phase");

            // Print initial values
            // Brain.Screen.printAt(10, 80, "Init L: %.2f, R: %.2f", motorVoltageLeft[i], motorVoltageRight[i]);

            // Call traction cotrol class and get adjusted motor voltage
            double leftTractionVoltage = tractionControlLeft.tractionControlSpeed(motorVoltageLeft[1], leftMotorRPM, leftEncoderRPM, ACCEL_FACTOR_LAUNCH);
            double rightTractionVoltage = tractionControlRight.tractionControlSpeed(motorVoltageRight[1], rightMotorRPM, rightEncoderRPM, ACCEL_FACTOR_LAUNCH);
            // PIDVoltageCapCorrection(motorVoltageLeft[i], motorVoltageRight[i], absoluteMaxVoltage);

            // Synchronized control - use the lower voltage (more conservative) for BOTH sides
            // Synchronized control - use the lower MAGNITUDE voltage (more conservative) for BOTH sides
            // For negative values, std::min picks the MORE negative (higher magnitude), so use fabs comparison
            double syncedMotorVoltage = (std::fabs(leftTractionVoltage) < std::fabs(rightTractionVoltage)) 
                ? leftTractionVoltage 
                : rightTractionVoltage;

            // Apply to all 3 motors on both sides with PID correction
            // Apply to all 3 motors on both sides with PID correction - CORRECTED SIGNS
            for (int i = 0; i < 3; i++)
            {
                motorVoltageLeft[i] = syncedMotorVoltage - (headingCorrection * accelHeadingScaling);
                motorVoltageRight[i] = syncedMotorVoltage + (headingCorrection * accelHeadingScaling);
            }

            maxEncoderRPM = std::max(maxEncoderRPM, fabs(avgEncoderRPM));

            if (fabs(avgMotorVoltage) >= (fabs(maxSpeedVoltage) - VOLTAGE_TOLERANCE))
            {

                //  Brain.Screen.setCursor(12,1);
                // Brain.Screen.print("Raw RPM L/R: %.1f/%.1f", leftMotor[0].velocity(velocityUnits::rpm), rightMotor[0].velocity(velocityUnits::rpm));

                accelCompleted = true;
                /*
                        // Stop all motors at end of routine after approach
                        for (int i = 0; i < 3; i++) {
                            motorVoltageLeft[i] = 0;
                            motorVoltageRight[i] = 0;
                            leftMotor[i].stop(brake);
                            rightMotor[i].stop(brake);
                        }
                  */
            }

            // Cruise Phase
        }
        else if (fabs(currentDistance) < (fabs(targetDistance) - breakDistance) && accelCompleted == true)
        {
            // break;
            // Brain.Screen.printAt(10, 20, "Cruise Phase"); // Else condition - specify actions here if the if condition is not met

            for (int i = 0; i < 3; i++)
            {
                // Example action: Set motor voltage to target voltage directly
                motorVoltageLeft[i] = maxSpeedVoltage - (headingCorrection);
                motorVoltageRight[i] = maxSpeedVoltage + (headingCorrection);
                // PIDVoltageCapCorrection(motorVoltageLeft[i], motorVoltageRight[i], absoluteMaxVoltage);
            }

            /*
                Brain.Screen.setCursor(1,1);
            Brain.Screen.print("Distance: %.1f Target: %.1f Break: %.1f",
                std::fabs(currentDistance), std::fabs(targetDistance), breakDistance);

            Brain.Screen.setCursor(2,1);
            Brain.Screen.print("Accel/Decel: %d/%d", accelCompleted, decel);

            Brain.Screen.setCursor(3,1);
            Brain.Screen.print("Dist Check: %d",
                std::fabs(currentDistance) >= std::fabs(targetDistance) - breakDistance);

            */

            // Decel Phase
            // If declerating then go to ABS routine
        }

        // Deceleration phase with adaptive ABS
        else if (fabs(currentDistance) >= (fabs(targetDistance) - breakDistance) && decelCompleted == false)
        {
// First entry - initialize ABS
            if (!decel)
            {
                adaptiveABSLeft.initialize(motorVoltageLeft[1]);
                adaptiveABSRight.initialize(motorVoltageRight[1]);
            }
            decel = true;

            // Get motor and encoder speeds
            double leftMotorRPM = leftMotor[1].velocity(vex::velocityUnits::rpm) * DRIVE_MOTOR_RPM_ADJ;
            double rightMotorRPM = rightMotor[1].velocity(vex::velocityUnits::rpm) * DRIVE_MOTOR_RPM_ADJ;
            double leftEncoderRPM = passiveEncoderLeft.velocity(velocityUnits::rpm) * (encoderWheelCircumferenceCM / wheelCircumferenceCM);
            double rightEncoderRPM = passiveEncoderRight.velocity(velocityUnits::rpm) * (encoderWheelCircumferenceCM / wheelCircumferenceCM);

            // Calculate brake voltages using adaptive ABS
            // Each side independently determines how much voltage to apply based on lockup detection
            double leftDecelVoltage = adaptiveABSLeft.decelControlSpeed(leftMotorRPM, leftEncoderRPM);
            double rightDecelVoltage = adaptiveABSRight.decelControlSpeed(rightMotorRPM, rightEncoderRPM);

            // Get brake modes from ABS
            // Returns coast if wheel is locking up, brake if wheel has traction
            vex::brakeType leftBrakeMode = adaptiveABSLeft.getBrakeMode();
            vex::brakeType rightBrakeMode = adaptiveABSRight.getBrakeMode();

            // SYNC brake mode: if EITHER side is locking up (coasting), BOTH sides coast
            // This keeps the robot straight - prevents one side braking harder than the other
            vex::brakeType syncedBrakeMode;
            if (leftBrakeMode == vex::coast || rightBrakeMode == vex::coast) {
                syncedBrakeMode = vex::coast;
            } else {
                syncedBrakeMode = vex::brake;
            }

            // SYNC voltage: use the MINIMUM magnitude (most conservative) for BOTH sides
            // Mirrors accel phase pattern - whichever side needs less power dictates both
            // This ensures both sides decelerate at the same rate to maintain straight tracking
            double syncedDecelVoltage = (std::fabs(leftDecelVoltage) < std::fabs(rightDecelVoltage))
                ? leftDecelVoltage
                : rightDecelVoltage;

            // Calculate steering correction scaled for decel phase
            double steeringCorrection = headingCorrection * decelHeadingScaling;

            // Apply synced voltage with heading correction (mirrors accel phase pattern)
            // Pattern: syncedBase + correction (left) / syncedBase - correction (right)
            for (int i = 0; i < 3; i++)
            {
                // Only apply voltage if we're actively braking AND have voltage to apply
                if (syncedBrakeMode == vex::brake && std::fabs(syncedDecelVoltage) > 0.0)
                {
                    double correctedLeft = syncedDecelVoltage - steeringCorrection;
                    double correctedRight = syncedDecelVoltage + steeringCorrection;
                    
                    // Clamp toward zero - don't let steering correction reverse motor direction
                    // Forward (positive voltage): use max(0, x) to prevent negative values
                    // Backward (negative voltage): use min(0, x) to prevent positive values
                    if (syncedDecelVoltage > 0) {
                        motorVoltageLeft[i] = std::max(0.0, correctedLeft);
                        motorVoltageRight[i] = std::max(0.0, correctedRight);
                    } else {
                        motorVoltageLeft[i] = std::min(0.0, correctedLeft);
                        motorVoltageRight[i] = std::min(0.0, correctedRight);
                    }
                }
                else
                {
                    // Coasting to release lockup OR fully stopped - zero voltage
                    motorVoltageLeft[i] = 0.0;
                    motorVoltageRight[i] = 0.0;
                }
                
                // Set synced brake mode for both sides to maintain symmetry
                leftMotor[i].setBrake(syncedBrakeMode);
                rightMotor[i].setBrake(syncedBrakeMode);
            }

            // Update rolling averages for exit detection
            leftEncoderRollingAverage = rollingAverage(leftEncoderRPM, leftEncoderRollingAverage, 10);
            rightEncoderRollingAverage = rollingAverage(rightEncoderRPM, rightEncoderRollingAverage, 10);

            // Exit when BOTH sides slowed to minimum speed, confirmed multiple times
            if (fabs(leftEncoderRollingAverage) <= fabs(minDriveMotorRPM) &&
                fabs(rightEncoderRollingAverage) <= fabs(minDriveMotorRPM))
            {
                consecutiveAtTarget++;
                if (consecutiveAtTarget >= REQUIRED_CONSECUTIVE)
                {
                    decelCompleted = true;
                }
            }
            else
            {
                consecutiveAtTarget = 0;  // Reset if we pop back above target
            }
        }

        // Final Approach Phase
        else if (decelCompleted == true)
        {
            // break;
            //    Brain.Screen.printAt(10, 20, "Approach Phase");
            //    Brain.Screen.printAt(10, 40, "Decel Compl: %d", decelCompleted);

            for (int i = 0; i < 3; i++)
            {
                // Example action: Set motor voltage to target voltage directly
                motorVoltageLeft[i] = minSpeedVoltage - (headingCorrection * approachHeadingScaling);
                motorVoltageRight[i] = minSpeedVoltage + (headingCorrection * approachHeadingScaling);
                // PIDVoltageCapCorrection(motorVoltageLeft[i], motorVoltageRight[i], absoluteMaxVoltage);
            }
        }

        // Track maximum voltages - both individual motors AND averages
        for (int i = 0; i < 3; i++)
        {
            // Track highest voltage for ANY motor on each side
            maxLeftVoltageReached = std::max(maxLeftVoltageReached, std::fabs(motorVoltageLeft[i]));
            maxRightVoltageReached = std::max(maxRightVoltageReached, std::fabs(motorVoltageRight[i]));

            maxLeftMotor[i] = std::max(maxLeftMotor[i], std::fabs(motorVoltageLeft[i]));
            maxRightMotor[i] = std::max(maxRightMotor[i], std::fabs(motorVoltageRight[i]));
        }

        // Calculate and track average voltage across all motors
        double currentAvgLeftVoltage = (std::fabs(motorVoltageLeft[0]) +
                                        std::fabs(motorVoltageLeft[1]) +
                                        std::fabs(motorVoltageLeft[2])) /
                                       3.0;

        double currentAvgRightVoltage = (std::fabs(motorVoltageRight[0]) +
                                         std::fabs(motorVoltageRight[1]) +
                                         std::fabs(motorVoltageRight[2])) /
                                        3.0;

        static double maxAvgLeftVoltage = 0;
        static double maxAvgRightVoltage = 0;
        maxAvgLeftVoltage = std::max(maxAvgLeftVoltage, currentAvgLeftVoltage);
        maxAvgRightVoltage = std::max(maxAvgRightVoltage, currentAvgRightVoltage);

        // Power Drive Motors

        // turnDirection = std::copysign(turnDirection, normTargetHeading);
        // if (!decel == true || decelCompleted == true)
        //{
        for (int i = 0; i < 3; i++)
        {
            leftMotor[i].spin(forward, motorVoltageLeft[i], voltageUnits::volt);
            rightMotor[i].spin(forward, motorVoltageRight[i], voltageUnits::volt);
        }
        //}
        /*
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("=== Movement Complete ===");

        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("LEFT: %.1f %.1f %.1f",
                           maxLeftMotor[0], maxLeftMotor[1], maxLeftMotor[2]);

        Brain.Screen.setCursor(3, 1);
        Brain.Screen.print("RIGHT: %.1f %.1f %.1f",
                           maxRightMotor[0], maxRightMotor[1], maxRightMotor[2]);

        Brain.Screen.setCursor(4, 1);
        Brain.Screen.print("MaxL: %.2f MaxR: %.2f",
                           maxLeftVoltageReached, maxRightVoltageReached);

        Brain.Screen.setCursor(5, 1);
        Brain.Screen.print("AvgL: %.2f AvgR: %.2f",
                           maxAvgLeftVoltage, maxAvgRightVoltage);

        Brain.Screen.setCursor(6, 1);
        Brain.Screen.print("Target: %.2fV", fabs(maxSpeedVoltage));

        Brain.Screen.setCursor(7, 1);
        Brain.Screen.print("Distance: %.1f / %.1f",
                           fabs(currentDistance), fabs(targetDistance));
        */
        vex::task::sleep(10);
    }

    // Stop all motors at end of routine after approach
    // Stop all motors at end of routine after approach
    for (int i = 0; i < 3; i++)
    {
        leftMotor[i].setBrake(brake);
        rightMotor[i].setBrake(brake);
        leftMotor[i].stop();
        rightMotor[i].stop();
    }

    //wait(120, msec); // brief pause to allow motors to settle
/*
       for (int i = 0; i < 3; i++)
    {
        leftMotor[i].setBrake(hold);
        rightMotor[i].setBrake(hold);
        leftMotor[i].stop();
        rightMotor[i].stop();
    }
*/
 
    // Display detailed movement summary with all 6 motors
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("=== Movement Complete ===");

    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("LEFT: %.1f %.1f %.1f",
                       maxLeftMotor[0], maxLeftMotor[1], maxLeftMotor[2]);

    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("RIGHT: %.1f %.1f %.1f",
                       maxRightMotor[0], maxRightMotor[1], maxRightMotor[2]);

    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("MaxL: %.2f MaxR: %.2f",
                       maxLeftVoltageReached, maxRightVoltageReached);

    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("AvgL: %.2f AvgR: %.2f",
                       maxAvgLeftVoltage, maxAvgRightVoltage);

    Brain.Screen.setCursor(6, 1);
    Brain.Screen.print("Target: %.2fV", fabs(maxSpeedVoltage));

    Brain.Screen.setCursor(7, 1);
    Brain.Screen.print("Distance: %.1f / %.1f",
                       fabs(currentDistance), fabs(targetDistance));

    // wait(10000, msec);
    /*
        currentDistance = (passiveEncoderLeft.position(degrees) + passiveEncoderRight.position(degrees) ) / 2 * (encoderWheelCircumferenceCM / 360.0);
        // Debug print: Stopping motors
        Brain.Screen.clearLine(8);
        Brain.Screen.setCursor(8, 1);
        Brain.Screen.print("Distance Complete");
        Brain.Screen.print("Current Distance: %.2f", currentDistance);
    */
}


void straightOdometryV2(double targetDistance,
                      double breakDistance,
                      double targetHeading,
                      double minSpeed,
                      double distanceTolerance,
                      double kp_heading,
                      double ki_heading,
                      double kd_heading,
                      double accelHeadingScaling,
                      double decelHeadingScaling,
                      double approachHeadingScaling,
                      double maxSpeed)
{
    // ========================================
    // CONFIGURATION CONSTANTS
    // ========================================
    const double LAUNCH_VOLTAGE = 6;
    const double ACCEL_FACTOR_LAUNCH = 1.25;
    const double SLIP_THRESHOLD_TRACTION = 0.3;
    const double DECEL_STEP_PERCENT = 20;
    const double LOCK_THRESHOLD_DECEL = 0.25;
    // ========================================

    vex::timer accelTimer;

    passiveEncoderLeft.resetPosition();
    passiveEncoderRight.resetPosition();

    PID headingPID(kp_heading, ki_heading, kd_heading);
    headingPID.pidReset();

    double currentDistance = 0;

    double maxSpeedVoltage = std::copysign(maxSpeed * 0.01 * absoluteMaxVoltage, targetDistance);
    double minSpeedVoltage = std::copysign(minSpeed * 0.01 * absoluteMaxVoltage, targetDistance);
    double launchVoltage = std::copysign(LAUNCH_VOLTAGE, targetDistance);
    double minLaunchSpeedVoltage = std::copysign(std::min(fabs(maxSpeedVoltage), fabs(LAUNCH_VOLTAGE)), targetDistance);

    double minDriveMotorRPM = (minSpeed * .01) * absoluteMaxRPM;
    double maxEncoderRPM = 0;

    double motorVoltageLeft[3] = {minLaunchSpeedVoltage, minLaunchSpeedVoltage, minLaunchSpeedVoltage};
    double motorVoltageRight[3] = {minLaunchSpeedVoltage, minLaunchSpeedVoltage, minLaunchSpeedVoltage};
    double leftMotorRPM = 0;
    double rightMotorRPM = 0;

    double avgMotorVoltage = 0;
    double leftEncoderRollingAverage = 0;
    double rightEncoderRollingAverage = 0;
    double voltageRollingAverage = 0;

    bool decel = false;
    bool decelCompleted = false;
    bool accelCompleted = false;
    int consecutiveAtTarget = 0;
    const int REQUIRED_CONSECUTIVE = 3;

    double maxLeftVoltageReached = 0;
    double maxRightVoltageReached = 0;
    double maxAvgLeftVoltage = 0;
    double maxAvgRightVoltage = 0;
    double maxLeftMotor[3] = {0, 0, 0};
    double maxRightMotor[3] = {0, 0, 0};

    tractionControl tractionControlLeft(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION);
    tractionControl tractionControlRight(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION);

    adaptiveABS adaptiveABSLeft(DECEL_STEP_PERCENT, LOCK_THRESHOLD_DECEL);
    adaptiveABS adaptiveABSRight(DECEL_STEP_PERCENT, LOCK_THRESHOLD_DECEL);

    while (std::fabs(currentDistance) <= fabs(targetDistance) - distanceTolerance)
    {
        currentDistance = ((passiveEncoderLeft.position(degrees) + passiveEncoderRight.position(degrees)) / 2.0 / 360.0) * encoderWheelCircumferenceCM;
        avgMotorVoltage = (motorVoltageLeft[0] + motorVoltageLeft[1] + motorVoltageLeft[2] + motorVoltageRight[0] + motorVoltageRight[1] + motorVoltageRight[2]) / numberDriveMotor;

        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Current: %.2f cm", std::fabs(currentDistance));
        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("Target: %.2f cm", std::fabs(targetDistance));

        // Use continuous heading to prevent wrap-around discontinuities at ±180°
        double currentHeading = getContinuousStandardHeading();
        // Adjust target heading to match current rotation count (prevents wrap-around jumps)
        double rotationsDiff = std::round((currentHeading - targetHeading) / 360.0);
        double adjustedTargetHeading = targetHeading + (rotationsDiff * 360.0);
        double headingCorrection = headingPID.calculate(adjustedTargetHeading, currentHeading);

        double leftEncoderRPM = passiveEncoderLeft.velocity(vex::velocityUnits::rpm) * (encoderWheelCircumferenceCM / wheelCircumferenceCM);
        double rightEncoderRPM = passiveEncoderRight.velocity(vex::velocityUnits::rpm) * (encoderWheelCircumferenceCM / wheelCircumferenceCM);
        double avgEncoderRPM = (leftEncoderRPM + rightEncoderRPM) / 2;

        double leftMotorRPM = leftMotor[1].velocity(vex::velocityUnits::rpm) * DRIVE_MOTOR_RPM_ADJ;
        double rightMotorRPM = rightMotor[1].velocity(vex::velocityUnits::rpm) * DRIVE_MOTOR_RPM_ADJ;

        // Launch Phase
        if (fabs(currentDistance) < (fabs(targetDistance) - breakDistance) && !accelCompleted && !decel)
        {
            double leftTractionVoltage = tractionControlLeft.tractionControlSpeed(motorVoltageLeft[1], leftMotorRPM, leftEncoderRPM, ACCEL_FACTOR_LAUNCH);
            double rightTractionVoltage = tractionControlRight.tractionControlSpeed(motorVoltageRight[1], rightMotorRPM, rightEncoderRPM, ACCEL_FACTOR_LAUNCH);

            double syncedMotorVoltage = (std::fabs(leftTractionVoltage) < std::fabs(rightTractionVoltage)) 
                ? leftTractionVoltage 
                : rightTractionVoltage;

            for (int i = 0; i < 3; i++)
            {
                motorVoltageLeft[i] = syncedMotorVoltage - (headingCorrection * accelHeadingScaling);
                motorVoltageRight[i] = syncedMotorVoltage + (headingCorrection * accelHeadingScaling);
            }

            maxEncoderRPM = std::max(maxEncoderRPM, fabs(avgEncoderRPM));

            if (fabs(avgMotorVoltage) >= (fabs(maxSpeedVoltage) - VOLTAGE_TOLERANCE))
            {
                accelCompleted = true;
            }
        }
        // Cruise Phase
        else if (fabs(currentDistance) < (fabs(targetDistance) - breakDistance) && accelCompleted == true)
        {
            for (int i = 0; i < 3; i++)
            {
                motorVoltageLeft[i] = maxSpeedVoltage - (headingCorrection);
                motorVoltageRight[i] = maxSpeedVoltage + (headingCorrection);
            }
        }
        // Deceleration phase with adaptive ABS
        else if (fabs(currentDistance) >= (fabs(targetDistance) - breakDistance) && decelCompleted == false)
        {
            if (!decel)
            {
                adaptiveABSLeft.initialize(motorVoltageLeft[1]);
                adaptiveABSRight.initialize(motorVoltageRight[1]);
            }
            decel = true;

            double leftMotorRPM = leftMotor[1].velocity(vex::velocityUnits::rpm) * DRIVE_MOTOR_RPM_ADJ;
            double rightMotorRPM = rightMotor[1].velocity(vex::velocityUnits::rpm) * DRIVE_MOTOR_RPM_ADJ;
            double leftEncoderRPM = passiveEncoderLeft.velocity(velocityUnits::rpm) * (encoderWheelCircumferenceCM / wheelCircumferenceCM);
            double rightEncoderRPM = passiveEncoderRight.velocity(velocityUnits::rpm) * (encoderWheelCircumferenceCM / wheelCircumferenceCM);

            double leftDecelVoltage = adaptiveABSLeft.decelControlSpeed(leftMotorRPM, leftEncoderRPM);
            double rightDecelVoltage = adaptiveABSRight.decelControlSpeed(rightMotorRPM, rightEncoderRPM);

            vex::brakeType leftBrakeMode = adaptiveABSLeft.getBrakeMode();
            vex::brakeType rightBrakeMode = adaptiveABSRight.getBrakeMode();

            vex::brakeType syncedBrakeMode;
            if (leftBrakeMode == vex::coast || rightBrakeMode == vex::coast) {
                syncedBrakeMode = vex::coast;
            } else {
                syncedBrakeMode = vex::brake;
            }

            double syncedDecelVoltage = (std::fabs(leftDecelVoltage) < std::fabs(rightDecelVoltage))
                ? leftDecelVoltage
                : rightDecelVoltage;

            double steeringCorrection = headingCorrection * decelHeadingScaling;

            for (int i = 0; i < 3; i++)
            {
                if (syncedBrakeMode == vex::brake && std::fabs(syncedDecelVoltage) > 0.0)
                {
                    double correctedLeft = syncedDecelVoltage - steeringCorrection;
                    double correctedRight = syncedDecelVoltage + steeringCorrection;
                    
                    if (syncedDecelVoltage > 0) {
                        motorVoltageLeft[i] = std::max(0.0, correctedLeft);
                        motorVoltageRight[i] = std::max(0.0, correctedRight);
                    } else {
                        motorVoltageLeft[i] = std::min(0.0, correctedLeft);
                        motorVoltageRight[i] = std::min(0.0, correctedRight);
                    }
                }
                else
                {
                    motorVoltageLeft[i] = 0.0;
                    motorVoltageRight[i] = 0.0;
                }
                
                leftMotor[i].setBrake(syncedBrakeMode);
                rightMotor[i].setBrake(syncedBrakeMode);
            }

            leftEncoderRollingAverage = rollingAverage(leftEncoderRPM, leftEncoderRollingAverage, 10);
            rightEncoderRollingAverage = rollingAverage(rightEncoderRPM, rightEncoderRollingAverage, 10);

            if (fabs(leftEncoderRollingAverage) <= fabs(minDriveMotorRPM) &&
                fabs(rightEncoderRollingAverage) <= fabs(minDriveMotorRPM))
            {
                consecutiveAtTarget++;
                if (consecutiveAtTarget >= REQUIRED_CONSECUTIVE)
                {
                    decelCompleted = true;
                }
            }
            else
            {
                consecutiveAtTarget = 0;
            }
        }
        // Final Approach Phase
        else if (decelCompleted == true)
        {
            for (int i = 0; i < 3; i++)
            {
                motorVoltageLeft[i] = minSpeedVoltage - (headingCorrection * approachHeadingScaling);
                motorVoltageRight[i] = minSpeedVoltage + (headingCorrection * approachHeadingScaling);
            }
        }

        for (int i = 0; i < 3; i++)
        {
            maxLeftVoltageReached = std::max(maxLeftVoltageReached, std::fabs(motorVoltageLeft[i]));
            maxRightVoltageReached = std::max(maxRightVoltageReached, std::fabs(motorVoltageRight[i]));

            maxLeftMotor[i] = std::max(maxLeftMotor[i], std::fabs(motorVoltageLeft[i]));
            maxRightMotor[i] = std::max(maxRightMotor[i], std::fabs(motorVoltageRight[i]));
        }

        double currentAvgLeftVoltage = (std::fabs(motorVoltageLeft[0]) +
                                        std::fabs(motorVoltageLeft[1]) +
                                        std::fabs(motorVoltageLeft[2])) /
                                       3.0;

        double currentAvgRightVoltage = (std::fabs(motorVoltageRight[0]) +
                                         std::fabs(motorVoltageRight[1]) +
                                         std::fabs(motorVoltageRight[2])) /
                                        3.0;

        static double maxAvgLeftVoltage = 0;
        static double maxAvgRightVoltage = 0;
        maxAvgLeftVoltage = std::max(maxAvgLeftVoltage, currentAvgLeftVoltage);
        maxAvgRightVoltage = std::max(maxAvgRightVoltage, currentAvgRightVoltage);

        for (int i = 0; i < 3; i++)
        {
            leftMotor[i].spin(forward, motorVoltageLeft[i], voltageUnits::volt);
            rightMotor[i].spin(forward, motorVoltageRight[i], voltageUnits::volt);
        }

        vex::task::sleep(10);
    }

    for (int i = 0; i < 3; i++)
    {
        leftMotor[i].setBrake(brake);
        rightMotor[i].setBrake(brake);
        leftMotor[i].stop();
        rightMotor[i].stop();
    }
}


/**
 * straightOdometryV3 - Advanced motion profiling with configurable stopping tolerance
 * 
 * Moves the robot forward or backward with precise heading control and smooth acceleration/deceleration.
 * Uses continuous heading tracking to prevent wrap-around issues and a configurable stopping distance
 * for different accuracy requirements.
 * 
 * @param targetDistance Distance to travel in cm (positive = forward, negative = backward)
 * @param breakDistance Distance in cm before target to begin deceleration
 * @param targetHeading Desired heading in degrees (0° = forward, uses continuous tracking)
 * @param minSpeed Minimum speed percentage during approach phase (0-100)
 * @param distanceTolerance How close to get to target before stopping (cm) - smaller = more precise
 * @param kp_heading Proportional gain for heading correction
 * @param ki_heading Integral gain for heading correction
 * @param kd_heading Derivative gain for heading correction
 * @param accelHeadingScaling Scales heading correction strength during acceleration (0-1)
 * @param decelHeadingScaling Scales heading correction strength during deceleration (0-1)
 * @param approachHeadingScaling Scales heading correction strength during final approach (0-1)
 * @param maxSpeed Maximum speed percentage during cruise phase (0-100)
 */

 void straightOdometryV3(double targetDistance, 
                        double breakDistance, 
                        double targetHeading,
                        double minSpeed,
                        double distanceTolerance,
                        double kp_heading, 
                        double ki_heading, 
                        double kd_heading,
                        double accelHeadingScaling, 
                        double decelHeadingScaling,
                        double approachHeadingScaling, 
                        double maxSpeed,
                        vex::brakeType brakeMode,
                        double timeout) 
{
    // ========================================
    // CONFIGURATION CONSTANTS
    // ========================================
    const double LAUNCH_VOLTAGE = 6.0;              // Initial acceleration voltage
    const double ACCEL_FACTOR_LAUNCH = 1.2;         // Voltage ramp rate during launch
    const double SLIP_THRESHOLD_TRACTION = 20.0;    // Wheel slip detection sensitivity
    const double DECEL_STEP_PERCENT = 0.45;         // ABS brake pressure reduction rate
    const double LOCK_THRESHOLD_DECEL = 0.25;       // Wheel lock detection threshold

    // ========================================
    // INITIALIZATION
    // ========================================
    
    // Record starting encoder position to measure relative distance traveled
    double startDist = getCurrentEncoderDistanceCM();
    double distanceTravelled = 0.0;
    
    // Direction scalar for forward (+1) or backward (-1) movement
    double dir = (targetDistance >= 0) ? 1.0 : -1.0;
    
    // Adjust target heading to nearest equivalent angle in continuous frame
    // Prevents large rotations when robot heading has wrapped past 360°
    double currentHeadingInitial = getContinuousStandardHeading();
    double rotationsDiff = std::round((currentHeadingInitial - targetHeading) / 360.0);
    targetHeading += rotationsDiff * 360.0;
    
    // Initialize PID controller for heading correction
    PID headingPID(kp_heading, ki_heading, kd_heading);
    headingPID.pidReset();
    
    // Convert speed percentages to voltages
    double maxSpeedVoltage = std::copysign(maxSpeed * 0.01 * absoluteMaxVoltage, targetDistance);
    double minSpeedVoltage = std::copysign(minSpeed * 0.01 * absoluteMaxVoltage, targetDistance);
    double launchVoltage = std::copysign(LAUNCH_VOLTAGE, targetDistance);
    
    // Cap launch voltage at lower of maxSpeed or LAUNCH_VOLTAGE to prevent tipping
    double minLaunchSpeedVoltage = std::copysign(std::min(fabs(maxSpeedVoltage), fabs(LAUNCH_VOLTAGE)), targetDistance);
    
    // Calculate minimum RPM threshold for deceleration exit detection
    double minDriveMotorRPM = (minSpeed * 0.01) * absoluteMaxRPM;
    
    // Initialize motor voltage arrays to safe launch speed
    double motorVoltageLeft[3]  = {minLaunchSpeedVoltage, minLaunchSpeedVoltage, minLaunchSpeedVoltage};
    double motorVoltageRight[3] = {minLaunchSpeedVoltage, minLaunchSpeedVoltage, minLaunchSpeedVoltage};
    
    // Phase tracking flags
    bool decel = false;
    bool decelCompleted = false;
    bool accelCompleted = false;

    // Rolling averages for stable speed detection during deceleration
    double leftEncoderRollingAverage = 0;
    double rightEncoderRollingAverage = 0;

    // Traction control instances for independent per-side slip management
    tractionControl tractionControlLeft(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION);
    tractionControl tractionControlRight(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION);

    // ABS controllers for independent per-side wheel lockup prevention
    adaptiveABS adaptiveABSLeft(DECEL_STEP_PERCENT, LOCK_THRESHOLD_DECEL); 
    adaptiveABS adaptiveABSRight(DECEL_STEP_PERCENT, LOCK_THRESHOLD_DECEL);

    // Timeout safety timer
    vex::timer safetyTimer;
    safetyTimer.reset();
    double timeoutMs = timeout * 1000.0;

    // ═══════════════════════════════════════════════════════════════════
    // MAIN CONTROL LOOP
    // ═══════════════════════════════════════════════════════════════════
    while (std::fabs(distanceTravelled) <= std::fabs(targetDistance) - distanceTolerance) 
    {
        // Update current position and heading
        distanceTravelled = getCurrentEncoderDistanceCM() - startDist;
        double currentHeading = getContinuousStandardHeading();
        double headingCorrection = headingPID.calculate(targetHeading, currentHeading);

        // ───────────────────────────────────────────────────────────────
        // TIMEOUT SAFETY CHECK
        // ───────────────────────────────────────────────────────────────
        if (safetyTimer.time(vex::msec) > timeoutMs) {
            break; // Prevent infinite loop on sensor failure or unreachable target
        }

        // Read encoder speeds (ground truth) scaled by wheel size ratio
        double leftEncoderRPM = passiveEncoderLeft.velocity(vex::velocityUnits::rpm) * (encoderWheelCircumferenceCM / wheelCircumferenceCM);
        double rightEncoderRPM = passiveEncoderRight.velocity(vex::velocityUnits::rpm) * (encoderWheelCircumferenceCM / wheelCircumferenceCM);
        
        // Read motor speeds from middle motor as representative sample
        double leftMotorRPM = leftMotor[1].velocity(vex::velocityUnits::rpm) * DRIVE_MOTOR_RPM_ADJ;
        double rightMotorRPM = rightMotor[1].velocity(vex::velocityUnits::rpm) * DRIVE_MOTOR_RPM_ADJ;

        // ───────────────────────────────────────────────────────────────
        // PHASE 1: LAUNCH / ACCELERATION
        // Per-side traction control with slip ratio calculation
        // ───────────────────────────────────────────────────────────────
        if (std::fabs(distanceTravelled) < (std::fabs(targetDistance) - breakDistance) && 
            !accelCompleted && !decel) 
        {
            // Calculate traction-controlled voltage for each side independently
            // Class compares motor speed vs encoder speed and adjusts voltage based on slip ratio
            double leftTractionVoltage = tractionControlLeft.tractionControlSpeed(
                motorVoltageLeft[1], 
                leftMotorRPM, 
                leftEncoderRPM, 
                ACCEL_FACTOR_LAUNCH
            );
            
            double rightTractionVoltage = tractionControlRight.tractionControlSpeed(
                motorVoltageRight[1], 
                rightMotorRPM, 
                rightEncoderRPM, 
                ACCEL_FACTOR_LAUNCH
            );

            // Apply traction-controlled voltages with heading correction to all motors
            // Voltage is already clamped by tractionControl class, but heading correction added after
            for (int i = 0; i < 3; i++) 
            { 
                motorVoltageLeft[i]  = leftTractionVoltage - (headingCorrection * accelHeadingScaling); 
                motorVoltageRight[i] = rightTractionVoltage + (headingCorrection * accelHeadingScaling); 
            }
            
            // Exit acceleration when both sides reach maximum voltage
            if (std::fabs(motorVoltageLeft[1]) >= std::fabs(maxSpeedVoltage) && 
                std::fabs(motorVoltageRight[1]) >= std::fabs(maxSpeedVoltage)) 
            {
                accelCompleted = true;
            }
        }
        
        // ───────────────────────────────────────────────────────────────
        // PHASE 2: CRUISE
        // Maintain maximum speed with heading correction
        // ───────────────────────────────────────────────────────────────
        else if (std::fabs(distanceTravelled) < (std::fabs(targetDistance) - breakDistance) && 
                 accelCompleted) 
        {
            for (int i = 0; i < 3; i++) 
            { 
                motorVoltageLeft[i]  = maxSpeedVoltage - headingCorrection; 
                motorVoltageRight[i] = maxSpeedVoltage + headingCorrection; 
            }
        }
        
        // ───────────────────────────────────────────────────────────────
        // PHASE 3: DECELERATION (Adaptive ABS)
        // Independent per-side brake control with rolling average exit detection
        // ───────────────────────────────────────────────────────────────
        else if (std::fabs(distanceTravelled) >= (std::fabs(targetDistance) - breakDistance) && 
                 !decelCompleted) 
        {
            // Initialize ABS controllers on first entry to deceleration phase
            if (!decel) 
            {
                decel = true;
                adaptiveABSLeft.initialize(std::fabs(motorVoltageLeft[1]));
                adaptiveABSRight.initialize(std::fabs(motorVoltageRight[1]));
            }

            // Update ABS state based on wheel lockup detection
            adaptiveABSLeft.decelControlSpeed(leftMotorRPM, leftEncoderRPM);
            adaptiveABSRight.decelControlSpeed(rightMotorRPM, rightEncoderRPM);

            // Get independent brake modes for each side
            vex::brakeType leftBrakeMode = adaptiveABSLeft.getBrakeMode();
            vex::brakeType rightBrakeMode = adaptiveABSRight.getBrakeMode();

            // Scale heading correction for deceleration phase
            double adjustedHeadingCorrection = headingCorrection * decelHeadingScaling * dir;

            // Apply brake modes and selective release steering
            for (int i = 0; i < 3; i++) 
            { 
                leftMotor[i].setBrake(leftBrakeMode);
                rightMotor[i].setBrake(rightBrakeMode);

                // Selective release: only apply positive correction values for steering
                motorVoltageLeft[i] = std::max(0.0, adjustedHeadingCorrection);
                motorVoltageRight[i] = std::max(0.0, -adjustedHeadingCorrection);
            }

            // Update rolling averages to filter sensor noise (averages last 3 readings)
            leftEncoderRollingAverage = rollingAverage(leftEncoderRPM, leftEncoderRollingAverage, 3);
            rightEncoderRollingAverage = rollingAverage(rightEncoderRPM, rightEncoderRollingAverage, 3);

            // Exit deceleration when both sides slow below minimum threshold
            // AND logic ensures both sides are slow before transitioning to approach
            if (std::fabs(leftEncoderRollingAverage) <= std::fabs(minDriveMotorRPM) && 
                std::fabs(rightEncoderRollingAverage) <= std::fabs(minDriveMotorRPM)) 
            {
                decelCompleted = true;
            }
        }
        
        // ───────────────────────────────────────────────────────────────
        // PHASE 4: APPROACH / FINAL SETTLING
        // Slow precision crawl to target position
        // ───────────────────────────────────────────────────────────────
        else if (decelCompleted) 
        {
            for (int i = 0; i < 3; i++) 
            { 
                // Reset brake mode to standard brake for precision control
                leftMotor[i].setBrake(brake);
                rightMotor[i].setBrake(brake);

                // Apply minimum speed with heading correction
                motorVoltageLeft[i]  = minSpeedVoltage - (headingCorrection * approachHeadingScaling); 
                motorVoltageRight[i] = minSpeedVoltage + (headingCorrection * approachHeadingScaling); 
            }
        }

        // ───────────────────────────────────────────────────────────────
        // VOLTAGE SATURATION LIMITER
        // ───────────────────────────────────────────────────────────────
        // When heading correction pushes one side above absoluteMaxVoltage, proportionally scale
        // both sides down to preserve the steering differential while staying within limits
        double maximumRequestedVoltage = std::max(std::fabs(motorVoltageLeft[0]), 
                                                   std::fabs(motorVoltageRight[0]));
        if (maximumRequestedVoltage > absoluteMaxVoltage) {
            double voltageScaleFactor = absoluteMaxVoltage / maximumRequestedVoltage;
            for (int i = 0; i < 3; i++) { 
                motorVoltageLeft[i] *= voltageScaleFactor;
                motorVoltageRight[i] *= voltageScaleFactor;
            }
        }

        // Apply calculated voltages to all motors
        for (int i = 0; i < 3; i++) 
        {
            leftMotor[i].spin(forward, motorVoltageLeft[i],  volt);
            rightMotor[i].spin(forward, motorVoltageRight[i], volt);
        }
        
        // 10ms delay for 100Hz control loop
        vex::task::sleep(10);
    }

    // ═══════════════════════════════════════════════════════════════════
    // CLEANUP - Stop all motors
    // ═══════════════════════════════════════════════════════════════════
    for (int i = 0; i < 3; i++) 
    {
        leftMotor[i].stop(brakeMode);
        rightMotor[i].stop(brakeMode);
    }
}

// ======================================================================
// Smart straight drive with heading PID and optional wall stall detection
// Fixed: continuous heading, direction scalar for reverse, target snapping
// ======================================================================
// ======================================================================
// Smart straight drive with heading PID
// FIXED: Converts input from Modified (North=0) to Standard (East=0)
// ======================================================================
void smartStraight(double targetDistance, double breakDistance, double targetHeading, 
                   double minSpeed, double wallStalledTimeMs, 
                   double kp_heading, double ki_heading, double kd_heading,
                   double accelHeadingScaling, double decelHeadingScaling,
                   double approachHeadingScaling, double maxSpeed)
{
    const double WALL_THRESHOLD = 5.0; 
    vex::timer wallTimer; 
    bool wallDetected = false, isStalled = false;
    
    double startDist = getCurrentEncoderDistanceCM();
    double distanceTravelled = 0;
    
    // 1. CONVERT HEADING: Modified (North=0) -> Standard (East=0)
    // This aligns the input target with the system used by getContinuousStandardHeading()
   // Target heading is already in Standard Cartesian (East=0°, CCW+)
    double targetHeadingStandard = targetHeading;

    // 2. SNAP TO NEAREST ROTATION
    // If the robot is at 360° and target is 0°, this makes the target 360° instead of 0°
    // to prevent unwinding the robot.
    double currentHeading = getContinuousStandardHeading();
    double rotationsDiff = std::round((currentHeading - targetHeadingStandard) / 360.0);
    targetHeadingStandard += rotationsDiff * 360.0;

    PID headingPID(kp_heading, ki_heading, kd_heading);
    headingPID.pidReset();
    
    // Set max/min voltage with correct sign for forward/backward
    double maxV = std::copysign(maxSpeed * 0.12, targetDistance);
    double minV = std::copysign(minSpeed * 0.12, targetDistance);
    
    double motorVoltageLeft[3]  = {minV, minV, minV};
    double motorVoltageRight[3] = {minV, minV, minV};

    while (std::fabs(distanceTravelled) <= std::fabs(targetDistance) - 6.9 && !wallDetected) 
    {
        distanceTravelled = getCurrentEncoderDistanceCM() - startDist;
        
        // Update Heading 
        currentHeading = getContinuousStandardHeading();
        double headingCorrection = headingPID.calculate(targetHeadingStandard, currentHeading);
        
        double avgRPM = (std::fabs(passiveEncoderLeft.velocity(rpm)) + 
                         std::fabs(passiveEncoderRight.velocity(rpm))) / 2.0;

        // Wall stall detection
        if (wallStalledTimeMs > 0) 
        {
            if (avgRPM < WALL_THRESHOLD) 
            { 
                if (!isStalled) { wallTimer.reset(); isStalled = true; } 
                else if (wallTimer.time(msec) >= wallStalledTimeMs) wallDetected = true; 
            }
            else isStalled = false;
        }

        // Apply differential drive
        // Note: headingCorrection sign logic assumes Standard Cartesian (Left Turn = Positive Error)
        for (int i = 0; i < 3; i++) 
        { 
            // Subtraction on Left / Addition on Right = Turn Left (CCW)
            motorVoltageLeft[i]  = maxV - headingCorrection; 
            motorVoltageRight[i] = maxV + headingCorrection; 
            
            leftMotor[i].spin(forward, motorVoltageLeft[i], volt); 
            rightMotor[i].spin(forward, motorVoltageRight[i], volt); 
        }
        
        vex::task::sleep(10);
    }
    
    for (int i = 0; i < 3; i++) 
    { 
        leftMotor[i].stop(brake); 
        rightMotor[i].stop(brake); 
    }
}

void forwardMP(double targetDistance, double breakDistance, double targetHeading, double minSpeed, double kp_heading, double ki_heading, double kd_heading, double accelHeadingScaling, double decelHeadingScaling, double approachHeadingScaling, double maxSpeed) {
    straightOdometry(targetDistance, breakDistance, targetHeading, minSpeed, kp_heading, ki_heading, kd_heading, accelHeadingScaling, decelHeadingScaling, approachHeadingScaling, maxSpeed);
}

// In backwardMP (wrapper for profiled straight backward)
void backwardMP(double targetDistance,
                double breakDistance,
                double targetHeading,  // Keep as-is (Modified Cartesian)
                double minSpeed,
                double kp_heading,
                double ki_heading,
                double kd_heading,
                double accelHeadingScaling,
                double decelHeadingScaling,
                double approachHeadingScaling,
                double maxSpeed) {

    // Convert once to Standard Cartesian for internal use
    double targetHeadingStandard = targetHeading;

    // NEGATE distance for reverse travel, but DO NOT add 180° to heading
    double negativeDistance = -fabs(targetDistance);
    if (maxSpeed > 0) maxSpeed = -maxSpeed;  // Optional: make maxSpeed negative for reverse

    straightOdometry(negativeDistance,
                     breakDistance,
                     targetHeadingStandard,  // <-- NO +180.0
                     minSpeed,
                     kp_heading, ki_heading, kd_heading,
                     accelHeadingScaling, decelHeadingScaling,
                     approachHeadingScaling, maxSpeed);
}

void leftMP(double turnAmount, double breakDistance, double minSpeed, double maxSpeed) {
    turnOdometry(getContinuousStandardHeading() + turnAmount, breakDistance, minSpeed, maxSpeed);
}

void rightMP(double turnAmount, double breakDistance, double minSpeed, double maxSpeed) {
    turnOdometry(getContinuousStandardHeading() - turnAmount, breakDistance, minSpeed, maxSpeed);
}

// ======================================================================
// Pivot turn to absolute target heading (continuous Standard Cartesian)
// One side brakes/stops, opposite side drives to pivot in place
// Original simple implementation — only fixed heading system to use continuous
// No new parameters, no profiling phases added — legacy behavior preserved
// ======================================================================
void pivotTurnOdometry(double targetHeading, double breakDistanceInDegrees,
                       double minSpeed, double maxSpeed)
{
    // Initial error calculation (uses continuous heading from the start)
    double headingError = targetHeading - getContinuousStandardHeading();

    // Calculate initial max/min voltages based on error direction
    double maxV = std::copysign(maxSpeed * 0.12, headingError);
    // double minV = std::copysign(minSpeed * 0.12, headingError);  // unused - commented to avoid warning

    // Motor voltage arrays were declared but never really used in original logic
    // double motorVoltageLeft[3] = {maxV, maxV, maxV};     // unused
    // double motorVoltageRight[3] = {-maxV, -maxV, -maxV}; // unused

    while (std::abs(headingError) > 9)  // Original exit threshold (9 degrees)
    {
        // FIXED: Use continuous standard heading (unbounded, no wrap-around jumps)
        // This replaces the old getAdjustedRotation() which caused discontinuities
        double currentHeading = getContinuousStandardHeading();
        headingError = targetHeading - currentHeading;

        // Recalculate voltage every loop based on current continuous error
        double volt = std::copysign(maxSpeed * 0.12, headingError);

        // Pivot logic: one side drives forward, opposite side brakes/holds
        if (volt > 0) {  // Positive voltage → pivot left (left motors forward)
            leftMotor[0].spin(forward, volt, voltageUnits::volt);
            rightMotor[0].stop(brake);
            leftMotor[1].spin(forward, volt, voltageUnits::volt);
            rightMotor[1].stop(hold);
            leftMotor[2].spin(forward, volt, voltageUnits::volt);
            rightMotor[2].stop(brake);
        }
        else {           // Negative voltage → pivot right (right motors forward)
            leftMotor[0].stop(brake);
            rightMotor[0].spin(forward, std::fabs(volt), voltageUnits::volt);
            leftMotor[1].stop(hold);
            rightMotor[1].spin(forward, std::fabs(volt), voltageUnits::volt);
            leftMotor[2].stop(brake);
            rightMotor[2].spin(forward, std::fabs(volt), voltageUnits::volt);
        }

        vex::task::sleep(10);
    }

    // Final stop with brake (original behavior)
    for (int i = 0; i < 3; i++) {
        leftMotor[i].stop(brake);
        rightMotor[i].stop(brake);
    }
}

// ======================================================================
// V2: Modernized pivot turn to target heading (continuous Standard Cartesian)
// One side brakes/stops, opposite side drives (true pivot)
// Uses motion profiling phases similar to turnOdometry
// Final stop uses brake mode
// ======================================================================
void pivotTurnOdometryV2(double targetHeading, double breakDistanceInDegrees, 
                         double minSpeed, double maxSpeed, double exitTolerance) 
{
    // ───────────────────────────────────────────────
    // SNAP TARGET TO NEAREST CONTINUOUS EQUIVALENT
    // Makes V2 robust if caller passes wrapped angle (e.g. 90° when at 1230°)
    // ───────────────────────────────────────────────
    double currentHeadingInitial = getContinuousStandardHeading();
    double rotationsDiff = std::round((currentHeadingInitial - targetHeading) / 360.0);
    targetHeading += rotationsDiff * 360.0;

    bool decelCompleted = false;
    bool accelCompleted = false;
    bool decel = false;

    // Voltage setup
    // Calculate initial error to determine direction
    double startError = targetHeading - currentHeadingInitial;
    double maxSpeedVoltage = std::copysign(maxSpeed * 0.01 * absoluteMaxVoltage, startError);
    double minSpeedVoltage = std::copysign(minSpeed * 0.01 * absoluteMaxVoltage, startError);
    double launchVoltage   = std::copysign(5.0, maxSpeedVoltage);  // gentle launch kick

    double minDriveMotorRPM = (minSpeed * 0.01) * absoluteMaxRPM;

    // Motor voltage arrays (one side will be 0 during pivot)
    double motorVoltageLeft[3]  = {0, 0, 0};
    double motorVoltageRight[3] = {0, 0, 0};

    // Main loop — continue until within tolerance
    while (true) 
    {
        double currentHeading = getContinuousStandardHeading();
        double headingError   = targetHeading - currentHeading;

        // Exit condition
        if (std::fabs(headingError) <= exitTolerance) {
            break;
        }

        // ───────────────────────────────────────────────
        // LAUNCH / ACCEL PHASE
        // ───────────────────────────────────────────────
        if (std::fabs(headingError) > breakDistanceInDegrees && !accelCompleted && !decel) 
        {
            double targetVolt = std::fabs(maxSpeedVoltage);
            
            // Get current highest voltage being applied
            double currentMax = std::max(std::fabs(motorVoltageLeft[1]), std::fabs(motorVoltageRight[1]));
            
            if (currentMax < std::fabs(launchVoltage)) {
                currentMax = std::fabs(launchVoltage);
            } else {
                currentMax = std::min(targetVolt, currentMax + 0.5);  // smooth ramp up
            }

            double pivotVolt = std::copysign(currentMax, headingError);

            // CORRECTION: 
            // To turn LEFT (Positive), Right motors must drive.
            // To turn RIGHT (Negative), Left motors must drive.
            if (pivotVolt > 0) {  // Pivot LEFT (CCW)
                for (int i = 0; i < 3; i++) {
                    motorVoltageLeft[i]  = 0;          // Pivot Point
                    motorVoltageRight[i] = pivotVolt;  // Driver
                }
            } else {              // Pivot RIGHT (CW)
                for (int i = 0; i < 3; i++) {
                    motorVoltageLeft[i]  = std::fabs(pivotVolt); // Driver
                    motorVoltageRight[i] = 0;                    // Pivot Point
                }
            }

            if (currentMax >= std::fabs(maxSpeedVoltage) - VOLTAGE_TOLERANCE) {
                accelCompleted = true;
            }
        }
        // ───────────────────────────────────────────────
        // CRUISE PHASE (full speed pivot)
        // ───────────────────────────────────────────────
        else if (std::fabs(headingError) > breakDistanceInDegrees && accelCompleted) 
        {
            double pivotVolt = maxSpeedVoltage;

            if (pivotVolt > 0) { // Pivot LEFT (CCW)
                for (int i = 0; i < 3; i++) {
                    motorVoltageLeft[i]  = 0;
                    motorVoltageRight[i] = pivotVolt;
                }
            } else {             // Pivot RIGHT (CW)
                for (int i = 0; i < 3; i++) {
                    motorVoltageLeft[i]  = std::fabs(pivotVolt);
                    motorVoltageRight[i] = 0;
                }
            }
        }
        // ───────────────────────────────────────────────
        // DECELERATION PHASE
        // ───────────────────────────────────────────────
        else if (std::fabs(headingError) <= breakDistanceInDegrees && !decelCompleted) 
        {
            if (!decel) decel = true;

            double currentMax = std::max(std::fabs(motorVoltageLeft[1]), std::fabs(motorVoltageRight[1]));
            double syncedDecel = std::max(0.0, currentMax - 0.4);  // adjustable decel step

            double pivotVolt = std::copysign(syncedDecel, headingError);

            if (pivotVolt > 0) { // Pivot LEFT (CCW)
                for (int i = 0; i < 3; i++) {
                    motorVoltageLeft[i]  = 0;
                    motorVoltageRight[i] = pivotVolt;
                }
            } else {             // Pivot RIGHT (CW)
                for (int i = 0; i < 3; i++) {
                    motorVoltageLeft[i]  = std::fabs(pivotVolt);
                    motorVoltageRight[i] = 0;
                }
            }

            if (syncedDecel <= std::fabs(minSpeedVoltage)) {
                decelCompleted = true;
            }
        }
        // ───────────────────────────────────────────────
        // APPROACH / FINAL SETTLING PHASE
        // ───────────────────────────────────────────────
        else if (decelCompleted) 
        {
            double pivotVolt = minSpeedVoltage;

            if (pivotVolt > 0) { // Pivot LEFT (CCW)
                for (int i = 0; i < 3; i++) {
                    motorVoltageLeft[i]  = 0;
                    motorVoltageRight[i] = pivotVolt;
                }
            } else {             // Pivot RIGHT (CW)
                for (int i = 0; i < 3; i++) {
                    motorVoltageLeft[i]  = std::fabs(pivotVolt);
                    motorVoltageRight[i] = 0;
                }
            }
        }

        // ───────────────────────────────────────────────
        // APPLY MOTOR COMMANDS (true pivot — one side always stopped)
        // ───────────────────────────────────────────────
        for (int i = 0; i < 3; i++) 
        {
            if (motorVoltageLeft[i] != 0) {
                leftMotor[i].spin(forward, motorVoltageLeft[i], voltageUnits::volt);
            } else {
                leftMotor[i].stop(brake); // Hard brake on pivot point
            }

            if (motorVoltageRight[i] != 0) {
                rightMotor[i].spin(forward, motorVoltageRight[i], voltageUnits::volt);
            } else {
                rightMotor[i].stop(brake); // Hard brake on pivot point
            }
        }

        vex::task::sleep(10);
    }

    // Final stop using brake mode (as requested)
    for (int i = 0; i < 3; i++) 
    {
        leftMotor[i].stop(brake);
        rightMotor[i].stop(brake);
    }
}

// ======================================================================
// Pivot left/right with motion profiling
// Turn amount is relative in degrees (positive for left/CCW)
// Converts to absolute target in continuous Standard Cartesian
// ======================================================================
void pivotLeftMP(double turnAmount, double breakDistance, double minSpeed, double maxSpeed) {
    double currentHeading = getContinuousStandardHeading();
    double targetRotation = currentHeading + turnAmount;
    pivotTurnOdometry(targetRotation, breakDistance, minSpeed, maxSpeed);
}

void pivotRightMP(double turnAmount, double breakDistance, double minSpeed, double maxSpeed) {
    double currentHeading = getContinuousStandardHeading();
    double targetRotation = currentHeading - turnAmount;
    pivotTurnOdometry(targetRotation, breakDistance, minSpeed, maxSpeed);
}

void driveForward(double targetDistance,
                  double breakDistance,
                  double targetHeading,
                  double minSpeed,
                  double kp_heading,
                  double ki_heading,
                  double kd_heading,
                  double accelHeadingScaling,
                  double decelHeadingScaling,
                  double approachHeadingScaling,
                  double maxSpeed)
{
    // No conversion needed - both use Modified Cartesian

    straightOdometryV3(targetDistance, breakDistance, targetHeading, minSpeed,
                     kp_heading, ki_heading, kd_heading,
                     accelHeadingScaling, decelHeadingScaling,
                     approachHeadingScaling, maxSpeed);
}

void driveBackward(double targetDistance,
                   double breakDistance,
                   double targetHeading,
                   double minSpeed,
                   double kp_heading,
                   double ki_heading,
                   double kd_heading,
                   double accelHeadingScaling,
                   double decelHeadingScaling,
                   double approachHeadingScaling,
                   double maxSpeed)
{
    targetDistance = -std::fabs(targetDistance);

    straightOdometryV3(targetDistance, breakDistance, targetHeading, minSpeed,
                     kp_heading, ki_heading, kd_heading,
                     accelHeadingScaling, decelHeadingScaling,
                     approachHeadingScaling, maxSpeed);
}


/**
 * Drive forward with precise heading control
 * 
 * Moves the robot forward using motion profiling (acceleration, cruise, deceleration,
 * and final approach phases) while maintaining a target heading. Accepts headings in
 * Modified Cartesian (North=0°) and converts them to Standard Cartesian for internal
 * calculations.
 * 
 * The distanceTolerance parameter allows tuning stopping precision per movement:
 * smaller values (3cm) for precise alignment, larger values (8cm) for speed.
 * 
 * @param targetDistance Distance to travel forward in cm (positive values only)
 * @param breakDistance Distance before target to begin deceleration (cm)
 * @param targetHeading Desired heading in Modified Cartesian (North=0°, 0-360°)
 * @param minSpeed Minimum motor speed during final approach (0-100%)
 * @param distanceTolerance How close to target before stopping (cm, smaller = more precise)
 * @param kp_heading Proportional gain for heading PID correction
 * @param ki_heading Integral gain for heading PID correction
 * @param kd_heading Derivative gain for heading PID correction
 * @param accelHeadingScaling Scales heading correction during acceleration (0-1)
 * @param decelHeadingScaling Scales heading correction during deceleration (0-1)
 * @param approachHeadingScaling Scales heading correction during final approach (0-1)
 * @param maxSpeed Maximum motor speed during cruise phase (0-100%)
 */
void driveForwardV3(double targetDistance, double breakDistance, double targetHeading, 
                  double minSpeed, double distanceTolerance,
                  double kp_heading, double ki_heading, double kd_heading, 
                  double accelHeadingScaling, double decelHeadingScaling, 
                  double approachHeadingScaling, double maxSpeed) {
    // Convert heading from Modified Cartesian (North=0°) to Standard Cartesian (East=0°)
                straightOdometryV3(targetDistance, breakDistance, targetHeading, 
                      minSpeed, distanceTolerance, kp_heading, ki_heading, kd_heading, 
                      accelHeadingScaling, decelHeadingScaling, approachHeadingScaling, maxSpeed);
}

/**
 * Drive backward with precise heading control
 * 
 * Identical to driveForward() but moves backward - forces distance negative.
 * See driveForward() for detailed parameter documentation.
 */
void driveBackwardV3(double targetDistance, double breakDistance, double targetHeading_modified, 
                   double minSpeed, double distanceTolerance,
                   double kp_heading, double ki_heading, double kd_heading, 
                   double accelHeadingScaling, double decelHeadingScaling, 
                   double approachHeadingScaling, double maxSpeed) {
    // Target heading is already in Standard Cartesian (East=0°, CCW+)
    // Force distance negative for backward movement
    straightOdometryV3(-std::fabs(targetDistance), breakDistance, targetHeading_modified,  // ← Use correct parameter name
                      minSpeed, distanceTolerance, kp_heading, ki_heading, kd_heading, 
                      accelHeadingScaling, decelHeadingScaling, approachHeadingScaling, maxSpeed);
}

/**
 * Turn right (clockwise) to an absolute field heading
 *
 * Forces a RIGHT turn by ensuring the target heading is HIGHER than the current
 * continuous heading. The function accepts headings in Modified Cartesian (North=0°)
 * and converts them to Standard Cartesian for internal calculations.
 *
 * The direction forcing loop adds 360° to the target until target > current,
 * creating positive heading error that drives clockwise motor rotation.
 *
 * @param absolutetargetHeading Desired heading in Modified Cartesian (North=0°, 0-360°)
 * @param breakDistance Distance before target to begin deceleration (degrees)
 * @param minSpeed Minimum motor speed during approach phase (0-100%)
 * @param maxSpeed Maximum motor speed during turn (0-100%)
 * @param exitTolerance Acceptable error to exit turn early (degrees)
 */
void turnRight(double absoluteTargetHeading_modified, double breakDistance, double minSpeed, double maxSpeed, double exitTolerance) {
    // Convert from Modified Cartesian (North=0°) to Standard Cartesian (East=0°)
    double target = absoluteTargetHeading_modified;
    
    // Get current continuous heading
    double current = getContinuousStandardHeading();
    
    // FORCE clockwise: subtract 360° until target < current (creates negative error)
    while (target >= current + 0.5) {
        target -= 360.0;
    }
    
    turnOdometry(target, breakDistance, minSpeed, maxSpeed, exitTolerance);
}

/**
 * Turn left (counter-clockwise) to an absolute field heading
 * 
 * Mirror function of turnRight() - forces CCW rotation by making target > current.
 * See turnRight() documentation for detailed explanation of the algorithm.
 * 
 * @param absoluteTargetHeading_modified Desired heading in Modified Cartesian (North=0°, 0-360°)
 * @param breakDistance Distance before target to begin deceleration (degrees)
 * @param minSpeed Minimum motor speed during approach phase (0-100%)
 * @param maxSpeed Maximum motor speed during turn (0-100%)
 * @param exitTolerance Acceptable error to exit turn early (degrees)
 */
void turnLeft(double absoluteTargetHeading_modified, double breakDistance, double minSpeed, double maxSpeed, double exitTolerance) {
    // Convert from Modified Cartesian to Standard Cartesian
     double target = absoluteTargetHeading_modified;
    
    // Get current continuous heading
    double current = getContinuousStandardHeading();
    
    // FORCE counter-clockwise: add 360° until target > current (creates positive error)
    while (target <= current - 0.5) { // small tolerance to avoid infinite loop on equality
        target += 360.0;
    }
    
    turnOdometry(target, breakDistance, minSpeed, maxSpeed, exitTolerance);
}
//=============================================================================
// ABSOLUTE HEADING WRAPPER FUNCTIONS - FINAL VERSION
// Add these functions to your navigation.cpp file
//=============================================================================

// Forward wrapper - uses absolute heading with forward as 0Â°
// Forward wrapper - uses absolute heading with forward as 0Â°
void driveForwardV1(double targetDistance,
                  double breakDistance,
                  double targetHeading,
                  double minSpeed,
                  double kp_heading,
                  double ki_heading,
                  double kd_heading,
                  double accelHeadingScaling,
                  double decelHeadingScaling,
                  double approachHeadingScaling,
                  double maxSpeed)
{
    // No conversion needed - both use Modified Cartesian

    straightOdometry(targetDistance, breakDistance, targetHeading, minSpeed,
                     kp_heading, ki_heading, kd_heading,
                     accelHeadingScaling, decelHeadingScaling,
                     approachHeadingScaling, maxSpeed);
}

void driveBackwardV1(double targetDistance,
                   double breakDistance,
                   double targetHeading,
                   double minSpeed,
                   double kp_heading,
                   double ki_heading,
                   double kd_heading,
                   double accelHeadingScaling,
                   double decelHeadingScaling,
                   double approachHeadingScaling,
                   double maxSpeed)
{
    targetDistance = -std::fabs(targetDistance);

    straightOdometry(targetDistance, breakDistance, targetHeading, minSpeed,
                     kp_heading, ki_heading, kd_heading,
                     accelHeadingScaling, decelHeadingScaling,
                     approachHeadingScaling, maxSpeed);
}

void pidlessForward(double timeMs, double speedPct) {
    vex::timer forwardTime; forwardTime.reset();
    double voltagePower = (speedPct / 8.34);
    while (forwardTime.time(timeUnits::msec) < timeMs) {
        for(int i=0; i<3; i++) { leftMotor[i].spin(forward, voltagePower, volt); rightMotor[i].spin(forward, voltagePower, volt); }
        vex::task::sleep(10);
    }
    for(int i=0; i<3; i++) { leftMotor[i].stop(coast); rightMotor[i].stop(coast); }
}

void driveForwardV2(double targetDistance,
                  double breakDistance,
                  double targetHeading,
                  double minSpeed,
                  double distanceTolerance,
                  double kp_heading,
                  double ki_heading,
                  double kd_heading,
                  double accelHeadingScaling,
                  double decelHeadingScaling,
                  double approachHeadingScaling,
                  double maxSpeed)
{
    double internalHeading = -targetHeading;

    straightOdometryV2(targetDistance, breakDistance, targetHeading, minSpeed,
                     distanceTolerance, kp_heading, ki_heading, kd_heading,
                     accelHeadingScaling, decelHeadingScaling,
                     approachHeadingScaling, maxSpeed);
}

void driveBackwardV2(double targetDistance,
                   double breakDistance,
                   double targetHeading,
                   double minSpeed,
                   double distanceTolerance,
                   double kp_heading,
                   double ki_heading,
                   double kd_heading,
                   double accelHeadingScaling,
                   double decelHeadingScaling,
                   double approachHeadingScaling,
                   double maxSpeed)
{
    targetDistance = -std::fabs(targetDistance);

    straightOdometryV2(targetDistance, breakDistance, targetHeading, minSpeed,
                     distanceTolerance, kp_heading, ki_heading, kd_heading,
                     accelHeadingScaling, decelHeadingScaling,
                     approachHeadingScaling, maxSpeed);
}

void visionDrive(vex::aivision::colordesc targetSignature, int targetPixelWidth, double timeoutDistanceCM, double targetHeading, double minSpeedPct, double maxSpeedPct, vex::brakeType brakeMode, double kp_head, double ki_head, double kd_head, double kp_dist, double ki_dist, double kd_dist, int minX, int maxX, int minY, int maxY, int maxObjectsToCheck, int consecutiveRequired) {
    double startDist = getCurrentEncoderDistanceCM();
    
    PID headingPID(kp_head, ki_head, kd_head), distPID(kp_dist, ki_dist, kd_dist);
    headingPID.pidReset(); distPID.pidReset();
    int consecutiveStableWidth = 0;

    while (true) {
        double distanceTravelled = getCurrentEncoderDistanceCM() - startDist;
        if (fabs(distanceTravelled) >= timeoutDistanceCM) break;
        
        AIVision20.takeSnapshot(targetSignature);
        int bestIdx = -1; double largestWidth = 0;
        for (int i = 0; i < std::min(maxObjectsToCheck, (int)AIVision20.objectCount); i++) {
            auto& obj = AIVision20.objects[i];
            if (obj.width >= 24 && obj.centerX >= minX && obj.centerX <= maxX && obj.centerY >= minY && obj.centerY <= maxY && obj.width > largestWidth) { largestWidth = obj.width; bestIdx = i; }
        }
        double turnCorrection = (bestIdx >= 0) ? headingPID.calculate(0.0, AIVision20.objects[bestIdx].centerX - VISION_CENTER_X) : headingPID.calculate(targetHeading, getContinuousStandardHeading());
        double currentWidth = (bestIdx >= 0) ? AIVision20.objects[bestIdx].width : 0;
        if (currentWidth >= targetPixelWidth) { if (++consecutiveStableWidth >= consecutiveRequired) break; } else consecutiveStableWidth = 0;
        double driveV = std::max(minSpeedPct * 0.12, std::min(maxSpeedPct * 0.12, distPID.calculate((double)targetPixelWidth, currentWidth) * 0.12));
        double turnV = std::max(-4.0, std::min(4.0, turnCorrection * 0.12));
        for (int i = 0; i < 3; i++) { leftMotor[i].spin(forward, driveV + turnV, volt); rightMotor[i].spin(forward, driveV - turnV, volt); }
        vex::task::sleep(20);
    }
    for (int i = 0; i < 3; i++) { leftMotor[i].stop(brakeMode); rightMotor[i].stop(brakeMode); }
}

/**
 * visionDriveMinimal - Vision-guided approach function
 *
 * Drives the robot toward a detected object using two PID controllers:
 *   - Heading PID:  Steers left/right to keep the object centered in the camera frame
 *   - Distance PID: Controls forward speed based on the object's apparent pixel width
 *
 * Distance-to-Heading Scaling adjusts steering sensitivity based on distance —
 * stronger corrections when far away, gentler corrections when close to prevent overshooting.
 *
 * Accepts colordesc (single color) or codedesc (color combination) descriptors.
 * Both public overloads share the same internal logic via a static template function.
 *
 * @param targetSignature      AI Vision descriptor — colordesc or codedesc
 * @param targetPixelWidth     Pixel width that means "close enough" (larger = closer)
 * @param targetHeading        Heading to snap to on startup (prevents wrap-around jumps)
 * @param minSpeedPct          Minimum forward speed percentage (0-100)
 * @param maxSpeedPct          Maximum forward speed percentage (0-100)
 * @param brakeMode            How motors stop: brake / coast / hold
 * @param kp_head              Heading PID proportional gain
 * @param ki_head              Heading PID integral gain
 * @param kd_head              Heading PID derivative gain
 * @param kp_distToHeadScaling Scales heading correction strength by distance (P-only)
 * @param kp_dist              Distance PID proportional gain
 * @param ki_dist              Distance PID integral gain
 * @param kd_dist              Distance PID derivative gain
 */

// ─── INTERNAL SHARED IMPLEMENTATION ───────────────────────────────────────
// Static template — only visible inside navigation.cpp, not callable externally.
// The template parameter T accepts any descriptor type that takeSnapshot() supports.
// The two public overloads below forward their descriptor directly into this function.
template <typename T>
static void visionDriveMinimal_impl(
    T      targetSignature,
    int    targetPixelWidth,
    double targetHeading,
    double minSpeedPct,
    double maxSpeedPct,
    vex::brakeType brakeMode,
    double kp_head, double ki_head, double kd_head,
    double kp_distToHeadScaling,
    double kp_dist, double ki_dist, double kd_dist)
{
    // ========================================
    // INITIALIZATION
    // ========================================

    // Maximum steering correction as a percentage of full motor power (0-100)
    double maxSteeringPct = 25.0;  // SET THIS VALUE HERE

    // Create PID controllers — one centers the object, one controls approach speed
    PID headingPID(kp_head, ki_head, kd_head);   // Controls left/right steering
    PID distancePID(kp_dist, ki_dist, kd_dist);  // Controls forward drive speed

    // Reset accumulators so old error doesn't carry into this run
    headingPID.pidReset();
    distancePID.pidReset();

    // Convert steering cap from percentage to volts (12V system: 1% = 0.12V)
    double maxSteeringVoltage = maxSteeringPct * 0.12;

    // Snap target heading to the nearest continuous equivalent
    // Prevents the heading from jumping across the 0/360 boundary mid-run
    double currentHeadingInitial = getContinuousStandardHeading();
    double rotationsDiff = std::round((currentHeadingInitial - targetHeading) / 360.0);
    targetHeading += rotationsDiff * 360.0;

    // ========================================
    // MAIN VISION TRACKING LOOP
    // ========================================

    while (true) {
        // Pass the full descriptor object — takeSnapshot is overloaded for each type
        AIVision20.takeSnapshot(targetSignature);

        // If the target disappears, stop immediately rather than driving blind
        if (AIVision20.objectCount == 0) {
            break;
        }

        // Use the highest-confidence detected object (index 0)
        auto& detectedObject = AIVision20.objects[0];

        // ========================================
        // CALCULATE DISTANCE-BASED HEADING SCALING
        // ========================================

        // How many pixels short of the target width are we? (larger value = farther away)
        double distanceErrorPixels = (double)targetPixelWidth - (double)detectedObject.width;

        // Scale steering strength by distance — farther away gets stronger corrections
        double headingScalingFactor = 1.0 + (kp_distToHeadScaling * distanceErrorPixels);

        // Clamp to a safe range: never negative, never more than 3x base correction
        headingScalingFactor = std::max(0.1, std::min(3.0, headingScalingFactor));

        // ========================================
        // CALCULATE STEERING CORRECTION
        // ========================================

        // Pixel offset from frame center — positive = object is right of center
        double pixelOffsetFromCenter = detectedObject.centerX - VISION_CENTER_X;

        // PID output converted from percentage to volts
        double baseHeadingCorrectionVoltage = headingPID.calculate(0.0, pixelOffsetFromCenter) * 0.12;

        // Apply distance scaling so far objects get steered toward more aggressively
        double steeringCorrectionVoltage = baseHeadingCorrectionVoltage * headingScalingFactor;

        // Clamp steering so it never overwhelms forward drive power
        steeringCorrectionVoltage = std::max(-maxSteeringVoltage, std::min(maxSteeringVoltage, steeringCorrectionVoltage));

        // ========================================
        // CALCULATE FORWARD DRIVE SPEED
        // ========================================

        // PID error = target width minus current width; larger gap = faster approach
        double baseDriveVoltage = distancePID.calculate((double)targetPixelWidth, (double)detectedObject.width) * 0.12;

        // Convert speed limits from percentage to volts and clamp drive output
        double minDriveVoltage     = minSpeedPct * 0.12;
        double maxDriveVoltage     = maxSpeedPct * 0.12;
        double clampedDriveVoltage = std::max(minDriveVoltage, std::min(maxDriveVoltage, baseDriveVoltage));

        // ========================================
        // APPLY MOTOR COMMANDS
        // ========================================

        // Differential drive: adding/subtracting steering from each side turns the robot
        // Left side slower = turn left; right side slower = turn right
        for (int i = 0; i < 3; i++) {
            leftMotor[i].spin(forward,  clampedDriveVoltage - steeringCorrectionVoltage, volt);
            rightMotor[i].spin(forward, clampedDriveVoltage + steeringCorrectionVoltage, volt);
        }

        // ========================================
        // EXIT CONDITION
        // ========================================

        // Object has reached target width — robot is close enough, stop the loop
        if (detectedObject.width >= targetPixelWidth) {
            break;
        }

        // 20ms delay = 50Hz control loop, matches the AI Vision sensor update rate
        vex::task::sleep(20);
    }

    // ========================================
    // CLEANUP — STOP ALL MOTORS
    // ========================================

    for (int i = 0; i < 3; i++) {
        leftMotor[i].stop(brakeMode);
        rightMotor[i].stop(brakeMode);
    }
}

// ─── PUBLIC OVERLOAD: colordesc (single color object) ─────────────────────
// Forwards the descriptor directly to the shared template implementation above.
void visionDriveMinimal(
    vex::aivision::colordesc targetSignature,
    int targetPixelWidth, double targetHeading,
    double minSpeedPct, double maxSpeedPct, vex::brakeType brakeMode,
    double kp_head, double ki_head, double kd_head,
    double kp_distToHeadScaling,
    double kp_dist, double ki_dist, double kd_dist)
{
    visionDriveMinimal_impl(targetSignature, targetPixelWidth, targetHeading,
        minSpeedPct, maxSpeedPct, brakeMode,
        kp_head, ki_head, kd_head, kp_distToHeadScaling,
        kp_dist, ki_dist, kd_dist);
}

// ─── PUBLIC OVERLOAD: codedesc (color combination object) ─────────────────
// Forwards the descriptor directly to the shared template implementation above.
void visionDriveMinimal(
    vex::aivision::codedesc targetSignature,
    int targetPixelWidth, double targetHeading,
    double minSpeedPct, double maxSpeedPct, vex::brakeType brakeMode,
    double kp_head, double ki_head, double kd_head,
    double kp_distToHeadScaling,
    double kp_dist, double ki_dist, double kd_dist)
{
    visionDriveMinimal_impl(targetSignature, targetPixelWidth, targetHeading,
        minSpeedPct, maxSpeedPct, brakeMode,
        kp_head, ki_head, kd_head, kp_distToHeadScaling,
        kp_dist, ki_dist, kd_dist);
}

/**
 * visionDriveV2 - Advanced AI Vision tracking with Priority Scaling
 * * Drives the robot toward a detected signature while centering it.
 * Uses a Priority Scaler to ensure steering is preserved at max power 
 * and a grace period to handle momentary sensor drops.
 */
void visionDriveV2(
    vex::aivision::colordesc targetSignature,
    int targetPixelWidth,
    double targetHeading,
    vex::brakeType brakeMode,
    double maxSpeedPct,
    double kp_head,
    double ki_head,
    double kd_head,
    double kp_distToHeadScaling,
    int minObjectWidth,
    int minX,
    int maxX,
    int minY,
    int maxY,
    double minSpeedPct,
    double timeoutDistanceCM,
    double kp_dist,
    double ki_dist,
    double kd_dist
) {
    // ========================================
    // CONFIGURATION & PID SETUP
    // ========================================
    double maxSteeringPct = 25.0;  // Max steering authority (0-100)
    const int MAX_OBJECTS_TO_CHECK = 3;
    const int MAX_LOST_FRAMES = 15;
    
    PID headingPID(kp_head, ki_head, kd_head);
    PID distancePID(kp_dist, ki_dist, kd_dist);
    headingPID.pidReset();
    distancePID.pidReset();
    
    double maxSteeringVoltage = maxSteeringPct * 0.12;
    
    // State Tracking
    bool hasDetectedBefore = false;
    double lastNormalizedOffset = 0.0;
    int lastDetectedWidth = 0;
    int lostFrameCounter = 0;

    // ========================================
    // MAIN VISION TRACKING LOOP
    // ========================================
    while (true) {
        AIVision20.takeSnapshot(targetSignature);
        
        int bestObjectIndex = -1;
        int largestWidth = 0;
        int objectsToCheck = std::min((int)AIVision20.objectCount, MAX_OBJECTS_TO_CHECK);
        
        // --- SECTION 1: FILTERING ---
        for (int i = 0; i < objectsToCheck; i++) {
            auto& obj = AIVision20.objects[i];
            int bottomY = obj.centerY + (obj.height / 2);
            
            if (obj.width < minObjectWidth) continue;
            if (bottomY < minY || bottomY > maxY) continue;
            if (obj.centerX < minX || obj.centerX > maxX) continue;
            
            if (obj.width > largestWidth) {
                largestWidth = obj.width;
                bestObjectIndex = i;
            }
        }
        
        double currentNormalizedOffset;
        int currentWidth;
        
        // --- SECTION 2: DETECTION & FALLBACK ---
        if (bestObjectIndex == -1) {
            lostFrameCounter++;
            
            // Give up after too many frames without detection
            if (lostFrameCounter > MAX_LOST_FRAMES) break;
            
            // Use fallback memory (only useful if we've detected before)
            if (hasDetectedBefore) {
                currentNormalizedOffset = lastNormalizedOffset;
                currentWidth = lastDetectedWidth;
            } else {
                // No detection yet and no memory — wait for next frame
                vex::task::sleep(20);
                continue;
            }
        } else {
            lostFrameCounter = 0;  // Reset counter when we see the target
            auto& detectedObject = AIVision20.objects[bestObjectIndex];
            
            // Normalize offset: -1.0 (left edge) to 1.0 (right edge)
            currentNormalizedOffset = (detectedObject.centerX - VISION_CENTER_X) / VISION_CENTER_X;
            currentWidth = detectedObject.width;
            
            // Update fallback memory
            lastNormalizedOffset = currentNormalizedOffset;
            lastDetectedWidth = currentWidth;
            hasDetectedBefore = true;
            
            // Success Exit
            if (currentWidth >= targetPixelWidth) break;
        }
        
        // --- SECTION 3: CALCULATIONS ---
        
        // Distance-based Heading Scaling: 0.2 floor prevents steering loss at target
        double distanceErrorPixels = (double)targetPixelWidth - (double)currentWidth;
        double headingScalingFactor = 0.2 + (kp_distToHeadScaling * distanceErrorPixels);
        headingScalingFactor = std::max(0.2, std::min(3.0, headingScalingFactor));
        
        // Base correction from Normalized PID (error is -1.0 to 1.0)
        double steeringCorrection = (headingPID.calculate(0.0, currentNormalizedOffset) * 12.0) * headingScalingFactor;
        steeringCorrection = std::max(-maxSteeringVoltage, std::min(maxSteeringVoltage, steeringCorrection));
        
        // Drive speed calculation
        double baseDriveVoltage = distancePID.calculate((double)targetPixelWidth, (double)currentWidth) * 0.12;
        double clampedDrive = std::max(minSpeedPct * 0.12, std::min(maxSpeedPct * 0.12, baseDriveVoltage));
        
        // --- SECTION 4: PRIORITY SCALER (Symmetry Fix) ---
        double leftRequest = clampedDrive - steeringCorrection;
        double rightRequest = clampedDrive + steeringCorrection;
        
        // Calculate the maximum magnitude requested across both motors
        double maxRequest = std::max(std::fabs(leftRequest), std::fabs(rightRequest));
        
        // If exceeding 12V, scale BOTH sides down to preserve the turn ratio
        if (maxRequest > 12.0) {
            double scaleFactor = 12.0 / maxRequest;
            leftRequest *= scaleFactor;
            rightRequest *= scaleFactor;
        }
        
        // --- SECTION 5: MOTOR OUTPUT ---
        for (int i = 0; i < 3; i++) {
            leftMotor[i].spin(forward, leftRequest, volt);
            rightMotor[i].spin(forward, rightRequest, volt);
        }
        
        vex::task::sleep(20);
    }
    
    // --- CLEANUP ---
    for (int i = 0; i < 3; i++) {
        leftMotor[i].stop(brakeMode);
        rightMotor[i].stop(brakeMode);
    }
}


/**
 * moveVisionOdometry - Advanced vision-fused autonomous movement
 * 
 * Drives the robot to a global (X, Y) coordinate while using the AI Vision sensor
 * to center a target object. It uses the Gyro as the "Truth" for heading fusion.
 * 
 * UPDATED: Now synchronized with straightOdometryV3 for all odometry movement logic
 * - Independent per-side traction control during acceleration
 * - Selective release steering during deceleration
 * - Rolling average exit detection
 * - Updated configuration constants to match latest values
 * 
 * @param targetX Global X coordinate destination (cm)
 * @param targetY Global Y coordinate destination (cm)
 * @param breakDistance Distance from target to begin deceleration (cm)
 * @param minSpeed Minimum speed percentage during approach (0-100)
 * @param distanceTolerance Radius of the stopping bubble (cm)
 * @param kp_heading Proportional gain for heading PID
 * @param ki_heading Integral gain for heading PID
 * @param kd_heading Derivative gain for heading PID
 * @param brakeMode Final motor stop behavior (brake/coast/hold)
 * @param accelHeadingScaling PID scaling during acceleration phase
 * @param decelHeadingScaling PID scaling during deceleration phase
 * @param approachHeadingScaling PID scaling during final approach phase
 * @param maxSpeed Maximum speed percentage during cruise (0-100)
 * @param targetSignature AI Vision color signature to track
 * @param kp_distanceToHeadingScaling Authority of Vision over Odometry (0.0 to 1.0)
 * @param minObjectWidth Minimum pixel width to consider a detection valid
 */
void moveVisionOdometry(vex::aivision::colordesc targetSignature,
                        int targetPixelWidth,
                        double targetX,
                        double targetY, 
                        double breakDistance,
                        vex::brakeType brakeMode,
                        double maxSpeed,
                        double kp_head,
                        double ki_head,
                        double kd_head,
                        double kp_distToHeadScaling,
                        int minObjectWidth,
                        int minX,
                        int maxX,
                        int minY,
                        int maxY,
                        double minSpeed,
                        double distanceTolerance,
                        double accelHeadingScaling,
                        double decelHeadingScaling,
                        double approachHeadingScaling,
                        double timeout)
{
    // ========================================
    // CONFIGURATION CONSTANTS
    // ========================================
    const double LAUNCH_VOLTAGE = 6.0;              // Initial acceleration voltage
    const double ACCEL_FACTOR_LAUNCH = 1.2;         // Voltage ramp rate during launch
    const double SLIP_THRESHOLD_TRACTION = 20.0;    // Wheel slip detection sensitivity
    const double DECEL_STEP_PERCENT = 0.45;         // ABS brake pressure reduction rate
    const double LOCK_THRESHOLD_DECEL = 0.25;       // Wheel lock detection threshold
    const int REQUIRED_CONSECUTIVE_STOPS = 3;       // Frames at target before exit (noise filter)

    // ========================================
    // INITIALIZATION
    // ========================================
    updateOdometry();
    double startCoordinateX = globalX;
    double startCoordinateY = globalY;

    // Calculate initial distance for direction scalar and voltage sign
    double pathVectorX = targetX - startCoordinateX;
    double pathVectorY = targetY - startCoordinateY;
    double initialDistance = sqrt(pathVectorX * pathVectorX + pathVectorY * pathVectorY);
    
    // Direction scalar: +1 forward, -1 backward
    double dir = (initialDistance >= 0) ? 1.0 : -1.0;

    // Initialize heading PID controller
    PID headingPID(kp_head, ki_head, kd_head);
    headingPID.pidReset();
    
    // Convert percentage speeds to voltage values
    double maxSpeedVoltage = std::copysign(maxSpeed * 0.01 * absoluteMaxVoltage, initialDistance);
    double minSpeedVoltage = std::copysign(minSpeed * 0.01 * absoluteMaxVoltage, initialDistance);
    double minLaunchSpeedVoltage = std::copysign(std::min(fabs(maxSpeedVoltage), fabs(LAUNCH_VOLTAGE)), initialDistance);
    
    // RPM threshold for detecting deceleration completion
    double minDriveMotorRPM = (minSpeed * 0.01) * absoluteMaxRPM;
        
    // Motor voltage arrays (left/right × 3 motors per side)
    double motorVoltageLeft[3] = {minLaunchSpeedVoltage, minLaunchSpeedVoltage, minLaunchSpeedVoltage};
    double motorVoltageRight[3] = {minLaunchSpeedVoltage, minLaunchSpeedVoltage, minLaunchSpeedVoltage};
    
    // Motion phase flags
    bool decel = false;
    bool decelCompleted = false;
    bool accelCompleted = false;
    
    // Rolling averages for stable deceleration exit detection
    double leftEncoderRollingAverage = 0;
    double rightEncoderRollingAverage = 0;
    
    // Exit condition tracking
    int consecutiveAtTargetCount = 0;

    // Vision-specific state tracking
    // lastVisionHorizontalOffset: stores the most recent valid horizontal offset (-1.0 to +1.0)
    //   so that if vision drops mid-run, the last known correction is preserved
    // visionEverTracked: becomes true the first time a valid detection is received
    // visionCurrentlyTracked: true only when vision saw a valid object THIS tick
    // lastFusedHeading: the most recent heading calculated while vision was active —
    //   used to compute a corrected target XY if vision drops out
    // distanceTraveled: accumulated odometry distance since function start —
    //   used to calculate remaining distance to target if vision drops
    double lastVisionHorizontalOffset = 0.0;
    bool visionEverTracked = false;
    bool visionCurrentlyTracked = false;
    bool visionDropoutHandled = false;  // Ensures dropout recovery only fires once — prevents
                                        // targetX/targetY from being recalculated every tick
                                        // while vision is down, which would keep shifting the target
    double lastFusedHeading = 0.0;
    double distanceTraveled = 0.0;
    double startX = globalX;
    double startY = globalY;
    int lastSnapshotCenterX = -999;
    int lastSnapshotWidth = -999;

    // Initialize traction and ABS control systems
    tractionControl tractionControlLeft(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION);
    tractionControl tractionControlRight(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION);
    adaptiveABS adaptiveABSLeft(DECEL_STEP_PERCENT, LOCK_THRESHOLD_DECEL);
    adaptiveABS adaptiveABSRight(DECEL_STEP_PERCENT, LOCK_THRESHOLD_DECEL);

    // Timeout safety timer
    vex::timer safetyTimer;
    safetyTimer.reset();
    double timeoutMs = timeout * 1000.0;

    // ═══════════════════════════════════════════════════════════════════
    // MAIN CONTROL LOOP
    // ═══════════════════════════════════════════════════════════════════
    while (true) 
    {
        updateOdometry();

        // ───────────────────────────────────────────────────────────────
        // 1. STATE CALCULATION
        // ───────────────────────────────────────────────────────────────
        double currentDistanceToTarget, odometryTargetHeading;
        calculatePathToTarget(globalX, globalY, targetX, targetY, currentDistanceToTarget, odometryTargetHeading);
        double currentGyroHeading = getContinuousStandardHeading();

        // ───────────────────────────────────────────────────────────────
        // 2. TIMEOUT SAFETY CHECK
        // ───────────────────────────────────────────────────────────────
        if (safetyTimer.time(vex::msec) > timeoutMs) {
            break; // Prevent infinite loop on sensor failure or unreachable target
        }

        // ───────────────────────────────────────────────────────────────
        // 3. EXIT CONDITIONS
        // ───────────────────────────────────────────────────────────────
        // Two exit conditions — whichever fires first wins:
        //
        // A. Vision pixel width exit (Section 4 below):
        //    Fires immediately when object width >= targetPixelWidth.
        //    This is the preferred exit — vision is ground truth for object proximity.
        //    Handles cases where the object is closer OR farther than the odometry coordinate.
        //
        // B. Odometry distance tolerance exit (below):
        //    Fires when the robot reaches the target XY coordinate within distanceTolerance.
        //    This is the fallback — handles cases where vision never acquired the object
        //    or dropped out before the pixel width threshold was reached.
        //    targetX/targetY may be updated mid-run if vision drops after having corrected
        //    the path (see Section 5), so this always navigates to the best known destination.
        if (currentDistanceToTarget <= distanceTolerance) {
            consecutiveAtTargetCount++;
            // Require multiple consecutive frames to filter out momentary odometry noise
            if (consecutiveAtTargetCount >= REQUIRED_CONSECUTIVE_STOPS) break;
        } else {
            consecutiveAtTargetCount = 0;
        }

        // ───────────────────────────────────────────────────────────────
        // 4. VISION SNAPSHOT
        // ───────────────────────────────────────────────────────────────
        // Poll the AI Vision sensor every tick. If a valid object is found:
        //   - Update lastVisionHorizontalOffset with the normalized screen position
        //   - Fire pixel width exit immediately if object is large enough
        // If no valid object is found this tick, visionCurrentlyTracked goes false
        // but lastVisionHorizontalOffset retains the last known value — this allows
        // the heading fusion below to hold the last correction during brief dropouts.
        visionCurrentlyTracked = false;
        AIVision20.takeSnapshot(targetSignature);
        if (AIVision20.objectCount > 0) {
            auto& primaryObject = AIVision20.objects[0];
            // Validate: object must meet minimum size AND fall within the allowed screen region
            if (primaryObject.width >= minObjectWidth &&
                primaryObject.centerX >= minX && primaryObject.centerX <= maxX &&
                primaryObject.centerY >= minY && primaryObject.centerY <= maxY) {
                // Only process if this is a new frame (center or width changed)
                if (primaryObject.centerX != lastSnapshotCenterX || primaryObject.width != lastSnapshotWidth) {
                    // Normalize horizontal offset: 0.0 = centered, -1.0 = full left, +1.0 = full right
                    lastVisionHorizontalOffset = (primaryObject.centerX - VISION_CENTER_X) / VISION_CENTER_X;
                    lastSnapshotCenterX = primaryObject.centerX;
                    lastSnapshotWidth = primaryObject.width;
                    visionEverTracked = true;
                    visionCurrentlyTracked = true;

                    // Vision proximity exit: object fills enough of the frame to be at target distance.
                    // This is the primary exit — fires before odometry distance tolerance in most cases.
                    if (primaryObject.width >= targetPixelWidth) break;
                }
            }
        }

        // ───────────────────────────────────────────────────────────────
        // 5. HEADING CALCULATION WITH VISION FUSION + DROPOUT RECOVERY
        // ───────────────────────────────────────────────────────────────
        // Snap odometry target heading to the nearest equivalent in continuous rotation space.
        // This prevents the heading PID from trying to unwind a full 360° when the robot
        // has rotated past a multiple of 360°.
        double rotationsDifference = std::round((currentGyroHeading - odometryTargetHeading) / 360.0);
        odometryTargetHeading += rotationsDifference * 360.0;

        // Start with pure odometry heading as the baseline
        double fusedTargetHeading = odometryTargetHeading;

        if (visionEverTracked) {
            // Vision has seen the object at least once — use vision as the primary heading source.
            // lastVisionHorizontalOffset retains the last valid reading even during brief dropouts,
            // so the robot continues correcting toward the last known object position.
            //
            // visualTruthHeading: converts screen offset to an absolute heading correction.
            //   - Object on right (+offset) means robot needs to turn right (lower heading in CCW+ system)
            //   - 30.5 scale factor converts normalized pixel offset to degrees of heading correction
            double visualTruthHeading = currentGyroHeading - (lastVisionHorizontalOffset * 30.5);

            // Blend odometry baseline with vision correction using kp_distToHeadScaling as blend weight:
            //   0.0 = pure odometry, 1.0 = pure vision, 0.9 = 90% trust in vision
            fusedTargetHeading = odometryTargetHeading +
                                ((visualTruthHeading - odometryTargetHeading) * kp_distToHeadScaling);

            // Track the distance traveled from start using current odometry position.
            // This is used below to compute a corrected fallback target if vision drops permanently.
            double dx = globalX - startX;
            double dy = globalY - startY;
            distanceTraveled = sqrt(dx * dx + dy * dy);

            // Save the current fused heading while vision is active.
            // If vision drops permanently, lastFusedHeading captures the last direction
            // the robot was confirmed to be steering toward the object.
            lastFusedHeading = fusedTargetHeading;

            // If vision reacquires after a dropout, clear the handled flag so a fresh
            // dropout in the future will trigger a new target recalculation
            visionDropoutHandled = false;

            // VISION DROPOUT RECOVERY:
            // If vision was tracking but lost the object this tick, recalculate targetX/targetY
            // based on the last confirmed vision heading and the remaining distance to travel.
            // visionDropoutHandled ensures this only fires ONCE per dropout event —
            // without it, targetX/targetY would shift every tick while vision is down.
            //
            // Math: remaining distance = original total distance - how far we've already gone.
            // New target = current position + (remaining distance in direction of last vision heading).
            // This gives odometry a corrected destination that reflects where vision was pointing,
            // not the original field coordinate — which may have been offset by wheel slip or drift.
            if (!visionCurrentlyTracked && !visionDropoutHandled) {
                double remainingDistance = initialDistance - distanceTraveled;
                if (remainingDistance > 0) {
                    // Convert lastFusedHeading (CCW+ standard cartesian) to X/Y components
                    double headingRad = lastFusedHeading * M_PI / 180.0;
                    targetX = globalX + (remainingDistance * cos(headingRad));
                    targetY = globalY + (remainingDistance * sin(headingRad));
                }
                visionDropoutHandled = true;  // Lock — don't recalculate until vision reacquires
            }
        }

        // Calculate the PID heading correction — how much to steer left/right this tick
        double headingCorrection = headingPID.calculate(fusedTargetHeading, currentGyroHeading);

        // ───────────────────────────────────────────────────────────────
        // 6. SENSOR READINGS
        // ───────────────────────────────────────────────────────────────
        double leftMotorRPM = leftMotor[1].velocity(rpm) * DRIVE_MOTOR_RPM_ADJ;
        double rightMotorRPM = rightMotor[1].velocity(rpm) * DRIVE_MOTOR_RPM_ADJ;
        double leftEncoderRPM = passiveEncoderLeft.velocity(rpm) * (encoderWheelCircumferenceCM / wheelCircumferenceCM);
        double rightEncoderRPM = passiveEncoderRight.velocity(rpm) * (encoderWheelCircumferenceCM / wheelCircumferenceCM);

        // ───────────────────────────────────────────────────────────────
        // 7. MOTION PHASE CONTROL
        // ───────────────────────────────────────────────────────────────
        
        // PHASE 1: LAUNCH - Traction-controlled acceleration
        if (currentDistanceToTarget > breakDistance && !accelCompleted && !decel) 
        {
            // Independent traction control per side (prevents wheel slip)
            double leftTractionVoltage = tractionControlLeft.tractionControlSpeed(
                motorVoltageLeft[1], leftMotorRPM, leftEncoderRPM, ACCEL_FACTOR_LAUNCH
            );
            
            double rightTractionVoltage = tractionControlRight.tractionControlSpeed(
                motorVoltageRight[1], rightMotorRPM, rightEncoderRPM, ACCEL_FACTOR_LAUNCH
            );

            // Apply voltages with heading correction
            for (int i = 0; i < 3; i++) 
            { 
                motorVoltageLeft[i]  = leftTractionVoltage - (headingCorrection * accelHeadingScaling); 
                motorVoltageRight[i] = rightTractionVoltage + (headingCorrection * accelHeadingScaling); 
            }
            
            // Exit when both sides reach max voltage
            if (std::fabs(motorVoltageLeft[1]) >= std::fabs(maxSpeedVoltage) && 
                std::fabs(motorVoltageRight[1]) >= std::fabs(maxSpeedVoltage)) 
            {
                accelCompleted = true;
            }
        }
        
        // PHASE 2: CRUISE - Maintain max speed
        else if (currentDistanceToTarget > breakDistance && accelCompleted) 
        {
            for (int i = 0; i < 3; i++) 
            {
                motorVoltageLeft[i]  = maxSpeedVoltage - headingCorrection;
                motorVoltageRight[i] = maxSpeedVoltage + headingCorrection;
            }
        }
        
        // PHASE 3: DECELERATION - Adaptive ABS braking
        else if (currentDistanceToTarget <= breakDistance && !decelCompleted) 
        {
            // Initialize ABS on first entry
            if (!decel) 
            {
                decel = true;
                adaptiveABSLeft.initialize(std::fabs(motorVoltageLeft[1]));
                adaptiveABSRight.initialize(std::fabs(motorVoltageRight[1]));
            }

            // Update ABS state (monitors for wheel lockup)
            adaptiveABSLeft.decelControlSpeed(leftMotorRPM, leftEncoderRPM);
            adaptiveABSRight.decelControlSpeed(rightMotorRPM, rightEncoderRPM);

            // Get brake mode per side (coast if locked, brake otherwise)
            vex::brakeType leftBrakeMode = adaptiveABSLeft.getBrakeMode();
            vex::brakeType rightBrakeMode = adaptiveABSRight.getBrakeMode();

            // Scale heading correction for deceleration
            double adjustedHeadingCorrection = headingCorrection * decelHeadingScaling * dir;

            // Apply selective release steering (only release locked wheels for turning)
            for (int i = 0; i < 3; i++) 
            { 
                leftMotor[i].setBrake(leftBrakeMode);
                rightMotor[i].setBrake(rightBrakeMode);

                // Only apply positive voltages (release, not drive)
                motorVoltageLeft[i] = std::max(0.0, adjustedHeadingCorrection);
                motorVoltageRight[i] = std::max(0.0, -adjustedHeadingCorrection);
            }

            // Update rolling averages for stable exit detection
            leftEncoderRollingAverage = rollingAverage(leftEncoderRPM, leftEncoderRollingAverage, 3);
            rightEncoderRollingAverage = rollingAverage(rightEncoderRPM, rightEncoderRollingAverage, 3);

            // Exit when both sides slow below threshold
            if (std::fabs(leftEncoderRollingAverage) <= std::fabs(minDriveMotorRPM) && 
                std::fabs(rightEncoderRollingAverage) <= std::fabs(minDriveMotorRPM)) 
            {
                decelCompleted = true;
            }
        }
        
        // PHASE 4: APPROACH - Slow precision movement to target
        else if (decelCompleted) 
        {
            for (int i = 0; i < 3; i++) 
            { 
                leftMotor[i].setBrake(brake);
                rightMotor[i].setBrake(brake);

                motorVoltageLeft[i]  = minSpeedVoltage - (headingCorrection * approachHeadingScaling); 
                motorVoltageRight[i] = minSpeedVoltage + (headingCorrection * approachHeadingScaling); 
            }
        }

        // ───────────────────────────────────────────────────────────────
        // 8. VOLTAGE SATURATION LIMITER
        // ───────────────────────────────────────────────────────────────
        // Scale voltages proportionally if either side exceeds max
        // Preserves steering differential while staying within battery limits
        double maximumRequestedVoltage = std::max(std::fabs(motorVoltageLeft[0]), 
                                                   std::fabs(motorVoltageRight[0]));
        if (maximumRequestedVoltage > absoluteMaxVoltage) {
            double voltageScaleFactor = absoluteMaxVoltage / maximumRequestedVoltage;
            for (int i = 0; i < 3; i++) { 
                motorVoltageLeft[i] *= voltageScaleFactor;
                motorVoltageRight[i] *= voltageScaleFactor;
            }
        }

        // ───────────────────────────────────────────────────────────────
        // 9. MOTOR OUTPUT
        // ───────────────────────────────────────────────────────────────
        for (int i = 0; i < 3; i++) {
            leftMotor[i].spin(forward, motorVoltageLeft[i], volt);
            rightMotor[i].spin(forward, motorVoltageRight[i], volt);
        }
        
        vex::task::sleep(10);  // 100Hz control loop
    }

    // ═══════════════════════════════════════════════════════════════════
    // CLEANUP - Stop all motors
    // ═══════════════════════════════════════════════════════════════════
    for (int i = 0; i < 3; i++) { 
        leftMotor[i].stop(brakeMode);
        rightMotor[i].stop(brakeMode);
    }
}

 

void moveOdometry(double targetX,
                  double targetY, 
                  double breakDistance, 
                  double minSpeed,
                  double distanceTolerance,
                  double kp_heading, 
                  double ki_heading, 
                  double kd_heading,
                  vex::brakeType brakeMode,
                  double accelHeadingScaling, 
                  double decelHeadingScaling,
                  double approachHeadingScaling, 
                  double maxSpeed,
                  double headingLockDistance,
                  double timeout)
{
    // ========================================
    // CONFIGURATION CONSTANTS
    // ========================================
    const double LAUNCH_VOLTAGE = 6.0;              // Initial acceleration voltage
    const double ACCEL_FACTOR_LAUNCH = 1.2;         // Voltage ramp rate during launch
    const double SLIP_THRESHOLD_TRACTION = 20.0;    // Wheel slip detection sensitivity
    const double DECEL_STEP_PERCENT = 0.45;         // ABS brake pressure reduction rate
    const double LOCK_THRESHOLD_DECEL = 0.25;       // Wheel lock detection threshold
    const int REQUIRED_CONSECUTIVE_STOPS = 3;       // Frames at target before exit (noise filter)
    const double DOT_PRODUCT_MAX_DISTANCE = 10.0;   // Max distance for plane-crossing detection
    
    // ========================================
    // INITIALIZATION
    // ========================================
    updateOdometry();
    double startCoordinateX = globalX;
    double startCoordinateY = globalY;

    // Calculate initial path vector for dot product termination
    double pathVectorX = targetX - startCoordinateX;
    double pathVectorY = targetY - startCoordinateY;
    double initialDistance = sqrt(pathVectorX * pathVectorX + pathVectorY * pathVectorY);
    
    // Direction scalar: +1 forward, -1 backward
    double dir = (initialDistance >= 0) ? 1.0 : -1.0;

    // Initialize heading PID controller
    PID headingPID(kp_heading, ki_heading, kd_heading);
    headingPID.pidReset();
    
    // Convert percentage speeds to voltage values
    double maxSpeedVoltage = std::copysign(maxSpeed * 0.01 * absoluteMaxVoltage, initialDistance);
    double minSpeedVoltage = std::copysign(minSpeed * 0.01 * absoluteMaxVoltage, initialDistance);
    double minLaunchSpeedVoltage = std::copysign(std::min(fabs(maxSpeedVoltage), fabs(LAUNCH_VOLTAGE)), initialDistance);
    
    // RPM threshold for detecting deceleration completion
    double minDriveMotorRPM = (minSpeed * 0.01) * absoluteMaxRPM;
        
    // Motor voltage arrays (left/right × 3 motors per side)
    double motorVoltageLeft[3] = {minLaunchSpeedVoltage, minLaunchSpeedVoltage, minLaunchSpeedVoltage};
    double motorVoltageRight[3] = {minLaunchSpeedVoltage, minLaunchSpeedVoltage, minLaunchSpeedVoltage};
    
    // Motion phase flags
    bool decel = false;
    bool decelCompleted = false;
    bool accelCompleted = false;
    
    // Rolling averages for stable deceleration exit detection
    double leftEncoderRollingAverage = 0;
    double rightEncoderRollingAverage = 0;
    
    // Exit condition tracking
    int consecutiveAtTargetCount = 0;

    // Heading lock state (prevents singularity near target)
    bool headingLocked = false;
    double lockedHeadingValue = 0;

    // Initialize traction and ABS control systems
    tractionControl tractionControlLeft(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION);
    tractionControl tractionControlRight(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION);
    adaptiveABS adaptiveABSLeft(DECEL_STEP_PERCENT, LOCK_THRESHOLD_DECEL);
    adaptiveABS adaptiveABSRight(DECEL_STEP_PERCENT, LOCK_THRESHOLD_DECEL);

    // Timeout safety timer
    vex::timer safetyTimer;
    safetyTimer.reset();
    double timeoutMs = timeout * 1000.0;

    // ═══════════════════════════════════════════════════════════════════
    // MAIN CONTROL LOOP
    // ═══════════════════════════════════════════════════════════════════
    while (true) 
    {
        updateOdometry();

        // ───────────────────────────────────────────────────────────────
        // 1. STATE CALCULATION
        // ───────────────────────────────────────────────────────────────
        double currentDistanceToTarget, odometryTargetHeading;
        calculatePathToTarget(globalX, globalY, targetX, targetY, currentDistanceToTarget, odometryTargetHeading);
        double currentGyroHeading = getContinuousStandardHeading();

        // ───────────────────────────────────────────────────────────────
        // 2. TIMEOUT SAFETY CHECK
        // ───────────────────────────────────────────────────────────────
        if (safetyTimer.time(vex::msec) > timeoutMs) {
            break; // Prevent infinite loop on sensor failure or unreachable target
        }

        // ───────────────────────────────────────────────────────────────
        // 3. EXIT CONDITIONS
        // ───────────────────────────────────────────────────────────────
        
        // A. Primary: Euclidean distance tolerance (normal success)
        if (currentDistanceToTarget <= distanceTolerance) {
            consecutiveAtTargetCount++;
            if (consecutiveAtTargetCount >= REQUIRED_CONSECUTIVE_STOPS) break;
        } else {
            consecutiveAtTargetCount = 0;
        }

        // B. Secondary: Dot product plane crossing (overshoot detection)
        // Detects when robot crosses perpendicular plane through target
        double vectorToTargetX = targetX - globalX;
        double vectorToTargetY = targetY - globalY;
        double progressScalar = (pathVectorX * vectorToTargetX) + (pathVectorY * vectorToTargetY);
        
        if (progressScalar < 0 && currentDistanceToTarget < DOT_PRODUCT_MAX_DISTANCE) {
            break; // Crossed the finish line
        }

        // ───────────────────────────────────────────────────────────────
        // 4. HEADING CALCULATION
        // ───────────────────────────────────────────────────────────────
        
        // Snap target heading to nearest 360° multiple (prevents discontinuity)
        double rotationsDifference = std::round((currentGyroHeading - odometryTargetHeading) / 360.0);
        odometryTargetHeading += rotationsDifference * 360.0;

        // Apply heading lock near target to prevent singularity
        double fusedTargetHeading = odometryTargetHeading;
        
        if (currentDistanceToTarget <= headingLockDistance) {
            // Lock heading to prevent erratic corrections when very close
            if (!headingLocked) {
                lockedHeadingValue = odometryTargetHeading;
                headingLocked = true;
            }
            fusedTargetHeading = lockedHeadingValue;
        } else {
            headingLocked = false;
        }

        // Calculate heading correction via PID
        double headingCorrection = headingPID.calculate(fusedTargetHeading, currentGyroHeading);

        // ───────────────────────────────────────────────────────────────
        // 5. SENSOR READINGS
        // ───────────────────────────────────────────────────────────────
        double leftMotorRPM = leftMotor[1].velocity(rpm) * DRIVE_MOTOR_RPM_ADJ;
        double rightMotorRPM = rightMotor[1].velocity(rpm) * DRIVE_MOTOR_RPM_ADJ;
        double leftEncoderRPM = passiveEncoderLeft.velocity(rpm) * (encoderWheelCircumferenceCM / wheelCircumferenceCM);
        double rightEncoderRPM = passiveEncoderRight.velocity(rpm) * (encoderWheelCircumferenceCM / wheelCircumferenceCM);

        // ───────────────────────────────────────────────────────────────
        // 6. MOTION PHASE CONTROL
        // ───────────────────────────────────────────────────────────────
        
        // PHASE 1: LAUNCH - Traction-controlled acceleration
        if (currentDistanceToTarget > breakDistance && !accelCompleted && !decel) 
        {
            // Independent traction control per side (prevents wheel slip)
            double leftTractionVoltage = tractionControlLeft.tractionControlSpeed(
                motorVoltageLeft[1], leftMotorRPM, leftEncoderRPM, ACCEL_FACTOR_LAUNCH
            );
            
            double rightTractionVoltage = tractionControlRight.tractionControlSpeed(
                motorVoltageRight[1], rightMotorRPM, rightEncoderRPM, ACCEL_FACTOR_LAUNCH
            );

            // Apply voltages with heading correction
            for (int i = 0; i < 3; i++) 
            { 
                motorVoltageLeft[i]  = leftTractionVoltage - (headingCorrection * accelHeadingScaling); 
                motorVoltageRight[i] = rightTractionVoltage + (headingCorrection * accelHeadingScaling); 
            }
            
            // Exit when both sides reach max voltage
            if (std::fabs(motorVoltageLeft[1]) >= std::fabs(maxSpeedVoltage) && 
                std::fabs(motorVoltageRight[1]) >= std::fabs(maxSpeedVoltage)) 
            {
                accelCompleted = true;
            }
        }
        
        // PHASE 2: CRUISE - Maintain max speed
        else if (currentDistanceToTarget > breakDistance && accelCompleted) 
        {
            for (int i = 0; i < 3; i++) 
            {
                motorVoltageLeft[i]  = maxSpeedVoltage - headingCorrection;
                motorVoltageRight[i] = maxSpeedVoltage + headingCorrection;
            }
        }
        
        // PHASE 3: DECELERATION - Adaptive ABS braking
        else if (currentDistanceToTarget <= breakDistance && !decelCompleted) 
        {
            // Initialize ABS on first entry
            if (!decel) 
            {
                decel = true;
                adaptiveABSLeft.initialize(std::fabs(motorVoltageLeft[1]));
                adaptiveABSRight.initialize(std::fabs(motorVoltageRight[1]));
            }

            // Update ABS state (monitors for wheel lockup)
            adaptiveABSLeft.decelControlSpeed(leftMotorRPM, leftEncoderRPM);
            adaptiveABSRight.decelControlSpeed(rightMotorRPM, rightEncoderRPM);

            // Get brake mode per side (coast if locked, brake otherwise)
            vex::brakeType leftBrakeMode = adaptiveABSLeft.getBrakeMode();
            vex::brakeType rightBrakeMode = adaptiveABSRight.getBrakeMode();

            // Scale heading correction for deceleration
            double adjustedHeadingCorrection = headingCorrection * decelHeadingScaling * dir;

            // Apply selective release steering (only release locked wheels for turning)
            for (int i = 0; i < 3; i++) 
            { 
                leftMotor[i].setBrake(leftBrakeMode);
                rightMotor[i].setBrake(rightBrakeMode);

                // Only apply positive voltages (release, not drive)
                motorVoltageLeft[i] = std::max(0.0, adjustedHeadingCorrection);
                motorVoltageRight[i] = std::max(0.0, -adjustedHeadingCorrection);
            }

            // Update rolling averages for stable exit detection
            leftEncoderRollingAverage = rollingAverage(leftEncoderRPM, leftEncoderRollingAverage, 3);
            rightEncoderRollingAverage = rollingAverage(rightEncoderRPM, rightEncoderRollingAverage, 3);

            // Exit when both sides slow below threshold
            if (std::fabs(leftEncoderRollingAverage) <= std::fabs(minDriveMotorRPM) && 
                std::fabs(rightEncoderRollingAverage) <= std::fabs(minDriveMotorRPM)) 
            {
                decelCompleted = true;
            }
        }
        
        // PHASE 4: APPROACH - Slow precision movement to target
        else if (decelCompleted) 
        {
            for (int i = 0; i < 3; i++) 
            { 
                leftMotor[i].setBrake(brake);
                rightMotor[i].setBrake(brake);

                motorVoltageLeft[i]  = minSpeedVoltage - (headingCorrection * approachHeadingScaling); 
                motorVoltageRight[i] = minSpeedVoltage + (headingCorrection * approachHeadingScaling); 
            }
        }

        // ───────────────────────────────────────────────────────────────
        // 7. VOLTAGE SATURATION LIMITER
        // ───────────────────────────────────────────────────────────────
        // Scale voltages proportionally if either side exceeds max
        // Preserves steering differential while staying within battery limits
        double maximumRequestedVoltage = std::max(std::fabs(motorVoltageLeft[0]), 
                                                   std::fabs(motorVoltageRight[0]));
        if (maximumRequestedVoltage > absoluteMaxVoltage) {
            double voltageScaleFactor = absoluteMaxVoltage / maximumRequestedVoltage;
            for (int i = 0; i < 3; i++) { 
                motorVoltageLeft[i] *= voltageScaleFactor;
                motorVoltageRight[i] *= voltageScaleFactor;
            }
        }

        // ───────────────────────────────────────────────────────────────
        // 8. MOTOR OUTPUT
        // ───────────────────────────────────────────────────────────────
        for (int i = 0; i < 3; i++) {
            leftMotor[i].spin(forward, motorVoltageLeft[i], volt);
            rightMotor[i].spin(forward, motorVoltageRight[i], volt);
        }
        
        vex::task::sleep(10);  // 100Hz control loop
    }

    // ═══════════════════════════════════════════════════════════════════
    // CLEANUP - Stop all motors
    // ═══════════════════════════════════════════════════════════════════
    for (int i = 0; i < 3; i++) { 
        leftMotor[i].stop(brakeMode);
        rightMotor[i].stop(brakeMode);
    }
}