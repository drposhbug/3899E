// ============================================================================
// NAVIGATION.CPP - Motion Control Implementation
// ============================================================================
#include "navigation.h"
#include "robot_config.h"
#include "vex.h"
#include "utils.h"
#include "pid.h"
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace vex;

// ======================== BASIC MOVEMENT ====================================

void move(double distanceCM, double maxSpeed, vex::directionType dir) {
    // Calculate rotations needed based on wheel circumference
    double targetRotations = distanceCM / wheelCircumferenceCM;

    // Set all motors to coast for smooth stopping
    for (int i = 0; i < 3; i++) {
        leftMotor[i].setBrake(brakeType::coast);
        rightMotor[i].setBrake(brakeType::coast);
    }

    // Command all motors to spin simultaneously
    for (int i = 0; i < 3; i++) {
        leftMotor[i].spinFor(dir, targetRotations, rotationUnits::rev, maxSpeed, velocityUnits::pct, false);
        rightMotor[i].spinFor(dir, targetRotations, rotationUnits::rev, maxSpeed, velocityUnits::pct, false);
    }
}

// ======================== ARC TURN ==========================================

void arcTurn(double targetDistance, double breakDistance, double minSpeed, double maxSpeed,
             double turnRadius, bool turnLeft) {
    
    // Calculate differential speeds for inner and outer wheels
    double innerRatio = (turnRadius - (TRACK_WIDTH / 2)) / turnRadius;
    double outerRatio = (turnRadius + (TRACK_WIDTH / 2)) / turnRadius;

    // Reset encoder positions
    passiveEncoderLeft.resetPosition();
    passiveEncoderRight.resetPosition();

    double currentDistance = 0;
    double maxVoltage = maxSpeed * 0.01 * absoluteMaxVoltage;
    double minVoltage = minSpeed * 0.01 * absoluteMaxVoltage;

    // Calculate target voltages maintaining speed ratio
    double innerVoltage = minVoltage * innerRatio;
    double outerVoltage = minVoltage * outerRatio;
    double innerMaxVoltage = maxVoltage * innerRatio;
    double outerMaxVoltage = maxVoltage * outerRatio;

    while (std::fabs(currentDistance) <= fabs(targetDistance)) {
        // Calculate average distance traveled
        currentDistance = ((passiveEncoderLeft.position(degrees) +
                           passiveEncoderRight.position(degrees)) / 2.0 / 360.0) *
                         encoderWheelCircumferenceCM;

        // Acceleration phase
        if (fabs(currentDistance) < (fabs(targetDistance) - breakDistance)) {
            innerVoltage = std::min(innerMaxVoltage, innerVoltage + 0.1);
            outerVoltage = std::min(outerMaxVoltage, outerVoltage + 0.1);
        }
        // Deceleration phase
        else {
            innerVoltage = std::max(minVoltage * innerRatio, innerVoltage - 0.1);
            outerVoltage = std::max(minVoltage * outerRatio, outerVoltage - 0.1);
        }

        // Apply voltages based on turn direction
        if (turnLeft) {
            for (int i = 0; i < 3; i++) {
                leftMotor[i].spin(forward, innerVoltage, voltageUnits::volt);
                rightMotor[i].spin(forward, outerVoltage, voltageUnits::volt);
            }
        } else {
            for (int i = 0; i < 3; i++) {
                leftMotor[i].spin(forward, outerVoltage, voltageUnits::volt);
                rightMotor[i].spin(forward, innerVoltage, voltageUnits::volt);
            }
        }

        vex::task::sleep(20);
    }

    // Stop all motors
    for (int i = 0; i < 3; i++) {
        leftMotor[i].setBrake(coast);
        rightMotor[i].setBrake(coast);
        leftMotor[i].stop();
        rightMotor[i].stop();
    }
}

// ======================== STRAIGHT WITH MOTION PROFILING ====================

void straight(double targetDistance, double breakDistance, double targetHeading,
              double minSpeed, double kp_heading, double ki_heading, double kd_heading,
              double accelHeadingScaling, double decelHeadingScaling,
              double approachHeadingScaling, double maxSpeed) {
    
    // Motion profiling configuration
    const double LAUNCH_VOLTAGE = 6;
    const double ACCEL_FACTOR_LAUNCH = 1.25;
    const double SLIP_THRESHOLD_TRACTION = 0.3;
    const double SLIP_THRESHOLD_ABS = 0.25;

    // Reset encoders
    passiveEncoderLeft.resetPosition();
    passiveEncoderRight.resetPosition();

    // Initialize PID controller for heading
    PID headingPID(kp_heading, ki_heading, kd_heading);
    headingPID.pidReset();

    // Motion state variables
    double currentDistance = 0;
    double maxSpeedVoltage = std::copysign(maxSpeed * 0.01 * absoluteMaxVoltage, targetDistance);
    double minSpeedVoltage = std::copysign(minSpeed * 0.01 * absoluteMaxVoltage, targetDistance);
    double minLaunchSpeedVoltage = std::copysign(std::min(fabs(maxSpeedVoltage), LAUNCH_VOLTAGE), targetDistance);

    double minDriveMotorRPM = (minSpeed * 0.01) * absoluteMaxRPM;
    
    // Motor control arrays
    double motorVoltageLeft[3] = {minLaunchSpeedVoltage, minLaunchSpeedVoltage, minLaunchSpeedVoltage};
    double motorVoltageRight[3] = {minLaunchSpeedVoltage, minLaunchSpeedVoltage, minLaunchSpeedVoltage};

    // Phase tracking
    bool decel = false;
    bool decelCompleted = false;
    bool accelCompleted = false;

    double leftEncoderRollingAverage = 0;
    double rightEncoderRollingAverage = 0;

    // Initialize traction control and ABS systems
    ABSController ABSControllerLeft(SLIP_THRESHOLD_ABS);
    ABSController ABSControllerRight(SLIP_THRESHOLD_ABS);
    tractionControl tractionControlLeft(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION);
    tractionControl tractionControlRight(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION);

    // Main control loop
    while (std::fabs(currentDistance) <= fabs(targetDistance) - 3) {
        
        // Update current position
        currentDistance = ((passiveEncoderLeft.position(degrees) + 
                          passiveEncoderRight.position(degrees)) / 2.0 / 360.0) * 
                         encoderWheelCircumferenceCM;

        // Calculate heading correction
        double currentHeading = InertialSensor.rotation(degrees) + headingOffset;
        double headingCorrection = headingPID.calculate(targetHeading, currentHeading);

        // Get sensor readings
        double leftEncoderRPM = passiveEncoderLeft.velocity(vex::velocityUnits::rpm) * 
                               (encoderWheelCircumferenceCM / wheelCircumferenceCM);
        double rightEncoderRPM = passiveEncoderRight.velocity(vex::velocityUnits::rpm) * 
                                (encoderWheelCircumferenceCM / wheelCircumferenceCM);

        double leftMotorRPM = leftMotor[1].velocity(vex::velocityUnits::rpm) * DRIVE_MOTOR_RPM_ADJ;
        double rightMotorRPM = rightMotor[1].velocity(vex::velocityUnits::rpm) * DRIVE_MOTOR_RPM_ADJ;

        // ==================== LAUNCH PHASE ====================
        if (fabs(currentDistance) < (fabs(targetDistance) - breakDistance) && !accelCompleted && !decel) {
            
            // Apply traction control to prevent wheel slip
            double leftTractionVoltage = tractionControlLeft.tractionControlSpeed(
                motorVoltageLeft[1], leftMotorRPM, leftEncoderRPM, ACCEL_FACTOR_LAUNCH);
            double rightTractionVoltage = tractionControlRight.tractionControlSpeed(
                motorVoltageRight[1], rightMotorRPM, rightEncoderRPM, ACCEL_FACTOR_LAUNCH);

            // Synchronize both sides to lower voltage (more conservative)
            double syncedMotorVoltage = std::min(leftTractionVoltage, rightTractionVoltage);

            // Apply heading correction
            for (int i = 0; i < 3; i++) {
                motorVoltageLeft[i] = syncedMotorVoltage + (headingCorrection * accelHeadingScaling);
                motorVoltageRight[i] = syncedMotorVoltage - (headingCorrection * accelHeadingScaling);
            }

            // Check if acceleration is complete
            double avgMotorVoltage = (motorVoltageLeft[0] + motorVoltageLeft[1] + motorVoltageLeft[2] +
                                     motorVoltageRight[0] + motorVoltageRight[1] + motorVoltageRight[2]) / 6.0;
            
            if (fabs(avgMotorVoltage) >= (fabs(maxSpeedVoltage) - VOLTAGE_TOLERANCE)) {
                accelCompleted = true;
            }
        }
        
        // ==================== CRUISE PHASE ====================
        else if (fabs(currentDistance) < (fabs(targetDistance) - breakDistance) && accelCompleted) {
            for (int i = 0; i < 3; i++) {
                motorVoltageLeft[i] = maxSpeedVoltage + headingCorrection;
                motorVoltageRight[i] = maxSpeedVoltage - headingCorrection;
            }
        }
        
        // ==================== DECELERATION PHASE ====================
        else if (fabs(currentDistance) >= (fabs(targetDistance) - breakDistance) && !decelCompleted) {
            decel = true;

            // Apply ABS braking
            vex::brakeType leftBrakeMode = ABSControllerLeft.ABSSpeedReduction(leftMotorRPM, leftEncoderRPM);
            vex::brakeType rightBrakeMode = ABSControllerRight.ABSSpeedReduction(rightMotorRPM, rightEncoderRPM);

            // Apply heading correction during deceleration
            for (int i = 0; i < 3; i++) {
                motorVoltageLeft[i] = std::max(0.0, headingCorrection * decelHeadingScaling);
                motorVoltageRight[i] = std::max(0.0, -headingCorrection * decelHeadingScaling);

                // Set brake modes
                if (leftBrakeMode == brakeType::coast) {
                    leftMotor[i].setBrake(coast);
                    motorVoltageLeft[i] = 0;
                } else if (motorVoltageLeft[i] == 0) {
                    leftMotor[i].setBrake(brake);
                }

                if (rightBrakeMode == brakeType::coast) {
                    rightMotor[i].setBrake(coast);
                    motorVoltageRight[i] = 0;
                } else if (motorVoltageRight[i] == 0) {
                    rightMotor[i].setBrake(brake);
                }
            }

            // Track deceleration progress
            leftEncoderRollingAverage = rollingAverage(leftEncoderRPM, leftEncoderRollingAverage, 10);
            rightEncoderRollingAverage = rollingAverage(rightEncoderRPM, rightEncoderRollingAverage, 10);

            if (fabs(leftEncoderRollingAverage) <= fabs(minDriveMotorRPM) ||
                fabs(rightEncoderRollingAverage) <= fabs(minDriveMotorRPM)) {
                decelCompleted = true;
            }
        }
        
        // ==================== APPROACH PHASE ====================
        else if (decelCompleted) {
            for (int i = 0; i < 3; i++) {
                motorVoltageLeft[i] = minSpeedVoltage + (headingCorrection * approachHeadingScaling);
                motorVoltageRight[i] = minSpeedVoltage - (headingCorrection * approachHeadingScaling);
            }
        }

        // Apply voltages to motors
        for (int i = 0; i < 3; i++) {
            leftMotor[i].spin(forward, motorVoltageLeft[i], voltageUnits::volt);
            rightMotor[i].spin(forward, motorVoltageRight[i], voltageUnits::volt);
        }

        vex::task::sleep(10);
    }

    // Stop all motors
    for (int i = 0; i < 3; i++) {
        leftMotor[i].setBrake(brake);
        rightMotor[i].setBrake(brake);
        leftMotor[i].stop();
        rightMotor[i].stop();
    }
}

// ======================== TURN WITH MOTION PROFILING ========================

void turn(double targetHeading, double breakDistanceInDegrees, double minSpeed, double maxSpeed) {
    
    bool decelCompleted = false;
    bool accelCompleted = false;
    bool decel = false;

    double currentHeading = InertialSensor.rotation(degrees) + headingOffset;
    int completeRotations = (int)(currentHeading / 360.0);
    double targetRotationHeading = targetHeading + (completeRotations * 360.0);
    double headingError = targetRotationHeading - currentHeading;

    // Convert speeds to voltages
    double maxSpeedVoltage = std::copysign(maxSpeed * 0.01 * absoluteMaxVoltage, headingError);
    double minSpeedVoltage = std::copysign(minSpeed * 0.01 * absoluteMaxVoltage, headingError);
    double launchVoltage = std::copysign(4, headingError);
    double minLaunchSpeedVoltage = std::copysign(std::min(fabs(maxSpeedVoltage), fabs(launchVoltage)), headingError);

    double minDriveMotorRPM = (minSpeed * 0.01) * absoluteMaxRPM;

    // Traction control parameters
    const double TURN_ACCEL_FACTOR_LAUNCH = 1.2;
    const double SLIP_THRESHOLD_TRACTION = 10;
    const double SLIP_THRESHOLD_ABS = 100;
    const double EXIT_TOLERANCE_DEGREES = 1.2;

    double motorVoltageLeft[3] = {minLaunchSpeedVoltage, minLaunchSpeedVoltage, minLaunchSpeedVoltage};
    double motorVoltageRight[3] = {minLaunchSpeedVoltage, minLaunchSpeedVoltage, minLaunchSpeedVoltage};

    double leftEncoderRollingAverage = 0;
    double rightEncoderRollingAverage = 0;
    double voltageRollingAverage = 0;

    // Initialize control systems
    ABSController ABSControllerLeft(SLIP_THRESHOLD_ABS);
    ABSController ABSControllerRight(SLIP_THRESHOLD_ABS);
    tractionControl tractionControlLeft(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION);
    tractionControl tractionControlRight(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION);

    // Main turn loop
    while ((maxSpeedVoltage > 0 && currentHeading <= targetRotationHeading - EXIT_TOLERANCE_DEGREES) ||
           (maxSpeedVoltage < 0 && currentHeading >= targetRotationHeading + EXIT_TOLERANCE_DEGREES)) {
        
        currentHeading = InertialSensor.rotation(degrees) + headingOffset;
        headingError = targetRotationHeading - currentHeading;

        // Get sensor readings
        double leftMotorRPM = fabs(leftMotor[1].velocity(vex::velocityUnits::rpm)) * DRIVE_MOTOR_RPM_ADJ;
        double rightMotorRPM = fabs(rightMotor[1].velocity(vex::velocityUnits::rpm)) * DRIVE_MOTOR_RPM_ADJ;

        double leftEncoderRPM = fabs(passiveEncoderLeft.velocity(velocityUnits::rpm)) *
                                (encoderWheelCircumferenceCM / wheelCircumferenceCM);
        double rightEncoderRPM = fabs(passiveEncoderRight.velocity(velocityUnits::rpm)) *
                                 (encoderWheelCircumferenceCM / wheelCircumferenceCM);

        double averageEncoderRPM = (leftEncoderRPM + rightEncoderRPM) / 2.0;

        // ==================== LAUNCH PHASE ====================
        if (std::fabs(headingError) > fabs(breakDistanceInDegrees) && !accelCompleted && !decel) {
            
            // Apply traction control
            double leftTractionVoltage = tractionControlLeft.tractionControlSpeed(
                motorVoltageLeft[1], leftMotorRPM, averageEncoderRPM, TURN_ACCEL_FACTOR_LAUNCH);
            double rightTractionVoltage = tractionControlRight.tractionControlSpeed(
                motorVoltageRight[1], rightMotorRPM, averageEncoderRPM, TURN_ACCEL_FACTOR_LAUNCH);

            // Synchronize voltages
            double syncedMotorVoltage = std::min(fabs(leftTractionVoltage), fabs(rightTractionVoltage));

            for (int i = 0; i < 3; i++) {
                motorVoltageLeft[i] = std::copysign(syncedMotorVoltage, motorVoltageLeft[i]);
                motorVoltageRight[i] = std::copysign(syncedMotorVoltage, motorVoltageRight[i]);
            }

            double averageMotorVoltage = (fabs(motorVoltageLeft[0]) + fabs(motorVoltageRight[0]) +
                                         fabs(motorVoltageLeft[1]) + fabs(motorVoltageRight[1]) +
                                         fabs(motorVoltageLeft[2]) + fabs(motorVoltageRight[2])) / 6.0;

            voltageRollingAverage = rollingAverage(averageMotorVoltage, voltageRollingAverage, 5);

            if (fabs(voltageRollingAverage) >= (fabs(maxSpeedVoltage) - VOLTAGE_TOLERANCE)) {
                accelCompleted = true;
            }
        }
        
        // ==================== CRUISE PHASE ====================
        else if (std::abs(headingError) > fabs(breakDistanceInDegrees) && accelCompleted) {
            for (int i = 0; i < 3; i++) {
                motorVoltageLeft[i] = maxSpeedVoltage;
                motorVoltageRight[i] = maxSpeedVoltage;
            }
        }
        
        // ==================== DECELERATION PHASE ====================
        else if (std::abs(headingError) <= fabs(breakDistanceInDegrees) && !decelCompleted) {
            
            if (!decel) {
                for (int i = 0; i < 3; i++) {
                    motorVoltageLeft[i] = 0;
                    motorVoltageRight[i] = 0;
                }
            }
            decel = true;

            // Apply ABS
            vex::brakeType leftBrakeMode = ABSControllerLeft.ABSSpeedReduction(leftMotorRPM, leftEncoderRPM);
            vex::brakeType rightBrakeMode = ABSControllerRight.ABSSpeedReduction(rightMotorRPM, rightEncoderRPM);

            brakeType syncedBrakeMode;
            if (leftBrakeMode == brakeType::coast || rightBrakeMode == brakeType::coast) {
                syncedBrakeMode = brakeType::coast;
            } else {
                syncedBrakeMode = brakeType::brake;
            }

            for (int i = 0; i < 3; i++) {
                motorVoltageLeft[i] = 0;
                motorVoltageRight[i] = 0;
                leftMotor[i].setBrake(syncedBrakeMode);
                rightMotor[i].setBrake(syncedBrakeMode);
            }

            leftEncoderRollingAverage = rollingAverage(leftEncoderRPM, leftEncoderRollingAverage, 3);
            rightEncoderRollingAverage = rollingAverage(rightEncoderRPM, rightEncoderRollingAverage, 3);

            if (fabs(leftEncoderRollingAverage) <= fabs(minDriveMotorRPM) &&
                fabs(rightEncoderRollingAverage) <= fabs(minDriveMotorRPM)) {
                decelCompleted = true;
            }
        }
        
        // ==================== APPROACH PHASE ====================
        else if (decelCompleted) {
            for (int i = 0; i < 3; i++) {
                motorVoltageLeft[i] = minSpeedVoltage;
                motorVoltageRight[i] = minSpeedVoltage;
            }
        }

        // Apply voltages (counter-rotating for turn)
        if (!decel || decelCompleted) {
            for (int i = 0; i < 3; i++) {
                leftMotor[i].spin(forward, motorVoltageLeft[i], voltageUnits::volt);
                rightMotor[i].spin(forward, -motorVoltageRight[i], voltageUnits::volt);
            }
        }

        vex::task::sleep(10);
    }

    // Stop motors
    for (int i = 0; i < 3; i++) {
        leftMotor[i].stop(coast);
        rightMotor[i].stop(coast);
    }
}

// ======================== PIVOT TURN ========================================

void pivotTurn(double targetHeading, double breakDistanceInDegrees, double minSpeed, double maxSpeed) {
    
    bool decelCompleted = false;
    bool accelCompleted = false;
    bool decel = false;

    double currentHeading = InertialSensor.rotation(degrees) + headingOffset;
    double headingError = targetHeading - currentHeading;

    // Convert speeds to voltages
    double maxSpeedVoltage = std::copysign(maxSpeed * 0.01 * 12, headingError);
    double minSpeedVoltage = std::copysign(minSpeed * 0.01 * 12, headingError);
    double launchVoltage = std::copysign(5, headingError);
    double minLaunchSpeedVoltage = std::copysign(std::min(fabs(maxSpeedVoltage), fabs(launchVoltage)), headingError);

    double minDriveMotorRPM = (minSpeed * 0.01) * absoluteMaxRPM;

    const double SLIP_THRESHOLD_TRACTION = 0.35;
    const double SLIP_THRESHOLD_ABS = 0.35;
    double accelFactorLaunch = 1.4;

    double motorVoltageLeft[3] = {minLaunchSpeedVoltage, minLaunchSpeedVoltage, minLaunchSpeedVoltage};
    double motorVoltageRight[3] = {minLaunchSpeedVoltage, minLaunchSpeedVoltage, minLaunchSpeedVoltage};

    double leftMotorRPM[3] = {0, 0, 0};
    double rightMotorRPM[3] = {0, 0, 0};
    vex::brakeType leftBrakeMode[3];
    vex::brakeType rightBrakeMode[3];

    double leftEncoderRollingAverage = 0;
    double rightEncoderRollingAverage = 0;
    double averageMotorVoltage = 0;

    // Initialize control systems
    ABSController ABSControllerLeft[3] = {
        ABSController(SLIP_THRESHOLD_ABS),
        ABSController(SLIP_THRESHOLD_ABS),
        ABSController(SLIP_THRESHOLD_ABS)};
    ABSController ABSControllerRight[3] = {
        ABSController(SLIP_THRESHOLD_ABS),
        ABSController(SLIP_THRESHOLD_ABS),
        ABSController(SLIP_THRESHOLD_ABS)};

    tractionControl tractionControlLeft[3] = {
        tractionControl(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION),
        tractionControl(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION),
        tractionControl(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION)};

    tractionControl tractionControlRight[3] = {
        tractionControl(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION),
        tractionControl(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION),
        tractionControl(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION)};

    // Main pivot loop
    while (std::abs(headingError) > 9) {
        currentHeading = InertialSensor.rotation(degrees) + headingOffset;
        headingError = targetHeading - currentHeading;

        // Get motor and encoder RPMs
        for (int i = 0; i < 3; i++) {
            leftMotorRPM[i] = fabs(leftMotor[i].velocity(velocityUnits::rpm));
            rightMotorRPM[i] = fabs(rightMotor[i].velocity(velocityUnits::rpm));
        }

        double leftEncoderRPM = fabs(passiveEncoderLeft.velocity(velocityUnits::rpm)) *
                                (encoderWheelCircumferenceCM / wheelCircumferenceCM) * ENCODER_RADIUS_RATIO;
        double rightEncoderRPM = fabs(passiveEncoderRight.velocity(velocityUnits::rpm)) *
                                 (encoderWheelCircumferenceCM / wheelCircumferenceCM) * ENCODER_RADIUS_RATIO;
        double avgEncoderRPM = (fabs(leftEncoderRPM) + fabs(rightEncoderRPM)) / 2;

        // ==================== LAUNCH PHASE ====================
        if (std::fabs(headingError) > fabs(breakDistanceInDegrees) && !accelCompleted && !decel) {
            
            for (int i = 0; i < 3; i++) {
                motorVoltageLeft[i] = tractionControlLeft[i].tractionControlSpeed(
                    motorVoltageLeft[i], leftMotorRPM[i], avgEncoderRPM, accelFactorLaunch);
                motorVoltageRight[i] = tractionControlRight[i].tractionControlSpeed(
                    motorVoltageRight[i], rightMotorRPM[i], avgEncoderRPM, accelFactorLaunch);
            }

            averageMotorVoltage = (fabs(motorVoltageLeft[0]) + fabs(motorVoltageRight[0]) +
                                  fabs(motorVoltageLeft[1]) + fabs(motorVoltageRight[1]) +
                                  fabs(motorVoltageLeft[2]) + fabs(motorVoltageRight[2])) / 6.0;

            if (std::fabs(averageMotorVoltage) >= std::fabs(maxSpeedVoltage)) {
                accelCompleted = true;
            }
        }
        
        // ==================== CRUISE PHASE ====================
        else if (std::abs(headingError) > fabs(breakDistanceInDegrees) && accelCompleted) {
            for (int i = 0; i < 3; i++) {
                motorVoltageLeft[i] = maxSpeedVoltage;
                motorVoltageRight[i] = maxSpeedVoltage;
            }
        }
        
        // ==================== DECELERATION PHASE ====================
        else if (std::abs(headingError) <= fabs(breakDistanceInDegrees) && !decelCompleted) {
            decel = true;

            for (int i = 0; i < 3; i++) {
                motorVoltageLeft[i] = 0;
                motorVoltageRight[i] = 0;

                // Apply ABS to each wheel
                leftBrakeMode[i] = ABSControllerLeft[i].ABSSpeedReduction(leftMotorRPM[i], leftEncoderRPM);
                rightBrakeMode[i] = ABSControllerRight[i].ABSSpeedReduction(rightMotorRPM[i], rightEncoderRPM);
            }

            // Synchronize brake modes for wheel pairs
            brakeType leadingPairMode = (leftBrakeMode[0] == brakeType::coast || rightBrakeMode[2] == brakeType::coast) ? 
                                       brakeType::coast : brakeType::brake;
            leftMotor[0].stop(leadingPairMode);
            rightMotor[2].stop(leadingPairMode);

            brakeType middlePairMode = (leftBrakeMode[1] == brakeType::coast || rightBrakeMode[1] == brakeType::coast) ? 
                                      brakeType::coast : brakeType::brake;
            leftMotor[1].stop(middlePairMode);
            rightMotor[1].stop(middlePairMode);

            brakeType trailingPairMode = (leftBrakeMode[2] == brakeType::coast || rightBrakeMode[0] == brakeType::coast) ? 
                                        brakeType::coast : brakeType::brake;
            leftMotor[2].stop(trailingPairMode);
            rightMotor[0].stop(trailingPairMode);

            leftEncoderRollingAverage = rollingAverage(leftEncoderRPM, leftEncoderRollingAverage, 3);
            rightEncoderRollingAverage = rollingAverage(rightEncoderRPM, rightEncoderRollingAverage, 3);

            if (fabs(leftEncoderRollingAverage) <= fabs(minDriveMotorRPM) &&
                fabs(rightEncoderRollingAverage) <= fabs(minDriveMotorRPM)) {
                decelCompleted = true;
            }
        }
        
        // ==================== APPROACH PHASE ====================
        else if (decelCompleted) {
            for (int i = 0; i < 3; i++) {
                motorVoltageLeft[i] = minSpeedVoltage;
                motorVoltageRight[i] = minSpeedVoltage;
            }
        }

        // Synchronize voltages for wheel pairs
        double lowerVoltageLeading = std::min(std::fabs(motorVoltageLeft[0]), std::fabs(motorVoltageRight[2]));
        motorVoltageLeft[0] = std::copysign(lowerVoltageLeading, motorVoltageLeft[0]);
        motorVoltageRight[2] = std::copysign(lowerVoltageLeading, motorVoltageRight[2]);

        double lowerVoltageMiddle = std::min(std::fabs(motorVoltageLeft[1]), std::fabs(motorVoltageRight[1]));
        motorVoltageLeft[1] = std::copysign(lowerVoltageMiddle, motorVoltageLeft[1]);
        motorVoltageRight[1] = std::copysign(lowerVoltageMiddle, motorVoltageRight[1]);

        double lowerVoltageTrailing = std::min(std::fabs(motorVoltageLeft[2]), std::fabs(motorVoltageRight[0]));
        motorVoltageLeft[2] = std::copysign(lowerVoltageTrailing, motorVoltageLeft[2]);
        motorVoltageRight[0] = std::copysign(lowerVoltageTrailing, motorVoltageRight[0]);

        // Apply voltages - pivot around stationary side
        if (!decel || decelCompleted) {
            if (motorVoltageLeft[0] > 0) {  // Pivot around right side
                leftMotor[0].spin(forward, motorVoltageLeft[0], voltageUnits::volt);
                rightMotor[0].stop(brake);
                leftMotor[1].spin(forward, motorVoltageLeft[1], voltageUnits::volt);
                rightMotor[1].stop(hold);
                leftMotor[2].spin(forward, motorVoltageLeft[2], voltageUnits::volt);
                rightMotor[2].stop(brake);
            } else {  // Pivot around left side
                leftMotor[0].stop(brake);
                rightMotor[0].spin(forward, fabs(motorVoltageRight[0]), voltageUnits::volt);
                leftMotor[1].stop(hold);
                rightMotor[1].spin(forward, fabs(motorVoltageRight[1]), voltageUnits::volt);
                leftMotor[2].stop(brake);
                rightMotor[2].spin(forward, fabs(motorVoltageRight[2]), voltageUnits::volt);
            }
        }

        vex::task::sleep(10);
    }

    // Stop all motors
    for (int i = 0; i < 3; i++) {
        leftMotor[i].stop(brake);
        rightMotor[2-i].stop(brake);
    }
}

// ======================== WRAPPER FUNCTIONS =================================

void forwardMP(double targetDistance, double breakDistance, double targetHeading,
               double minSpeed, double kp_heading, double ki_heading, double kd_heading,
               double accelHeadingScaling, double decelHeadingScaling,
               double approachHeadingScaling, double maxSpeed) {
    straight(targetDistance, breakDistance, targetHeading, minSpeed,
            kp_heading, ki_heading, kd_heading,
            accelHeadingScaling, decelHeadingScaling,
            approachHeadingScaling, maxSpeed);
}

void backwardMP(double targetDistance, double breakDistance, double targetHeading,
                double minSpeed, double kp_heading, double ki_heading, double kd_heading,
                double accelHeadingScaling, double decelHeadingScaling,
                double approachHeadingScaling, double maxSpeed) {
    // Force negative distance for backward movement
    targetDistance = -std::fabs(targetDistance);
    straight(targetDistance, breakDistance, targetHeading, minSpeed,
            kp_heading, ki_heading, kd_heading,
            accelHeadingScaling, decelHeadingScaling,
            approachHeadingScaling, maxSpeed);
}

void leftMP(double turnAmount, double breakDistance, double minSpeed, double maxSpeed) {
    // Get current rotation and calculate target (CCW = subtract)
    double currentHeading = InertialSensor.rotation(degrees) + headingOffset;
    double targetRotation = currentHeading - turnAmount;
    turn(targetRotation, breakDistance, minSpeed, maxSpeed);
}

void rightMP(double turnAmount, double breakDistance, double minSpeed, double maxSpeed) {
    // Get current rotation and calculate target (CW = add)
    double currentHeading = InertialSensor.rotation(degrees) + headingOffset;
    double targetRotation = currentHeading + turnAmount;
    turn(targetRotation, breakDistance, minSpeed, maxSpeed);
}

void pivotLeftMP(double turnAmount, double breakDistance, double minSpeed, double maxSpeed) {
    // Get current rotation and calculate target (CCW = add)
    double currentHeading = InertialSensor.rotation(degrees) + headingOffset;
    double targetRotation = currentHeading + turnAmount;
    pivotTurn(targetRotation, breakDistance, minSpeed, maxSpeed);
}

void pivotRightMP(double turnAmount, double breakDistance, double minSpeed, double maxSpeed) {
    // Get current rotation and calculate target (CW = subtract)
    double currentHeading = InertialSensor.rotation(degrees) + headingOffset;
    double targetRotation = currentHeading - turnAmount;
    pivotTurn(targetRotation, breakDistance, minSpeed, maxSpeed);
}

// ======================== TRACTION CONTROL ==================================

tractionControl::tractionControl(double minSpeedVoltage, double maxSpeedVoltage, double slipThreshold)
    : minSpeedVoltage(minSpeedVoltage), maxSpeedVoltage(maxSpeedVoltage), slipThreshold(slipThreshold) {}

double tractionControl::tractionControlSpeed(double motorVoltage, double wheelSpeed, 
                                            double robotSpeed, double accelFactor) {
    
    // Calculate slip ratio (wheel speed vs ground speed)
    double slipRatio = calculateSlipRatio(wheelSpeed, robotSpeed);

    // Adjust power based on slip
    if (slipRatio > slipThreshold) {
        motorVoltage = motorVoltage / accelFactor;  // Reduce power when slipping
    } else {
        motorVoltage = motorVoltage * accelFactor;  // Increase power when grip is good
    }

    // Clamp voltage to safe range
    motorVoltage = std::copysign(
        std::max(std::fabs(minSpeedVoltage), 
                std::min(std::fabs(motorVoltage), std::fabs(maxSpeedVoltage))),
        motorVoltage);

    return motorVoltage;
}

// ======================== ABS CONTROLLER ====================================

ABSController::ABSController(double lockThreshold) : ABSLockThreshold(lockThreshold) {}

vex::brakeType ABSController::ABSSpeedReduction(double wheelSpeed, double robotSpeed) {
    // Calculate slip ratio
    double slipRatio = calculateSlipRatio(wheelSpeed, robotSpeed);

    // If wheel is locking up, switch to coast mode
    if (slipRatio > ABSLockThreshold) {
        return vex::coast;
    } else {
        return vex::brake;
    }
}