#include "navigation.hpp"
#include "robot_config.hpp"
#include "utils.hpp"
#include "pid.hpp"
#include <cmath>
#include <algorithm>
#include <atomic>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


inline int32_t voltageToMillivolts(double voltage) {
    return static_cast<int32_t>(voltage * 1000.0);
}

void move(double distanceCM, double maxSpeed, int dir) {
    double targetDegrees = (distanceCM / wheelCircumferenceCM) * 360.0;
    for (int i = 0; i < 3; i++) {
        leftMotor[i]->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        rightMotor[i]->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    }
    double velocity = maxSpeed * 6.0;
    for (int i = 0; i < 3; i++) {
        leftMotor[i]->move_relative(dir * targetDegrees, velocity);
        rightMotor[i]->move_relative(dir * targetDegrees, velocity);
    }
}

void smartMove(double distanceCM, double maxSpeed, int dir, double wallStalledTimeMs) {
    const double WALL_STOP_THRESHOLD_RPM = 5.0;
    uint32_t wallStallStartTime = 0;
    bool wallDetected = false, wallDetectEnabled = (wallStalledTimeMs > 0), isCurrentlyStalled = false;
    passiveEncoderLeft.reset_position();
    passiveEncoderRight.reset_position();
    for (int i = 0; i < 3; i++) {
        leftMotor[i]->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        rightMotor[i]->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    }
    int32_t voltage = voltageToMillivolts(maxSpeed * 0.01 * absoluteMaxVoltage);
    if (dir == -1) voltage = -voltage;
    double currentDistance = 0;
    while (fabs(currentDistance) < fabs(distanceCM) && !wallDetected) {
        double leftDeg = passiveEncoderLeft.get_position() / 100.0;
        double rightDeg = passiveEncoderRight.get_position() / 100.0;
        currentDistance = ((leftDeg + rightDeg) / 2.0 / 360.0) * encoderWheelCircumferenceCM;
        double leftEncoderRPM = fabs(passiveEncoderLeft.get_velocity() / 100.0 / 6.0);
        double rightEncoderRPM = fabs(passiveEncoderRight.get_velocity() / 100.0 / 6.0);
        if (wallDetectEnabled) {
            double avgEncoderSpeed = (leftEncoderRPM + rightEncoderRPM) / 2.0;
            if (avgEncoderSpeed < WALL_STOP_THRESHOLD_RPM) {
                if (!isCurrentlyStalled) { wallStallStartTime = pros::millis(); isCurrentlyStalled = true; }
                else if ((pros::millis() - wallStallStartTime) >= wallStalledTimeMs) { wallDetected = true; }
            } else { isCurrentlyStalled = false; }
        }
        for (int i = 0; i < 3; i++) {
            leftMotor[i]->move_voltage(voltage);
            rightMotor[i]->move_voltage(voltage);
        }
        pros::delay(10);
    }
    for (int i = 0; i < 3; i++) {
        leftMotor[i]->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        rightMotor[i]->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        leftMotor[i]->move_voltage(0);
        rightMotor[i]->move_voltage(0);
    }
}

tractionControl::tractionControl(double minSpeedVoltage, double maxSpeedVoltage, double slipThreshold)
    : minSpeedVoltage(minSpeedVoltage), maxSpeedVoltage(maxSpeedVoltage), slipThreshold(slipThreshold) {}

double tractionControl::tractionControlSpeed(double motorVoltage, double wheelSpeed, double robotSpeed, double accelFactor) {
    double slipRatio = calculateSlipRatio(wheelSpeed, robotSpeed);
    if (slipRatio > slipThreshold) motorVoltage = motorVoltage / accelFactor;
    else motorVoltage = motorVoltage * accelFactor;
    motorVoltage = std::copysign(std::max(std::fabs(minSpeedVoltage), std::min(std::fabs(motorVoltage), std::fabs(maxSpeedVoltage))), motorVoltage);
    return motorVoltage;
}

adaptiveABS::adaptiveABS(double decelStepPercent, double lockThreshold)
    : lockThreshold(lockThreshold), lastAttemptedVoltage(0.0), wasLockedLastCycle(false), currentBrakeMode(pros::E_MOTOR_BRAKE_BRAKE) {
    decelStepVoltage = absoluteMaxVoltage * (decelStepPercent / 100.0);
}

void adaptiveABS::initialize(double startingVoltage) {
    lastAttemptedVoltage = startingVoltage;
    wasLockedLastCycle = false;
    currentBrakeMode = pros::E_MOTOR_BRAKE_BRAKE;
}

double adaptiveABS::decelControlSpeed(double wheelSpeed, double robotSpeed) {
    double lockupRatio = calculateLockupRatio(wheelSpeed, robotSpeed);
    double outputVoltage;
    if (lockupRatio > lockThreshold) {
        outputVoltage = 0.0; currentBrakeMode = pros::E_MOTOR_BRAKE_COAST; wasLockedLastCycle = true;
    } else if (wasLockedLastCycle) {
        outputVoltage = lastAttemptedVoltage; currentBrakeMode = pros::E_MOTOR_BRAKE_BRAKE; wasLockedLastCycle = false;
    } else {
        lastAttemptedVoltage = std::copysign(std::max(0.0, std::fabs(lastAttemptedVoltage) - decelStepVoltage), lastAttemptedVoltage);
        outputVoltage = lastAttemptedVoltage; currentBrakeMode = pros::E_MOTOR_BRAKE_BRAKE;
    }
    return outputVoltage;
}

void turnOdometry(double targetHeading, double breakDistanceInDegrees, double minSpeed, double maxSpeed, double exitTolerance) {
    bool decelCompleted = false, accelCompleted = false, decel = false;
    double currentHeading = inertialSensor.get_rotation() - headingOffset;
    int completeRotations = (int)(currentHeading / 360.0);
    double targetRotationHeading = targetHeading + (completeRotations * 360.0);
    double headingError = targetRotationHeading - currentHeading;
    double maxSpeedVoltage = std::copysign(maxSpeed * 0.01 * absoluteMaxVoltage, headingError);
    double minSpeedVoltage = std::copysign(minSpeed * 0.01 * absoluteMaxVoltage, headingError);
    double minLaunchSpeedVoltage = std::copysign(std::min(fabs(maxSpeedVoltage), 4.0), headingError);
    double minDriveMotorRPM = (minSpeed * .01) * absoluteMaxRPM;
    const double TURN_ACCEL_FACTOR_LAUNCH = 1.5, SLIP_THRESHOLD_TRACTION = 10, DECEL_STEP_PERCENT = 20, LOCK_THRESHOLD_DECEL = 10;
    double motorVoltageLeft[3] = {minLaunchSpeedVoltage, minLaunchSpeedVoltage, minLaunchSpeedVoltage};
    double motorVoltageRight[3] = {minLaunchSpeedVoltage, minLaunchSpeedVoltage, minLaunchSpeedVoltage};
    double leftEncoderRollingAverage = 0, rightEncoderRollingAverage = 0, voltageRollingAverage = 0;
    adaptiveABS adaptiveABSLeft(DECEL_STEP_PERCENT, LOCK_THRESHOLD_DECEL);
    adaptiveABS adaptiveABSRight(DECEL_STEP_PERCENT, LOCK_THRESHOLD_DECEL);
    tractionControl tractionControlLeft(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION);
    tractionControl tractionControlRight(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION);

    while ((maxSpeedVoltage > 0 && currentHeading <= targetRotationHeading - exitTolerance) ||
           (maxSpeedVoltage < 0 && currentHeading >= targetRotationHeading + exitTolerance)) {
        currentHeading = inertialSensor.get_rotation() - headingOffset;
        headingError = targetRotationHeading - currentHeading;
        double leftMotorRPM = fabs(leftMotor[1]->get_actual_velocity()) * DRIVE_MOTOR_RPM_ADJ;
        double rightMotorRPM = fabs(rightMotor[1]->get_actual_velocity()) * DRIVE_MOTOR_RPM_ADJ;
        double leftEncoderRPM = fabs(passiveEncoderLeft.get_velocity() / 100.0 / 6.0) * (encoderWheelCircumferenceCM / wheelCircumferenceCM);
        double rightEncoderRPM = fabs(passiveEncoderRight.get_velocity() / 100.0 / 6.0) * (encoderWheelCircumferenceCM / wheelCircumferenceCM);
        double averageEncoderRPM = (leftEncoderRPM + rightEncoderRPM) / 2.0;

        if ((std::fabs(headingError) > fabs(breakDistanceInDegrees)) && !accelCompleted && !decel) {
            double leftTractionVoltage = tractionControlLeft.tractionControlSpeed(motorVoltageLeft[1], leftMotorRPM, averageEncoderRPM, TURN_ACCEL_FACTOR_LAUNCH);
            double rightTractionVoltage = tractionControlRight.tractionControlSpeed(motorVoltageRight[1], rightMotorRPM, averageEncoderRPM, TURN_ACCEL_FACTOR_LAUNCH);
            double syncedMotorVoltage = std::min(fabs(leftTractionVoltage), fabs(rightTractionVoltage));
            for (int i = 0; i < 3; i++) {
                motorVoltageLeft[i] = std::copysign(syncedMotorVoltage, motorVoltageLeft[i]);
                motorVoltageRight[i] = std::copysign(syncedMotorVoltage, motorVoltageRight[i]);
            }
            double averageMotorVoltage = (fabs(motorVoltageLeft[0]) + fabs(motorVoltageRight[0]) + fabs(motorVoltageLeft[1]) + fabs(motorVoltageRight[1]) + fabs(motorVoltageLeft[2]) + fabs(motorVoltageRight[2])) / numberDriveMotor;
            voltageRollingAverage = rollingAverage(averageMotorVoltage, voltageRollingAverage, 5);
            if (fabs(voltageRollingAverage) >= (fabs(maxSpeedVoltage) - VOLTAGE_TOLERANCE)) accelCompleted = true;
        } else if ((std::abs(headingError) > fabs(breakDistanceInDegrees)) && accelCompleted) {
            for (int i = 0; i < 3; i++) { motorVoltageLeft[i] = maxSpeedVoltage; motorVoltageRight[i] = maxSpeedVoltage; }
        } else if ((std::abs(headingError) <= fabs(breakDistanceInDegrees)) && decelCompleted == false) {
            if (!decel) { adaptiveABSLeft.initialize(motorVoltageLeft[1]); adaptiveABSRight.initialize(motorVoltageRight[1]); }
            decel = true;
            double leftDecelVoltage = adaptiveABSLeft.decelControlSpeed(leftMotorRPM, leftEncoderRPM);
            double rightDecelVoltage = adaptiveABSRight.decelControlSpeed(rightMotorRPM, rightEncoderRPM);
            pros::motor_brake_mode_e_t syncedBrakeMode = (adaptiveABSLeft.getBrakeMode() == pros::E_MOTOR_BRAKE_COAST || adaptiveABSRight.getBrakeMode() == pros::E_MOTOR_BRAKE_COAST) ? pros::E_MOTOR_BRAKE_COAST : pros::E_MOTOR_BRAKE_BRAKE;
            double syncedDecelVoltage = std::max(fabs(leftDecelVoltage), fabs(rightDecelVoltage));
            for (int i = 0; i < 3; i++) {
                leftMotor[i]->set_brake_mode(syncedBrakeMode); rightMotor[i]->set_brake_mode(syncedBrakeMode);
                if (syncedBrakeMode == pros::E_MOTOR_BRAKE_BRAKE && syncedDecelVoltage > 0.0) {
                    motorVoltageLeft[i] = std::copysign(syncedDecelVoltage, motorVoltageLeft[i]);
                    motorVoltageRight[i] = std::copysign(syncedDecelVoltage, motorVoltageRight[i]);
                } else { motorVoltageLeft[i] = 0.0; motorVoltageRight[i] = 0.0; }
            }
            leftEncoderRollingAverage = rollingAverage(leftEncoderRPM, leftEncoderRollingAverage, 3);
            rightEncoderRollingAverage = rollingAverage(rightEncoderRPM, rightEncoderRollingAverage, 3);
            if (fabs(leftEncoderRollingAverage) <= fabs(minDriveMotorRPM) && fabs(rightEncoderRollingAverage) <= fabs(minDriveMotorRPM)) decelCompleted = true;
        } else if (decelCompleted == true) {
            for (int i = 0; i < 3; i++) { motorVoltageLeft[i] = minSpeedVoltage; motorVoltageRight[i] = minSpeedVoltage; }
        }
        for (int i = 0; i < 3; i++) {
            leftMotor[i]->move_voltage(voltageToMillivolts(motorVoltageLeft[i]));
            rightMotor[i]->move_voltage(voltageToMillivolts(-motorVoltageRight[i]));
        }
        pros::delay(10);
    }
    for (int i = 0; i < 3; i++) {
        leftMotor[i]->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE); rightMotor[i]->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        leftMotor[i]->move_voltage(0); rightMotor[i]->move_voltage(0);
    }
}

void straightOdometry(double targetDistance, double breakDistance, double targetHeading, double minSpeed,
                      double kp_heading, double ki_heading, double kd_heading, double accelHeadingScaling,
                      double decelHeadingScaling, double approachHeadingScaling, double maxSpeed) {
    const double LAUNCH_VOLTAGE = 6, ACCEL_FACTOR_LAUNCH = 1.25, SLIP_THRESHOLD_TRACTION = 0.3;
    const double DECEL_STEP_PERCENT = 20, LOCK_THRESHOLD_DECEL = 0.25;
    passiveEncoderLeft.reset_position(); passiveEncoderRight.reset_position();
    PID headingPID(kp_heading, ki_heading, kd_heading); headingPID.pidReset();
    double currentDistance = 0;
    double maxSpeedVoltage = std::copysign(maxSpeed * 0.01 * absoluteMaxVoltage, targetDistance);
    double minSpeedVoltage = std::copysign(minSpeed * 0.01 * absoluteMaxVoltage, targetDistance);
    double minLaunchSpeedVoltage = std::copysign(std::min(fabs(maxSpeedVoltage), fabs(LAUNCH_VOLTAGE)), targetDistance);
    double minDriveMotorRPM = (minSpeed * .01) * absoluteMaxRPM;
    double motorVoltageLeft[3] = {minLaunchSpeedVoltage, minLaunchSpeedVoltage, minLaunchSpeedVoltage};
    double motorVoltageRight[3] = {minLaunchSpeedVoltage, minLaunchSpeedVoltage, minLaunchSpeedVoltage};
    double avgMotorVoltage = 0, leftEncoderRollingAverage = 0, rightEncoderRollingAverage = 0;
    bool decel = false, decelCompleted = false, accelCompleted = false;
    int consecutiveAtTarget = 0; const int REQUIRED_CONSECUTIVE = 3;
    tractionControl tractionControlLeft(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION);
    tractionControl tractionControlRight(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION);
    adaptiveABS adaptiveABSLeft(DECEL_STEP_PERCENT, LOCK_THRESHOLD_DECEL);
    adaptiveABS adaptiveABSRight(DECEL_STEP_PERCENT, LOCK_THRESHOLD_DECEL);

    while (std::fabs(currentDistance) <= fabs(targetDistance) - 6) {
        double leftDeg = passiveEncoderLeft.get_position() / 100.0;
        double rightDeg = passiveEncoderRight.get_position() / 100.0;
        currentDistance = ((leftDeg + rightDeg) / 2.0 / 360.0) * encoderWheelCircumferenceCM;
        avgMotorVoltage = (motorVoltageLeft[0]+motorVoltageLeft[1]+motorVoltageLeft[2]+motorVoltageRight[0]+motorVoltageRight[1]+motorVoltageRight[2]) / numberDriveMotor;
        double currentHeading = inertialSensor.get_rotation() - headingOffset;
        double headingCorrection = headingPID.calculate(targetHeading, currentHeading);
        double leftEncoderRPM = passiveEncoderLeft.get_velocity() / 100.0 / 6.0 * (encoderWheelCircumferenceCM / wheelCircumferenceCM);
        double rightEncoderRPM = passiveEncoderRight.get_velocity() / 100.0 / 6.0 * (encoderWheelCircumferenceCM / wheelCircumferenceCM);
        double leftMotorRPM = leftMotor[1]->get_actual_velocity() * DRIVE_MOTOR_RPM_ADJ;
        double rightMotorRPM = rightMotor[1]->get_actual_velocity() * DRIVE_MOTOR_RPM_ADJ;

        if (fabs(currentDistance) < (fabs(targetDistance) - breakDistance) && !accelCompleted && !decel) {
            double leftTractionVoltage = tractionControlLeft.tractionControlSpeed(motorVoltageLeft[1], leftMotorRPM, leftEncoderRPM, ACCEL_FACTOR_LAUNCH);
            double rightTractionVoltage = tractionControlRight.tractionControlSpeed(motorVoltageRight[1], rightMotorRPM, rightEncoderRPM, ACCEL_FACTOR_LAUNCH);
            double syncedMotorVoltage = (std::fabs(leftTractionVoltage) < std::fabs(rightTractionVoltage)) ? leftTractionVoltage : rightTractionVoltage;
            for (int i = 0; i < 3; i++) {
                motorVoltageLeft[i] = syncedMotorVoltage + (headingCorrection * accelHeadingScaling);
                motorVoltageRight[i] = syncedMotorVoltage - (headingCorrection * accelHeadingScaling);
            }
            if (fabs(avgMotorVoltage) >= (fabs(maxSpeedVoltage) - VOLTAGE_TOLERANCE)) accelCompleted = true;
        } else if (fabs(currentDistance) < (fabs(targetDistance) - breakDistance) && accelCompleted == true) {
            for (int i = 0; i < 3; i++) {
                motorVoltageLeft[i] = maxSpeedVoltage + headingCorrection;
                motorVoltageRight[i] = maxSpeedVoltage - headingCorrection;
            }
        } else if (fabs(currentDistance) >= (fabs(targetDistance) - breakDistance) && decelCompleted == false) {
            if (!decel) { adaptiveABSLeft.initialize(motorVoltageLeft[1]); adaptiveABSRight.initialize(motorVoltageRight[1]); }
            decel = true;
            double leftDecelVoltage = adaptiveABSLeft.decelControlSpeed(leftMotorRPM, leftEncoderRPM);
            double rightDecelVoltage = adaptiveABSRight.decelControlSpeed(rightMotorRPM, rightEncoderRPM);
            pros::motor_brake_mode_e_t syncedBrakeMode = (adaptiveABSLeft.getBrakeMode() == pros::E_MOTOR_BRAKE_COAST || adaptiveABSRight.getBrakeMode() == pros::E_MOTOR_BRAKE_COAST) ? pros::E_MOTOR_BRAKE_COAST : pros::E_MOTOR_BRAKE_BRAKE;
            double syncedDecelVoltage = (std::fabs(leftDecelVoltage) < std::fabs(rightDecelVoltage)) ? leftDecelVoltage : rightDecelVoltage;
            double steeringCorrection = headingCorrection * decelHeadingScaling;
            for (int i = 0; i < 3; i++) {
                if (syncedBrakeMode == pros::E_MOTOR_BRAKE_BRAKE && std::fabs(syncedDecelVoltage) > 0.0) {
                    double correctedLeft = syncedDecelVoltage + steeringCorrection;
                    double correctedRight = syncedDecelVoltage - steeringCorrection;
                    if (syncedDecelVoltage > 0) { motorVoltageLeft[i] = std::max(0.0, correctedLeft); motorVoltageRight[i] = std::max(0.0, correctedRight); }
                    else { motorVoltageLeft[i] = std::min(0.0, correctedLeft); motorVoltageRight[i] = std::min(0.0, correctedRight); }
                } else { motorVoltageLeft[i] = 0.0; motorVoltageRight[i] = 0.0; }
                leftMotor[i]->set_brake_mode(syncedBrakeMode); rightMotor[i]->set_brake_mode(syncedBrakeMode);
            }
            leftEncoderRollingAverage = rollingAverage(leftEncoderRPM, leftEncoderRollingAverage, 10);
            rightEncoderRollingAverage = rollingAverage(rightEncoderRPM, rightEncoderRollingAverage, 10);
            if (fabs(leftEncoderRollingAverage) <= fabs(minDriveMotorRPM) && fabs(rightEncoderRollingAverage) <= fabs(minDriveMotorRPM)) {
                consecutiveAtTarget++; if (consecutiveAtTarget >= REQUIRED_CONSECUTIVE) decelCompleted = true;
            } else consecutiveAtTarget = 0;
        } else if (decelCompleted == true) {
            for (int i = 0; i < 3; i++) {
                motorVoltageLeft[i] = minSpeedVoltage + (headingCorrection * approachHeadingScaling);
                motorVoltageRight[i] = minSpeedVoltage - (headingCorrection * approachHeadingScaling);
            }
        }
        for (int i = 0; i < 3; i++) {
            leftMotor[i]->move_voltage(voltageToMillivolts(motorVoltageLeft[i]));
            rightMotor[i]->move_voltage(voltageToMillivolts(motorVoltageRight[i]));
        }
        pros::delay(10);
    }
    for (int i = 0; i < 3; i++) {
        leftMotor[i]->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE); rightMotor[i]->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        leftMotor[i]->move_voltage(0); rightMotor[i]->move_voltage(0);
    }
}

void forwardMP(double targetDistance, double breakDistance, double targetHeading, double minSpeed,
               double kp_heading, double ki_heading, double kd_heading, double accelHeadingScaling,
               double decelHeadingScaling, double approachHeadingScaling, double maxSpeed) {
    straightOdometry(targetDistance, breakDistance, targetHeading, minSpeed, kp_heading, ki_heading, kd_heading, accelHeadingScaling, decelHeadingScaling, approachHeadingScaling, maxSpeed);
}

void backwardMP(double targetDistance, double breakDistance, double targetHeading, double minSpeed,
                double kp_heading, double ki_heading, double kd_heading, double accelHeadingScaling,
                double decelHeadingScaling, double approachHeadingScaling, double maxSpeed) {
    targetDistance = -std::fabs(targetDistance);
    straightOdometry(targetDistance, breakDistance, targetHeading, minSpeed, kp_heading, ki_heading, kd_heading, accelHeadingScaling, decelHeadingScaling, approachHeadingScaling, maxSpeed);
}

void leftMP(double turnAmount, double breakDistance, double minSpeed, double maxSpeed) {
    double currentHeading = inertialSensor.get_rotation() - headingOffset;
    turnOdometry(currentHeading - turnAmount, breakDistance, minSpeed, maxSpeed);
}

void rightMP(double turnAmount, double breakDistance, double minSpeed, double maxSpeed) {
    double currentHeading = inertialSensor.get_rotation() - headingOffset;
    turnOdometry(currentHeading + turnAmount, breakDistance, minSpeed, maxSpeed);
}

void driveForward(double targetDistance, double breakDistance, double targetHeading, double minSpeed,
                  double kp_heading, double ki_heading, double kd_heading, double accelHeadingScaling,
                  double decelHeadingScaling, double approachHeadingScaling, double maxSpeed) {
    straightOdometry(targetDistance, breakDistance, -targetHeading, minSpeed, kp_heading, ki_heading, kd_heading, accelHeadingScaling, decelHeadingScaling, approachHeadingScaling, maxSpeed);
}

void driveBackward(double targetDistance, double breakDistance, double targetHeading, double minSpeed,
                   double kp_heading, double ki_heading, double kd_heading, double accelHeadingScaling,
                   double decelHeadingScaling, double approachHeadingScaling, double maxSpeed) {
    targetDistance = -std::fabs(targetDistance);
    straightOdometry(targetDistance, breakDistance, -targetHeading, minSpeed, kp_heading, ki_heading, kd_heading, accelHeadingScaling, decelHeadingScaling, approachHeadingScaling, maxSpeed);
}

void turnRight(double absoluteTargetHeading, double breakDistance, double minSpeed, double maxSpeed, double exitTolerance) {
    double currentHeading = inertialSensor.get_rotation() - headingOffset;
    double targetHeading = -absoluteTargetHeading;
    while (targetHeading <= currentHeading) targetHeading += 360.0;
    turnOdometry(targetHeading, breakDistance, minSpeed, maxSpeed, exitTolerance);
}

void turnLeft(double absoluteTargetHeading, double breakDistance, double minSpeed, double maxSpeed, double exitTolerance) {
    double currentHeading = inertialSensor.get_rotation() - headingOffset;
    double targetHeading = -absoluteTargetHeading;
    while (targetHeading >= currentHeading) targetHeading -= 360.0;
    turnOdometry(targetHeading, breakDistance, minSpeed, maxSpeed, exitTolerance);
}

void pidlessForward(double timeMs, double speedPct) {
    uint32_t startTime = pros::millis();
    int32_t voltage = voltageToMillivolts(speedPct / 100.0 * 12.0);
    while ((pros::millis() - startTime) < timeMs) {
        leftMotor1.move_voltage(voltage); leftMotor2.move_voltage(voltage); leftMotor3.move_voltage(voltage);
        rightMotor1.move_voltage(voltage); rightMotor2.move_voltage(voltage); rightMotor3.move_voltage(voltage);
        pros::delay(10);
    }
    for (int i = 0; i < 3; i++) {
        leftMotor[i]->set_brake_mode(pros::E_MOTOR_BRAKE_COAST); rightMotor[i]->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        leftMotor[i]->move_voltage(0); rightMotor[i]->move_voltage(0);
    }
}

void pivotTurnOdometry(double targetHeading, double breakDistanceInDegrees, double minSpeed, double maxSpeed) {
    turnOdometry(targetHeading, breakDistanceInDegrees, minSpeed, maxSpeed);
}

void pivotLeftMP(double turnAmount, double breakDistance, double minSpeed, double maxSpeed) {
    double currentHeading = inertialSensor.get_rotation() - headingOffset;
    pivotTurnOdometry(currentHeading + turnAmount, breakDistance, minSpeed, maxSpeed);
}

void pivotRightMP(double turnAmount, double breakDistance, double minSpeed, double maxSpeed) {
    double currentHeading = inertialSensor.get_rotation() - headingOffset;
    pivotTurnOdometry(currentHeading - turnAmount, breakDistance, minSpeed, maxSpeed);
}

// REPLACE the simplified smartStraight with this full version

void smartStraight(double targetDistance, double breakDistance, double targetHeading, double minSpeed,
                   double wallStalledTimeMs, double kp_heading, double ki_heading, double kd_heading,
                   double accelHeadingScaling, double decelHeadingScaling, double approachHeadingScaling, double maxSpeed) {
    const double LAUNCH_VOLTAGE = 6;
    const double ACCEL_FACTOR_LAUNCH = 1.25;
    const double SLIP_THRESHOLD_TRACTION = 0.3;
    const double DECEL_STEP_PERCENT = 20;
    const double LOCK_THRESHOLD_DECEL = 0.25;
    const double WALL_STOP_THRESHOLD_RPM = 5.0;

    uint32_t wallStallStartTime = 0;
    bool wallDetected = false;
    bool wallDetectEnabled = (wallStalledTimeMs > 0);
    bool isCurrentlyStalled = false;

    passiveEncoderLeft.reset_position();
    passiveEncoderRight.reset_position();

    PID headingPID(kp_heading, ki_heading, kd_heading);
    headingPID.pidReset();
    targetHeading = -targetHeading;

    double currentDistance = 0;
    double maxSpeedVoltage = std::copysign(maxSpeed * 0.01 * absoluteMaxVoltage, targetDistance);
    double minSpeedVoltage = std::copysign(minSpeed * 0.01 * absoluteMaxVoltage, targetDistance);
    double minLaunchSpeedVoltage = std::copysign(std::min(fabs(maxSpeedVoltage), fabs(LAUNCH_VOLTAGE)), targetDistance);
    double minDriveMotorRPM = (minSpeed * .01) * absoluteMaxRPM;

    double motorVoltageLeft[3] = {minLaunchSpeedVoltage, minLaunchSpeedVoltage, minLaunchSpeedVoltage};
    double motorVoltageRight[3] = {minLaunchSpeedVoltage, minLaunchSpeedVoltage, minLaunchSpeedVoltage};

    double avgMotorVoltage = 0;
    double leftEncoderRollingAverage = 0;
    double rightEncoderRollingAverage = 0;

    bool decel = false;
    bool decelCompleted = false;
    bool accelCompleted = false;
    int consecutiveAtTarget = 0;
    const int REQUIRED_CONSECUTIVE = 3;

    tractionControl tractionControlLeft(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION);
    tractionControl tractionControlRight(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION);
    adaptiveABS adaptiveABSLeft(DECEL_STEP_PERCENT, LOCK_THRESHOLD_DECEL);
    adaptiveABS adaptiveABSRight(DECEL_STEP_PERCENT, LOCK_THRESHOLD_DECEL);

    while (std::fabs(currentDistance) <= fabs(targetDistance) - 6.9 && !wallDetected) {
        double leftDeg = passiveEncoderLeft.get_position() / 100.0;
        double rightDeg = passiveEncoderRight.get_position() / 100.0;
        currentDistance = ((leftDeg + rightDeg) / 2.0 / 360.0) * encoderWheelCircumferenceCM;
        avgMotorVoltage = (motorVoltageLeft[0] + motorVoltageLeft[1] + motorVoltageLeft[2] + 
                          motorVoltageRight[0] + motorVoltageRight[1] + motorVoltageRight[2]) / numberDriveMotor;

        double currentHeading = inertialSensor.get_rotation() - headingOffset;
        double headingCorrection = headingPID.calculate(targetHeading, currentHeading);

        double leftEncoderRPM = passiveEncoderLeft.get_velocity() / 100.0 / 6.0 * (encoderWheelCircumferenceCM / wheelCircumferenceCM);
        double rightEncoderRPM = passiveEncoderRight.get_velocity() / 100.0 / 6.0 * (encoderWheelCircumferenceCM / wheelCircumferenceCM);

        double leftMotorRPM = leftMotor[1]->get_actual_velocity() * DRIVE_MOTOR_RPM_ADJ;
        double rightMotorRPM = rightMotor[1]->get_actual_velocity() * DRIVE_MOTOR_RPM_ADJ;

        // WALL DETECTION LOGIC
        if (wallDetectEnabled) {
            double avgEncoderSpeed = (fabs(leftEncoderRPM) + fabs(rightEncoderRPM)) / 2.0;
            if (avgEncoderSpeed < WALL_STOP_THRESHOLD_RPM) {
                if (!isCurrentlyStalled) {
                    wallStallStartTime = pros::millis();
                    isCurrentlyStalled = true;
                } else if ((pros::millis() - wallStallStartTime) >= wallStalledTimeMs) {
                    wallDetected = true;
                }
            } else {
                isCurrentlyStalled = false;
            }
        }

        // Launch Phase
        if (fabs(currentDistance) < (fabs(targetDistance) - breakDistance) && !accelCompleted && !decel) {
            double leftTractionVoltage = tractionControlLeft.tractionControlSpeed(motorVoltageLeft[1], leftMotorRPM, leftEncoderRPM, ACCEL_FACTOR_LAUNCH);
            double rightTractionVoltage = tractionControlRight.tractionControlSpeed(motorVoltageRight[1], rightMotorRPM, rightEncoderRPM, ACCEL_FACTOR_LAUNCH);
            double syncedMotorVoltage = (std::fabs(leftTractionVoltage) < std::fabs(rightTractionVoltage)) ? leftTractionVoltage : rightTractionVoltage;
            for (int i = 0; i < 3; i++) {
                motorVoltageLeft[i] = syncedMotorVoltage + (headingCorrection * accelHeadingScaling);
                motorVoltageRight[i] = syncedMotorVoltage - (headingCorrection * accelHeadingScaling);
            }
            if (fabs(avgMotorVoltage) >= (fabs(maxSpeedVoltage) - VOLTAGE_TOLERANCE)) {
                accelCompleted = true;
            }
        }
        // Cruise Phase
        else if (fabs(currentDistance) < (fabs(targetDistance) - breakDistance) && accelCompleted == true) {
            for (int i = 0; i < 3; i++) {
                motorVoltageLeft[i] = maxSpeedVoltage + headingCorrection;
                motorVoltageRight[i] = maxSpeedVoltage - headingCorrection;
            }
        }
        // Decel Phase
        else if (fabs(currentDistance) >= (fabs(targetDistance) - breakDistance) && decelCompleted == false) {
            if (!decel) {
                adaptiveABSLeft.initialize(motorVoltageLeft[1]);
                adaptiveABSRight.initialize(motorVoltageRight[1]);
            }
            decel = true;

            double leftMotorRPMD = leftMotor[1]->get_actual_velocity() * DRIVE_MOTOR_RPM_ADJ;
            double rightMotorRPMD = rightMotor[1]->get_actual_velocity() * DRIVE_MOTOR_RPM_ADJ;
            double leftEncoderRPMD = passiveEncoderLeft.get_velocity() / 100.0 / 6.0 * (encoderWheelCircumferenceCM / wheelCircumferenceCM);
            double rightEncoderRPMD = passiveEncoderRight.get_velocity() / 100.0 / 6.0 * (encoderWheelCircumferenceCM / wheelCircumferenceCM);

            double leftDecelVoltage = adaptiveABSLeft.decelControlSpeed(leftMotorRPMD, leftEncoderRPMD);
            double rightDecelVoltage = adaptiveABSRight.decelControlSpeed(rightMotorRPMD, rightEncoderRPMD);

            pros::motor_brake_mode_e_t syncedBrakeMode = pros::E_MOTOR_BRAKE_HOLD;
            double syncedDecelVoltage = (std::fabs(leftDecelVoltage) < std::fabs(rightDecelVoltage)) ? leftDecelVoltage : rightDecelVoltage;
            double steeringCorrection = headingCorrection * decelHeadingScaling;

            for (int i = 0; i < 3; i++) {
                if (syncedBrakeMode == pros::E_MOTOR_BRAKE_BRAKE && std::fabs(syncedDecelVoltage) > 0.0) {
                    double correctedLeft = syncedDecelVoltage + steeringCorrection;
                    double correctedRight = syncedDecelVoltage - steeringCorrection;
                    if (syncedDecelVoltage > 0) {
                        motorVoltageLeft[i] = std::max(0.0, correctedLeft);
                        motorVoltageRight[i] = std::max(0.0, correctedRight);
                    } else {
                        motorVoltageLeft[i] = std::min(0.0, correctedLeft);
                        motorVoltageRight[i] = std::min(0.0, correctedRight);
                    }
                } else {
                    motorVoltageLeft[i] = 0.0;
                    motorVoltageRight[i] = 0.0;
                }
                leftMotor[i]->set_brake_mode(syncedBrakeMode);
                rightMotor[i]->set_brake_mode(syncedBrakeMode);
            }

            leftEncoderRollingAverage = rollingAverage(leftEncoderRPMD, leftEncoderRollingAverage, 10);
            rightEncoderRollingAverage = rollingAverage(rightEncoderRPMD, rightEncoderRollingAverage, 10);

            if (fabs(leftEncoderRollingAverage) <= fabs(minDriveMotorRPM) &&
                fabs(rightEncoderRollingAverage) <= fabs(minDriveMotorRPM)) {
                consecutiveAtTarget++;
                if (consecutiveAtTarget >= REQUIRED_CONSECUTIVE) {
                    decelCompleted = true;
                }
            } else {
                consecutiveAtTarget = 0;
            }
        }
        // Final Approach Phase
        else if (decelCompleted == true) {
            for (int i = 0; i < 3; i++) {
                motorVoltageLeft[i] = minSpeedVoltage + (headingCorrection * approachHeadingScaling);
                motorVoltageRight[i] = minSpeedVoltage - (headingCorrection * approachHeadingScaling);
            }
        }

        // Power Drive Motors
        for (int i = 0; i < 3; i++) {
            leftMotor[i]->move_voltage(voltageToMillivolts(motorVoltageLeft[i]));
            rightMotor[i]->move_voltage(voltageToMillivolts(motorVoltageRight[i]));
        }

        pros::delay(10);
    }

    // Stop all motors
    for (int i = 0; i < 3; i++) {
        leftMotor[i]->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        rightMotor[i]->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        leftMotor[i]->move_voltage(0);
        rightMotor[i]->move_voltage(0);
    }

    // Display summary
    pros::lcd::clear();
    if (wallDetected) {
        pros::lcd::print(0, "=== WALL DETECTED ===");
    } else {
        pros::lcd::print(0, "=== Movement Complete ===");
    }
    pros::lcd::print(1, "Distance: %.1f / %.1f", fabs(currentDistance), fabs(targetDistance));
}