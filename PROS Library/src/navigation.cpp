#include "navigation.hpp"
#include "robot_config.hpp"
#include "utils.hpp"
#include "pid.hpp"
#include "main.h" 
#include <cmath>
#include <algorithm>
#include <iterator>
#include <cstring>
#include <atomic>
#include "odometry.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Helper function to convert PROS centidegrees to degrees
inline double getEncoderPositionDeg(pros::Rotation &encoder) {
    return encoder.get_position() / 100.0;
}

// Helper function to convert PROS Rotation velocity (cdeg/s) to RPM
inline double getEncoderVelocityRPM(pros::Rotation &encoder) {
    return encoder.get_velocity() / 6.0;
}

// Function to move the six wheel motors based on a given distance
void move(double distanceCM, double maxSpeed, int dir) 
{
    double targetRotations = distanceCM / wheelCircumferenceCM;
    
    // Convert to degrees for PROS (rotations * 360)
    // Map maxSpeed (0-100 pct) to RPM (0-600)
    double targetDegrees = targetRotations * 360.0 * (dir == 1 ? 1 : -1); 
    double velocityRPM = (maxSpeed / 100.0) * absoluteMaxRPM;

    // Use MotorGroups for uniform operations
    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    // Non-blocking relative move
    leftMotors.move_relative(targetDegrees, velocityRPM);
    rightMotors.move_relative(targetDegrees, velocityRPM);
}

void smartMove(double distanceCM, double maxSpeed, int dir, double wallStalledTimeMs)
{
    const double WALL_STOP_THRESHOLD_RPM = 5.0;
    uint32_t wallStallStartTime = 0; 
    bool wallDetected = false;
    bool wallDetectEnabled = (wallStalledTimeMs > 0);
    bool isCurrentlyStalled = false;

    passiveEncoderLeft.reset_position();
    passiveEncoderRight.reset_position();

    // Use MotorGroups for uniform settings
    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    double voltage = maxSpeed * 0.01 * absoluteMaxVoltage; 
    if (dir == -1) voltage = -voltage;
    
    // PROS uses millivolts
    double mvVoltage = voltage * 1000.0;

    double currentDistance = 0;
    while (fabs(currentDistance) < fabs(distanceCM) && !wallDetected)
    {
        double leftDeg = getEncoderPositionDeg(passiveEncoderLeft);
        double rightDeg = getEncoderPositionDeg(passiveEncoderRight);
        currentDistance = ((leftDeg + rightDeg) / 2.0 / 360.0) * encoderWheelCircumferenceCM;

        double leftEncoderRPM = fabs(getEncoderVelocityRPM(passiveEncoderLeft));
        double rightEncoderRPM = fabs(getEncoderVelocityRPM(passiveEncoderRight));

        // Stall Detection
        if (wallDetectEnabled)
        {
            double avgEncoderSpeed = (leftEncoderRPM + rightEncoderRPM) / 2.0;
            if (avgEncoderSpeed < WALL_STOP_THRESHOLD_RPM)
            {
                if (!isCurrentlyStalled) {
                    wallStallStartTime = pros::millis();
                    isCurrentlyStalled = true;
                } else if ((pros::millis() - wallStallStartTime) >= wallStalledTimeMs) {
                    wallDetected = true;
                }
            }
            else {
                isCurrentlyStalled = false;
            }
        }

        // Use MotorGroups for movement
        leftMotors.move_voltage(mvVoltage);
        rightMotors.move_voltage(mvVoltage);

        pros::delay(10);
    }

    // Use MotorGroups for stop
    leftMotors.brake();
    rightMotors.brake();
}

void turnOdometry(double targetHeading, double breakDistanceInDegrees, double minSpeed, double maxSpeed, double exitTolerance)
{
    bool decelCompleted = false;
    bool accelCompleted = false;
    bool decel = false;

    double currentHeading = inertialSensor.get_rotation() - headingOffset;
    int completeRotations = (int)(currentHeading / 360.0);
    double targetRotationHeading = targetHeading + (completeRotations * 360.0);
    double headingError = targetRotationHeading - currentHeading;
    
    pros::lcd::print(4, "Target Head: %.2f", targetHeading);
    pros::lcd::print(5, "Curr Rotation: %.2f", currentHeading);

    double maxSpeedVoltage = std::copysign(maxSpeed * 0.01 * absoluteMaxVoltage, headingError);
    double minSpeedVoltage = std::copysign(minSpeed * 0.01 * absoluteMaxVoltage, headingError);
    double launchVoltage = std::copysign(4, headingError);
    double minLaunchSpeedVoltage = std::copysign(std::min(fabs(maxSpeedVoltage), fabs(launchVoltage)), headingError);

    double minDriveMotorRPM = (minSpeed * .01) * absoluteMaxRPM;

    const double TURN_ACCEL_FACTOR_LAUNCH = 1.5;
    const double SLIP_THRESHOLD_TRACTION = 10; 
    const double DECEL_STEP_PERCENT = 20;     
    const double LOCK_THRESHOLD_DECEL = 10;

    double averageMotorVoltage = 0;
    
    // We use scalars instead of arrays because turnOdometry applies uniform voltage to the side
    double currentVoltageLeft = minLaunchSpeedVoltage;
    double currentVoltageRight = minLaunchSpeedVoltage;

    double leftEncoderRollingAverage = 0;
    double rightEncoderRollingAverage = 0;
    double voltageRollingAverage = 0;

    adaptiveABS adaptiveABSLeft(DECEL_STEP_PERCENT, LOCK_THRESHOLD_DECEL);
    adaptiveABS adaptiveABSRight(DECEL_STEP_PERCENT, LOCK_THRESHOLD_DECEL);

    tractionControl tractionControlLeft(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION);
    tractionControl tractionControlRight(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION);

  while ((maxSpeedVoltage > 0 && currentHeading <= targetRotationHeading - exitTolerance) ||
               (maxSpeedVoltage < 0 && currentHeading >= targetRotationHeading + exitTolerance))
    {
        currentHeading = inertialSensor.get_rotation() - headingOffset;
        headingError = targetRotationHeading - currentHeading;

        pros::lcd::print(5, "Curr Rotation: %.2f", currentHeading);
        pros::lcd::print(7, "Target: %.2f", targetHeading);

        double leftMotorRPM = fabs(leftMotor2.get_actual_velocity()) * DRIVE_MOTOR_RPM_ADJ;
        double rightMotorRPM = fabs(rightMotor2.get_actual_velocity()) * DRIVE_MOTOR_RPM_ADJ;

        double leftEncoderRPM = fabs(getEncoderVelocityRPM(passiveEncoderLeft)) * (encoderWheelCircumferenceCM / wheelCircumferenceCM);
        double rightEncoderRPM = fabs(getEncoderVelocityRPM(passiveEncoderRight)) * (encoderWheelCircumferenceCM / wheelCircumferenceCM);
        double averageEncoderRPM = (leftEncoderRPM + rightEncoderRPM) / 2.0;

        // Launch Phase
        if ((std::fabs(headingError) > fabs(breakDistanceInDegrees)) && !accelCompleted && !decel)
        {
            double leftTractionVoltage = tractionControlLeft.tractionControlSpeed(currentVoltageLeft, leftMotorRPM, averageEncoderRPM, TURN_ACCEL_FACTOR_LAUNCH);
            double rightTractionVoltage = tractionControlRight.tractionControlSpeed(currentVoltageRight, rightMotorRPM, averageEncoderRPM, TURN_ACCEL_FACTOR_LAUNCH);

            // Sync voltages to the conservative one
            double syncedMotorVoltage = std::min(fabs(leftTractionVoltage), fabs(rightTractionVoltage));
            currentVoltageLeft = std::copysign(syncedMotorVoltage, currentVoltageLeft);
            currentVoltageRight = std::copysign(syncedMotorVoltage, currentVoltageRight);

            averageMotorVoltage = (fabs(currentVoltageLeft) * 3 + fabs(currentVoltageRight) * 3) / numberDriveMotor;
            voltageRollingAverage = rollingAverage(averageMotorVoltage, voltageRollingAverage, 5);

            if (fabs(voltageRollingAverage) >= (fabs(maxSpeedVoltage) - VOLTAGE_TOLERANCE)) {
                accelCompleted = true;
                pros::lcd::print(1, "Launch Phase");
            }
        }
        // Cruise Phase
        else if ((std::abs(headingError) > fabs(breakDistanceInDegrees)) && accelCompleted)
        {
            pros::lcd::print(1, "Cruise Phase");
            currentVoltageLeft = maxSpeedVoltage;
            currentVoltageRight = maxSpeedVoltage;
        }
        // Decel Phase
        else if ((std::abs(headingError) <= fabs(breakDistanceInDegrees)) && decelCompleted == false)
        {
            if (!decel) {
                adaptiveABSLeft.initialize(currentVoltageLeft);
                adaptiveABSRight.initialize(currentVoltageRight);
            }
            pros::lcd::print(1, "Decel Phase");
            decel = true;

            double leftMotorRPMDecel = fabs(leftMotor2.get_actual_velocity()) * DRIVE_MOTOR_RPM_ADJ;
            double rightMotorRPMDecel = fabs(rightMotor2.get_actual_velocity()) * DRIVE_MOTOR_RPM_ADJ;
            
            double leftEncoderRPMDecel = fabs(getEncoderVelocityRPM(passiveEncoderLeft)) * (encoderWheelCircumferenceCM / wheelCircumferenceCM);
            double rightEncoderRPMDecel = fabs(getEncoderVelocityRPM(passiveEncoderRight)) * (encoderWheelCircumferenceCM / wheelCircumferenceCM);

            double leftDecelVoltage = adaptiveABSLeft.decelControlSpeed(leftMotorRPMDecel, leftEncoderRPMDecel);
            double rightDecelVoltage = adaptiveABSRight.decelControlSpeed(rightMotorRPMDecel, rightEncoderRPMDecel);

            pros::motor_brake_mode_e_t leftBrakeMode = adaptiveABSLeft.getBrakeMode();
            pros::motor_brake_mode_e_t rightBrakeMode = adaptiveABSRight.getBrakeMode();

            pros::motor_brake_mode_e_t syncedBrakeMode;
            if (leftBrakeMode == pros::E_MOTOR_BRAKE_COAST || rightBrakeMode == pros::E_MOTOR_BRAKE_COAST) {
                syncedBrakeMode = pros::E_MOTOR_BRAKE_COAST;
            } else {
                syncedBrakeMode = pros::E_MOTOR_BRAKE_BRAKE;
            }

            double syncedDecelVoltage = std::max(fabs(leftDecelVoltage), fabs(rightDecelVoltage));

            // Use Groups for braking
            leftMotors.set_brake_mode(syncedBrakeMode);
            rightMotors.set_brake_mode(syncedBrakeMode);

            if (syncedBrakeMode == pros::E_MOTOR_BRAKE_BRAKE && syncedDecelVoltage > 0.0) {
                currentVoltageLeft = std::copysign(syncedDecelVoltage, currentVoltageLeft);
                currentVoltageRight = std::copysign(syncedDecelVoltage, currentVoltageRight);
            } else {
                currentVoltageLeft = 0.0;
                currentVoltageRight = 0.0;
            }

            leftEncoderRollingAverage = rollingAverage(leftEncoderRPMDecel, leftEncoderRollingAverage, 3);
            rightEncoderRollingAverage = rollingAverage(rightEncoderRPMDecel, rightEncoderRollingAverage, 3);

            if (fabs(leftEncoderRollingAverage) <= fabs(minDriveMotorRPM) &&
                fabs(rightEncoderRollingAverage) <= fabs(minDriveMotorRPM)) {
                decelCompleted = true;
            }
        }
        // Approach Phase
        else if (decelCompleted == true)
        {
            pros::lcd::print(1, "Approach Phase");
            currentVoltageLeft = minSpeedVoltage;
            currentVoltageRight = minSpeedVoltage;
        }

        // Apply voltages using Groups
        leftMotors.move_voltage(currentVoltageLeft * 1000.0);
        // Invert right side manually for the turn since the voltages have signs relative to rotation
        rightMotors.move_voltage(-currentVoltageRight * 1000.0);

        pros::delay(10);
    }

    // Use MotorGroups for final stop
    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    leftMotors.brake();
    rightMotors.brake();
}

tractionControl::tractionControl(double minSpeedVoltage, double maxSpeedVoltage, double slipThreshold)
    : minSpeedVoltage(minSpeedVoltage), maxSpeedVoltage(maxSpeedVoltage), slipThreshold(slipThreshold) {}

double tractionControl::tractionControlSpeed(double motorVoltage, double wheelSpeed, double robotSpeed, double accelFactor)
{
    double slipRatio = calculateSlipRatio(wheelSpeed, robotSpeed); 
    if (slipRatio > slipThreshold) motorVoltage = motorVoltage / accelFactor; 
    else motorVoltage = motorVoltage * accelFactor; 

    motorVoltage = std::copysign(std::max(std::fabs(minSpeedVoltage), std::min(std::fabs(motorVoltage), std::fabs(maxSpeedVoltage))), motorVoltage);
    return motorVoltage;
}

adaptiveABS::adaptiveABS(double decelStepPercent, double lockThreshold)
    : lockThreshold(lockThreshold), lastAttemptedVoltage(0.0), wasLockedLastCycle(false), 
      currentBrakeMode(pros::E_MOTOR_BRAKE_BRAKE)
{
    decelStepVoltage = absoluteMaxVoltage * (decelStepPercent / 100.0);
}

void adaptiveABS::initialize(double startingVoltage)
{
    lastAttemptedVoltage = startingVoltage;
    wasLockedLastCycle = false;
    currentBrakeMode = pros::E_MOTOR_BRAKE_BRAKE;
}

double adaptiveABS::decelControlSpeed(double wheelSpeed, double robotSpeed)
{
    double lockupRatio = calculateLockupRatio(wheelSpeed, robotSpeed);
    double outputVoltage;
    
    if (lockupRatio > lockThreshold) {
        outputVoltage = 0.0;
        currentBrakeMode = pros::E_MOTOR_BRAKE_COAST;
        wasLockedLastCycle = true;
    } else if (wasLockedLastCycle) {
        outputVoltage = lastAttemptedVoltage;
        currentBrakeMode = pros::E_MOTOR_BRAKE_BRAKE;
        wasLockedLastCycle = false;
    } else {
        lastAttemptedVoltage = std::copysign(std::max(0.0, std::fabs(lastAttemptedVoltage) - decelStepVoltage), lastAttemptedVoltage);
        outputVoltage = lastAttemptedVoltage;
        currentBrakeMode = pros::E_MOTOR_BRAKE_BRAKE;
    }
    return outputVoltage;
}

void arcTurn(double targetDistance, double breakDistance, double minSpeed, double maxSpeed, double turnRadius, bool turnLeft)
{ 
    double innerRatio = (turnRadius - (TRACK_WIDTH / 2)) / turnRadius;
    double outerRatio = (turnRadius + (TRACK_WIDTH / 2)) / turnRadius;

    passiveEncoderLeft.reset_position();
    passiveEncoderRight.reset_position();

    double currentDistance = 0;
    double maxVoltage = maxSpeed * 0.01 * absoluteMaxVoltage;
    double minVoltage = minSpeed * 0.01 * absoluteMaxVoltage;
    double innerVoltage = minVoltage * innerRatio;
    double outerVoltage = minVoltage * outerRatio;
    double innerMaxVoltage = maxVoltage * innerRatio;
    double outerMaxVoltage = maxVoltage * outerRatio;

    while (std::fabs(currentDistance) <= fabs(targetDistance))
    {
        double leftDeg = getEncoderPositionDeg(passiveEncoderLeft);
        double rightDeg = getEncoderPositionDeg(passiveEncoderRight);
        currentDistance = ((leftDeg + rightDeg) / 2.0 / 360.0) * encoderWheelCircumferenceCM;

        if (fabs(currentDistance) < (fabs(targetDistance) - breakDistance)) {
            innerVoltage = std::min(innerMaxVoltage, innerVoltage + 0.1);
            outerVoltage = std::min(outerMaxVoltage, outerVoltage + 0.1);
        } else {
            innerVoltage = std::max(minVoltage * innerRatio, innerVoltage - 0.1);
            outerVoltage = std::max(minVoltage * outerRatio, outerVoltage - 0.1);
        }

        if (turnLeft) {
            leftMotors.move_voltage(innerVoltage * 1000.0);
            rightMotors.move_voltage(outerVoltage * 1000.0);
        } else {
            leftMotors.move_voltage(outerVoltage * 1000.0);
            rightMotors.move_voltage(innerVoltage * 1000.0);
        }
        pros::delay(20);
    }

    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    leftMotors.brake();
    rightMotors.brake();
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
    const double LAUNCH_VOLTAGE = 6;         
    const double ACCEL_FACTOR_LAUNCH = 1.25; 
    const double SLIP_THRESHOLD_TRACTION = 0.3; 
    const double DECEL_STEP_PERCENT = 20;    
    const double LOCK_THRESHOLD_DECEL = 0.25; 

    passiveEncoderLeft.reset_position();
    passiveEncoderRight.reset_position();

    PID headingPID(kp_heading, ki_heading, kd_heading);
    headingPID.pidReset();

    double currentDistance = 0;
    double maxSpeedVoltage = std::copysign(maxSpeed * 0.01 * absoluteMaxVoltage, targetDistance);
    double minSpeedVoltage = std::copysign(minSpeed * 0.01 * absoluteMaxVoltage, targetDistance);
    double launchVoltage = std::copysign(LAUNCH_VOLTAGE, targetDistance);
    double minLaunchSpeedVoltage = std::copysign(std::min(fabs(maxSpeedVoltage), fabs(LAUNCH_VOLTAGE)), targetDistance);

    double minDriveMotorRPM = (minSpeed * .01) * absoluteMaxRPM;
    double maxEncoderRPM = 0;
    
    // Using synced voltages for whole side
    double currentVoltageLeft = minLaunchSpeedVoltage;
    double currentVoltageRight = minLaunchSpeedVoltage;

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

    while (std::fabs(currentDistance) <= fabs(targetDistance) - 6)
    {
        double leftDeg = getEncoderPositionDeg(passiveEncoderLeft);
        double rightDeg = getEncoderPositionDeg(passiveEncoderRight);
        currentDistance = ((leftDeg + rightDeg) / 2.0 / 360.0) * encoderWheelCircumferenceCM;
        avgMotorVoltage = (fabs(currentVoltageLeft) * 3 + fabs(currentVoltageRight) * 3) / numberDriveMotor;

        pros::lcd::print(1, "Current: %.2f cm", std::fabs(currentDistance));
        pros::lcd::print(2, "Target: %.2f cm", std::fabs(targetDistance));

        double currentHeading = inertialSensor.get_rotation() - headingOffset;
        double headingCorrection = headingPID.calculate(targetHeading, currentHeading);

        double leftEncoderRPM = getEncoderVelocityRPM(passiveEncoderLeft) * (encoderWheelCircumferenceCM / wheelCircumferenceCM);
        double rightEncoderRPM = getEncoderVelocityRPM(passiveEncoderRight) * (encoderWheelCircumferenceCM / wheelCircumferenceCM);
        double avgEncoderRPM = (leftEncoderRPM + rightEncoderRPM) / 2;

        double leftMotorRPM = leftMotor2.get_actual_velocity() * DRIVE_MOTOR_RPM_ADJ;
        double rightMotorRPM = rightMotor2.get_actual_velocity() * DRIVE_MOTOR_RPM_ADJ;

        if (fabs(currentDistance) < (fabs(targetDistance) - breakDistance) && !accelCompleted && !decel)
        { 
            double leftTractionVoltage = tractionControlLeft.tractionControlSpeed(currentVoltageLeft, leftMotorRPM, leftEncoderRPM, ACCEL_FACTOR_LAUNCH);
            double rightTractionVoltage = tractionControlRight.tractionControlSpeed(currentVoltageRight, rightMotorRPM, rightEncoderRPM, ACCEL_FACTOR_LAUNCH);

            double syncedMotorVoltage = (std::fabs(leftTractionVoltage) < std::fabs(rightTractionVoltage)) 
                ? leftTractionVoltage : rightTractionVoltage;

            currentVoltageLeft = syncedMotorVoltage + (headingCorrection * accelHeadingScaling);
            currentVoltageRight = syncedMotorVoltage - (headingCorrection * accelHeadingScaling);

            maxEncoderRPM = std::max(maxEncoderRPM, fabs(avgEncoderRPM));
            if (fabs(avgMotorVoltage) >= (fabs(maxSpeedVoltage) - VOLTAGE_TOLERANCE)) accelCompleted = true;
        }
        else if (fabs(currentDistance) < (fabs(targetDistance) - breakDistance) && accelCompleted == true)
        {
            currentVoltageLeft = maxSpeedVoltage + (headingCorrection);
            currentVoltageRight = maxSpeedVoltage - (headingCorrection);
        }
        else if (fabs(currentDistance) >= (fabs(targetDistance) - breakDistance) && decelCompleted == false)
        {
            if (!decel) {
                adaptiveABSLeft.initialize(currentVoltageLeft);
                adaptiveABSRight.initialize(currentVoltageRight);
            }
            decel = true;

            double leftDecelVoltage = adaptiveABSLeft.decelControlSpeed(leftMotorRPM, leftEncoderRPM);
            double rightDecelVoltage = adaptiveABSRight.decelControlSpeed(rightMotorRPM, rightEncoderRPM);

            pros::motor_brake_mode_e_t leftBrakeMode = adaptiveABSLeft.getBrakeMode();
            pros::motor_brake_mode_e_t rightBrakeMode = adaptiveABSRight.getBrakeMode();

            pros::motor_brake_mode_e_t syncedBrakeMode;
            if (leftBrakeMode == pros::E_MOTOR_BRAKE_COAST || rightBrakeMode == pros::E_MOTOR_BRAKE_COAST) {
                syncedBrakeMode = pros::E_MOTOR_BRAKE_COAST;
            } else {
                syncedBrakeMode = pros::E_MOTOR_BRAKE_BRAKE;
            }

            double syncedDecelVoltage = (std::fabs(leftDecelVoltage) < std::fabs(rightDecelVoltage))
                ? leftDecelVoltage : rightDecelVoltage;

            double steeringCorrection = headingCorrection * decelHeadingScaling;

            if (syncedBrakeMode == pros::E_MOTOR_BRAKE_BRAKE && std::fabs(syncedDecelVoltage) > 0.0) {
                double correctedLeft = syncedDecelVoltage + steeringCorrection;
                double correctedRight = syncedDecelVoltage - steeringCorrection;
                
                if (syncedDecelVoltage > 0) {
                    currentVoltageLeft = std::max(0.0, correctedLeft);
                    currentVoltageRight = std::max(0.0, correctedRight);
                } else {
                    currentVoltageLeft = std::min(0.0, correctedLeft);
                    currentVoltageRight = std::min(0.0, correctedRight);
                }
            } else {
                currentVoltageLeft = 0.0;
                currentVoltageRight = 0.0;
            }
            
            leftMotors.set_brake_mode(syncedBrakeMode);
            rightMotors.set_brake_mode(syncedBrakeMode);

            leftEncoderRollingAverage = rollingAverage(leftEncoderRPM, leftEncoderRollingAverage, 10);
            rightEncoderRollingAverage = rollingAverage(rightEncoderRPM, rightEncoderRollingAverage, 10);

            if (fabs(leftEncoderRollingAverage) <= fabs(minDriveMotorRPM) &&
                fabs(rightEncoderRollingAverage) <= fabs(minDriveMotorRPM)) {
                decelCompleted = true;
            }
        }
        else if (decelCompleted == true)
        {
            currentVoltageLeft = minSpeedVoltage + (headingCorrection * approachHeadingScaling);
            currentVoltageRight = minSpeedVoltage - (headingCorrection * approachHeadingScaling);
        }

        leftMotors.move_voltage(currentVoltageLeft * 1000.0);
        rightMotors.move_voltage(currentVoltageRight * 1000.0);
        pros::delay(10);
    }

    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    leftMotors.brake();
    rightMotors.brake();
}

void straightOdometryV2(double targetDistance, double breakDistance, double targetHeading, double minSpeed, 
                      double distanceTolerance, double kp_heading, double ki_heading, double kd_heading, 
                      double accelHeadingScaling, double decelHeadingScaling, double approachHeadingScaling, double maxSpeed)
{
    const double LAUNCH_VOLTAGE = 6;
    const double ACCEL_FACTOR_LAUNCH = 1.25;
    const double SLIP_THRESHOLD_TRACTION = 0.3;
    const double DECEL_STEP_PERCENT = 20;
    const double LOCK_THRESHOLD_DECEL = 0.25;

    passiveEncoderLeft.reset_position();
    passiveEncoderRight.reset_position();

    PID headingPID(kp_heading, ki_heading, kd_heading);
    headingPID.pidReset();

    double currentDistance = 0;
    double maxSpeedVoltage = std::copysign(maxSpeed * 0.01 * absoluteMaxVoltage, targetDistance);
    double minSpeedVoltage = std::copysign(minSpeed * 0.01 * absoluteMaxVoltage, targetDistance);
    double launchVoltage = std::copysign(LAUNCH_VOLTAGE, targetDistance);
    double minLaunchSpeedVoltage = std::copysign(std::min(fabs(maxSpeedVoltage), fabs(LAUNCH_VOLTAGE)), targetDistance);

    double minDriveMotorRPM = (minSpeed * .01) * absoluteMaxRPM;
    double maxEncoderRPM = 0;

    double currentVoltageLeft = minLaunchSpeedVoltage;
    double currentVoltageRight = minLaunchSpeedVoltage;

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

    while (std::fabs(currentDistance) <= fabs(targetDistance) - distanceTolerance)
    {
        double leftDeg = getEncoderPositionDeg(passiveEncoderLeft);
        double rightDeg = getEncoderPositionDeg(passiveEncoderRight);
        currentDistance = ((leftDeg + rightDeg) / 2.0 / 360.0) * encoderWheelCircumferenceCM;
        avgMotorVoltage = (fabs(currentVoltageLeft) * 3 + fabs(currentVoltageRight) * 3) / numberDriveMotor;

        pros::lcd::print(1, "Current: %.2f cm", std::fabs(currentDistance));
        pros::lcd::print(2, "Target: %.2f cm", std::fabs(targetDistance));

        double currentHeading = inertialSensor.get_rotation() - headingOffset;
        double headingCorrection = headingPID.calculate(targetHeading, currentHeading);

        double leftEncoderRPM = getEncoderVelocityRPM(passiveEncoderLeft) * (encoderWheelCircumferenceCM / wheelCircumferenceCM);
        double rightEncoderRPM = getEncoderVelocityRPM(passiveEncoderRight) * (encoderWheelCircumferenceCM / wheelCircumferenceCM);
        double avgEncoderRPM = (leftEncoderRPM + rightEncoderRPM) / 2;

        double leftMotorRPM = leftMotor2.get_actual_velocity() * DRIVE_MOTOR_RPM_ADJ;
        double rightMotorRPM = rightMotor2.get_actual_velocity() * DRIVE_MOTOR_RPM_ADJ;

        if (fabs(currentDistance) < (fabs(targetDistance) - breakDistance) && !accelCompleted && !decel)
        {
            double leftTractionVoltage = tractionControlLeft.tractionControlSpeed(currentVoltageLeft, leftMotorRPM, leftEncoderRPM, ACCEL_FACTOR_LAUNCH);
            double rightTractionVoltage = tractionControlRight.tractionControlSpeed(currentVoltageRight, rightMotorRPM, rightEncoderRPM, ACCEL_FACTOR_LAUNCH);

            double syncedMotorVoltage = (std::fabs(leftTractionVoltage) < std::fabs(rightTractionVoltage)) 
                ? leftTractionVoltage : rightTractionVoltage;

            currentVoltageLeft = syncedMotorVoltage + (headingCorrection * accelHeadingScaling);
            currentVoltageRight = syncedMotorVoltage - (headingCorrection * accelHeadingScaling);

            maxEncoderRPM = std::max(maxEncoderRPM, fabs(avgEncoderRPM));
            if (fabs(avgMotorVoltage) >= (fabs(maxSpeedVoltage) - VOLTAGE_TOLERANCE)) accelCompleted = true;
        }
        else if (fabs(currentDistance) < (fabs(targetDistance) - breakDistance) && accelCompleted == true)
        {
            currentVoltageLeft = maxSpeedVoltage + (headingCorrection);
            currentVoltageRight = maxSpeedVoltage - (headingCorrection);
        }
        else if (fabs(currentDistance) >= (fabs(targetDistance) - breakDistance) && decelCompleted == false)
        {
            if (!decel) {
                adaptiveABSLeft.initialize(currentVoltageLeft);
                adaptiveABSRight.initialize(currentVoltageRight);
            }
            decel = true;

            double leftDecelVoltage = adaptiveABSLeft.decelControlSpeed(leftMotorRPM, leftEncoderRPM);
            double rightDecelVoltage = adaptiveABSRight.decelControlSpeed(rightMotorRPM, rightEncoderRPM);

            pros::motor_brake_mode_e_t leftBrakeMode = adaptiveABSLeft.getBrakeMode();
            pros::motor_brake_mode_e_t rightBrakeMode = adaptiveABSRight.getBrakeMode();

            pros::motor_brake_mode_e_t syncedBrakeMode;
            if (leftBrakeMode == pros::E_MOTOR_BRAKE_COAST || rightBrakeMode == pros::E_MOTOR_BRAKE_COAST) {
                syncedBrakeMode = pros::E_MOTOR_BRAKE_COAST;
            } else {
                syncedBrakeMode = pros::E_MOTOR_BRAKE_BRAKE;
            }

            double syncedDecelVoltage = (std::fabs(leftDecelVoltage) < std::fabs(rightDecelVoltage))
                ? leftDecelVoltage : rightDecelVoltage;

            double steeringCorrection = headingCorrection * decelHeadingScaling;

            if (syncedBrakeMode == pros::E_MOTOR_BRAKE_BRAKE && std::fabs(syncedDecelVoltage) > 0.0) {
                double correctedLeft = syncedDecelVoltage + steeringCorrection;
                double correctedRight = syncedDecelVoltage - steeringCorrection;
                
                if (syncedDecelVoltage > 0) {
                    currentVoltageLeft = std::max(0.0, correctedLeft);
                    currentVoltageRight = std::max(0.0, correctedRight);
                } else {
                    currentVoltageLeft = std::min(0.0, correctedLeft);
                    currentVoltageRight = std::min(0.0, correctedRight);
                }
            } else {
                currentVoltageLeft = 0.0;
                currentVoltageRight = 0.0;
            }
            
            leftMotors.set_brake_mode(syncedBrakeMode);
            rightMotors.set_brake_mode(syncedBrakeMode);

            leftEncoderRollingAverage = rollingAverage(leftEncoderRPM, leftEncoderRollingAverage, 10);
            rightEncoderRollingAverage = rollingAverage(rightEncoderRPM, rightEncoderRollingAverage, 10);

            if (fabs(leftEncoderRollingAverage) <= fabs(minDriveMotorRPM) &&
                fabs(rightEncoderRollingAverage) <= fabs(minDriveMotorRPM)) {
                decelCompleted = true;
            }
        }
        else if (decelCompleted == true)
        {
            currentVoltageLeft = minSpeedVoltage + (headingCorrection * approachHeadingScaling);
            currentVoltageRight = minSpeedVoltage - (headingCorrection * approachHeadingScaling);
        }

        leftMotors.move_voltage(currentVoltageLeft * 1000.0);
        rightMotors.move_voltage(currentVoltageRight * 1000.0);
        pros::delay(10);
    }

    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    leftMotors.brake();
    rightMotors.brake();
}

void smartStraight(double targetDistance, double breakDistance, double targetHeading, double minSpeed, 
                   double wallStalledTimeMs, double kp_heading, double ki_heading, double kd_heading, 
                   double accelHeadingScaling, double decelHeadingScaling, double approachHeadingScaling, double maxSpeed)
{
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
    double launchVoltage = std::copysign(LAUNCH_VOLTAGE, targetDistance);
    double minLaunchSpeedVoltage = std::copysign(std::min(fabs(maxSpeedVoltage), fabs(LAUNCH_VOLTAGE)), targetDistance);

    double minDriveMotorRPM = (minSpeed * .01) * absoluteMaxRPM;
    double maxEncoderRPM = 0;
    
    double currentVoltageLeft = minLaunchSpeedVoltage;
    double currentVoltageRight = minLaunchSpeedVoltage;

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

    while (std::fabs(currentDistance) <= fabs(targetDistance) - 6.9 && !wallDetected)
    {
        double leftDeg = getEncoderPositionDeg(passiveEncoderLeft);
        double rightDeg = getEncoderPositionDeg(passiveEncoderRight);
        currentDistance = ((leftDeg + rightDeg) / 2.0 / 360.0) * encoderWheelCircumferenceCM;
        avgMotorVoltage = (fabs(currentVoltageLeft) * 3 + fabs(currentVoltageRight) * 3) / numberDriveMotor;

        pros::lcd::print(1, "Current: %.2f cm", std::fabs(currentDistance));
        pros::lcd::print(2, "Target: %.2f cm", std::fabs(targetDistance));

        double currentHeading = inertialSensor.get_rotation() - headingOffset;
        double headingCorrection = headingPID.calculate(targetHeading, currentHeading);

        double leftEncoderRPM = getEncoderVelocityRPM(passiveEncoderLeft) * (encoderWheelCircumferenceCM / wheelCircumferenceCM);
        double rightEncoderRPM = getEncoderVelocityRPM(passiveEncoderRight) * (encoderWheelCircumferenceCM / wheelCircumferenceCM);
        double avgEncoderRPM = (leftEncoderRPM + rightEncoderRPM) / 2;

        double leftMotorRPM = leftMotor2.get_actual_velocity() * DRIVE_MOTOR_RPM_ADJ;
        double rightMotorRPM = rightMotor2.get_actual_velocity() * DRIVE_MOTOR_RPM_ADJ;

        if (wallDetectEnabled)
        {
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

        if (fabs(currentDistance) < (fabs(targetDistance) - breakDistance) && !accelCompleted && !decel)
        {
            double leftTractionVoltage = tractionControlLeft.tractionControlSpeed(currentVoltageLeft, leftMotorRPM, leftEncoderRPM, ACCEL_FACTOR_LAUNCH);
            double rightTractionVoltage = tractionControlRight.tractionControlSpeed(currentVoltageRight, rightMotorRPM, rightEncoderRPM, ACCEL_FACTOR_LAUNCH);

            double syncedMotorVoltage = (std::fabs(leftTractionVoltage) < std::fabs(rightTractionVoltage)) 
                ? leftTractionVoltage : rightTractionVoltage;

            currentVoltageLeft = syncedMotorVoltage + (headingCorrection * accelHeadingScaling);
            currentVoltageRight = syncedMotorVoltage - (headingCorrection * accelHeadingScaling);

            maxEncoderRPM = std::max(maxEncoderRPM, fabs(avgEncoderRPM));
            if (fabs(avgMotorVoltage) >= (fabs(maxSpeedVoltage) - VOLTAGE_TOLERANCE)) accelCompleted = true;
        }
        else if (fabs(currentDistance) < (fabs(targetDistance) - breakDistance) && accelCompleted == true)
        {
            currentVoltageLeft = maxSpeedVoltage + (headingCorrection);
            currentVoltageRight = maxSpeedVoltage - (headingCorrection);
        }
        else if (fabs(currentDistance) >= (fabs(targetDistance) - breakDistance) && decelCompleted == false)
        {
            if (!decel) {
                adaptiveABSLeft.initialize(currentVoltageLeft);
                adaptiveABSRight.initialize(currentVoltageRight);
            }
            decel = true;

            double leftDecelVoltage = adaptiveABSLeft.decelControlSpeed(leftMotorRPM, leftEncoderRPM);
            double rightDecelVoltage = adaptiveABSRight.decelControlSpeed(rightMotorRPM, rightEncoderRPM);

            pros::motor_brake_mode_e_t leftBrakeMode = adaptiveABSLeft.getBrakeMode();
            pros::motor_brake_mode_e_t rightBrakeMode = adaptiveABSRight.getBrakeMode();

            pros::motor_brake_mode_e_t syncedBrakeMode;
            if (leftBrakeMode == pros::E_MOTOR_BRAKE_HOLD || rightBrakeMode == pros::E_MOTOR_BRAKE_HOLD) {
                syncedBrakeMode = pros::E_MOTOR_BRAKE_HOLD;
            } else {
                syncedBrakeMode = pros::E_MOTOR_BRAKE_HOLD;
            }

            double syncedDecelVoltage = (std::fabs(leftDecelVoltage) < std::fabs(rightDecelVoltage))
                ? leftDecelVoltage : rightDecelVoltage;

            double steeringCorrection = headingCorrection * decelHeadingScaling;

            if (syncedBrakeMode == pros::E_MOTOR_BRAKE_BRAKE && std::fabs(syncedDecelVoltage) > 0.0) {
                double correctedLeft = syncedDecelVoltage + steeringCorrection;
                double correctedRight = syncedDecelVoltage - steeringCorrection;
                
                if (syncedDecelVoltage > 0) {
                    currentVoltageLeft = std::max(0.0, correctedLeft);
                    currentVoltageRight = std::max(0.0, correctedRight);
                } else {
                    currentVoltageLeft = std::min(0.0, correctedLeft);
                    currentVoltageRight = std::min(0.0, correctedRight);
                }
            } else {
                currentVoltageLeft = 0.0;
                currentVoltageRight = 0.0;
            }
            
            leftMotors.set_brake_mode(syncedBrakeMode);
            rightMotors.set_brake_mode(syncedBrakeMode);

            leftEncoderRollingAverage = rollingAverage(leftEncoderRPM, leftEncoderRollingAverage, 10);
            rightEncoderRollingAverage = rollingAverage(rightEncoderRPM, rightEncoderRollingAverage, 10);

            if (fabs(leftEncoderRollingAverage) <= fabs(minDriveMotorRPM) &&
                fabs(rightEncoderRollingAverage) <= fabs(minDriveMotorRPM)) {
                decelCompleted = true;
            }
        }
        else if (decelCompleted == true)
        {
            currentVoltageLeft = minSpeedVoltage + (headingCorrection * approachHeadingScaling);
            currentVoltageRight = minSpeedVoltage - (headingCorrection * approachHeadingScaling);
        }

        leftMotors.move_voltage(currentVoltageLeft * 1000.0);
        rightMotors.move_voltage(currentVoltageRight * 1000.0);
        pros::delay(10);
    }

    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    leftMotors.brake();
    rightMotors.brake();
}

void forwardMP(double targetDistance, double breakDistance, double targetHeading, double minSpeed, 
               double kp_heading, double ki_heading, double kd_heading, double accelHeadingScaling, 
               double decelHeadingScaling, double approachHeadingScaling, double maxSpeed)
{
    straightOdometry(targetDistance, breakDistance, targetHeading, minSpeed,
                     kp_heading, ki_heading, kd_heading,
                     accelHeadingScaling, decelHeadingScaling,
                     approachHeadingScaling, maxSpeed);
}

void backwardMP(double targetDistance, double breakDistance, double targetHeading, double minSpeed, 
                double kp_heading, double ki_heading, double kd_heading, double accelHeadingScaling, 
                double decelHeadingScaling, double approachHeadingScaling, double maxSpeed)
{
    targetDistance = -std::fabs(targetDistance);
    straightOdometry(targetDistance, breakDistance, targetHeading, minSpeed,
                     kp_heading, ki_heading, kd_heading,
                     accelHeadingScaling, decelHeadingScaling,
                     approachHeadingScaling, maxSpeed);
}

void leftMP(double turnAmount, double breakDistance, double minSpeed, double maxSpeed)
{
    double currentHeading = inertialSensor.get_rotation() - headingOffset;
    double targetRotation = currentHeading - turnAmount;
    turnOdometry(targetRotation, breakDistance, minSpeed, maxSpeed);
}

void rightMP(double turnAmount, double breakDistance, double minSpeed, double maxSpeed)
{
    double currentHeading = inertialSensor.get_rotation() - headingOffset;
    double targetRotation = currentHeading + turnAmount;
    turnOdometry(targetRotation, breakDistance, minSpeed, maxSpeed);
}

void pivotTurnOdometry(double targetHeading, double breakDistanceInDegrees, double minSpeed, double maxSpeed)
{
    bool decelCompleted = false;
    bool accelCompleted = false;
    bool decel = false;

    double currentHeading = inertialSensor.get_rotation() - headingOffset;
    double headingError = targetHeading - currentHeading;
    double currentDistanceInDegrees = headingError;

    pros::lcd::print(4, "Target Rotation: %.2f", targetHeading);
    pros::lcd::print(5, "Curr Rotation: %.2f", currentHeading);

    double maxSpeedVoltage = std::copysign(maxSpeed * 0.01 * 12, headingError);
    double minSpeedVoltage = std::copysign(minSpeed * 0.01 * 12, headingError);
    double launchVoltage = std::copysign(5, headingError);
    double minLaunchSpeedVoltage = std::copysign(std::min(fabs(maxSpeedVoltage), fabs(launchVoltage)), headingError);

    double minDriveMotorRPM = (minSpeed * .01) * absoluteMaxRPM;
    const double SLIP_THRESHOLD_TRACTION = 0.35; 
    double accelFactorLaunch = 1.4;
    const double DECEL_STEP_PERCENT = 2.0;
    const double LOCK_THRESHOLD_DECEL = 0.25;

    // Arrays to store individual motor states (required for algorithm logic)
    double leftMotorRPM[3] = {0, 0, 0};
    double rightMotorRPM[3] = {0, 0, 0};
    double motorVoltageLeft[3] = {minLaunchSpeedVoltage, minLaunchSpeedVoltage, minLaunchSpeedVoltage}; 
    double motorVoltageRight[3] = {minLaunchSpeedVoltage, minLaunchSpeedVoltage, minLaunchSpeedVoltage}; 
    
    double averageMotorVoltage = 0;
    double leftEncoderRollingAverage = 0;
    double rightEncoderRollingAverage = 0;

    adaptiveABS adaptiveABSLeft(DECEL_STEP_PERCENT, LOCK_THRESHOLD_DECEL);
    adaptiveABS adaptiveABSRight(DECEL_STEP_PERCENT, LOCK_THRESHOLD_DECEL);

    tractionControl tractionControlLeft[3] = {tractionControl(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION),
                                              tractionControl(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION),
                                              tractionControl(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION)};

    tractionControl tractionControlRight[3] = {tractionControl(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION),
                                               tractionControl(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION),
                                               tractionControl(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION)};

    while (std::abs(headingError) > 9)
    {
        currentHeading = inertialSensor.get_rotation() - headingOffset;
        headingError = targetHeading - currentHeading;
        currentDistanceInDegrees = headingError;

        pros::lcd::print(4, "Target Rotation: %.2f", targetHeading);
        pros::lcd::print(5, "Curr Rotation: %.2f", currentHeading);
        pros::lcd::print(7, "Target: %.2f", targetHeading);

        // Update RPMs individually (unrolled to avoid manual pointers)
        leftMotorRPM[0] = fabs(leftMotor1.get_actual_velocity());
        leftMotorRPM[1] = fabs(leftMotor2.get_actual_velocity());
        leftMotorRPM[2] = fabs(leftMotor3.get_actual_velocity());
        rightMotorRPM[0] = fabs(rightMotor1.get_actual_velocity());
        rightMotorRPM[1] = fabs(rightMotor2.get_actual_velocity());
        rightMotorRPM[2] = fabs(rightMotor3.get_actual_velocity());

        double leftEncoderRPM = fabs(getEncoderVelocityRPM(passiveEncoderLeft)) * (encoderWheelCircumferenceCM / wheelCircumferenceCM) * ENCODER_RADIUS_RATIO;
        double rightEncoderRPM = fabs(getEncoderVelocityRPM(passiveEncoderRight)) * (encoderWheelCircumferenceCM / wheelCircumferenceCM) * ENCODER_RADIUS_RATIO;
        double avgEncoderRPM = (fabs(leftEncoderRPM) + fabs(rightEncoderRPM)) / 2;
        
        // Launch Phase
        if ((std::fabs(headingError) > fabs(breakDistanceInDegrees)) && !accelCompleted && !decel)
        {
            // Calculate individual traction control
            for(int i=0; i<3; i++) {
                motorVoltageLeft[i] = tractionControlLeft[i].tractionControlSpeed(motorVoltageLeft[i], leftMotorRPM[i], avgEncoderRPM, accelFactorLaunch);
                motorVoltageRight[i] = tractionControlRight[i].tractionControlSpeed(motorVoltageRight[i], rightMotorRPM[i], avgEncoderRPM, accelFactorLaunch);
            }

            averageMotorVoltage = (fabs(motorVoltageLeft[0]) + fabs(motorVoltageRight[0]) + fabs(motorVoltageLeft[1]) + fabs(motorVoltageRight[1]) + fabs(motorVoltageLeft[2]) + fabs(motorVoltageRight[2])) / numberDriveMotor;

            if (std::fabs(averageMotorVoltage) >= std::fabs(maxSpeedVoltage)) {
                accelCompleted = true;
                pros::lcd::print(1, "Accel Completed");
            }
        }
        // Cruise Phase
        else if ((std::abs(headingError) > fabs(breakDistanceInDegrees)) && accelCompleted)
        {
            for (int i = 0; i < 3; i++) {
                motorVoltageLeft[i] = maxSpeedVoltage;
                motorVoltageRight[i] = maxSpeedVoltage;
            }
        }
        // Decel Phase
        else if ((std::abs(headingError) <= fabs(breakDistanceInDegrees)) && decelCompleted == false)
        {
            if (!decel) {
                // Initialize uniform braking
                leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
                rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
                adaptiveABSLeft.initialize(motorVoltageLeft[1]);
                adaptiveABSRight.initialize(motorVoltageRight[1]);
            }

            decel = true;
            pros::lcd::print(1, "Decel Phase");

            double leftMotorRPM_Mid = fabs(leftMotor2.get_actual_velocity());
            double rightMotorRPM_Mid = fabs(rightMotor2.get_actual_velocity());

            double leftDecelVoltage = adaptiveABSLeft.decelControlSpeed(leftMotorRPM_Mid, leftEncoderRPM);
            double rightDecelVoltage = adaptiveABSRight.decelControlSpeed(rightMotorRPM_Mid, rightEncoderRPM);

            double syncedDecelVoltage = std::max(fabs(leftDecelVoltage), fabs(rightDecelVoltage));

            for (int i = 0; i < 3; i++) {
                motorVoltageLeft[i] = std::copysign(syncedDecelVoltage, motorVoltageLeft[i]);
                motorVoltageRight[i] = std::copysign(syncedDecelVoltage, motorVoltageRight[i]);
            }

            leftEncoderRollingAverage = rollingAverage(leftEncoderRPM, leftEncoderRollingAverage, 3);
            rightEncoderRollingAverage = rollingAverage(rightEncoderRPM, rightEncoderRollingAverage, 3);

            if (fabs(leftEncoderRollingAverage) <= fabs(minDriveMotorRPM) &&
                fabs(rightEncoderRollingAverage) <= fabs(minDriveMotorRPM)) {
                decelCompleted = true;
            }
        }

        // Apply voltages (Complex individual pivot logic preserved)
        // Pair logic
        double lowerVoltageLeading = std::min(std::fabs(motorVoltageLeft[0]), std::fabs(motorVoltageRight[2]));
        motorVoltageLeft[0] = std::copysign(lowerVoltageLeading, motorVoltageLeft[0]);
        motorVoltageRight[2] = std::copysign(lowerVoltageLeading, motorVoltageRight[2]);

        double lowerVoltageMiddle = std::min(std::fabs(motorVoltageLeft[1]), std::fabs(motorVoltageRight[1]));
        motorVoltageLeft[1] = std::copysign(lowerVoltageMiddle, motorVoltageLeft[1]);
        motorVoltageRight[1] = std::copysign(lowerVoltageMiddle, motorVoltageRight[1]);

        double lowerVoltageTrailing = std::min(std::fabs(motorVoltageLeft[2]), std::fabs(motorVoltageRight[0]));
        motorVoltageLeft[2] = std::copysign(lowerVoltageTrailing, motorVoltageLeft[2]);
        motorVoltageRight[0] = std::copysign(lowerVoltageTrailing, motorVoltageRight[0]);

        if (!decel == true || decelCompleted == true)
        {
            if (motorVoltageLeft[0] > 0)
            { 
                leftMotor1.move_voltage(motorVoltageLeft[0] * 1000.0);
                rightMotor1.brake();
                leftMotor2.move_voltage(motorVoltageLeft[1] * 1000.0);
                rightMotor2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
                rightMotor2.brake();
                leftMotor3.move_voltage(motorVoltageLeft[2] * 1000.0);
                rightMotor3.brake();
            }
            else
            { 
                leftMotor1.brake();
                rightMotor1.move_voltage(fabs(motorVoltageRight[0]) * 1000.0);
                leftMotor2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
                leftMotor2.brake();
                rightMotor2.move_voltage(fabs(motorVoltageRight[1]) * 1000.0);
                leftMotor3.brake();
                rightMotor3.move_voltage(fabs(motorVoltageRight[2]) * 1000.0);
            }
        }
        pros::delay(10);
    }

    // Final braking using Groups (Replaces manual loop)
    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    leftMotors.brake();
    rightMotors.brake();
}

void pivotLeftMP(double turnAmount, double breakDistance, double minSpeed, double maxSpeed)
{
    double currentHeading = inertialSensor.get_rotation() - headingOffset;
    double targetRotation = currentHeading + turnAmount;
    pivotTurnOdometry(targetRotation, breakDistance, minSpeed, maxSpeed);
}

void pivotRightMP(double turnAmount, double breakDistance, double minSpeed, double maxSpeed)
{
    double currentHeading = inertialSensor.get_rotation() - headingOffset;
    double targetRotation = currentHeading - turnAmount;
    pivotTurnOdometry(targetRotation, breakDistance, minSpeed, maxSpeed);
}

void driveForward(double targetDistance, double breakDistance, double targetHeading, double minSpeed, 
                  double kp_heading, double ki_heading, double kd_heading, double accelHeadingScaling, 
                  double decelHeadingScaling, double approachHeadingScaling, double maxSpeed)
{
    double internalHeading = -targetHeading; 
    straightOdometry(targetDistance, breakDistance, internalHeading, minSpeed,
                     kp_heading, ki_heading, kd_heading,
                     accelHeadingScaling, decelHeadingScaling,
                     approachHeadingScaling, maxSpeed);
}

void driveBackward(double targetDistance, double breakDistance, double targetHeading, double minSpeed, 
                   double kp_heading, double ki_heading, double kd_heading, double accelHeadingScaling, 
                   double decelHeadingScaling, double approachHeadingScaling, double maxSpeed)
{
    targetDistance = -std::fabs(targetDistance);
    double internalHeading = -targetHeading; 
    straightOdometry(targetDistance, breakDistance, internalHeading, minSpeed,
                     kp_heading, ki_heading, kd_heading,
                     accelHeadingScaling, decelHeadingScaling,
                     approachHeadingScaling, maxSpeed);
}

void turnRight(double absoluteTargetHeading, double breakDistance, double minSpeed, double maxSpeed, double exitTolerance)
{
    double currentHeading = inertialSensor.get_rotation() - headingOffset;
    double targetHeading = -absoluteTargetHeading;
    while (targetHeading <= currentHeading) targetHeading += 360.0;
    turnOdometry(targetHeading, breakDistance, minSpeed, maxSpeed, exitTolerance);
}

void turnLeft(double absoluteTargetHeading, double breakDistance, double minSpeed, double maxSpeed, double exitTolerance)
{
    double currentHeading = inertialSensor.get_rotation() - headingOffset;
    double targetHeading = -absoluteTargetHeading;
    while (targetHeading >= currentHeading) targetHeading -= 360.0;
    turnOdometry(targetHeading, breakDistance, minSpeed, maxSpeed, exitTolerance);
}

void driveForwardV2(double targetDistance, double breakDistance, double targetHeading, double minSpeed, 
                  double distanceTolerance, double kp_heading, double ki_heading, double kd_heading, 
                  double accelHeadingScaling, double decelHeadingScaling, double approachHeadingScaling, double maxSpeed)
{
    double internalHeading = -targetHeading;
    straightOdometryV2(targetDistance, breakDistance, internalHeading, minSpeed,
                     distanceTolerance, kp_heading, ki_heading, kd_heading,
                     accelHeadingScaling, decelHeadingScaling,
                     approachHeadingScaling, maxSpeed);
}

void driveBackwardV2(double targetDistance, double breakDistance, double targetHeading, double minSpeed, 
                   double distanceTolerance, double kp_heading, double ki_heading, double kd_heading, 
                   double accelHeadingScaling, double decelHeadingScaling, double approachHeadingScaling, double maxSpeed)
{
    targetDistance = -std::fabs(targetDistance);
    double internalHeading = -targetHeading;
    straightOdometryV2(targetDistance, breakDistance, internalHeading, minSpeed,
                     distanceTolerance, kp_heading, ki_heading, kd_heading,
                     accelHeadingScaling, decelHeadingScaling,
                     approachHeadingScaling, maxSpeed);
}

void pidlessForward(double timeMs, double speedPct)
{
    uint32_t startTime = pros::millis();
    double voltagePower = (speedPct / 8.34) * 1000.0; 

    while ((pros::millis() - startTime) < timeMs)
    {
        leftMotors.move_voltage(voltagePower);
        rightMotors.move_voltage(voltagePower);
        pros::delay(10);
    }
    
    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    leftMotors.brake();
    rightMotors.brake();
}
