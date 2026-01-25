#include "navigation.h"
#include "robot_config.h"
#include "vex.h"
#include "utils.h"
#include "pid.h" 
#include <cmath>
#include <algorithm>
#include <iterator>
#include <cstring> 
#include <atomic>
#include "odometry.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace vex;

const int VISION_CENTER_X = 160;      
const int MIN_OBJECT_WIDTH = 25;      

enum MotionPhase { READY, LAUNCH, CRUISE, DECELERATE, APPROACH, STOP };

double getCurrentEncoderDistanceCM() {
    double avgDeg = (passiveEncoderLeft.position(degrees) + passiveEncoderRight.position(degrees)) / 2.0;
    return (avgDeg / 360.0) * encoderWheelCircumferenceCM;
}

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
    bool decelCompleted = false, accelCompleted = false, decel = false;
    double currentHeading = getAdjustedRotation();
    int completeRotations = (int)(currentHeading / 360.0);
    double targetRotationHeading = targetHeading + (completeRotations * 360.0);
    double headingError = targetRotationHeading - currentHeading;
    double maxSpeedVoltage = std::copysign(maxSpeed * 0.01 * absoluteMaxVoltage, headingError);
    double minSpeedVoltage = std::copysign(minSpeed * 0.01 * absoluteMaxVoltage, headingError);
    double minLaunchSpeedVoltage = std::copysign(std::min(fabs(maxSpeedVoltage), 4.0), headingError);
    double minDriveMotorRPM = (minSpeed * .01) * absoluteMaxRPM;

    adaptiveABS ABSLeft(20, 10), ABSRight(20, 10);
    tractionControl trackLeft(minLaunchSpeedVoltage, maxSpeedVoltage, 10), trackRight(minLaunchSpeedVoltage, maxSpeedVoltage, 10);

    double motorVoltageLeft[3] = {minLaunchSpeedVoltage, minLaunchSpeedVoltage, minLaunchSpeedVoltage};
    double motorVoltageRight[3] = {minLaunchSpeedVoltage, minLaunchSpeedVoltage, minLaunchSpeedVoltage};

    while ((maxSpeedVoltage > 0 && currentHeading <= targetRotationHeading - exitTolerance) || (maxSpeedVoltage < 0 && currentHeading >= targetRotationHeading + exitTolerance)) {
        currentHeading = getAdjustedRotation();
        headingError = targetRotationHeading - currentHeading;
        double leftMotorRPM = fabs(leftMotor[1].velocity(rpm)) * DRIVE_MOTOR_RPM_ADJ, rightMotorRPM = fabs(rightMotor[1].velocity(rpm)) * DRIVE_MOTOR_RPM_ADJ;
        double avgEncoderRPM = (fabs(passiveEncoderLeft.velocity(rpm)) + fabs(passiveEncoderRight.velocity(rpm))) / 2.0;

        if ((std::fabs(headingError) > fabs(breakDistanceInDegrees)) && !accelCompleted && !decel) {
            double syncedVolt = std::min(fabs(trackLeft.tractionControlSpeed(motorVoltageLeft[1], leftMotorRPM, avgEncoderRPM, 1.5)), fabs(trackRight.tractionControlSpeed(motorVoltageRight[1], rightMotorRPM, avgEncoderRPM, 1.5)));
            for (int i = 0; i < 3; i++) { motorVoltageLeft[i] = std::copysign(syncedVolt, motorVoltageLeft[i]); motorVoltageRight[i] = std::copysign(syncedVolt, motorVoltageRight[i]); }
            if (fabs(motorVoltageLeft[1]) >= (fabs(maxSpeedVoltage) - VOLTAGE_TOLERANCE)) accelCompleted = true;
        } else if ((std::abs(headingError) > fabs(breakDistanceInDegrees)) && accelCompleted) {
            for (int i = 0; i < 3; i++) { motorVoltageLeft[i] = maxSpeedVoltage; motorVoltageRight[i] = maxSpeedVoltage; }
        } else if ((std::abs(headingError) <= fabs(breakDistanceInDegrees)) && !decelCompleted) {
            if (!decel) { ABSLeft.initialize(motorVoltageLeft[1]); ABSRight.initialize(motorVoltageRight[1]); decel = true; }
            double syncedDecel = std::max(fabs(ABSLeft.decelControlSpeed(leftMotorRPM, avgEncoderRPM)), fabs(ABSRight.decelControlSpeed(rightMotorRPM, avgEncoderRPM)));
            vex::brakeType syncedBrake = (ABSLeft.getBrakeMode() == vex::coast || ABSRight.getBrakeMode() == vex::coast) ? vex::coast : vex::brake;
            for (int i = 0; i < 3; i++) { leftMotor[i].setBrake(syncedBrake); rightMotor[i].setBrake(syncedBrake); motorVoltageLeft[i] = (syncedBrake == vex::brake) ? std::copysign(syncedDecel, motorVoltageLeft[i]) : 0; motorVoltageRight[i] = (syncedBrake == vex::brake) ? std::copysign(syncedDecel, motorVoltageRight[i]) : 0; }
            if (fabs(leftMotorRPM) <= fabs(minDriveMotorRPM) && fabs(rightMotorRPM) <= fabs(minDriveMotorRPM)) decelCompleted = true;
        } else if (decelCompleted) {
            for (int i = 0; i < 3; i++) { motorVoltageLeft[i] = minSpeedVoltage; motorVoltageRight[i] = minSpeedVoltage; }
        }

        for (int i = 0; i < 3; i++) { leftMotor[i].spin(forward, motorVoltageLeft[i], volt); rightMotor[i].spin(forward, -motorVoltageRight[i], volt); }
        vex::task::sleep(10);
    }
    for (int i = 0; i < 3; i++) { leftMotor[i].setBrake(brake); rightMotor[i].setBrake(brake); leftMotor[i].stop(); rightMotor[i].stop(); }
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

void straightOdometry(double targetDistance, double breakDistance, double targetHeading, double minSpeed, double kp_heading, double ki_heading, double kd_heading, double accelHeadingScaling, double decelHeadingScaling, double approachHeadingScaling, double maxSpeed) {
    double startDist = getCurrentEncoderDistanceCM();
    double distanceTravelled = 0;
    
    PID headingPID(kp_heading, ki_heading, kd_heading); headingPID.pidReset();
    double maxV = std::copysign(maxSpeed * 0.12, targetDistance), minV = std::copysign(minSpeed * 0.12, targetDistance);
    double motorVoltageLeft[3] = {minV, minV, minV}, motorVoltageRight[3] = {minV, minV, minV};
    bool decel = false, decelCompleted = false, accelCompleted = false;

    while (std::fabs(distanceTravelled) <= fabs(targetDistance) - 6) {
        distanceTravelled = getCurrentEncoderDistanceCM() - startDist;
        double headingCorrection = headingPID.calculate(targetHeading, getAdjustedRotation());
        double leftMotorRPM = leftMotor[1].velocity(rpm) * DRIVE_MOTOR_RPM_ADJ, rightMotorRPM = rightMotor[1].velocity(rpm) * DRIVE_MOTOR_RPM_ADJ;
        double avgEncoderRPM = (fabs(passiveEncoderLeft.velocity(rpm)) + fabs(passiveEncoderRight.velocity(rpm))) / 2.0;

        if (fabs(distanceTravelled) < (fabs(targetDistance) - breakDistance) && !accelCompleted && !decel) {
            double syncedV = std::min(fabs(maxV), std::max(fabs(minV), (leftMotorRPM < avgEncoderRPM ? fabs(motorVoltageLeft[1]) * 1.25 : fabs(motorVoltageLeft[1]) / 1.25)));
            for (int i = 0; i < 3; i++) { motorVoltageLeft[i] = std::copysign(syncedV, maxV) + (headingCorrection * accelHeadingScaling); motorVoltageRight[i] = std::copysign(syncedV, maxV) - (headingCorrection * accelHeadingScaling); }
            if (fabs(motorVoltageLeft[1]) >= fabs(maxV)) accelCompleted = true;
        } else if (fabs(distanceTravelled) < (fabs(targetDistance) - breakDistance) && accelCompleted) {
            for (int i = 0; i < 3; i++) { motorVoltageLeft[i] = maxV + headingCorrection; motorVoltageRight[i] = maxV - headingCorrection; }
        } else if (fabs(distanceTravelled) >= (fabs(targetDistance) - breakDistance) && !decelCompleted) {
            if (!decel) decel = true; 
            double syncedDecel = std::max(0.0, fabs(motorVoltageLeft[1]) - 0.2);
            for (int i = 0; i < 3; i++) { motorVoltageLeft[i] = std::copysign(syncedDecel, maxV) + headingCorrection * decelHeadingScaling; motorVoltageRight[i] = std::copysign(syncedDecel, maxV) - headingCorrection * decelHeadingScaling; }
            if (syncedDecel <= fabs(minV)) decelCompleted = true;
        } else if (decelCompleted) {
            for (int i = 0; i < 3; i++) { motorVoltageLeft[i] = minV + headingCorrection * approachHeadingScaling; motorVoltageRight[i] = minV - headingCorrection * approachHeadingScaling; }
        }

        for (int i = 0; i < 3; i++) { leftMotor[i].spin(forward, motorVoltageLeft[i], volt); rightMotor[i].spin(forward, motorVoltageRight[i], volt); }
        vex::task::sleep(10);
    }
    for (int i = 0; i < 3; i++) { leftMotor[i].stop(brake); rightMotor[i].stop(brake); }
}

void straightOdometryV2(double targetDistance, double breakDistance, double targetHeading, double minSpeed, double distanceTolerance, double kp_heading, double ki_heading, double kd_heading, double accelHeadingScaling, double decelHeadingScaling, double approachHeadingScaling, double maxSpeed) {
    double startDist = getCurrentEncoderDistanceCM();
    double distanceTravelled = 0;
    
    PID headingPID(kp_heading, ki_heading, kd_heading); headingPID.pidReset();
    double maxV = std::copysign(maxSpeed * 0.12, targetDistance), minV = std::copysign(minSpeed * 0.12, targetDistance);
    double motorVoltageLeft[3] = {minV, minV, minV}, motorVoltageRight[3] = {minV, minV, minV};
    bool decel = false, decelCompleted = false, accelCompleted = false;

    while (std::fabs(distanceTravelled) <= fabs(targetDistance) - distanceTolerance) {
        distanceTravelled = getCurrentEncoderDistanceCM() - startDist;
        double headingCorrection = headingPID.calculate(targetHeading, getAdjustedRotation());
        if (fabs(distanceTravelled) < (fabs(targetDistance) - breakDistance) && !accelCompleted) {
            double syncedV = std::min(fabs(maxV), fabs(motorVoltageLeft[1]) + 0.1);
            for (int i = 0; i < 3; i++) { motorVoltageLeft[i] = std::copysign(syncedV, maxV) + headingCorrection * accelHeadingScaling; motorVoltageRight[i] = std::copysign(syncedV, maxV) - headingCorrection * accelHeadingScaling; }
            if (syncedV >= fabs(maxV)) accelCompleted = true;
        } else if (fabs(distanceTravelled) < (fabs(targetDistance) - breakDistance) && accelCompleted) {
            for (int i = 0; i < 3; i++) { motorVoltageLeft[i] = maxV + headingCorrection; motorVoltageRight[i] = maxV - headingCorrection; }
        } else if (fabs(distanceTravelled) >= (fabs(targetDistance) - breakDistance) && !decelCompleted) {
            double syncedDecel = std::max(fabs(minV), fabs(motorVoltageLeft[1]) - 0.2);
            for (int i = 0; i < 3; i++) { motorVoltageLeft[i] = std::copysign(syncedDecel, maxV) + headingCorrection * decelHeadingScaling; motorVoltageRight[i] = std::copysign(syncedDecel, maxV) - headingCorrection * decelHeadingScaling; }
            if (syncedDecel <= fabs(minV)) decelCompleted = true;
        } else if (decelCompleted) {
            for (int i = 0; i < 3; i++) { motorVoltageLeft[i] = minV + headingCorrection * approachHeadingScaling; motorVoltageRight[i] = minV - headingCorrection * approachHeadingScaling; }
        }
        for (int i = 0; i < 3; i++) { leftMotor[i].spin(forward, motorVoltageLeft[i], volt); rightMotor[i].spin(forward, motorVoltageRight[i], volt); }
        vex::task::sleep(10);
    }
    for (int i = 0; i < 3; i++) { leftMotor[i].stop(brake); rightMotor[i].stop(brake); }
}

void smartStraight(double targetDistance, double breakDistance, double targetHeading, double minSpeed, double wallStalledTimeMs, double kp_heading, double ki_heading, double kd_heading, double accelHeadingScaling, double decelHeadingScaling, double approachHeadingScaling, double maxSpeed) {
    const double WALL_THRESHOLD = 5.0; 
    vex::timer wallTimer; 
    bool wallDetected = false, isStalled = false;
    
    double startDist = getCurrentEncoderDistanceCM();
    double distanceTravelled = 0;
    
    PID headingPID(kp_heading, ki_heading, kd_heading); headingPID.pidReset();
    double maxV = std::copysign(maxSpeed * 0.12, targetDistance), minV = std::copysign(minSpeed * 0.12, targetDistance);
    double motorVoltageLeft[3] = {minV, minV, minV}, motorVoltageRight[3] = {minV, minV, minV};

    while (std::fabs(distanceTravelled) <= fabs(targetDistance) - 6.9 && !wallDetected) {
        distanceTravelled = getCurrentEncoderDistanceCM() - startDist;
        double headingCorrection = headingPID.calculate(targetHeading, getAdjustedRotation());
        double avgRPM = (fabs(passiveEncoderLeft.velocity(rpm)) + fabs(passiveEncoderRight.velocity(rpm))) / 2.0;

        if (wallStalledTimeMs > 0) {
            if (avgRPM < WALL_THRESHOLD) { if (!isStalled) { wallTimer.reset(); isStalled = true; } else if (wallTimer.time(msec) >= wallStalledTimeMs) wallDetected = true; }
            else isStalled = false;
        }

        for (int i = 0; i < 3; i++) { motorVoltageLeft[i] = maxV + headingCorrection; motorVoltageRight[i] = maxV - headingCorrection; leftMotor[i].spin(forward, motorVoltageLeft[i], volt); rightMotor[i].spin(forward, motorVoltageRight[i], volt); }
        vex::task::sleep(10);
    }
    for (int i = 0; i < 3; i++) { leftMotor[i].stop(brake); rightMotor[i].stop(brake); }
}

void forwardMP(double targetDistance, double breakDistance, double targetHeading_modified, double minSpeed, double kp_heading, double ki_heading, double kd_heading, double accelHeadingScaling, double decelHeadingScaling, double approachHeadingScaling, double maxSpeed) {
    straightOdometry(targetDistance, breakDistance, modifiedToStandardCartesian(targetHeading_modified), minSpeed, kp_heading, ki_heading, kd_heading, accelHeadingScaling, decelHeadingScaling, approachHeadingScaling, maxSpeed);
}

void backwardMP(double targetDistance, double breakDistance, double targetHeading_modified, double minSpeed, double kp_heading, double ki_heading, double kd_heading, double accelHeadingScaling, double decelHeadingScaling, double approachHeadingScaling, double maxSpeed) {
    straightOdometry(-fabs(targetDistance), breakDistance, modifiedToStandardCartesian(targetHeading_modified) + 180.0, minSpeed, kp_heading, ki_heading, kd_heading, accelHeadingScaling, decelHeadingScaling, approachHeadingScaling, maxSpeed);
}

void leftMP(double turnAmount, double breakDistance, double minSpeed, double maxSpeed) {
    turnOdometry(getAdjustedRotation() + turnAmount, breakDistance, minSpeed, maxSpeed);
}

void rightMP(double turnAmount, double breakDistance, double minSpeed, double maxSpeed) {
    turnOdometry(getAdjustedRotation() - turnAmount, breakDistance, minSpeed, maxSpeed);
}

void pivotTurnOdometry(double targetHeading, double breakDistanceInDegrees, double minSpeed, double maxSpeed) {
    double currentHeading = getAdjustedRotation();
    double headingError = targetHeading - currentHeading;
    double maxV = std::copysign(maxSpeed * 0.12, headingError);
    double minV = std::copysign(minSpeed * 0.12, headingError);
    
    // Declare the motor voltage arrays that were missing
    double motorVoltageLeft[3] = {maxV, maxV, maxV};
    double motorVoltageRight[3] = {-maxV, -maxV, -maxV};
    
    while (std::abs(headingError) > 9) {
        currentHeading = getAdjustedRotation(); 
        headingError = targetHeading - currentHeading;
        
        // Recalculate voltage based on current error
        double volt = std::copysign(maxSpeed * 0.12, headingError);
        motorVoltageLeft[0] = volt;
        motorVoltageRight[0] = -volt;
        
        if (motorVoltageLeft[0] > 0) { 
            leftMotor[0].spin(forward, volt, voltageUnits::volt);
            rightMotor[0].stop(brake); 
        }
        else { 
            leftMotor[0].stop(brake); 
            rightMotor[0].spin(forward, fabs(volt), voltageUnits::volt);
        }
        vex::task::sleep(10);
    }
    
    for (int i = 0; i < 3; i++) { 
        leftMotor[i].stop(brake); 
        rightMotor[i].stop(brake); 
    }
}

void pivotLeftMP(double turnAmount, double breakDistance, double minSpeed, double maxSpeed) { pivotTurnOdometry(getAdjustedRotation() + turnAmount, breakDistance, minSpeed, maxSpeed); }
void pivotRightMP(double turnAmount, double breakDistance, double minSpeed, double maxSpeed) { pivotTurnOdometry(getAdjustedRotation() - turnAmount, breakDistance, minSpeed, maxSpeed); }

void driveForward(double targetDistance, double breakDistance, double targetHeading_modified, double minSpeed, double kp_heading, double ki_heading, double kd_heading, double accelHeadingScaling, double decelHeadingScaling, double approachHeadingScaling, double maxSpeed) {
    straightOdometry(targetDistance, breakDistance, modifiedToStandardCartesian(targetHeading_modified), minSpeed, kp_heading, ki_heading, kd_heading, accelHeadingScaling, decelHeadingScaling, approachHeadingScaling, maxSpeed);
}

void driveBackward(double targetDistance, double breakDistance, double targetHeading_modified, double minSpeed, double kp_heading, double ki_heading, double kd_heading, double accelHeadingScaling, double decelHeadingScaling, double approachHeadingScaling, double maxSpeed) {
    straightOdometry(-std::fabs(targetDistance), breakDistance, modifiedToStandardCartesian(targetHeading_modified) + 180.0, minSpeed, kp_heading, ki_heading, kd_heading, accelHeadingScaling, decelHeadingScaling, approachHeadingScaling, maxSpeed);
}

void turnRight(double absoluteTargetHeading_modified, double breakDistance, double minSpeed, double maxSpeed, double exitTolerance) {
    double target = modifiedToStandardCartesian(absoluteTargetHeading_modified), current = getAdjustedRotation();
    while (target >= current) target -= 360.0;
    turnOdometry(target, breakDistance, minSpeed, maxSpeed, exitTolerance);
}

void turnLeft(double absoluteTargetHeading_modified, double breakDistance, double minSpeed, double maxSpeed, double exitTolerance) {
    double target = modifiedToStandardCartesian(absoluteTargetHeading_modified), current = getAdjustedRotation();
    while (target <= current) target += 360.0;
    turnOdometry(target, breakDistance, minSpeed, maxSpeed, exitTolerance);
}

void driveForwardV2(double targetDistance, double breakDistance, double targetHeading_modified, double minSpeed, double distanceTolerance, double kp_heading, double ki_heading, double kd_heading, double accelHeadingScaling, double decelHeadingScaling, double approachHeadingScaling, double maxSpeed) {
    straightOdometryV2(targetDistance, breakDistance, modifiedToStandardCartesian(targetHeading_modified), minSpeed, distanceTolerance, kp_heading, ki_heading, kd_heading, accelHeadingScaling, decelHeadingScaling, approachHeadingScaling, maxSpeed);
}

void driveBackwardV2(double targetDistance, double breakDistance, double targetHeading_modified, double minSpeed, double distanceTolerance, double kp_heading, double ki_heading, double kd_heading, double accelHeadingScaling, double decelHeadingScaling, double approachHeadingScaling, double maxSpeed) {
    straightOdometryV2(-std::fabs(targetDistance), breakDistance, modifiedToStandardCartesian(targetHeading_modified) + 180.0, minSpeed, distanceTolerance, kp_heading, ki_heading, kd_heading, accelHeadingScaling, decelHeadingScaling, approachHeadingScaling, maxSpeed);
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
            if (obj.width >= MIN_OBJECT_WIDTH && obj.centerX >= minX && obj.centerX <= maxX && obj.centerY >= minY && obj.centerY <= maxY && obj.width > largestWidth) { largestWidth = obj.width; bestIdx = i; }
        }
        double turnCorrection = (bestIdx >= 0) ? headingPID.calculate(0.0, AIVision20.objects[bestIdx].centerX - VISION_CENTER_X) : headingPID.calculate(targetHeading, getAdjustedRotation());
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
 * Vision-guided movement function that drives toward a detected object
 * Uses two PID controllers plus distance-based heading scaling:
 * - Heading PID: Centers the object horizontally in the camera view
 * - Distance PID: Controls forward speed based on object's apparent size (width in pixels)
 * - Distance-to-Heading Scaling: Scales heading correction based on distance (P-only)
 *   → Farther away (small pixel width) = larger heading corrections
 *   → Closer (large pixel width) = smaller heading corrections (prevents overshoot)
 * 
 * @param targetSignature      AI Vision color signature to detect
 * @param targetPixelWidth     Desired object width in pixels (larger = closer)
 * @param targetHeading        Target heading to maintain (currently unused)
 * @param minSpeedPct          Minimum drive speed (0-100%)
 * @param maxSpeedPct          Maximum drive speed (0-100%)
 * @param brakeMode            Brake mode when stopping (brake/coast/hold)
 * @param kp_head, ki_head, kd_head        Heading PID gains
 * @param kp_distToHeadScaling             Proportional gain for distance-based heading scaling
 * @param kp_dist, ki_dist, kd_dist        Distance PID gains
 */
void visionDriveMinimal(
    vex::aivision::colordesc targetSignature, 
    int targetPixelWidth, 
    double targetHeading,           // Note: Currently not used in implementation
    double minSpeedPct, 
    double maxSpeedPct, 
    vex::brakeType brakeMode,
    double kp_head, double ki_head, double kd_head,
    double kp_distToHeadScaling,    // P-only scaling factor for heading based on distance
    double kp_dist, double ki_dist, double kd_dist
) {
    // ========================================
    // INITIALIZATION
    // ========================================
    
    // Maximum steering correction as percentage (0-100)
    double maxSteeringPct = 25.0;  // SET THIS VALUE HERE
    
    // Create PID controllers for heading (centering) and distance (forward movement)
    PID headingPID(kp_head, ki_head, kd_head);  // Controls left/right turn to center object
    PID distancePID(kp_dist, ki_dist, kd_dist);  // Controls forward speed based on object size
    
    // Reset PID accumulators to start fresh
    headingPID.pidReset();
    distancePID.pidReset();
    
    // Convert steering cap to voltage
    double maxSteeringVoltage = maxSteeringPct * 0.12;
    
    // ========================================
    // MAIN VISION TRACKING LOOP
    // ========================================
    
    while (true) {
        // Capture a frame and detect objects matching the target signature
        AIVision20.takeSnapshot(targetSignature);
        
        // Exit if no objects detected
        if (AIVision20.objectCount == 0) {
            break;
        }
        
        // Get the first detected object (assumes sorted by confidence)
        auto& detectedObject = AIVision20.objects[0];
        
        // ========================================
        // CALCULATE DISTANCE-BASED HEADING SCALING
        // ========================================
        
        // Calculate how far we are from the target distance (in pixels)
        // Larger error = farther away, needs more aggressive heading corrections
        // Smaller error = closer, needs gentler heading corrections
        double distanceErrorPixels = (double)targetPixelWidth - (double)detectedObject.width;
        
        // P-only scaling factor based on distance error
        // Far away (large error) → scaling factor > 1.0 → amplify heading correction
        // Close (small error) → scaling factor < 1.0 → reduce heading correction
        double headingScalingFactor = 1.0 + (kp_distToHeadScaling * distanceErrorPixels);
        
        // Clamp scaling factor to prevent negative or excessive values
        // Range: 0.1 to 3.0 (prevents reversal and limits max amplification)
        headingScalingFactor = std::max(0.1, std::min(3.0, headingScalingFactor));
        
        // ========================================
        // CALCULATE STEERING CORRECTION
        // ========================================
        
        // Calculate how far off-center the object is (in pixels)
        // Positive error = object is to the right, negative = object is to the left
        double pixelOffsetFromCenter = detectedObject.centerX - VISION_CENTER_X;
        
        // Base heading correction from PID
        double baseHeadingCorrectionVoltage = headingPID.calculate(0.0, pixelOffsetFromCenter) * 0.12;
        
        // Apply distance-based scaling to heading correction
        double steeringCorrectionVoltage = baseHeadingCorrectionVoltage * headingScalingFactor;
        
        // Clamp steering to maximum allowed value
        steeringCorrectionVoltage = std::max(-maxSteeringVoltage, std::min(maxSteeringVoltage, steeringCorrectionVoltage));
        
        // ========================================
        // CALCULATE FORWARD DRIVE SPEED
        // ========================================
        
        // PID outputs base drive voltage (larger error = faster approach)
        double baseDriveVoltage = distancePID.calculate((double)targetPixelWidth, (double)detectedObject.width) * 0.12;
        
        // Clamp drive voltage between min and max speeds (converted to voltage)
        double minDriveVoltage = minSpeedPct * 0.12;
        double maxDriveVoltage = maxSpeedPct * 0.12;
        double clampedDriveVoltage = std::max(minDriveVoltage, std::min(maxDriveVoltage, baseDriveVoltage));
        
        // ========================================
        // APPLY MOTOR COMMANDS
        // ========================================
        
        // Apply differential drive: base speed ± steering correction
        // Left side: subtract steering (turn left when object is left)
        // Right side: add steering (turn right when object is left)
        for (int i = 0; i < 3; i++) {
            leftMotor[i].spin(forward, clampedDriveVoltage - steeringCorrectionVoltage, volt);
            rightMotor[i].spin(forward, clampedDriveVoltage + steeringCorrectionVoltage, volt);
        }
        
        // ========================================
        // EXIT CONDITION CHECK
        // ========================================
        
        // Stop when object reaches target width (close enough to target)
        if (detectedObject.width >= targetPixelWidth) {
            break;
        }
        
        // Wait 20ms before next iteration (50Hz update rate)
        vex::task::sleep(20);
    }
    
    // ========================================
    // CLEANUP - STOP ALL MOTORS
    // ========================================
    
    for (int i = 0; i < 3; i++) {
        leftMotor[i].stop(brakeMode);
        rightMotor[i].stop(brakeMode);
    }
}