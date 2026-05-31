#include "robot_config.h"
#include "main.h"
#include "utils.h"
#include "pid.h"
#include "motion_config.h"
#include <cmath>
#include <atomic>
#include "odometry.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Horizontal center of the vision sensor frame (pixels)
const double VISION_CENTER_X = 160;

// ──────────────────────────────────────────────────────────────────────────────
// DRIVE PHASE — global enum readable by auto-calibration system.
// Written by each core motion function on every phase transition.
// Reset to PHASE_IDLE on function exit.
// ──────────────────────────────────────────────────────────────────────────────
DrivePhase currentDrivePhase = PHASE_IDLE;

// Returns the average distance traveled by the passive encoder wheels (cm).
// Uses centidegree PROS output (÷100 → degrees), then converts to distance.
double getCurrentEncoderDistanceCM() {
    double avgDeg = (passiveEncoderLeft.get_position()  / 100.0 +
                     passiveEncoderRight.get_position() / 100.0) / 2.0;
    return (avgDeg / 360.0) * encoderWheelCircumferenceCM;
}

// ======================================================================
// move — Open-loop drive for a fixed distance at maxSpeed%
// Non-blocking: motors coast to their target position, then the function returns.
// ======================================================================
void move(double distanceCM, double maxSpeed, bool reversed) {
    double targetRotations = distanceCM / wheelCircumferenceCM;

    // Coast mode so motors stop naturally when target rotation is reached
    leftDrive.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    rightDrive.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    // move_relative takes degrees; negate for reverse. Velocity in RPM (maxSpeed% of max).
    double degrees = targetRotations * 360.0;
    if (reversed) degrees = -degrees;
    leftDrive.move_relative(degrees, maxSpeed / 100.0 * absoluteMaxRPM);
    rightDrive.move_relative(degrees, maxSpeed / 100.0 * absoluteMaxRPM);
}

// Drive for a set distance with optional wall-stall abort.
// wallStalledTimeMs < 0 disables stall detection.
void smartMove(double distanceCM, double maxSpeed, bool reversed, double wallStalledTimeMs) {
    const double WALL_STOP_THRESHOLD_RPM = 5.0;  // RPM below which a side is considered stalled
    uint32_t wallStallStart = 0;                  // millis() timestamp when stall began
    bool wallDetected = false, wallDetectEnabled = (wallStalledTimeMs > 0), isCurrentlyStalled = false;

    // Coast so motors don't fight themselves mid-drive
    leftDrive.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    rightDrive.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    // Compute signed voltage: forward positive, reversed negative
    double voltage = maxSpeed * 0.01 * absoluteMaxVoltage;
    if (reversed) voltage = -voltage;

    double startDist = getCurrentEncoderDistanceCM();
    double distanceTravelled = 0;

    while (fabs(distanceTravelled) < fabs(distanceCM) && !wallDetected) {
        distanceTravelled = getCurrentEncoderDistanceCM() - startDist;
        double avgEncoderSpeed = (fabs(passiveEncoderLeft.get_velocity()) +
                                  fabs(passiveEncoderRight.get_velocity())) / 2.0;

        // Wall stall: if encoder barely moving, start (or extend) stall timer
        if (wallDetectEnabled) {
            if (avgEncoderSpeed < WALL_STOP_THRESHOLD_RPM) {
                if (!isCurrentlyStalled) { wallStallStart = pros::millis(); isCurrentlyStalled = true; }
                else if (pros::millis() - wallStallStart >= (uint32_t)wallStalledTimeMs) wallDetected = true;
            } else isCurrentlyStalled = false;
        }

        // Apply voltage to both sides (sign encodes direction)
        leftDrive.move_voltage((int32_t)(voltage * 1000));
        rightDrive.move_voltage((int32_t)(voltage * 1000));
        pros::delay(10);
    }
    // Brake stop
    leftDrive.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    leftDrive.move(0);
    rightDrive.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    rightDrive.move(0);
}

// ======================================================================
// turnOdometry — Point turn to absolute heading using motion profiling
//
// Accepts targetHeading in continuous VEX Coordinate space (North = 0°, CW+).
// Callers (turnRight / turnLeft) are responsible for adjusting targetHeading
// to the correct continuous value before passing it in — this function
// uses it as-is. Phases: LAUNCH → CRUISE → DECEL (adaptive ABS) → APPROACH.
// ======================================================================
void turnOdometry(double targetHeading, const TurnProfile& p) {
    bool decelCompleted = false;
    bool accelCompleted = false;
    bool decel = false;

    double currentHeading = getContinuousStandardHeading();

    // Caller has already set targetHeading to the correct continuous value;
    // assign directly and compute error from it.
    double targetRotationHeading = targetHeading;
    double headingError          = targetRotationHeading - currentHeading;

    pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Target Head: %.2f", targetHeading);
    pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Curr Rotation: %.2f", currentHeading);

    // copysign on voltage: positive headingError (CW turn) → negative maxSpeedVoltage,
    // which the motor output negation converts to a correct CW spin.
    double maxSpeedVoltage       = std::copysign(p.maxSpeed * 0.01 * absoluteMaxVoltage, -headingError);
    double minSpeedVoltage       = std::copysign(p.minSpeed * 0.01 * absoluteMaxVoltage, -headingError);
    double launchVoltage         = std::copysign(4, -headingError);
    double minLaunchSpeedVoltage = std::copysign(std::min(fabs(maxSpeedVoltage), fabs(launchVoltage)), -headingError);
    double minDriveMotorRPM      = (p.minSpeed * 0.01) * absoluteMaxRPM;

    double averageMotorVoltage  = 0;
    double motorVoltageLeft     = minLaunchSpeedVoltage;
    double motorVoltageRight    = minLaunchSpeedVoltage;

    double leftEncoderRollingAverage  = 0;
    double rightEncoderRollingAverage = 0;
    double voltageRollingAverage      = 0;

    adaptiveABS adaptiveABSLeft(p.decelStepPercent, p.lockThreshold);
    adaptiveABS adaptiveABSRight(p.decelStepPercent, p.lockThreshold);

    tractionControl tractionControlLeft(minLaunchSpeedVoltage, maxSpeedVoltage, p.slipThreshold);
    tractionControl tractionControlRight(minLaunchSpeedVoltage, maxSpeedVoltage, p.slipThreshold);

    // Loop to continuously adjust motor power
    while ((maxSpeedVoltage > 0 && currentHeading >= targetRotationHeading + p.exitTolerance) ||
           (maxSpeedVoltage < 0 && currentHeading <= targetRotationHeading - p.exitTolerance))
    {
        currentHeading = getContinuousStandardHeading();
        headingError = targetRotationHeading - currentHeading;

        pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Curr Rotation: %.2f", currentHeading);
        pros::screen::print(pros::E_TEXT_MEDIUM, 7, "Target: %.2f", targetHeading);

        // Read speeds; MotorGroup get_actual_velocity() returns average across all motors
        double leftMotorRPM  = fabs(leftDrive.get_actual_velocity())  * DRIVE_MOTOR_RPM_ADJ;
        double rightMotorRPM = fabs(rightDrive.get_actual_velocity()) * DRIVE_MOTOR_RPM_ADJ;

        // Encoder RPM scaled to the same units as drive motor RPM
        double leftEncoderRPM  = fabs(passiveEncoderLeft.get_velocity())  *
                                 (encoderWheelCircumferenceCM / wheelCircumferenceCM);
        double rightEncoderRPM = fabs(passiveEncoderRight.get_velocity()) *
                                 (encoderWheelCircumferenceCM / wheelCircumferenceCM);

        double averageEncoderRPM = (leftEncoderRPM + rightEncoderRPM) / 2.0;

        // Launch Phase
        if ((std::fabs(headingError) > fabs(p.breakDistance)) && !accelCompleted && !decel)
        {
            currentDrivePhase = PHASE_LAUNCH;
            double leftTractionVoltage  = tractionControlLeft.tractionControlSpeed(
                motorVoltageLeft, leftMotorRPM, averageEncoderRPM, p.accelFactor);
            double rightTractionVoltage = tractionControlRight.tractionControlSpeed(
                motorVoltageRight, rightMotorRPM, averageEncoderRPM, p.accelFactor);

            // Sync both sides to the slower of the two (prevents one side pulling ahead)
            double syncedMotorVoltage = std::min(fabs(leftTractionVoltage), fabs(rightTractionVoltage));
            motorVoltageLeft  = std::copysign(syncedMotorVoltage, motorVoltageLeft);
            motorVoltageRight = std::copysign(syncedMotorVoltage, motorVoltageRight);

            // Average of both sides — used to detect when cruise speed is reached
            averageMotorVoltage   = (fabs(motorVoltageLeft) + fabs(motorVoltageRight)) / 2.0;
            voltageRollingAverage = rollingAverage(averageMotorVoltage, voltageRollingAverage, 5);

            if (fabs(voltageRollingAverage) >= (fabs(maxSpeedVoltage) - VOLTAGE_TOLERANCE)) {
                accelCompleted = true;
            }
        }
        // Cruise Phase
        else if ((std::abs(headingError) > fabs(p.breakDistance)) && accelCompleted)
        {
            currentDrivePhase = PHASE_CRUISE;
            motorVoltageLeft  = maxSpeedVoltage;
            motorVoltageRight = maxSpeedVoltage;
        }
        // Decel Phase
        else if ((std::abs(headingError) <= fabs(p.breakDistance)) && !decelCompleted)
        {
            currentDrivePhase = PHASE_DECEL;
            if (!decel) {
                adaptiveABSLeft.initialize(motorVoltageLeft);
                adaptiveABSRight.initialize(motorVoltageRight);
            }
            decel = true;

            double leftMotorRPMDecel    = fabs(leftDrive.get_actual_velocity())    * DRIVE_MOTOR_RPM_ADJ;
            double rightMotorRPMDecel   = fabs(rightDrive.get_actual_velocity())   * DRIVE_MOTOR_RPM_ADJ;
            double leftEncoderRPMDecel  = fabs(passiveEncoderLeft.get_velocity())  * (encoderWheelCircumferenceCM / wheelCircumferenceCM);
            double rightEncoderRPMDecel = fabs(passiveEncoderRight.get_velocity()) * (encoderWheelCircumferenceCM / wheelCircumferenceCM);

            double leftDecelVoltage  = adaptiveABSLeft.decelControlSpeed(leftMotorRPMDecel,  leftEncoderRPMDecel);
            double rightDecelVoltage = adaptiveABSRight.decelControlSpeed(rightMotorRPMDecel, rightEncoderRPMDecel);

            pros::motor_brake_mode_e_t leftBrakeMode  = adaptiveABSLeft.getBrakeMode();
            pros::motor_brake_mode_e_t rightBrakeMode = adaptiveABSRight.getBrakeMode();

            // If either side wants to coast (lockup), sync both to coast
            pros::motor_brake_mode_e_t syncedBrakeMode =
                (leftBrakeMode == pros::E_MOTOR_BRAKE_COAST || rightBrakeMode == pros::E_MOTOR_BRAKE_COAST)
                ? pros::E_MOTOR_BRAKE_COAST
                : pros::E_MOTOR_BRAKE_BRAKE;

            double syncedDecelVoltage = std::max(fabs(leftDecelVoltage), fabs(rightDecelVoltage));

            leftDrive.set_brake_mode(syncedBrakeMode);
            rightDrive.set_brake_mode(syncedBrakeMode);

            // Apply decel voltage or zero-out and let brake mode hold
            if (syncedBrakeMode == pros::E_MOTOR_BRAKE_BRAKE && syncedDecelVoltage > 0.0) {
                motorVoltageLeft  = std::copysign(syncedDecelVoltage, motorVoltageLeft);
                motorVoltageRight = std::copysign(syncedDecelVoltage, motorVoltageRight);
            } else {
                motorVoltageLeft  = 0.0;
                motorVoltageRight = 0.0;
            }

            leftEncoderRollingAverage  = rollingAverage(leftEncoderRPMDecel,  leftEncoderRollingAverage,  3);
            rightEncoderRollingAverage = rollingAverage(rightEncoderRPMDecel, rightEncoderRollingAverage, 3);

            if (fabs(leftEncoderRollingAverage)  <= fabs(minDriveMotorRPM) &&
                fabs(rightEncoderRollingAverage) <= fabs(minDriveMotorRPM))
            {
                decelCompleted = true;
            }
        }
        // Final Approach
        else if (decelCompleted)
        {
            currentDrivePhase = PHASE_APPROACH;
            motorVoltageLeft  = minSpeedVoltage;
            motorVoltageRight = minSpeedVoltage;
        }

        // Apply voltages — left side is negated because it faces opposite direction to right
        leftDrive.move_voltage((int32_t)(-motorVoltageLeft  * 1000));
        rightDrive.move_voltage((int32_t)( motorVoltageRight * 1000));

        pros::delay(10);
    }

    // Final stop with brake
    currentDrivePhase = PHASE_IDLE;
    leftDrive.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    leftDrive.move(0);
    rightDrive.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    rightDrive.move(0);
}


tractionControl::tractionControl(double minSpeedVoltage, double maxSpeedVoltage, double slipThreshold) : minSpeedVoltage(minSpeedVoltage), maxSpeedVoltage(maxSpeedVoltage), slipThreshold(slipThreshold) {}
double tractionControl::tractionControlSpeed(double motorVoltage, double wheelSpeed, double robotSpeed, double accelFactor) {
    double slipRatio = calculateSlipRatio(wheelSpeed, robotSpeed);
    motorVoltage = (slipRatio > slipThreshold) ? motorVoltage / accelFactor : motorVoltage * accelFactor;
    return std::copysign(std::max(std::fabs(minSpeedVoltage), std::min(std::fabs(motorVoltage), std::fabs(maxSpeedVoltage))), motorVoltage);
}

adaptiveABS::adaptiveABS(double decelStepPercent, double lockThreshold)
    : lockThreshold(lockThreshold), lastAttemptedVoltage(0.0),
      wasLockedLastCycle(false), currentBrakeMode(pros::E_MOTOR_BRAKE_BRAKE) {
    decelStepVoltage = absoluteMaxVoltage * (decelStepPercent / 100.0);
}

// Resets ABS state at the start of a deceleration phase
void adaptiveABS::initialize(double startingVoltage) {
    lastAttemptedVoltage = startingVoltage;
    wasLockedLastCycle   = false;
    currentBrakeMode     = pros::E_MOTOR_BRAKE_BRAKE;
}

// Steps down voltage; switches to coast if lockup detected, back to brake once cleared
double adaptiveABS::decelControlSpeed(double wheelSpeed, double robotSpeed) {
    double lockupRatio = calculateLockupRatio(wheelSpeed, robotSpeed);
    if (lockupRatio > lockThreshold) {
        lastAttemptedVoltage = 0.0;
        currentBrakeMode     = pros::E_MOTOR_BRAKE_COAST;
        wasLockedLastCycle   = true;
    } else if (wasLockedLastCycle) {
        currentBrakeMode   = pros::E_MOTOR_BRAKE_BRAKE;
        wasLockedLastCycle = false;
    } else {
        lastAttemptedVoltage = std::copysign(
            std::max(0.0, std::fabs(lastAttemptedVoltage) - decelStepVoltage),
            lastAttemptedVoltage);
        currentBrakeMode = pros::E_MOTOR_BRAKE_BRAKE;
    }
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
        // Ramp up before break point, ramp down after
        if (fabs(distanceTravelled) < (fabs(targetDistance) - breakDistance)) {
            innerV = std::min(maxV * innerRatio, innerV + 0.1);
            outerV = std::min(maxV * outerRatio, outerV + 0.1);
        } else {
            innerV = std::max(minV * innerRatio, innerV - 0.1);
            outerV = std::max(minV * outerRatio, outerV - 0.1);
        }
        // Left turn: left is inner (slower), right is outer (faster); vice versa for right
        leftDrive.move_voltage((int32_t)((turnLeft ? innerV : outerV) * 1000));
        rightDrive.move_voltage((int32_t)((turnLeft ? outerV : innerV) * 1000));
        pros::delay(20);
    }
    leftDrive.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    leftDrive.move(0);
    rightDrive.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    rightDrive.move(0);
}

// ======================================================================
// straightDistance — Open-loop motion-profiled straight drive
//
// Drives targetDistance using encoder tracking with heading PID correction.
// Does not use field position — all tracking is relative to function entry.
// Phase: LAUNCH → CRUISE → DECEL (adaptive ABS) → APPROACH.
//
// @param targetDistance  cm to travel (positive = forward, negative = backward)
// @param targetHeading   heading to hold (VEX Coordinates, North = 0°, CW+)
// @param p               motion profile
// ======================================================================
void straightDistance(double targetDistance, double targetHeading, const StraightProfile& p)
{
    // ========================================
    // INITIALIZATION
    // ========================================

    // Record starting encoder position to measure relative distance traveled
    double startDist         = getCurrentEncoderDistanceCM();
    double distanceTravelled = 0.0;

    // Direction scalar for forward (+1) or backward (-1) movement
    double dir = (targetDistance >= 0) ? 1.0 : -1.0;

    // Adjust target heading to nearest equivalent angle in continuous frame
    // Prevents large rotations when robot heading has wrapped past 360°
    double currentHeadingInitial = getContinuousStandardHeading();
    double rotationsDiff = std::round((currentHeadingInitial - targetHeading) / 360.0);
    targetHeading += rotationsDiff * 360.0;

    // Initialize PID controller for heading correction
    PID headingPID(p.kp_heading, p.ki_heading, p.kd_heading);
    headingPID.pidReset();

    // Convert speed percentages to voltages
    double maxSpeedVoltage       = std::copysign(p.maxSpeed * 0.01 * absoluteMaxVoltage, targetDistance);
    double minSpeedVoltage       = std::copysign(p.minSpeed * 0.01 * absoluteMaxVoltage, targetDistance);
    double minLaunchSpeedVoltage = std::copysign(std::min(fabs(maxSpeedVoltage), fabs(p.launchVoltage)), targetDistance);

    // Calculate minimum RPM threshold for deceleration exit detection
    double minDriveMotorRPM = (p.minSpeed * 0.01) * absoluteMaxRPM;

    // Per-side voltage tracking — plain doubles, one per side
    // Separate traction state from PID-corrected output — prevents heading correction
    // from corrupting the traction ramp direction on the next tick
    double tractionVoltageLeft  = minLaunchSpeedVoltage;
    double tractionVoltageRight = minLaunchSpeedVoltage;
    double motorVoltageLeft     = minLaunchSpeedVoltage;
    double motorVoltageRight    = minLaunchSpeedVoltage;

    // Phase tracking flags
    bool decel          = false;
    bool decelCompleted = false;
    bool accelCompleted = false;

    // Rolling averages for stable speed detection during deceleration
    double leftEncoderRollingAverage  = 0;
    double rightEncoderRollingAverage = 0;

    // Traction control instances for independent per-side slip management
    tractionControl tractionControlLeft(minLaunchSpeedVoltage, maxSpeedVoltage, p.slipThreshold);
    tractionControl tractionControlRight(minLaunchSpeedVoltage, maxSpeedVoltage, p.slipThreshold);

    // ABS controllers for independent per-side wheel lockup prevention
    adaptiveABS adaptiveABSLeft(p.decelStepPercent, p.lockThreshold);
    adaptiveABS adaptiveABSRight(p.decelStepPercent, p.lockThreshold);

    // Timeout safety: record entry time, compare each tick
    uint32_t safetyStart = pros::millis();
    double timeoutMs     = p.timeout * 1000.0;

    // ═══════════════════════════════════════════════════════════════════
    // MAIN CONTROL LOOP
    // ═══════════════════════════════════════════════════════════════════
    while (std::fabs(distanceTravelled) <= std::fabs(targetDistance) - p.distanceTolerance)
    {
        // Update current position and heading
        distanceTravelled = getCurrentEncoderDistanceCM() - startDist;
        double currentHeading    = getContinuousStandardHeading();
        double headingCorrection = headingPID.calculate(targetHeading, currentHeading);

        // ───────────────────────────────────────────────────────────────
        // TIMEOUT SAFETY CHECK
        // ───────────────────────────────────────────────────────────────
        if (pros::millis() - safetyStart > (uint32_t)timeoutMs) {
            break;  // Prevent infinite loop on sensor failure or unreachable target
        }

        // Read encoder speeds (ground truth) scaled by wheel size ratio
        double leftEncoderRPM  = passiveEncoderLeft.get_velocity()  * (encoderWheelCircumferenceCM / wheelCircumferenceCM);
        double rightEncoderRPM = passiveEncoderRight.get_velocity() * (encoderWheelCircumferenceCM / wheelCircumferenceCM);

        // MotorGroup get_actual_velocity() returns average across all motors on that side
        double leftMotorRPM  = leftDrive.get_actual_velocity()  * DRIVE_MOTOR_RPM_ADJ;
        double rightMotorRPM = rightDrive.get_actual_velocity() * DRIVE_MOTOR_RPM_ADJ;

        // ───────────────────────────────────────────────────────────────
        // PHASE 1: LAUNCH / ACCELERATION
        // Per-side traction control with slip ratio calculation
        // ───────────────────────────────────────────────────────────────
        if (std::fabs(distanceTravelled) < (std::fabs(targetDistance) - p.breakDistance) &&
            !accelCompleted && !decel)
        {
            currentDrivePhase = PHASE_LAUNCH;

            // Update traction state independently — heading correction does not feed back in
            tractionVoltageLeft  = tractionControlLeft.tractionControlSpeed(
                tractionVoltageLeft, leftMotorRPM, leftEncoderRPM, p.accelFactor);
            tractionVoltageRight = tractionControlRight.tractionControlSpeed(
                tractionVoltageRight, rightMotorRPM, rightEncoderRPM, p.accelFactor);

            // Apply heading correction on top of traction base — forgotten next tick
            motorVoltageLeft  = tractionVoltageLeft  + (headingCorrection * p.accelHeadingScaling);
            motorVoltageRight = tractionVoltageRight - (headingCorrection * p.accelHeadingScaling);

            // Exit condition checks traction base, not PID-skewed motor output
            if (std::fabs(tractionVoltageLeft)  >= std::fabs(maxSpeedVoltage) &&
                std::fabs(tractionVoltageRight) >= std::fabs(maxSpeedVoltage))
            {
                accelCompleted = true;
            }
        }

        // ───────────────────────────────────────────────────────────────
        // PHASE 2: CRUISE
        // Maintain maximum speed with heading correction
        // ───────────────────────────────────────────────────────────────
        else if (std::fabs(distanceTravelled) < (std::fabs(targetDistance) - p.breakDistance) &&
                 accelCompleted)
        {
            currentDrivePhase = PHASE_CRUISE;
            motorVoltageLeft  = maxSpeedVoltage + headingCorrection;
            motorVoltageRight = maxSpeedVoltage - headingCorrection;
        }

        // ───────────────────────────────────────────────────────────────
        // PHASE 3: DECELERATION (Adaptive ABS)
        // Independent per-side brake control with rolling average exit detection
        // ───────────────────────────────────────────────────────────────
        else if (std::fabs(distanceTravelled) >= (std::fabs(targetDistance) - p.breakDistance) &&
                 !decelCompleted)
        {
            currentDrivePhase = PHASE_DECEL;

            // Initialize ABS controllers on first entry to deceleration phase
            if (!decel) {
                decel = true;
                adaptiveABSLeft.initialize(std::fabs(motorVoltageLeft));
                adaptiveABSRight.initialize(std::fabs(motorVoltageRight));
            }

            // Update ABS state based on wheel lockup detection
            adaptiveABSLeft.decelControlSpeed(leftMotorRPM, leftEncoderRPM);
            adaptiveABSRight.decelControlSpeed(rightMotorRPM, rightEncoderRPM);

            // Get independent brake modes for each side (coast = locked wheel, brake = normal)
            pros::motor_brake_mode_e_t leftBrakeMode  = adaptiveABSLeft.getBrakeMode();
            pros::motor_brake_mode_e_t rightBrakeMode = adaptiveABSRight.getBrakeMode();

            // Scale heading correction for deceleration phase.
            // dir is NOT applied here — dirSign handles motor direction at output.
            // Applying dir here would reverse the steering correction for backward driving.
            double adjustedHeadingCorrection = headingCorrection * p.decelHeadingScaling;

            // Set per-side brake modes; apply selective-release steering voltages
            leftDrive.set_brake_mode(leftBrakeMode);
            rightDrive.set_brake_mode(rightBrakeMode);

            // Selective release: only apply positive correction values for steering
            motorVoltageLeft  = std::max(0.0,  adjustedHeadingCorrection);
            motorVoltageRight = std::max(0.0, -adjustedHeadingCorrection);

            // Rolling averages filter momentary encoder spikes
            leftEncoderRollingAverage  = rollingAverage(leftEncoderRPM,  leftEncoderRollingAverage,  3);
            rightEncoderRollingAverage = rollingAverage(rightEncoderRPM, rightEncoderRollingAverage, 3);

            // Both sides must slow below threshold before transitioning to approach
            if (std::fabs(leftEncoderRollingAverage)  <= std::fabs(minDriveMotorRPM) &&
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
            currentDrivePhase = PHASE_APPROACH;

            // Reset to standard brake for precision control
            leftDrive.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
            rightDrive.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

            // Minimum speed with heading correction blended in
            motorVoltageLeft  = minSpeedVoltage + (headingCorrection * p.approachHeadingScaling);
            motorVoltageRight = minSpeedVoltage - (headingCorrection * p.approachHeadingScaling);
        }

        // ───────────────────────────────────────────────────────────────
        // VOLTAGE SATURATION LIMITER
        // When heading correction pushes one side above absoluteMaxVoltage, proportionally
        // scale both sides down to preserve the steering differential within limits
        // ───────────────────────────────────────────────────────────────
        double maximumRequestedVoltage = std::max(std::fabs(motorVoltageLeft),
                                                   std::fabs(motorVoltageRight));
        if (maximumRequestedVoltage > absoluteMaxVoltage) {
            double voltageScaleFactor = absoluteMaxVoltage / maximumRequestedVoltage;
            motorVoltageLeft  *= voltageScaleFactor;
            motorVoltageRight *= voltageScaleFactor;
        }

        // Apply calculated voltages to both drive sides (convert V → mV)
        leftDrive.move_voltage(static_cast<int32_t>(motorVoltageLeft  * 1000));
        rightDrive.move_voltage(static_cast<int32_t>(motorVoltageRight * 1000));

        pros::delay(10);  // 100Hz control loop
    }

    // ═══════════════════════════════════════════════════════════════════
    // CLEANUP - Stop all motors with the requested brake mode
    // ═══════════════════════════════════════════════════════════════════
    currentDrivePhase = PHASE_IDLE;
    leftDrive.set_brake_mode(p.brakeMode);
    rightDrive.set_brake_mode(p.brakeMode);
    leftDrive.move(0);
    rightDrive.move(0);
}

// ======================================================================
// smartStraight — Straight drive with heading PID and optional wall stall detection
//
// Drives targetDistance at maxSpeed with heading correction applied each tick.
// Wall stall detection (wallStalledTimeMs > 0) monitors encoder RPM; if both
// sides fall below WALL_THRESHOLD for the full wallStalledTimeMs, the loop exits.
// ======================================================================
void smartStraight(double targetDistance, double breakDistance, double targetHeading,
                   double wallStalledTimeMs, const StraightProfile& p)
{
    const double WALL_THRESHOLD = 5.0;
    uint32_t wallTimerStart = 0;
    bool wallDetected = false, isStalled = false;

    double startDist         = getCurrentEncoderDistanceCM();
    double distanceTravelled = 0;

    // Snap targetHeading to the nearest equivalent in continuous rotation space —
    // prevents PID unwinding when the robot heading has crossed a 360° boundary.
    double targetHeadingStandard = targetHeading;
    double currentHeading        = getContinuousStandardHeading();
    double rotationsDiff         = std::round((currentHeading - targetHeadingStandard) / 360.0);
    targetHeadingStandard += rotationsDiff * 360.0;

    PID headingPID(p.kp_heading, p.ki_heading, p.kd_heading);
    headingPID.pidReset();

    // Set max/min voltage with correct sign for forward/backward
    double maxV = std::copysign(p.maxSpeed * 0.12, targetDistance);
    double minV = std::copysign(p.minSpeed * 0.12, targetDistance);

    double motorVoltageLeft  = minV;
    double motorVoltageRight = minV;

    while (std::fabs(distanceTravelled) <= std::fabs(targetDistance) - 6.9 && !wallDetected)
    {
        distanceTravelled = getCurrentEncoderDistanceCM() - startDist;

        // Update heading
        currentHeading = getContinuousStandardHeading();
        double headingCorrection = headingPID.calculate(targetHeadingStandard, currentHeading);

        double avgRPM = (std::fabs(passiveEncoderLeft.get_velocity()) +
                         std::fabs(passiveEncoderRight.get_velocity())) / 2.0;

        // Wall stall detection
        if (wallStalledTimeMs > 0) {
            if (avgRPM < WALL_THRESHOLD) {
                if (!isStalled) { wallTimerStart = pros::millis(); isStalled = true; }
                else if (pros::millis() - wallTimerStart >= (uint32_t)wallStalledTimeMs) wallDetected = true;
            } else isStalled = false;
        }

        // Positive correction adds to left, subtracts from right → steers right
        motorVoltageLeft  = maxV + headingCorrection;
        motorVoltageRight = maxV - headingCorrection;

        leftDrive.move_voltage(static_cast<int32_t>(motorVoltageLeft  * 1000));
        rightDrive.move_voltage(static_cast<int32_t>(motorVoltageRight * 1000));

        pros::delay(10);
    }

    // Stop with brake
    leftDrive.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    rightDrive.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    leftDrive.move(0);
    rightDrive.move(0);
}

// ======================================================================
// driveForward — forward drive, heading defaults to 0 (hold current heading)
// driveBackward — backward drive, negates distance
// Both call straightDistance — thin wrappers for coder convenience.
// ======================================================================
void driveForward(double targetDistance, double targetHeading, const StraightProfile& p) {
    straightDistance(targetDistance, targetHeading, p);
}

void driveBackward(double targetDistance, double targetHeading, const StraightProfile& p) {
    straightDistance(-std::fabs(targetDistance), targetHeading, p);
}

// ======================================================================
// V2: Modernized pivot turn to target heading (continuous VEX Coordinate space)
// One side brakes while the other drives (true pivot — zero turn radius).
// Uses motion profiling: LAUNCH → CRUISE → DECEL → APPROACH.
// ======================================================================
void pivotTurnOdometryV2(double targetHeading, const TurnProfile& p)
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

    // Calculate initial error to determine direction
    double startError      = targetHeading - currentHeadingInitial;
    double maxSpeedVoltage = std::copysign(p.maxSpeed * 0.01 * absoluteMaxVoltage, -startError);
    double minSpeedVoltage = std::copysign(p.minSpeed * 0.01 * absoluteMaxVoltage, -startError);
    double launchVoltage   = std::copysign(5.0, maxSpeedVoltage);  // gentle launch kick

    double minDriveMotorRPM = (p.minSpeed * 0.01) * absoluteMaxRPM;

    // Per-side voltage tracking — one side will be 0 during pivot
    double motorVoltageLeft  = 0;
    double motorVoltageRight = 0;

    // Main loop — continue until within tolerance
    while (true)
    {
        double currentHeading = getContinuousStandardHeading();
        double headingError   = targetHeading - currentHeading;

        // Exit condition
        if (std::fabs(headingError) <= p.exitTolerance) break;

        // ───────────────────────────────────────────────
        // LAUNCH / ACCEL PHASE
        // ───────────────────────────────────────────────
        if (std::fabs(headingError) > p.breakDistance && !accelCompleted && !decel)
        {
            currentDrivePhase = PHASE_LAUNCH;
            double targetVolt = std::fabs(maxSpeedVoltage);

            // Get current highest voltage being applied — used to track ramp progress
            double currentMax = std::max(std::fabs(motorVoltageLeft), std::fabs(motorVoltageRight));

            if (currentMax < std::fabs(launchVoltage)) {
                currentMax = std::fabs(launchVoltage);
            } else {
                currentMax = std::min(targetVolt, currentMax + 0.5);  // smooth ramp up
            }

            double pivotVolt = std::copysign(currentMax, -headingError);

            // Assign driving side based on turn direction:
            // CW (negative pivotVolt) → left side drives, right side brakes.
            // CCW (positive pivotVolt) → right side drives, left side brakes.
            if (pivotVolt > 0) {  // Pivot LEFT (CCW)
                motorVoltageLeft  = 0;
                motorVoltageRight = pivotVolt;
            } else {              // Pivot RIGHT (CW)
                motorVoltageLeft  = std::fabs(pivotVolt);
                motorVoltageRight = 0;
            }

            if (currentMax >= std::fabs(maxSpeedVoltage) - VOLTAGE_TOLERANCE) {
                accelCompleted = true;
            }
        }
        // ───────────────────────────────────────────────
        // CRUISE PHASE (full speed pivot)
        // ───────────────────────────────────────────────
        else if (std::fabs(headingError) > p.breakDistance && accelCompleted)
        {
            currentDrivePhase = PHASE_CRUISE;
            double pivotVolt  = maxSpeedVoltage;

            if (pivotVolt > 0) {  // Pivot LEFT (CCW)
                motorVoltageLeft  = 0;
                motorVoltageRight = pivotVolt;
            } else {              // Pivot RIGHT (CW)
                motorVoltageLeft  = std::fabs(pivotVolt);
                motorVoltageRight = 0;
            }
        }
        // ───────────────────────────────────────────────
        // DECELERATION PHASE
        // ───────────────────────────────────────────────
        else if (std::fabs(headingError) <= p.breakDistance && !decelCompleted)
        {
            currentDrivePhase = PHASE_DECEL;
            if (!decel) decel = true;

            double currentMax  = std::max(std::fabs(motorVoltageLeft), std::fabs(motorVoltageRight));
            double syncedDecel = std::max(0.0, currentMax - 0.4);  // adjustable decel step

            double pivotVolt = std::copysign(syncedDecel, -headingError);

            if (pivotVolt > 0) {  // Pivot LEFT (CCW)
                motorVoltageLeft  = 0;
                motorVoltageRight = pivotVolt;
            } else {              // Pivot RIGHT (CW)
                motorVoltageLeft  = std::fabs(pivotVolt);
                motorVoltageRight = 0;
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
            currentDrivePhase = PHASE_APPROACH;
            double pivotVolt  = minSpeedVoltage;

            if (pivotVolt > 0) {  // Pivot LEFT (CCW)
                motorVoltageLeft  = 0;
                motorVoltageRight = pivotVolt;
            } else {              // Pivot RIGHT (CW)
                motorVoltageLeft  = std::fabs(pivotVolt);
                motorVoltageRight = 0;
            }
        }

        // ───────────────────────────────────────────────
        // APPLY MOTOR COMMANDS (true pivot — one side braked, one side driven)
        // ───────────────────────────────────────────────
        if (motorVoltageLeft != 0) {
            leftDrive.move_voltage(static_cast<int32_t>(motorVoltageLeft * 1000));
        } else {
            leftDrive.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
            leftDrive.move(0);
        }

        if (motorVoltageRight != 0) {
            rightDrive.move_voltage(static_cast<int32_t>(motorVoltageRight * 1000));
        } else {
            rightDrive.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
            rightDrive.move(0);
        }

        pros::delay(10);
    }

    // Final stop
    currentDrivePhase = PHASE_IDLE;
    leftDrive.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    rightDrive.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    leftDrive.move(0);
    rightDrive.move(0);
}

// ======================================================================
// pivotLeftMP / pivotRightMP — Relative pivot turns
// turnAmount is in degrees relative to the current heading.
// Converts to an absolute continuous heading and delegates to pivotTurnOdometryV2.
// ======================================================================
void pivotLeftMP(double turnAmount, const TurnProfile& p) {
    double currentHeading = getContinuousStandardHeading();
    pivotTurnOdometryV2(currentHeading - turnAmount, p);
}

void pivotRightMP(double turnAmount, const TurnProfile& p) {
    double currentHeading = getContinuousStandardHeading();
    pivotTurnOdometryV2(currentHeading + turnAmount, p);
}

// ======================================================================
// turnRight — Force a CW turn to an absolute field heading.
// Adds 360° until target sits above current — creates positive error → CW turn.
// ======================================================================
void turnRight(double absoluteTargetHeading, const TurnProfile& p) {
    double target  = absoluteTargetHeading;
    double current = getContinuousStandardHeading();
    while (target <= current + 0.5) target += 360.0;
    turnOdometry(target, p);
}

// ======================================================================
// turnLeft — Force a CCW turn to an absolute field heading.
// Subtracts 360° until target sits below current — creates negative error → CCW turn.
// ======================================================================
void turnLeft(double absoluteTargetHeading, const TurnProfile& p) {
    double target  = absoluteTargetHeading;
    double current = getContinuousStandardHeading();
    while (target >= current - 0.5) target -= 360.0;
    turnOdometry(target, p);
}

void pidlessForward(double timeMs, double speedPct) {
    uint32_t forwardStart = pros::millis();
    int32_t mv = static_cast<int32_t>(speedPct / 100.0 * 12000.0);
    while (pros::millis() - forwardStart < (uint32_t)timeMs) {
        leftDrive.move_voltage(mv);
        rightDrive.move_voltage(mv);
    }
    leftDrive.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    rightDrive.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    leftDrive.move(0);
    rightDrive.move(0);
}


// ─── SHARED IMPLEMENTATION ────────────────────────────────────────────────────
// T = pros::vision_signature_s_t  →  reads with get_by_sig(0, sig.id)
// T = pros::vision_color_code_t   →  reads with get_by_code(0, code)
// ======================================================================
// visionDriveMinimal_impl — Shared template core for simplified vision approach.
//
// Two PIDs: distance (pixel width → forward speed) and heading (screen offset
// → steering). Object to the right produces a negative steer value.
// Exits when targetPixelWidth is reached or object lost for MAX_LOST_FRAMES.
// ======================================================================
template<typename T>
static void visionDriveMinimal_impl(
    T targetSignature,
    int targetPixelWidth, double targetHeading,
    double minSpeedPct, double maxSpeedPct, pros::motor_brake_mode_e_t brakeMode,
    double kp_head, double ki_head, double kd_head,
    double kp_distToHeadScaling,
    double kp_dist, double ki_dist, double kd_dist)
{
    PID headingPID(kp_head, ki_head, kd_head);
    PID distancePID(kp_dist, ki_dist, kd_dist);
    headingPID.pidReset();
    distancePID.pidReset();

    const int MAX_LOST_FRAMES = 15;
    int  lostFrameCounter  = 0;
    bool hasDetectedBefore = false;
    double lastNormOffset  = 0.0;
    int    lastWidth       = 0;

    while (true) {
        pros::vision_object_s_t obj;
        if constexpr (std::is_same_v<T, pros::vision_signature_s_t>) {
            obj = aiVision.get_by_sig(0, targetSignature.id);
        } else {
            obj = aiVision.get_by_code(0, targetSignature);
        }

        bool detected = (obj.signature != VISION_OBJECT_ERR_SIG && obj.width > 0);

        double normOffset;
        int    width;

        if (!detected) {
            if (++lostFrameCounter > MAX_LOST_FRAMES) break;
            if (!hasDetectedBefore) { pros::delay(20); continue; }
            normOffset = lastNormOffset;
            width      = lastWidth;
        } else {
            lostFrameCounter  = 0;
            normOffset        = (obj.x_middle_coord - VISION_CENTER_X) / VISION_CENTER_X;
            width             = obj.width;
            lastNormOffset    = normOffset;
            lastWidth         = width;
            hasDetectedBefore = true;
            if (width >= targetPixelWidth) break;
        }

        double distError = (double)targetPixelWidth - (double)width;
        double headScale = std::max(0.2, std::min(3.0, 0.2 + kp_distToHeadScaling * distError));
        double steer     = headingPID.calculate(0.0, normOffset) * 12.0 * headScale;
        steer            = std::max(-3.0, std::min(3.0, steer));

        double drive = distancePID.calculate((double)targetPixelWidth, (double)width) * 0.12;
        drive        = std::max(minSpeedPct * 0.12, std::min(maxSpeedPct * 0.12, drive));

        double leftV  = drive - steer;
        double rightV = drive + steer;
        double maxV   = std::max(std::fabs(leftV), std::fabs(rightV));
        if (maxV > 12.0) { leftV *= 12.0 / maxV; rightV *= 12.0 / maxV; }

        leftDrive.move_voltage(static_cast<int32_t>(leftV  * 1000.0));
        rightDrive.move_voltage(static_cast<int32_t>(rightV * 1000.0));
        pros::delay(20);
    }

    leftDrive.set_brake_mode(brakeMode);
    rightDrive.set_brake_mode(brakeMode);
    leftDrive.move(0);
    rightDrive.move(0);
}

// ─── PUBLIC OVERLOAD: vision_signature_s_t (single color object) ──────────
void visionDriveMinimal(
    pros::vision_signature_s_t targetSignature,
    int targetPixelWidth, double targetHeading,
    double minSpeedPct, double maxSpeedPct, pros::motor_brake_mode_e_t brakeMode,
    double kp_head, double ki_head, double kd_head,
    double kp_distToHeadScaling,
    double kp_dist, double ki_dist, double kd_dist)
{
    visionDriveMinimal_impl(targetSignature, targetPixelWidth, targetHeading,
        minSpeedPct, maxSpeedPct, brakeMode,
        kp_head, ki_head, kd_head, kp_distToHeadScaling,
        kp_dist, ki_dist, kd_dist);
}

// ─── PUBLIC OVERLOAD: vision_color_code_t (color combination object) ──────
void visionDriveMinimal(
    pros::vision_color_code_t targetSignature,
    int targetPixelWidth, double targetHeading,
    double minSpeedPct, double maxSpeedPct, pros::motor_brake_mode_e_t brakeMode,
    double kp_head, double ki_head, double kd_head,
    double kp_distToHeadScaling,
    double kp_dist, double ki_dist, double kd_dist)
{
    visionDriveMinimal_impl(targetSignature, targetPixelWidth, targetHeading,
        minSpeedPct, maxSpeedPct, brakeMode,
        kp_head, ki_head, kd_head, kp_distToHeadScaling,
        kp_dist, ki_dist, kd_dist);
}

// ======================================================================
// visionDriveV2 - Advanced AI Vision tracking with Priority Scaling
//
// Drives toward a detected signature while centering it on screen.
// Uses a Priority Scaler to preserve steering at max power.
// ======================================================================
void visionDriveV2(
    pros::vision_signature_s_t targetSignature,
    int targetPixelWidth, double targetHeading,
    pros::motor_brake_mode_e_t brakeMode,
    double maxSpeedPct, double kp_head, double ki_head, double kd_head,
    double kp_distToHeadScaling,
    int minObjectWidth, int minX, int maxX, int minY, int maxY,
    double minSpeedPct, double timeoutDistanceCM,
    double kp_dist, double ki_dist, double kd_dist)
{
    double maxSteeringPct = 25.0;
    const int MAX_OBJECTS_TO_CHECK = 3;
    const int MAX_LOST_FRAMES = 15;

    PID headingPID(kp_head, ki_head, kd_head);
    PID distancePID(kp_dist, ki_dist, kd_dist);
    headingPID.pidReset();
    distancePID.pidReset();

    double maxSteeringVoltage = maxSteeringPct * 0.12;

    bool   hasDetectedBefore    = false;
    double lastNormalizedOffset = 0.0;
    int    lastDetectedWidth    = 0;
    int    lostFrameCounter     = 0;

    while (true) {
        int bestObjectIndex = -1;
        int largestWidth    = 0;

        // --- SECTION 1: FILTERING ---
        for (int i = 0; i < MAX_OBJECTS_TO_CHECK; i++) {
            pros::vision_object_s_t obj = aiVision.get_by_sig(i, targetSignature.id);
            if (obj.signature == VISION_OBJECT_ERR_SIG) break;

            int bottomY = obj.y_middle_coord + (obj.height / 2);
            if (obj.width < minObjectWidth) continue;
            if (bottomY < minY || bottomY > maxY) continue;
            if (obj.x_middle_coord < minX || obj.x_middle_coord > maxX) continue;

            if (obj.width > largestWidth) {
                largestWidth    = obj.width;
                bestObjectIndex = i;
            }
        }

        double currentNormalizedOffset;
        int    currentWidth;

        // --- SECTION 2: DETECTION & FALLBACK ---
        if (bestObjectIndex == -1) {
            lostFrameCounter++;
            if (lostFrameCounter > MAX_LOST_FRAMES) break;
            if (hasDetectedBefore) {
                currentNormalizedOffset = lastNormalizedOffset;
                currentWidth            = lastDetectedWidth;
            } else {
                pros::delay(20);
                continue;
            }
        } else {
            lostFrameCounter = 0;
            pros::vision_object_s_t detectedObject = aiVision.get_by_sig(bestObjectIndex, targetSignature.id);
            currentNormalizedOffset = (detectedObject.x_middle_coord - VISION_CENTER_X) / VISION_CENTER_X;
            currentWidth            = detectedObject.width;
            lastNormalizedOffset    = currentNormalizedOffset;
            lastDetectedWidth       = currentWidth;
            hasDetectedBefore       = true;
            if (currentWidth >= targetPixelWidth) break;
        }

        // --- SECTION 3: CALCULATIONS ---
        double distanceErrorPixels  = (double)targetPixelWidth - (double)currentWidth;
        double headingScalingFactor = 0.2 + (kp_distToHeadScaling * distanceErrorPixels);
        headingScalingFactor        = std::max(0.2, std::min(3.0, headingScalingFactor));

        double steeringCorrection = (headingPID.calculate(0.0, currentNormalizedOffset) * 12.0) * headingScalingFactor;
        steeringCorrection        = std::max(-maxSteeringVoltage, std::min(maxSteeringVoltage, steeringCorrection));

        double baseDriveVoltage = distancePID.calculate((double)targetPixelWidth, (double)currentWidth) * 0.12;
        double clampedDrive     = std::max(minSpeedPct * 0.12, std::min(maxSpeedPct * 0.12, baseDriveVoltage));

        // --- SECTION 4: PRIORITY SCALER ---
        double leftRequest  = clampedDrive - steeringCorrection;
        double rightRequest = clampedDrive + steeringCorrection;
        double maxRequest   = std::max(std::fabs(leftRequest), std::fabs(rightRequest));
        if (maxRequest > 12.0) {
            double scaleFactor = 12.0 / maxRequest;
            leftRequest  *= scaleFactor;
            rightRequest *= scaleFactor;
        }

        // --- SECTION 5: MOTOR OUTPUT ---
        leftDrive.move_voltage(static_cast<int32_t>(leftRequest  * 1000.0));
        rightDrive.move_voltage(static_cast<int32_t>(rightRequest * 1000.0));
        pros::delay(20);
    }

    leftDrive.set_brake_mode(brakeMode);
    rightDrive.set_brake_mode(brakeMode);
    leftDrive.move(0);
    rightDrive.move(0);
}


// ======================================================================
// forwardToPoint — Closed-loop point-to-point drive using live odometry.
//
// Re-computes heading and remaining distance to (targetX, targetY) each tick.
//
// backwardToPoint calls this with reversed = true. In that case the heading
// is flipped 180° once before the loop so the robot's rear faces the target,
// and all phase voltages are negated so the robot drives backward. The heading
// PID steers correctly throughout — positive correction still steers right
// relative to the robot's facing direction.
// ======================================================================
// ======================================================================
// forwardToPoint — Closed-loop point-to-point drive using live odometry.
//
// Re-computes heading and remaining distance to (targetX, targetY) each tick.
// Drives forward only. Use backwardToPoint for reverse approach.
// ======================================================================
void forwardToPoint(double targetX, double targetY, const StraightProfile& p, bool /*reversed*/)
{
    const int REQUIRED_CONSECUTIVE_STOPS = 3;

    updateOdometry();
    double startCoordinateX = globalX;
    double startCoordinateY = globalY;

    double pathVectorX     = targetX - startCoordinateX;
    double pathVectorY     = targetY - startCoordinateY;

    PID headingPID(p.kp_heading, p.ki_heading, p.kd_heading);
    headingPID.pidReset();

    double maxSpeedVoltage       = p.maxSpeed * 0.01 * absoluteMaxVoltage;
    double minSpeedVoltage       = p.minSpeed * 0.01 * absoluteMaxVoltage;
    double minLaunchSpeedVoltage = std::min(maxSpeedVoltage, p.launchVoltage);
    double minDriveMotorRPM      = (p.minSpeed * 0.01) * absoluteMaxRPM;

    // Separate traction state from PID-corrected output — prevents heading correction
    // from corrupting the traction ramp direction on the next tick
    double tractionVoltageLeft  = minLaunchSpeedVoltage;
    double tractionVoltageRight = minLaunchSpeedVoltage;
    double motorVoltageLeft     = minLaunchSpeedVoltage;
    double motorVoltageRight    = minLaunchSpeedVoltage;

    bool decel          = false;
    bool decelCompleted = false;
    bool accelCompleted = false;

    double leftEncoderRollingAverage  = 0;
    double rightEncoderRollingAverage = 0;
    int consecutiveAtTargetCount = 0;
    bool   headingLocked      = false;
    double lockedHeadingValue = 0;

    tractionControl tractionControlLeft(minLaunchSpeedVoltage, maxSpeedVoltage, p.slipThreshold);
    tractionControl tractionControlRight(minLaunchSpeedVoltage, maxSpeedVoltage, p.slipThreshold);
    adaptiveABS adaptiveABSLeft(p.decelStepPercent, p.lockThreshold);
    adaptiveABS adaptiveABSRight(p.decelStepPercent, p.lockThreshold);

    uint32_t safetyStart = pros::millis();
    double timeoutMs     = p.timeout * 1000.0;

    while (true)
    {
        updateOdometry();

        // ── 1. STATE CALCULATION ───────────────────────────────────────
        double currentDistanceToTarget, odometryTargetHeading;
        calculatePathToTarget(globalX, globalY, targetX, targetY, currentDistanceToTarget, odometryTargetHeading);
        double currentGyroHeading = getContinuousStandardHeading();

        // ── 2. TIMEOUT ─────────────────────────────────────────────────
        if (pros::millis() - safetyStart > (uint32_t)timeoutMs) break;

        // ── 3. EXIT CONDITIONS ─────────────────────────────────────────
        if (currentDistanceToTarget <= p.distanceTolerance) {
            if (++consecutiveAtTargetCount >= REQUIRED_CONSECUTIVE_STOPS) break;
        } else {
            consecutiveAtTargetCount = 0;
        }

        double vectorToTargetX = targetX - globalX;
        double vectorToTargetY = targetY - globalY;
        if ((pathVectorX * vectorToTargetX) + (pathVectorY * vectorToTargetY) < 0) break;

        // ── 4. HEADING CALCULATION ─────────────────────────────────────
        double rotationsDifference = std::round((currentGyroHeading - odometryTargetHeading) / 360.0);
        odometryTargetHeading += rotationsDifference * 360.0;

        double fusedTargetHeading = odometryTargetHeading;
        if (currentDistanceToTarget <= p.headingLockDistance) {
            if (!headingLocked) { lockedHeadingValue = odometryTargetHeading; headingLocked = true; }
            fusedTargetHeading = lockedHeadingValue;
        } else {
            headingLocked = false;
        }

        double headingCorrection = headingPID.calculate(fusedTargetHeading, currentGyroHeading);

        // ── 5. SENSOR READINGS ─────────────────────────────────────────
        double leftMotorRPM    = leftDrive.get_actual_velocity()  * DRIVE_MOTOR_RPM_ADJ;
        double rightMotorRPM   = rightDrive.get_actual_velocity() * DRIVE_MOTOR_RPM_ADJ;
        double leftEncoderRPM  = passiveEncoderLeft.get_velocity()  * (encoderWheelCircumferenceCM / wheelCircumferenceCM);
        double rightEncoderRPM = passiveEncoderRight.get_velocity() * (encoderWheelCircumferenceCM / wheelCircumferenceCM);

        // ── 6. MOTION PHASES ───────────────────────────────────────────

        // PHASE 1: LAUNCH
        if (currentDistanceToTarget > p.breakDistance && !accelCompleted && !decel)
        {
            currentDrivePhase = PHASE_LAUNCH;
            // Update traction state independently — heading correction does not feed back in
            tractionVoltageLeft  = tractionControlLeft.tractionControlSpeed(
                tractionVoltageLeft, leftMotorRPM, leftEncoderRPM, p.accelFactor);
            tractionVoltageRight = tractionControlRight.tractionControlSpeed(
                tractionVoltageRight, rightMotorRPM, rightEncoderRPM, p.accelFactor);

            // Add heading correction on top — forgotten next tick
            motorVoltageLeft  = tractionVoltageLeft  + (headingCorrection * p.accelHeadingScaling);
            motorVoltageRight = tractionVoltageRight - (headingCorrection * p.accelHeadingScaling);

            // Exit checks traction base, not PID-skewed motor output
            if (std::fabs(tractionVoltageLeft)  >= maxSpeedVoltage &&
                std::fabs(tractionVoltageRight) >= maxSpeedVoltage)
                accelCompleted = true;
        }

        // PHASE 2: CRUISE
        else if (currentDistanceToTarget > p.breakDistance && accelCompleted)
        {
            currentDrivePhase = PHASE_CRUISE;
            motorVoltageLeft  = maxSpeedVoltage + headingCorrection;
            motorVoltageRight = maxSpeedVoltage - headingCorrection;
        }

        // PHASE 3: DECELERATION
        else if (currentDistanceToTarget <= p.breakDistance && !decelCompleted)
        {
            currentDrivePhase = PHASE_DECEL;
            if (!decel) {
                decel = true;
                adaptiveABSLeft.initialize(std::fabs(motorVoltageLeft));
                adaptiveABSRight.initialize(std::fabs(motorVoltageRight));
            }

            adaptiveABSLeft.decelControlSpeed(leftMotorRPM, leftEncoderRPM);
            adaptiveABSRight.decelControlSpeed(rightMotorRPM, rightEncoderRPM);

            pros::motor_brake_mode_e_t leftBrakeMode  = adaptiveABSLeft.getBrakeMode();
            pros::motor_brake_mode_e_t rightBrakeMode = adaptiveABSRight.getBrakeMode();
            double adjustedHeadingCorrection = headingCorrection * p.decelHeadingScaling;

            leftDrive.set_brake_mode(leftBrakeMode);
            rightDrive.set_brake_mode(rightBrakeMode);
            motorVoltageLeft  = std::max(0.0,  adjustedHeadingCorrection);
            motorVoltageRight = std::max(0.0, -adjustedHeadingCorrection);

            leftEncoderRollingAverage  = rollingAverage(leftEncoderRPM,  leftEncoderRollingAverage,  3);
            rightEncoderRollingAverage = rollingAverage(rightEncoderRPM, rightEncoderRollingAverage, 3);

            if (std::fabs(leftEncoderRollingAverage) <= std::fabs(minDriveMotorRPM) &&
                std::fabs(rightEncoderRollingAverage) <= std::fabs(minDriveMotorRPM))
                decelCompleted = true;
        }

        // PHASE 4: APPROACH
        else if (decelCompleted)
        {
            currentDrivePhase = PHASE_APPROACH;
            leftDrive.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
            rightDrive.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
            motorVoltageLeft  = minSpeedVoltage + (headingCorrection * p.approachHeadingScaling);
            motorVoltageRight = minSpeedVoltage - (headingCorrection * p.approachHeadingScaling);
        }

        // ── 7. VOLTAGE SATURATION LIMITER ─────────────────────────────
        double maximumRequestedVoltage = std::max(std::fabs(motorVoltageLeft), std::fabs(motorVoltageRight));
        if (maximumRequestedVoltage > absoluteMaxVoltage) {
            double voltageScaleFactor = absoluteMaxVoltage / maximumRequestedVoltage;
            motorVoltageLeft  *= voltageScaleFactor;
            motorVoltageRight *= voltageScaleFactor;
        }

        // ── 8. MOTOR OUTPUT ────────────────────────────────────────────
        leftDrive.move_voltage(static_cast<int32_t>(motorVoltageLeft  * 1000));
        rightDrive.move_voltage(static_cast<int32_t>(motorVoltageRight * 1000));

        pros::delay(10);
    }

    currentDrivePhase = PHASE_IDLE;
    leftDrive.set_brake_mode(p.brakeMode);
    rightDrive.set_brake_mode(p.brakeMode);
    leftDrive.move(0);
    rightDrive.move(0);
}

// ======================================================================
// backwardToPoint — Closed-loop backward drive to (targetX, targetY).
//
// Same architecture as forwardToPoint. Heading points toward the target —
// no 180° flip needed. All phase voltages are negative so the robot drives
// backward while the heading PID steers correctly.
//
// Heading correction sign is flipped relative to forwardToPoint:
//   Forward: left += correction, right -= correction  (positive = steer right)
//   Backward: left -= correction, right += correction (positive = steer right
//             relative to direction of travel, i.e. robot's rear)
// ======================================================================
void backwardToPoint(double targetX, double targetY, const StraightProfile& p)
{
    const int REQUIRED_CONSECUTIVE_STOPS = 3;

    updateOdometry();
    double startCoordinateX = globalX;
    double startCoordinateY = globalY;

    double pathVectorX     = targetX - startCoordinateX;
    double pathVectorY     = targetY - startCoordinateY;

    PID headingPID(p.kp_heading, p.ki_heading, p.kd_heading);
    headingPID.pidReset();

    // Negative voltages — robot drives backward throughout
    double maxSpeedVoltage       = -(p.maxSpeed * 0.01 * absoluteMaxVoltage);
    double minSpeedVoltage       = -(p.minSpeed * 0.01 * absoluteMaxVoltage);
    double minLaunchSpeedVoltage = std::copysign(std::min(std::fabs(maxSpeedVoltage), p.launchVoltage), maxSpeedVoltage);
    double minDriveMotorRPM      = (p.minSpeed * 0.01) * absoluteMaxRPM;

    // Separate traction state from PID-corrected output — prevents heading correction
    // from corrupting the traction ramp direction on the next tick
    double tractionVoltageLeft  = minLaunchSpeedVoltage;
    double tractionVoltageRight = minLaunchSpeedVoltage;
    double motorVoltageLeft     = minLaunchSpeedVoltage;
    double motorVoltageRight    = minLaunchSpeedVoltage;

    bool decel          = false;
    bool decelCompleted = false;
    bool accelCompleted = false;

    double leftEncoderRollingAverage  = 0;
    double rightEncoderRollingAverage = 0;
    int consecutiveAtTargetCount = 0;
    bool   headingLocked      = false;
    double lockedHeadingValue = 0;

    tractionControl tractionControlLeft(minLaunchSpeedVoltage, maxSpeedVoltage, p.slipThreshold);
    tractionControl tractionControlRight(minLaunchSpeedVoltage, maxSpeedVoltage, p.slipThreshold);
    adaptiveABS adaptiveABSLeft(p.decelStepPercent, p.lockThreshold);
    adaptiveABS adaptiveABSRight(p.decelStepPercent, p.lockThreshold);

    uint32_t safetyStart = pros::millis();
    double timeoutMs     = p.timeout * 1000.0;

    while (true)
    {
        updateOdometry();

        // ── 1. STATE CALCULATION ───────────────────────────────────────
        double currentDistanceToTarget, odometryTargetHeading;
        calculatePathToTarget(globalX, globalY, targetX, targetY, currentDistanceToTarget, odometryTargetHeading);
        double currentGyroHeading = getContinuousStandardHeading();

        // ── 2. TIMEOUT ─────────────────────────────────────────────────
        if (pros::millis() - safetyStart > (uint32_t)timeoutMs) break;

        // ── 3. EXIT CONDITIONS ─────────────────────────────────────────
        if (currentDistanceToTarget <= p.distanceTolerance) {
            if (++consecutiveAtTargetCount >= REQUIRED_CONSECUTIVE_STOPS) break;
        } else {
            consecutiveAtTargetCount = 0;
        }

        double vectorToTargetX = targetX - globalX;
        double vectorToTargetY = targetY - globalY;
        if ((pathVectorX * vectorToTargetX) + (pathVectorY * vectorToTargetY) < 0) break;

        // ── 4. HEADING CALCULATION ─────────────────────────────────────
        // Flip 180° every tick — robot's rear faces the target dynamically.
        // Must stay inside the loop because calculatePathToTarget recalculates each tick.
        odometryTargetHeading += 180.0;

        double rotationsDifference = std::round((currentGyroHeading - odometryTargetHeading) / 360.0);
        odometryTargetHeading     += rotationsDifference * 360.0;

        double fusedTargetHeading = odometryTargetHeading;
        if (currentDistanceToTarget <= p.headingLockDistance) {
            if (!headingLocked) { lockedHeadingValue = odometryTargetHeading; headingLocked = true; }
            fusedTargetHeading = lockedHeadingValue;
        } else {
            headingLocked = false;
        }

        double headingCorrection = headingPID.calculate(fusedTargetHeading, currentGyroHeading);

        // ── 5. SENSOR READINGS ─────────────────────────────────────────
        double leftMotorRPM    = leftDrive.get_actual_velocity()  * DRIVE_MOTOR_RPM_ADJ;
        double rightMotorRPM   = rightDrive.get_actual_velocity() * DRIVE_MOTOR_RPM_ADJ;
        double leftEncoderRPM  = passiveEncoderLeft.get_velocity()  * (encoderWheelCircumferenceCM / wheelCircumferenceCM);
        double rightEncoderRPM = passiveEncoderRight.get_velocity() * (encoderWheelCircumferenceCM / wheelCircumferenceCM);

        // ── 6. MOTION PHASES ───────────────────────────────────────────
        // Base voltages are negative — direction is already encoded.
        // Heading correction applied same way as forwardToPoint — the negative
        // base voltage naturally produces correct backward steering.

        // PHASE 1: LAUNCH
        if (currentDistanceToTarget > p.breakDistance && !accelCompleted && !decel)
        {
            currentDrivePhase = PHASE_LAUNCH;
            // Update traction state independently — heading correction does not feed back in
            tractionVoltageLeft  = tractionControlLeft.tractionControlSpeed(
                tractionVoltageLeft, leftMotorRPM, leftEncoderRPM, p.accelFactor);
            tractionVoltageRight = tractionControlRight.tractionControlSpeed(
                tractionVoltageRight, rightMotorRPM, rightEncoderRPM, p.accelFactor);

            // Add heading correction on top — forgotten next tick
            motorVoltageLeft  = tractionVoltageLeft  + (headingCorrection * p.accelHeadingScaling);
            motorVoltageRight = tractionVoltageRight - (headingCorrection * p.accelHeadingScaling);

            // Exit checks traction base, not PID-skewed motor output
            if (std::fabs(tractionVoltageLeft)  >= std::fabs(maxSpeedVoltage) &&
                std::fabs(tractionVoltageRight) >= std::fabs(maxSpeedVoltage))
                accelCompleted = true;
        }

        // PHASE 2: CRUISE
        else if (currentDistanceToTarget > p.breakDistance && accelCompleted)
        {
            currentDrivePhase = PHASE_CRUISE;
            motorVoltageLeft  = maxSpeedVoltage + headingCorrection;
            motorVoltageRight = maxSpeedVoltage - headingCorrection;
        }

        // PHASE 3: DECELERATION
        else if (currentDistanceToTarget <= p.breakDistance && !decelCompleted)
        {
            currentDrivePhase = PHASE_DECEL;
            if (!decel) {
                decel = true;
                adaptiveABSLeft.initialize(std::fabs(motorVoltageLeft));
                adaptiveABSRight.initialize(std::fabs(motorVoltageRight));
            }

            adaptiveABSLeft.decelControlSpeed(leftMotorRPM, leftEncoderRPM);
            adaptiveABSRight.decelControlSpeed(rightMotorRPM, rightEncoderRPM);

            pros::motor_brake_mode_e_t leftBrakeMode  = adaptiveABSLeft.getBrakeMode();
            pros::motor_brake_mode_e_t rightBrakeMode = adaptiveABSRight.getBrakeMode();
            double adjustedHeadingCorrection = headingCorrection * p.decelHeadingScaling;

            leftDrive.set_brake_mode(leftBrakeMode);
            rightDrive.set_brake_mode(rightBrakeMode);
            motorVoltageLeft  = std::min(0.0, -adjustedHeadingCorrection);
            motorVoltageRight = std::min(0.0,  adjustedHeadingCorrection);

            leftEncoderRollingAverage  = rollingAverage(leftEncoderRPM,  leftEncoderRollingAverage,  3);
            rightEncoderRollingAverage = rollingAverage(rightEncoderRPM, rightEncoderRollingAverage, 3);

            if (std::fabs(leftEncoderRollingAverage) <= std::fabs(minDriveMotorRPM) &&
                std::fabs(rightEncoderRollingAverage) <= std::fabs(minDriveMotorRPM))
                decelCompleted = true;
        }

        // PHASE 4: APPROACH
        else if (decelCompleted)
        {
            currentDrivePhase = PHASE_APPROACH;
            leftDrive.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
            rightDrive.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
            motorVoltageLeft  = minSpeedVoltage + (headingCorrection * p.approachHeadingScaling);
            motorVoltageRight = minSpeedVoltage - (headingCorrection * p.approachHeadingScaling);
        }

        // ── 7. VOLTAGE SATURATION LIMITER ─────────────────────────────
        double maximumRequestedVoltage = std::max(std::fabs(motorVoltageLeft), std::fabs(motorVoltageRight));
        if (maximumRequestedVoltage > absoluteMaxVoltage) {
            double voltageScaleFactor = absoluteMaxVoltage / maximumRequestedVoltage;
            motorVoltageLeft  *= voltageScaleFactor;
            motorVoltageRight *= voltageScaleFactor;
        }

        // ── 8. MOTOR OUTPUT ────────────────────────────────────────────
        leftDrive.move_voltage(static_cast<int32_t>(motorVoltageLeft  * 1000));
        rightDrive.move_voltage(static_cast<int32_t>(motorVoltageRight * 1000));

        pros::delay(10);
    }

    currentDrivePhase = PHASE_IDLE;
    leftDrive.set_brake_mode(p.brakeMode);
    rightDrive.set_brake_mode(p.brakeMode);
    leftDrive.move(0);
    rightDrive.move(0);
}



// ======================================================================
// visionForwardToPoint — Closed-loop vision + odometry approach to a field coordinate.
//
// Fuses live odometry with AI Vision heading correction.
// Pre-acquisition:  odometry heading toward (targetX, targetY).
// Post-acquisition: vision heading via screen pixel offset.
// reversed = true: robot drives backward toward the target.
//
// Exit priority:
//   1. Vision pixel width — 3 consecutive unique frames >= targetPixelWidth
//   2. Odometry tolerance — 3 consecutive ticks within distanceTolerance
//   3. Timeout
//
// Dropout recovery: projects new targetX/targetY along lastFusedHeading
// on vision loss. Fires once per dropout; resets on reacquire.
// ======================================================================
void visionForwardToPoint(pros::vision_signature_s_t targetSignature,
                          int    targetPixelWidth,
                          double targetX,
                          double targetY,
                          const VisionProfile& p,
                          bool   reversed)
{
    // ========================================
    // CONFIGURATION CONSTANTS
    // ========================================
    const int REQUIRED_CONSECUTIVE_STOPS = 3;
    const int REQUIRED_CONSECUTIVE_WIDTH = 3;

    // ========================================
    // INITIALIZATION
    // ========================================
    updateOdometry();
    double startCoordinateX = globalX;
    double startCoordinateY = globalY;

    double pathVectorX   = targetX - startCoordinateX;
    double pathVectorY   = targetY - startCoordinateY;
    double initialDistance = sqrt(pathVectorX * pathVectorX + pathVectorY * pathVectorY);

    // dirSign negated at motor output each tick — see forwardToPoint for explanation
    double dirSign = reversed ? -1.0 : 1.0;

    PID headingPID(p.kp_head, p.ki_head, p.kd_head);
    headingPID.pidReset();

    double maxSpeedVoltage       = p.maxSpeed * 0.01 * absoluteMaxVoltage;
    double minSpeedVoltage       = p.minSpeed * 0.01 * absoluteMaxVoltage;
    double minLaunchSpeedVoltage = std::min(maxSpeedVoltage, p.launchVoltage);
    double minDriveMotorRPM      = (p.minSpeed * 0.01) * absoluteMaxRPM;

    // Separate traction state from PID-corrected output — prevents heading correction
    // from corrupting the traction ramp direction on the next tick
    double tractionVoltageLeft  = minLaunchSpeedVoltage;
    double tractionVoltageRight = minLaunchSpeedVoltage;
    double motorVoltageLeft     = minLaunchSpeedVoltage;
    double motorVoltageRight    = minLaunchSpeedVoltage;

    bool decel          = false;
    bool decelCompleted = false;
    bool accelCompleted = false;

    double leftEncoderRollingAverage  = 0;
    double rightEncoderRollingAverage = 0;

    int consecutiveAtTargetCount = 0;
    int consecutiveWidthCount    = 0;

    // Vision state
    double lastVisionHorizontalOffset = 0.0;  // Normalized screen X; held across dropouts
    bool   visionEverTracked          = false; // Latches true on first valid detection
    bool   visionCurrentlyTracked     = false; // True only if valid object this tick
    bool   visionDropoutHandled       = false; // Prevents dropout block re-firing every tick
    double lastFusedHeading           = 0.0;   // Last heading while vision was active

    // Heading lock state — prevents atan2 singularity near target
    bool   headingLocked      = false;
    double lockedHeadingValue = 0;

    // Dedup cache — exit counter only increments on unique sensor frames
    int lastSnapshotCenterX = -999;
    int lastSnapshotWidth   = -999;

    tractionControl tractionControlLeft(minLaunchSpeedVoltage, maxSpeedVoltage, p.slipThreshold);
    tractionControl tractionControlRight(minLaunchSpeedVoltage, maxSpeedVoltage, p.slipThreshold);
    adaptiveABS adaptiveABSLeft(p.decelStepPercent, p.lockThreshold);
    adaptiveABS adaptiveABSRight(p.decelStepPercent, p.lockThreshold);

    uint32_t safetyStart = pros::millis();
    double timeoutMs     = p.timeout * 1000.0;

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

        // Backward driving: flip heading 180° so the robot's rear faces the target.
        // Without this, negative voltage would drive the nose away from the target.
        if (reversed) odometryTargetHeading += 180.0;

        double currentGyroHeading = getContinuousStandardHeading();

        // ───────────────────────────────────────────────────────────────
        // 2. TIMEOUT
        // ───────────────────────────────────────────────────────────────
        if (pros::millis() - safetyStart > (uint32_t)timeoutMs) break;

        // ───────────────────────────────────────────────────────────────
        // 3. EXIT CONDITIONS
        // ───────────────────────────────────────────────────────────────
        if (currentDistanceToTarget <= p.distanceTolerance) {
            consecutiveAtTargetCount++;
            if (consecutiveAtTargetCount >= REQUIRED_CONSECUTIVE_STOPS) break;
        } else {
            consecutiveAtTargetCount = 0;
        }

        // Dot product overshoot detection
        double vectorToTargetX = targetX - globalX;
        double vectorToTargetY = targetY - globalY;
        double progressScalar  = (pathVectorX * vectorToTargetX) + (pathVectorY * vectorToTargetY);
        if (progressScalar < 0) break;

        // ───────────────────────────────────────────────────────────────
        // 4. VISION SNAPSHOT
        // ───────────────────────────────────────────────────────────────
        visionCurrentlyTracked = false;
        int objCount = aiVision.get_object_count();

        pros::screen::erase();
        pros::screen::print(pros::E_TEXT_MEDIUM, 1, "objCount: %d  dist: %.1f", objCount, currentDistanceToTarget);

        pros::vision_object_s_t primaryObject = aiVision.get_by_sig(0, targetSignature.id);
        bool primaryValid = (primaryObject.signature != VISION_OBJECT_ERR_SIG);

        if (primaryValid) {
            pros::screen::print(pros::E_TEXT_MEDIUM, 2, "w:%d  cx:%d  cy:%d",
                primaryObject.width, primaryObject.x_middle_coord, primaryObject.y_middle_coord);
            pros::screen::print(pros::E_TEXT_MEDIUM, 3, "minW:%d  X:%d-%d  Y:%d-%d",
                p.minObjectWidth, p.minX, p.maxX, p.minY, p.maxY);
            int bottomY = primaryObject.y_middle_coord + (primaryObject.height / 2);
            pros::screen::print(pros::E_TEXT_MEDIUM, 4, "wPass:%d  xyPass:%d",
                (int)(primaryObject.width >= p.minObjectWidth),
                (int)(primaryObject.x_middle_coord >= p.minX && primaryObject.x_middle_coord <= p.maxX &&
                      bottomY >= p.minY && bottomY <= p.maxY));
            pros::screen::print(pros::E_TEXT_MEDIUM, 5, "gate: cx%d==%d  w%d==%d",
                primaryObject.x_middle_coord, lastSnapshotCenterX, primaryObject.width, lastSnapshotWidth);
        }
        pros::screen::print(pros::E_TEXT_MEDIUM, 6, "everTracked:%d  currTracked:%d",
            (int)visionEverTracked, (int)visionCurrentlyTracked);

        if (primaryValid) {
            int bottomY = primaryObject.y_middle_coord + (primaryObject.height / 2);
            if (primaryObject.width >= p.minObjectWidth &&
                primaryObject.x_middle_coord >= p.minX && primaryObject.x_middle_coord <= p.maxX &&
                bottomY >= p.minY && bottomY <= p.maxY)
            {
                visionCurrentlyTracked = true;
                if (primaryObject.x_middle_coord != lastSnapshotCenterX || primaryObject.width != lastSnapshotWidth)
                {
                    lastVisionHorizontalOffset = (primaryObject.x_middle_coord - VISION_CENTER_X) / VISION_CENTER_X;
                    lastSnapshotCenterX        = primaryObject.x_middle_coord;
                    lastSnapshotWidth          = primaryObject.width;
                    visionEverTracked          = true;
                    if (primaryObject.width >= targetPixelWidth) {
                        consecutiveWidthCount++;
                        if (consecutiveWidthCount >= REQUIRED_CONSECUTIVE_WIDTH) break;
                    } else {
                        consecutiveWidthCount = 0;
                    }
                }
            }
        }

        // ───────────────────────────────────────────────────────────────
        // 5. HEADING CALCULATION
        // ───────────────────────────────────────────────────────────────
        double rotationsDifference = std::round((currentGyroHeading - odometryTargetHeading) / 360.0);
        odometryTargetHeading += rotationsDifference * 360.0;

        double fusedTargetHeading;
        if (!visionEverTracked) {
            if (currentDistanceToTarget <= p.headingLockDistance) {
                if (!headingLocked) { lockedHeadingValue = odometryTargetHeading; headingLocked = true; }
                fusedTargetHeading = lockedHeadingValue;
            } else {
                headingLocked      = false;
                fusedTargetHeading = odometryTargetHeading;
            }
        } else {
            double visualTruthHeading = currentGyroHeading + (lastVisionHorizontalOffset * 25.5);
            fusedTargetHeading        = currentGyroHeading +
                                       ((visualTruthHeading - currentGyroHeading) * p.kp_distToHeadScaling);
            lastFusedHeading = fusedTargetHeading;

            if (visionCurrentlyTracked) {
                visionDropoutHandled = false;
            } else if (!visionDropoutHandled) {
                double remainingDistance = currentDistanceToTarget;
                if (remainingDistance > 0) {
                    double headingRad = lastFusedHeading * M_PI / 180.0;
                    targetX = globalX + (remainingDistance * sin(headingRad));
                    targetY = globalY + (remainingDistance * cos(headingRad));
                }
                visionDropoutHandled = true;
            }
        }

        double headingCorrection = headingPID.calculate(fusedTargetHeading, currentGyroHeading);

        pros::screen::print(pros::E_TEXT_MEDIUM, 7, "odomH:%.1f fusedH:%.1f", odometryTargetHeading, fusedTargetHeading);
        pros::screen::print(pros::E_TEXT_MEDIUM, 8, "correction:%.3f  vOffset:%.2f", headingCorrection, lastVisionHorizontalOffset);

        // ───────────────────────────────────────────────────────────────
        // 6. SENSOR READINGS
        // ───────────────────────────────────────────────────────────────
        double leftMotorRPM    = leftDrive.get_actual_velocity()  * DRIVE_MOTOR_RPM_ADJ;
        double rightMotorRPM   = rightDrive.get_actual_velocity() * DRIVE_MOTOR_RPM_ADJ;
        double leftEncoderRPM  = passiveEncoderLeft.get_velocity()  * (encoderWheelCircumferenceCM / wheelCircumferenceCM);
        double rightEncoderRPM = passiveEncoderRight.get_velocity() * (encoderWheelCircumferenceCM / wheelCircumferenceCM);

        // ───────────────────────────────────────────────────────────────
        // 7. MOTION PHASE CONTROL
        // ───────────────────────────────────────────────────────────────

        // PHASE 1: LAUNCH
        if (currentDistanceToTarget > p.breakDistance && !accelCompleted && !decel)
        {
            currentDrivePhase = PHASE_LAUNCH;
            // Update traction state independently — heading correction does not feed back in
            tractionVoltageLeft  = tractionControlLeft.tractionControlSpeed(
                tractionVoltageLeft, leftMotorRPM, leftEncoderRPM, p.accelFactor);
            tractionVoltageRight = tractionControlRight.tractionControlSpeed(
                tractionVoltageRight, rightMotorRPM, rightEncoderRPM, p.accelFactor);

            // Add heading correction on top — forgotten next tick
            motorVoltageLeft  = tractionVoltageLeft  + (headingCorrection * p.accelHeadingScaling);
            motorVoltageRight = tractionVoltageRight - (headingCorrection * p.accelHeadingScaling);

            // Exit checks traction base, not PID-skewed motor output
            if (std::fabs(tractionVoltageLeft)  >= maxSpeedVoltage &&
                std::fabs(tractionVoltageRight) >= maxSpeedVoltage)
            {
                accelCompleted = true;
            }
        }

        // PHASE 2: CRUISE
        else if (currentDistanceToTarget > p.breakDistance && accelCompleted)
        {
            currentDrivePhase = PHASE_CRUISE;
            motorVoltageLeft  = maxSpeedVoltage + headingCorrection;
            motorVoltageRight = maxSpeedVoltage - headingCorrection;
        }

        // PHASE 3: DECELERATION
        else if (currentDistanceToTarget <= p.breakDistance && !decelCompleted)
        {
            currentDrivePhase = PHASE_DECEL;
            if (!decel) {
                decel = true;
                adaptiveABSLeft.initialize(std::fabs(motorVoltageLeft));
                adaptiveABSRight.initialize(std::fabs(motorVoltageRight));
            }

            adaptiveABSLeft.decelControlSpeed(leftMotorRPM, leftEncoderRPM);
            adaptiveABSRight.decelControlSpeed(rightMotorRPM, rightEncoderRPM);

            pros::motor_brake_mode_e_t leftBrakeMode  = adaptiveABSLeft.getBrakeMode();
            pros::motor_brake_mode_e_t rightBrakeMode = adaptiveABSRight.getBrakeMode();

            double adjustedHeadingCorrection = headingCorrection * p.decelHeadingScaling;

            leftDrive.set_brake_mode(leftBrakeMode);
            rightDrive.set_brake_mode(rightBrakeMode);
            motorVoltageLeft  = std::max(0.0,  adjustedHeadingCorrection);
            motorVoltageRight = std::max(0.0, -adjustedHeadingCorrection);

            leftEncoderRollingAverage  = rollingAverage(leftEncoderRPM,  leftEncoderRollingAverage,  3);
            rightEncoderRollingAverage = rollingAverage(rightEncoderRPM, rightEncoderRollingAverage, 3);

            if (std::fabs(leftEncoderRollingAverage)  <= std::fabs(minDriveMotorRPM) &&
                std::fabs(rightEncoderRollingAverage) <= std::fabs(minDriveMotorRPM))
            {
                decelCompleted = true;
            }
        }

        // PHASE 4: APPROACH
        else if (decelCompleted)
        {
            currentDrivePhase = PHASE_APPROACH;
            leftDrive.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
            rightDrive.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
            motorVoltageLeft  = minSpeedVoltage + (headingCorrection * p.approachHeadingScaling);
            motorVoltageRight = minSpeedVoltage - (headingCorrection * p.approachHeadingScaling);
        }

        // ───────────────────────────────────────────────────────────────
        // 8. VOLTAGE SATURATION LIMITER
        // ───────────────────────────────────────────────────────────────
        double maximumRequestedVoltage = std::max(std::fabs(motorVoltageLeft),
                                                   std::fabs(motorVoltageRight));
        if (maximumRequestedVoltage > absoluteMaxVoltage) {
            double voltageScaleFactor = absoluteMaxVoltage / maximumRequestedVoltage;
            motorVoltageLeft  *= voltageScaleFactor;
            motorVoltageRight *= voltageScaleFactor;
        }

        // ───────────────────────────────────────────────────────────────
        // 9. MOTOR OUTPUT — dirSign applied here, not in phase logic
        // ───────────────────────────────────────────────────────────────
        leftDrive.move_voltage(static_cast<int32_t>(dirSign * motorVoltageLeft  * 1000));
        rightDrive.move_voltage(static_cast<int32_t>(dirSign * motorVoltageRight * 1000));

        pros::delay(10);
    }

    // ═══════════════════════════════════════════════════════════════════
    // CLEANUP
    // ═══════════════════════════════════════════════════════════════════
    currentDrivePhase = PHASE_IDLE;
    leftDrive.set_brake_mode(p.brakeMode);
    rightDrive.set_brake_mode(p.brakeMode);
    leftDrive.move(0);
    rightDrive.move(0);
}

// ======================================================================
// visionBackwardToPoint — drives backward to (targetX, targetY) with vision.
// Thin wrapper — calls visionForwardToPoint with reversed = true.
// ======================================================================
void visionBackwardToPoint(pros::vision_signature_s_t targetSignature,
                           int    targetPixelWidth,
                           double targetX,
                           double targetY,
                           const VisionProfile& p) {
    visionForwardToPoint(targetSignature, targetPixelWidth, targetX, targetY, p, true);
}


// ======================================================================
// visionDriveForward — Open-loop encoder distance drive with vision heading.
//
// Drives a fixed distance using encoder tracking. Vision steers the robot
// once it acquires the target — before acquisition, holds targetHeading.
// Open-loop encoder distance drive with vision heading overlay.
// explicit distance and heading, no coordinate-to-distance conversion.
//
// reversed = true: drives backward toward the target.
// visionDriveBackward calls this with reversed = true.
//
// Exit priority:
//   1. Vision pixel width — 3 consecutive unique frames >= targetPixelWidth
//   2. Encoder overshoot  — wheels traveled >= targetDistance (safety net)
//   3. Timeout
// ======================================================================
void visionDriveForward(pros::vision_signature_s_t targetSignature,
                        int    targetPixelWidth,
                        double targetDistance,
                        double targetHeading,
                        const VisionProfile& p,
                        bool   reversed)
{
    const int REQUIRED_CONSECUTIVE_WIDTH = 3;

    // Encoder snapshot at entry — all distance tracking is relative to this.
    double startDist = getCurrentEncoderDistanceCM();

    // dirSign negated at motor output for backward driving
    double dirSign = reversed ? -1.0 : 1.0;

    // Snap heading to continuous rotation space
    double currentGyroHeadingInit = getContinuousStandardHeading();
    double rotationsDiff = std::round((currentGyroHeadingInit - targetHeading) / 360.0);
    targetHeading += rotationsDiff * 360.0;

    PID headingPID(p.kp_head, p.ki_head, p.kd_head);
    headingPID.pidReset();

    double maxSpeedVoltage       = p.maxSpeed * 0.01 * absoluteMaxVoltage;
    double minSpeedVoltage       = p.minSpeed * 0.01 * absoluteMaxVoltage;
    double minLaunchSpeedVoltage = std::min(maxSpeedVoltage, p.launchVoltage);
    double minDriveMotorRPM      = (p.minSpeed * 0.01) * absoluteMaxRPM;

    // Separate traction state from PID-corrected output — prevents heading correction
    // from corrupting the traction ramp direction on the next tick
    double tractionVoltageLeft  = minLaunchSpeedVoltage;
    double tractionVoltageRight = minLaunchSpeedVoltage;
    double motorVoltageLeft     = minLaunchSpeedVoltage;
    double motorVoltageRight    = minLaunchSpeedVoltage;

    bool decel          = false;
    bool decelCompleted = false;
    bool accelCompleted = false;

    double leftEncoderRollingAverage  = 0;
    double rightEncoderRollingAverage = 0;

    int consecutiveWidthCount = 0;

    // Vision state
    double lastVisionHorizontalOffset = 0.0;  // Normalized screen X; held across dropouts
    bool   visionEverTracked          = false; // Latches true on first valid detection
    bool   visionCurrentlyTracked     = false; // True only if valid object this tick
    bool   visionDropoutHandled       = false; // Prevents dropout block re-firing every tick
    double lastFusedHeading           = 0.0;   // Last heading while vision was active

    // Dedup cache
    int lastSnapshotCenterX = -999;
    int lastSnapshotWidth   = -999;

    tractionControl tractionControlLeft(minLaunchSpeedVoltage, maxSpeedVoltage, p.slipThreshold);
    tractionControl tractionControlRight(minLaunchSpeedVoltage, maxSpeedVoltage, p.slipThreshold);
    adaptiveABS adaptiveABSLeft(p.decelStepPercent, p.lockThreshold);
    adaptiveABS adaptiveABSRight(p.decelStepPercent, p.lockThreshold);

    uint32_t safetyStart = pros::millis();
    double timeoutMs     = p.timeout * 1000.0;

    // ═══════════════════════════════════════════════════════════════════
    // MAIN CONTROL LOOP
    // ═══════════════════════════════════════════════════════════════════
    while (true)
    {
        // ───────────────────────────────────────────────────────────────
        // 1. STATE CALCULATION — open-loop distance tracking
        // currentDistanceToTarget counts down from targetDistance to 0.
        // Phase gates are encoder-based, not field-position-based.
        // ───────────────────────────────────────────────────────────────
        double currentDistance         = getCurrentEncoderDistanceCM() - startDist;
        double currentDistanceToTarget = targetDistance - fabs(currentDistance);
        double currentGyroHeading      = getContinuousStandardHeading();

        // ───────────────────────────────────────────────────────────────
        // 2. TIMEOUT
        // ───────────────────────────────────────────────────────────────
        if (pros::millis() - safetyStart > (uint32_t)timeoutMs) break;

        // ───────────────────────────────────────────────────────────────
        // 3. ENCODER OVERSHOOT GUARD
        // Safety exit if vision never acquires and encoder limit reached.
        // ───────────────────────────────────────────────────────────────
        if (fabs(currentDistance) >= targetDistance) break;

        // ───────────────────────────────────────────────────────────────
        // 4. VISION SNAPSHOT
        // ───────────────────────────────────────────────────────────────
        visionCurrentlyTracked = false;
        int objCount = aiVision.get_object_count();

        pros::screen::erase();
        pros::screen::print(pros::E_TEXT_MEDIUM, 1, "objCount: %d  dist: %.1f", objCount, currentDistanceToTarget);

        pros::vision_object_s_t primaryObject = aiVision.get_by_sig(0, targetSignature.id);
        bool primaryValid = (primaryObject.signature != VISION_OBJECT_ERR_SIG);

        if (primaryValid) {
            int bottomY = primaryObject.y_middle_coord + (primaryObject.height / 2);
            pros::screen::print(pros::E_TEXT_MEDIUM, 2, "w:%d  cx:%d  cy:%d",
                primaryObject.width, primaryObject.x_middle_coord, primaryObject.y_middle_coord);
            pros::screen::print(pros::E_TEXT_MEDIUM, 3, "minW:%d  X:%d-%d  Y:%d-%d",
                p.minObjectWidth, p.minX, p.maxX, p.minY, p.maxY);
            pros::screen::print(pros::E_TEXT_MEDIUM, 4, "wPass:%d  xyPass:%d",
                (int)(primaryObject.width >= p.minObjectWidth),
                (int)(primaryObject.x_middle_coord >= p.minX && primaryObject.x_middle_coord <= p.maxX &&
                      bottomY >= p.minY && bottomY <= p.maxY));
            pros::screen::print(pros::E_TEXT_MEDIUM, 5, "gate: cx%d==%d  w%d==%d",
                primaryObject.x_middle_coord, lastSnapshotCenterX, primaryObject.width, lastSnapshotWidth);
        }
        pros::screen::print(pros::E_TEXT_MEDIUM, 6, "everTracked:%d  currTracked:%d",
            (int)visionEverTracked, (int)visionCurrentlyTracked);

        if (primaryValid) {
            int bottomY = primaryObject.y_middle_coord + (primaryObject.height / 2);
            if (primaryObject.width >= p.minObjectWidth &&
                primaryObject.x_middle_coord >= p.minX && primaryObject.x_middle_coord <= p.maxX &&
                bottomY >= p.minY && bottomY <= p.maxY)
            {
                visionCurrentlyTracked = true;
                if (primaryObject.x_middle_coord != lastSnapshotCenterX || primaryObject.width != lastSnapshotWidth)
                {
                    lastVisionHorizontalOffset = (primaryObject.x_middle_coord - VISION_CENTER_X) / VISION_CENTER_X;
                    lastSnapshotCenterX        = primaryObject.x_middle_coord;
                    lastSnapshotWidth          = primaryObject.width;
                    visionEverTracked          = true;
                    if (primaryObject.width >= targetPixelWidth) {
                        consecutiveWidthCount++;
                        if (consecutiveWidthCount >= REQUIRED_CONSECUTIVE_WIDTH) break;
                    } else {
                        consecutiveWidthCount = 0;
                    }
                }
            }
        }

        // ───────────────────────────────────────────────────────────────
        // 5. HEADING CALCULATION
        // Pre-acquisition:  hold targetHeading (snapped to continuous frame).
        // Post-acquisition: vision heading via screen pixel offset.
        // Dropout: hold lastFusedHeading until vision reacquires.
        // ───────────────────────────────────────────────────────────────
        double preAcqHeading       = targetHeading;
        double rotationsDifference = std::round((currentGyroHeading - preAcqHeading) / 360.0);
        preAcqHeading += rotationsDifference * 360.0;

        double fusedTargetHeading;
        if (!visionEverTracked) {
            fusedTargetHeading = preAcqHeading;
        } else {
            double visualTruthHeading = currentGyroHeading + (lastVisionHorizontalOffset * 25.5);
            fusedTargetHeading        = currentGyroHeading +
                                       ((visualTruthHeading - currentGyroHeading) * p.kp_distToHeadScaling);
            lastFusedHeading = fusedTargetHeading;

            if (visionCurrentlyTracked) {
                visionDropoutHandled = false;
            } else if (!visionDropoutHandled) {
                // Vision dropped out — steer on lastFusedHeading until reacquire.
                // No coordinate projection needed — distance is encoder-based.
                visionDropoutHandled = true;
            }
        }

        double headingCorrection = headingPID.calculate(fusedTargetHeading, currentGyroHeading);

        pros::screen::print(pros::E_TEXT_MEDIUM, 7, "targetH:%.1f fusedH:%.1f", targetHeading, fusedTargetHeading);
        pros::screen::print(pros::E_TEXT_MEDIUM, 8, "correction:%.3f  vOffset:%.2f", headingCorrection, lastVisionHorizontalOffset);

        // ───────────────────────────────────────────────────────────────
        // 6. SENSOR READINGS
        // ───────────────────────────────────────────────────────────────
        double leftMotorRPM    = leftDrive.get_actual_velocity()  * DRIVE_MOTOR_RPM_ADJ;
        double rightMotorRPM   = rightDrive.get_actual_velocity() * DRIVE_MOTOR_RPM_ADJ;
        double leftEncoderRPM  = passiveEncoderLeft.get_velocity()  * (encoderWheelCircumferenceCM / wheelCircumferenceCM);
        double rightEncoderRPM = passiveEncoderRight.get_velocity() * (encoderWheelCircumferenceCM / wheelCircumferenceCM);

        // ───────────────────────────────────────────────────────────────
        // 7. MOTION PHASE CONTROL
        // Phase gates use encoder distance — identical to straightDistance.
        // ───────────────────────────────────────────────────────────────

        // PHASE 1: LAUNCH
        if (currentDistanceToTarget > p.breakDistance && !accelCompleted && !decel)
        {
            currentDrivePhase = PHASE_LAUNCH;
            // Update traction state independently — heading correction does not feed back in
            tractionVoltageLeft  = tractionControlLeft.tractionControlSpeed(
                tractionVoltageLeft, leftMotorRPM, leftEncoderRPM, p.accelFactor);
            tractionVoltageRight = tractionControlRight.tractionControlSpeed(
                tractionVoltageRight, rightMotorRPM, rightEncoderRPM, p.accelFactor);

            // Add heading correction on top — forgotten next tick
            motorVoltageLeft  = tractionVoltageLeft  + (headingCorrection * p.accelHeadingScaling);
            motorVoltageRight = tractionVoltageRight - (headingCorrection * p.accelHeadingScaling);

            // Exit checks traction base, not PID-skewed motor output
            if (std::fabs(tractionVoltageLeft)  >= maxSpeedVoltage &&
                std::fabs(tractionVoltageRight) >= maxSpeedVoltage)
            {
                accelCompleted = true;
            }
        }

        // PHASE 2: CRUISE
        else if (currentDistanceToTarget > p.breakDistance && accelCompleted)
        {
            currentDrivePhase = PHASE_CRUISE;
            motorVoltageLeft  = maxSpeedVoltage + headingCorrection;
            motorVoltageRight = maxSpeedVoltage - headingCorrection;
        }

        // PHASE 3: DECELERATION
        else if (currentDistanceToTarget <= p.breakDistance && !decelCompleted)
        {
            currentDrivePhase = PHASE_DECEL;
            if (!decel) {
                decel = true;
                adaptiveABSLeft.initialize(std::fabs(motorVoltageLeft));
                adaptiveABSRight.initialize(std::fabs(motorVoltageRight));
            }

            adaptiveABSLeft.decelControlSpeed(leftMotorRPM, leftEncoderRPM);
            adaptiveABSRight.decelControlSpeed(rightMotorRPM, rightEncoderRPM);

            pros::motor_brake_mode_e_t leftBrakeMode  = adaptiveABSLeft.getBrakeMode();
            pros::motor_brake_mode_e_t rightBrakeMode = adaptiveABSRight.getBrakeMode();

            double adjustedHeadingCorrection = headingCorrection * p.decelHeadingScaling;

            leftDrive.set_brake_mode(leftBrakeMode);
            rightDrive.set_brake_mode(rightBrakeMode);
            motorVoltageLeft  = std::max(0.0,  adjustedHeadingCorrection);
            motorVoltageRight = std::max(0.0, -adjustedHeadingCorrection);

            leftEncoderRollingAverage  = rollingAverage(leftEncoderRPM,  leftEncoderRollingAverage,  3);
            rightEncoderRollingAverage = rollingAverage(rightEncoderRPM, rightEncoderRollingAverage, 3);

            if (std::fabs(leftEncoderRollingAverage)  <= std::fabs(minDriveMotorRPM) &&
                std::fabs(rightEncoderRollingAverage) <= std::fabs(minDriveMotorRPM))
            {
                decelCompleted = true;
            }
        }

        // PHASE 4: APPROACH
        else if (decelCompleted)
        {
            currentDrivePhase = PHASE_APPROACH;
            leftDrive.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
            rightDrive.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
            motorVoltageLeft  = minSpeedVoltage + (headingCorrection * p.approachHeadingScaling);
            motorVoltageRight = minSpeedVoltage - (headingCorrection * p.approachHeadingScaling);
        }

        // ───────────────────────────────────────────────────────────────
        // 8. VOLTAGE SATURATION LIMITER
        // ───────────────────────────────────────────────────────────────
        double maximumRequestedVoltage = std::max(std::fabs(motorVoltageLeft),
                                                   std::fabs(motorVoltageRight));
        if (maximumRequestedVoltage > absoluteMaxVoltage) {
            double voltageScaleFactor = absoluteMaxVoltage / maximumRequestedVoltage;
            motorVoltageLeft  *= voltageScaleFactor;
            motorVoltageRight *= voltageScaleFactor;
        }

        // ───────────────────────────────────────────────────────────────
        // 9. MOTOR OUTPUT — dirSign applied here for reversed driving
        // ───────────────────────────────────────────────────────────────
        leftDrive.move_voltage(static_cast<int32_t>(dirSign * motorVoltageLeft  * 1000));
        rightDrive.move_voltage(static_cast<int32_t>(dirSign * motorVoltageRight * 1000));

        pros::delay(10);  // 100Hz control loop
    }

    // ═══════════════════════════════════════════════════════════════════
    // CLEANUP
    // ═══════════════════════════════════════════════════════════════════
    currentDrivePhase = PHASE_IDLE;
    leftDrive.set_brake_mode(p.brakeMode);
    rightDrive.set_brake_mode(p.brakeMode);
    leftDrive.move(0);
    rightDrive.move(0);
}

// ======================================================================
// visionDriveBackward — drives backward with vision steering.
// Thin wrapper — calls visionDriveForward with reversed = true.
// ======================================================================
void visionDriveBackward(pros::vision_signature_s_t targetSignature,
                         int    targetPixelWidth,
                         double targetDistance,
                         double targetHeading,
                         const VisionProfile& p) {
    visionDriveForward(targetSignature, targetPixelWidth, targetDistance, targetHeading, p, true);
}


// ======================================================================
// visionOnly — Pure vision-guided approach, no odometry position updates.
//
// Pre-acquisition:  holds entry gyro heading until vision locks on.
// Post-acquisition: corrects heading using vision lateral error only.
// Exit:             vision pixel-width >= targetPixelWidth (primary),
//                   encoder distance >= targetDistance (safety),
//                   or timeout elapsed.
// ======================================================================
void visionOnly(pros::vision_signature_s_t targetSignature,
                int    targetPixelWidth,
                double targetDistance,
                const VisionProfile& p)
{
    const int REQUIRED_CONSECUTIVE_WIDTH = 3;

    // Encoder snapshot at entry — distance tracking is relative to this, no odometry needed.
    double startDist = getCurrentEncoderDistanceCM();

    // Entry gyro heading — held as pre-acquisition steering target until vision acquires.
    double initialGyroHeading = getContinuousStandardHeading();

    // dir is always +1.0 (targetDistance always positive); retained for formula consistency
    double dir = 1.0;

    PID headingPID(p.kp_head, p.ki_head, p.kd_head);
    headingPID.pidReset();

    double maxSpeedVoltage       = std::copysign(p.maxSpeed * 0.01 * absoluteMaxVoltage, targetDistance);
    double minSpeedVoltage       = std::copysign(p.minSpeed * 0.01 * absoluteMaxVoltage, targetDistance);
    double minLaunchSpeedVoltage = std::copysign(std::min(fabs(maxSpeedVoltage), fabs(p.launchVoltage)), targetDistance);
    double minDriveMotorRPM      = (p.minSpeed * 0.01) * absoluteMaxRPM;

    // Separate traction state from PID-corrected output — prevents heading correction
    // from corrupting the traction ramp direction on the next tick
    double tractionVoltageLeft  = minLaunchSpeedVoltage;
    double tractionVoltageRight = minLaunchSpeedVoltage;
    double motorVoltageLeft     = minLaunchSpeedVoltage;
    double motorVoltageRight    = minLaunchSpeedVoltage;

    bool decel          = false;
    bool decelCompleted = false;
    bool accelCompleted = false;

    double leftEncoderRollingAverage  = 0;
    double rightEncoderRollingAverage = 0;

    int consecutiveWidthCount = 0;

    double lastVisionHorizontalOffset = 0.0;  // Normalized screen X; held across dropouts
    bool   visionEverTracked          = false; // Latches true on first valid detection
    bool   visionCurrentlyTracked     = false; // True only if valid object found this tick
    bool   visionDropoutHandled       = false; // Prevents dropout block re-firing every tick
    double lastFusedHeading           = 0.0;   // Last heading while vision was active

    int lastSnapshotCenterX = -999;
    int lastSnapshotWidth   = -999;

    tractionControl tractionControlLeft(minLaunchSpeedVoltage, maxSpeedVoltage, p.slipThreshold);
    tractionControl tractionControlRight(minLaunchSpeedVoltage, maxSpeedVoltage, p.slipThreshold);
    adaptiveABS adaptiveABSLeft(p.decelStepPercent, p.lockThreshold);
    adaptiveABS adaptiveABSRight(p.decelStepPercent, p.lockThreshold);

    uint32_t safetyStart = pros::millis();
    double timeoutMs     = p.timeout * 1000.0;

    while (true)
    {
        // Encoder distance — counts down to 0 as robot closes on target
        double currentDistance         = getCurrentEncoderDistanceCM() - startDist;
        double currentDistanceToTarget = targetDistance - fabs(currentDistance);
        double currentGyroHeading      = getContinuousStandardHeading();

        if (pros::millis() - safetyStart > (uint32_t)timeoutMs) break;

        // Safety exit if vision never acquires
        if (fabs(currentDistance) >= targetDistance) break;

        // --- VISION SNAPSHOT ---
        visionCurrentlyTracked = false;
        int objCount = aiVision.get_object_count();

        pros::screen::erase();
        pros::screen::print(pros::E_TEXT_MEDIUM, 1, "objCount: %d  dist: %.1f", objCount, currentDistanceToTarget);

        pros::vision_object_s_t primaryObject = aiVision.get_by_sig(0, targetSignature.id);
        bool primaryValid = (primaryObject.signature != VISION_OBJECT_ERR_SIG);

        if (primaryValid) {
            int bottomY = primaryObject.y_middle_coord + (primaryObject.height / 2);
            pros::screen::print(pros::E_TEXT_MEDIUM, 2, "w:%d  cx:%d  cy:%d",
                primaryObject.width, primaryObject.x_middle_coord, primaryObject.y_middle_coord);
            pros::screen::print(pros::E_TEXT_MEDIUM, 3, "minW:%d  X:%d-%d  Y:%d-%d",
                p.minObjectWidth, p.minX, p.maxX, p.minY, p.maxY);
            pros::screen::print(pros::E_TEXT_MEDIUM, 4, "wPass:%d  xyPass:%d",
                (int)(primaryObject.width >= p.minObjectWidth),
                (int)(primaryObject.x_middle_coord >= p.minX && primaryObject.x_middle_coord <= p.maxX &&
                      bottomY >= p.minY && bottomY <= p.maxY));
            pros::screen::print(pros::E_TEXT_MEDIUM, 5, "gate: cx%d==%d  w%d==%d",
                primaryObject.x_middle_coord, lastSnapshotCenterX, primaryObject.width, lastSnapshotWidth);
        }
        pros::screen::print(pros::E_TEXT_MEDIUM, 6, "everTracked:%d  currTracked:%d",
            (int)visionEverTracked, (int)visionCurrentlyTracked);

        if (primaryValid) {
            int bottomY = primaryObject.y_middle_coord + (primaryObject.height / 2);
            if (primaryObject.width >= p.minObjectWidth &&
                primaryObject.x_middle_coord >= p.minX && primaryObject.x_middle_coord <= p.maxX &&
                bottomY >= p.minY && bottomY <= p.maxY)
            {
                visionCurrentlyTracked = true;
                if (primaryObject.x_middle_coord != lastSnapshotCenterX || primaryObject.width != lastSnapshotWidth)
                {
                    lastVisionHorizontalOffset = (primaryObject.x_middle_coord - VISION_CENTER_X) / VISION_CENTER_X;
                    lastSnapshotCenterX        = primaryObject.x_middle_coord;
                    lastSnapshotWidth          = primaryObject.width;
                    visionEverTracked          = true;
                    if (primaryObject.width >= targetPixelWidth) {
                        consecutiveWidthCount++;
                        if (consecutiveWidthCount >= REQUIRED_CONSECUTIVE_WIDTH) break;
                    } else {
                        consecutiveWidthCount = 0;
                    }
                }
            }
        }

        // --- HEADING ---
        // Pre-acquisition: hold entry gyro heading (snapped to continuous rotation space).
        // Post-acquisition: steer toward object screen position via vision pixel offset.
        // Dropout: hold lastFusedHeading until vision reacquires.
        double preAcqHeading       = initialGyroHeading;
        double rotationsDifference = std::round((currentGyroHeading - preAcqHeading) / 360.0);
        preAcqHeading += rotationsDifference * 360.0;

        double fusedTargetHeading;
        if (!visionEverTracked) {
            fusedTargetHeading = preAcqHeading;
        } else {
            double visualTruthHeading = currentGyroHeading + (lastVisionHorizontalOffset * 25.5);
            fusedTargetHeading        = currentGyroHeading +
                                       ((visualTruthHeading - currentGyroHeading) * p.kp_distToHeadScaling);
            lastFusedHeading = fusedTargetHeading;

            if (visionCurrentlyTracked) {
                visionDropoutHandled = false;
            } else if (!visionDropoutHandled) {
                visionDropoutHandled = true;
            }
        }

        double headingCorrection = headingPID.calculate(fusedTargetHeading, currentGyroHeading);

        pros::screen::print(pros::E_TEXT_MEDIUM, 7, "initH:%.1f fusedH:%.1f", initialGyroHeading, fusedTargetHeading);
        pros::screen::print(pros::E_TEXT_MEDIUM, 8, "correction:%.3f  vOffset:%.2f", headingCorrection, lastVisionHorizontalOffset);

        // --- MOTOR RPM READINGS ---
        double leftMotorRPM    = leftDrive.get_actual_velocity()  * DRIVE_MOTOR_RPM_ADJ;
        double rightMotorRPM   = rightDrive.get_actual_velocity() * DRIVE_MOTOR_RPM_ADJ;
        double leftEncoderRPM  = passiveEncoderLeft.get_velocity()  * (encoderWheelCircumferenceCM / wheelCircumferenceCM);
        double rightEncoderRPM = passiveEncoderRight.get_velocity() * (encoderWheelCircumferenceCM / wheelCircumferenceCM);

        // --- MOTION PHASES ---

        // PHASE 1: LAUNCH
        if (currentDistanceToTarget > p.breakDistance && !accelCompleted && !decel)
        {
            currentDrivePhase = PHASE_LAUNCH;
            // Update traction state independently — heading correction does not feed back in
            tractionVoltageLeft  = tractionControlLeft.tractionControlSpeed(
                tractionVoltageLeft, leftMotorRPM, leftEncoderRPM, p.accelFactor);
            tractionVoltageRight = tractionControlRight.tractionControlSpeed(
                tractionVoltageRight, rightMotorRPM, rightEncoderRPM, p.accelFactor);

            // Add heading correction on top — forgotten next tick
            motorVoltageLeft  = tractionVoltageLeft  + (headingCorrection * p.accelHeadingScaling);
            motorVoltageRight = tractionVoltageRight - (headingCorrection * p.accelHeadingScaling);

            // Exit checks traction base, not PID-skewed motor output
            if (std::fabs(tractionVoltageLeft)  >= std::fabs(maxSpeedVoltage) &&
                std::fabs(tractionVoltageRight) >= std::fabs(maxSpeedVoltage))
            {
                accelCompleted = true;
            }
        }

        // PHASE 2: CRUISE
        else if (currentDistanceToTarget > p.breakDistance && accelCompleted)
        {
            currentDrivePhase = PHASE_CRUISE;
            motorVoltageLeft  = maxSpeedVoltage + headingCorrection;
            motorVoltageRight = maxSpeedVoltage - headingCorrection;
        }

        // PHASE 3: DECELERATION
        else if (currentDistanceToTarget <= p.breakDistance && !decelCompleted)
        {
            currentDrivePhase = PHASE_DECEL;
            if (!decel) {
                decel = true;
                adaptiveABSLeft.initialize(std::fabs(motorVoltageLeft));
                adaptiveABSRight.initialize(std::fabs(motorVoltageRight));
            }

            adaptiveABSLeft.decelControlSpeed(leftMotorRPM, leftEncoderRPM);
            adaptiveABSRight.decelControlSpeed(rightMotorRPM, rightEncoderRPM);

            pros::motor_brake_mode_e_t leftBrakeMode  = adaptiveABSLeft.getBrakeMode();
            pros::motor_brake_mode_e_t rightBrakeMode = adaptiveABSRight.getBrakeMode();

            double adjustedHeadingCorrection = headingCorrection * p.decelHeadingScaling * dir;

            leftDrive.set_brake_mode(leftBrakeMode);
            rightDrive.set_brake_mode(rightBrakeMode);
            motorVoltageLeft  = std::max(0.0,  adjustedHeadingCorrection);
            motorVoltageRight = std::max(0.0, -adjustedHeadingCorrection);

            leftEncoderRollingAverage  = rollingAverage(leftEncoderRPM,  leftEncoderRollingAverage,  3);
            rightEncoderRollingAverage = rollingAverage(rightEncoderRPM, rightEncoderRollingAverage, 3);

            if (std::fabs(leftEncoderRollingAverage)  <= std::fabs(minDriveMotorRPM) &&
                std::fabs(rightEncoderRollingAverage) <= std::fabs(minDriveMotorRPM))
            {
                decelCompleted = true;
            }
        }

        // PHASE 4: APPROACH
        else if (decelCompleted)
        {
            currentDrivePhase = PHASE_APPROACH;
            leftDrive.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
            rightDrive.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
            motorVoltageLeft  = minSpeedVoltage + (headingCorrection * p.approachHeadingScaling);
            motorVoltageRight = minSpeedVoltage - (headingCorrection * p.approachHeadingScaling);
        }

        // Proportional scale-down if either side exceeds absoluteMaxVoltage
        double maximumRequestedVoltage = std::max(std::fabs(motorVoltageLeft),
                                                   std::fabs(motorVoltageRight));
        if (maximumRequestedVoltage > absoluteMaxVoltage) {
            double voltageScaleFactor = absoluteMaxVoltage / maximumRequestedVoltage;
            motorVoltageLeft  *= voltageScaleFactor;
            motorVoltageRight *= voltageScaleFactor;
        }

        leftDrive.move_voltage(static_cast<int32_t>(motorVoltageLeft  * 1000));
        rightDrive.move_voltage(static_cast<int32_t>(motorVoltageRight * 1000));
        pros::delay(10);  // 100Hz control loop
    }

    currentDrivePhase = PHASE_IDLE;
    leftDrive.set_brake_mode(p.brakeMode);
    rightDrive.set_brake_mode(p.brakeMode);
    leftDrive.move(0);
    rightDrive.move(0);
}

// ======================================================================
// driveToWall — Wall alignment with per-side independent stall detection
//
// Drives the robot into a wall at a set speed and uses left and right
// encoder readings independently to detect when each side contacts the
// wall. When one side stalls first (robot approaching at an angle), that
// side cuts to stalledSidePower while the free side keeps pushing —
// naturally rotating the robot flush against the wall without any PID.
// Once both sides are stalled, or a safety exit triggers, all motors stop.
//
// Why no heading PID:
//   Unlike a normal drive function, we WANT the robot to self-correct its
//   angle by letting one side stall while the other catches up. A heading
//   PID would fight that process.
// ======================================================================
void driveToWall(double targetDistance, double targetHeading, double minSpeed,
                 double wallStalledTimeMs, double stalledSidePower,
                 pros::motor_brake_mode_e_t brakeMode, double timeoutMs, double maxSpeed)
{
    const double WALL_THRESHOLD = 5.0;  // RPM below which a side is stalled

    double maxSpeedVoltage    = std::copysign(maxSpeed * 0.01 * absoluteMaxVoltage, targetDistance);
    double stalledSideVoltage = std::copysign(stalledSidePower * 0.01 * absoluteMaxVoltage, targetDistance);

    // Brake mode set upfront so it engages the moment voltage drops to 0
    leftDrive.set_brake_mode(brakeMode);
    rightDrive.set_brake_mode(brakeMode);

    // Snap heading to continuous frame (reference only — no PID correction applied)
    double currentHeading       = getContinuousStandardHeading();
    double rotationsDiff        = std::round((currentHeading - targetHeading) / 360.0);
    double targetHeadingSnapped = targetHeading + (rotationsDiff * 360.0);

    double startDist         = getCurrentEncoderDistanceCM();
    double distanceTravelled = 0;

    bool leftStalled       = false;
    bool leftWallDetected  = false;
    bool rightStalled      = false;
    bool rightWallDetected = false;

    uint32_t leftWallTimerStart  = 0;
    uint32_t rightWallTimerStart = 0;
    uint32_t timeoutStart        = pros::millis();

    while (!(leftWallDetected && rightWallDetected) &&
           std::fabs(distanceTravelled) <= std::fabs(targetDistance) &&
           pros::millis() - timeoutStart < (uint32_t)timeoutMs)
    {
        distanceTravelled = getCurrentEncoderDistanceCM() - startDist;

        double leftRPM  = std::fabs(passiveEncoderLeft.get_velocity());
        double rightRPM = std::fabs(passiveEncoderRight.get_velocity());

        // Left side stall detection
        if (leftRPM < WALL_THRESHOLD) {
            if (!leftStalled) { leftWallTimerStart = pros::millis(); leftStalled = true; }
            else if (pros::millis() - leftWallTimerStart >= (uint32_t)wallStalledTimeMs) leftWallDetected = true;
        } else {
            leftStalled = false;
        }

        // Right side stall detection
        if (rightRPM < WALL_THRESHOLD) {
            if (!rightStalled) { rightWallTimerStart = pros::millis(); rightStalled = true; }
            else if (pros::millis() - rightWallTimerStart >= (uint32_t)wallStalledTimeMs) rightWallDetected = true;
        } else {
            rightStalled = false;
        }

        // Once a side confirms wall contact, switch to stalledSideVoltage
        double leftOutput  = leftWallDetected  ? stalledSideVoltage : maxSpeedVoltage;
        double rightOutput = rightWallDetected ? stalledSideVoltage : maxSpeedVoltage;

        leftDrive.move_voltage(static_cast<int32_t>(leftOutput  * 1000));
        rightDrive.move_voltage(static_cast<int32_t>(rightOutput * 1000));

        pros::delay(10);  // 100Hz control loop
    }

    leftDrive.move(0);
    rightDrive.move(0);
}

// ======================================================================
// gpsReset — GPS-based position correction
//
// Collects SAMPLE_COUNT position readings over ~500ms while the robot is
// stationary and computes a weighted mean (weight = 1/error — lower error
// samples contribute more to the result). Samples failing the confidence
// threshold (get_error() > GPS_MAX_ERROR_M) or returning PROS_ERR_F are
// discarded before averaging.
//
// Only globalX and globalY are updated — IMU heading is intentionally left
// untouched because the GPS heading is less accurate than the inertial sensor.
//
// Returns true  — at least one sample accepted; globalX/globalY updated.
// Returns false — all samples rejected (GPS signal too weak); odometry unchanged.
//
// Call only when the robot is stationary. Motion during sampling corrupts readings.
// ======================================================================
bool gpsReset() {
    const int SAMPLE_COUNT    = 20;  // Total samples over 500ms
    const int SAMPLE_INTERVAL = 25;  // ms between samples (20 × 25 = 500ms)

    double weightedSumX = 0.0;
    double weightedSumY = 0.0;
    double totalWeight  = 0.0;
    int    accepted     = 0;

    double lowX  =  1e9, highX = -1e9;
    double lowY  =  1e9, highY = -1e9;

    for (int i = 0; i < SAMPLE_COUNT; i++) {
        double posError = gpsSensor.get_error();

        if (posError == PROS_ERR_F || posError > GPS_MAX_ERROR_M) {
            pros::delay(SAMPLE_INTERVAL);
            continue;
        }

        pros::gps_position_s_t pos = gpsSensor.get_position();

        if (pos.x == PROS_ERR_F || pos.y == PROS_ERR_F) {
            pros::delay(SAMPLE_INTERVAL);
            continue;
        }

        // Weight = 1/error — lower error = higher confidence = higher weight
        double weight = 1.0 / posError;
        weightedSumX += pos.x * weight;
        weightedSumY += pos.y * weight;
        totalWeight  += weight;
        accepted++;

        if (pos.x < lowX)  lowX  = pos.x;
        if (pos.x > highX) highX = pos.x;
        if (pos.y < lowY)  lowY  = pos.y;
        if (pos.y > highY) highY = pos.y;

        pros::delay(SAMPLE_INTERVAL);
    }

    if (accepted == 0 || totalWeight == 0.0) {
        pros::screen::print(pros::E_TEXT_MEDIUM, 5, "GPS FAIL — 0/%d accepted", SAMPLE_COUNT);
        return false;
    }

    double resultX_m = weightedSumX / totalWeight;
    double resultY_m = weightedSumY / totalWeight;

    double varX_cm = (highX - lowX) * 100.0;
    double varY_cm = (highY - lowY) * 100.0;

    pros::screen::print(pros::E_TEXT_MEDIUM, 5, "accepted:%d/%d  varX:%.1f varY:%.1fcm",
        accepted, SAMPLE_COUNT, varX_cm, varY_cm);
    pros::screen::print(pros::E_TEXT_MEDIUM, 6, "X low:%.1f high:%.1fcm", lowX * 100.0, highX * 100.0);
    pros::screen::print(pros::E_TEXT_MEDIUM, 7, "Y low:%.1f high:%.1fcm", lowY * 100.0, highY * 100.0);
    pros::screen::print(pros::E_TEXT_MEDIUM, 8, "result X:%.1f Y:%.1fcm", resultX_m * 100.0, resultY_m * 100.0);

    // Snap odometry to weighted mean. Heading NOT updated — trust IMU.
    globalX = resultX_m * 100.0;
    globalY = resultY_m * 100.0;

    return true;
}