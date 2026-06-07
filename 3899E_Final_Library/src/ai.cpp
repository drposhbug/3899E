/*----------------------------------------------------------------------------
 * ai.cpp — VAIRC-specific AI functions for the V5 Brain (Team 3899E)
 *
 * Contains all VAIRC match logic:
 *   SECTION 1 — Jetson data accessors
 *   SECTION 2 — moveVisionOdometryAI (vision navigation)
 *   SECTION 3 — Three-phase navigation wrapper (navigateToTarget)
 *   SECTION 4 — Strategy functions
 *   SECTION 5 — Match dispatch loop
 *   SECTION 6 — Behavior stubs
 *
 * g_jetson is defined in jetson_comms.cpp. This file only calls its
 * public interface (get_data(), get_strategy()) — never touches internals.
 *----------------------------------------------------------------------------*/

#include "ai.h"
#include "robot_config.h"
#include "robot_geometry.h"
#include "route_planner.h"
#include "odometry.h"
#include "navigation.h"
#include "autontasks.h"
#include "pid.h"
#include "utils.h"
#include "field_targets.h"   // FIELD_TARGETS table, getTarget(), waitAndResetGPS()
#include <cmath>
#include <atomic>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace RobotGeometry;

// Horizontal FOV constant for the e-CAM25_CUONX (AR0234CS) at 640px wide.
// AR0234CS HFOV ≈ 70° → half-frame = 35°. hOffset of 1.0 = 35° from center.
// Tune after physical calibration — replace 25.5 (VEX AI Vision) with this.
static constexpr double AI_CAM_DEG_PER_UNIT = 35.0;

// ============================================================================
// SECTION 1 — Jetson data accessors
// ============================================================================

// Atomic serial state — populated by the serial receive task in jetson_comms.cpp
// Declared extern; defined in jetson_comms.cpp alongside the serial task.
extern std::atomic<bool>   jetsonTargetTrackedAtomic;
extern std::atomic<double> jetsonTargetDistanceCmAtomic;
extern std::atomic<bool>   jetsonObstacleDetectedAtomic;

bool   jetsonObstacleDetected() { return jetsonObstacleDetectedAtomic.load(); }
double jetsonTargetDistance()   { return jetsonTargetDistanceCmAtomic.load(); }
bool   jetsonTargetTracked()    { return jetsonTargetTrackedAtomic.load(); }

// getLatestDetection — searches latest AI_RECORD for classID match
bool getLatestDetection(int classID, float minConfidence, JetsonDetection* out) {
    if (out == nullptr) return false;

    AI_RECORD rec;
    int32_t   len = g_jetson.get_data(&rec);
    if (len == 0) return false;

    for (int32_t i = 0; i < rec.detectionCount; i++) {
        const DETECTION_OBJECT& d = rec.detections[i];
        if (d.classID == classID && d.probability >= minConfidence) {
            float cx        = static_cast<float>(d.screenLocation.x) +
                              static_cast<float>(d.screenLocation.width) * 0.5f;
            out->hOffset    = (cx - 320.0f) / 320.0f;
            out->distanceCm = d.depth * 100.0f;
            out->classID    = d.classID;
            out->confidence = d.probability;
            out->mapX       = d.mapLocation.x;
            out->mapY       = d.mapLocation.y;
            out->seqNum     = rec.pos.framecnt;
            return true;
        }
    }
    return false;
}

// getStrategy — returns strategyCode from latest AI_RECORD
int32_t getStrategy(void) {
    return g_jetson.get_strategy();
}

// ============================================================================
// SECTION 2 — moveVisionOdometryAI
// ============================================================================

void moveVisionOdometryAI(int    objectClassID,
                          float  targetStopDistanceCm,
                          double targetX,
                          double targetY,
                          double breakDistance,
                          pros::motor_brake_mode_e_t brakeMode,
                          double maxSpeed,
                          double kp_head,
                          double ki_head,
                          double kd_head,
                          double kp_distToHeadScaling,
                          double minSpeed,
                          double accelHeadingScaling,
                          double decelHeadingScaling,
                          double approachHeadingScaling,
                          double headingLockDistance,
                          double timeout)
{
    const double LAUNCH_VOLTAGE          = 6.0;
    const double ACCEL_FACTOR_LAUNCH     = 1.2;
    const double SLIP_THRESHOLD_TRACTION = 20.0;
    const double DECEL_STEP_PERCENT      = 0.45;
    const double LOCK_THRESHOLD_DECEL    = 0.25;
    const int    REQUIRED_CONSECUTIVE    = 3;

    updateOdometry();

    double pathVectorX            = targetX - globalX;
    double pathVectorY            = targetY - globalY;
    double initialDistance        = sqrt(pathVectorX * pathVectorX + pathVectorY * pathVectorY);
    // atan2(X, Y) gives North=0°, CW+ heading — matches VEX Coordinates throughout codebase
    double initialOdometryHeading = atan2(pathVectorX, pathVectorY) * 180.0 / M_PI;

    double startDist = getCurrentEncoderDistanceCM();
    double dir       = 1.0;

    PID headingPID(kp_head, ki_head, kd_head);
    headingPID.pidReset();

    double maxSpeedVoltage       = std::copysign(maxSpeed * 0.01 * absoluteMaxVoltage, initialDistance);
    double minSpeedVoltage       = std::copysign(minSpeed * 0.01 * absoluteMaxVoltage, initialDistance);
    double minLaunchSpeedVoltage = std::copysign(std::min(fabs(maxSpeedVoltage), fabs(LAUNCH_VOLTAGE)), initialDistance);
    double minDriveMotorRPM      = (minSpeed * 0.01) * absoluteMaxRPM;

    double motorVoltageLeft  = minLaunchSpeedVoltage;
    double motorVoltageRight = minLaunchSpeedVoltage;

    bool decel = false, decelCompleted = false, accelCompleted = false;
    double leftEncoderRollingAverage = 0, rightEncoderRollingAverage = 0;
    int consecutiveDistCount = 0;

    double lastVisionHorizontalOffset = 0.0;
    bool   visionEverTracked = false, visionCurrentlyTracked = false;
    bool   visionDropoutHandled = false;
    double lastFusedHeading = 0.0;
    bool   headingLocked = false;
    double lockedHeadingValue = 0.0;
    int    lastProcessedSeq = -1;

    tractionControl tractionControlLeft (minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION);
    tractionControl tractionControlRight(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION);
    adaptiveABS adaptiveABSLeft (DECEL_STEP_PERCENT, LOCK_THRESHOLD_DECEL);
    adaptiveABS adaptiveABSRight(DECEL_STEP_PERCENT, LOCK_THRESHOLD_DECEL);

    uint32_t safetyStart = pros::millis();
    double   timeoutMs   = timeout * 1000.0;

    pros::screen::erase();
    pros::screen::print(pros::E_TEXT_MEDIUM, 0, "AI classID:%d stop:%.1fcm", objectClassID, targetStopDistanceCm);

    while (true)
    {
        // 1. DISTANCE STATE
        double currentDistance         = getCurrentEncoderDistanceCM() - startDist;
        double currentDistanceToTarget = initialDistance - fabs(currentDistance);
        double odometryTargetHeading   = initialOdometryHeading;
        double currentGyroHeading      = getContinuousStandardHeading();

        // 2. TIMEOUT
        if (pros::millis() - safetyStart > (uint32_t)timeoutMs) break;

        // 3. ENCODER OVERSHOOT GUARD
        if (fabs(currentDistance) >= initialDistance) break;

        // 4. JETSON DATA READ
        visionCurrentlyTracked = false;
        JetsonDetection det;
        if (getLatestDetection(objectClassID, 0.4f, &det)) {
            if (det.seqNum != lastProcessedSeq) {
                visionCurrentlyTracked     = true;
                lastProcessedSeq           = det.seqNum;
                lastVisionHorizontalOffset = det.hOffset;
                visionEverTracked          = true;
                if (det.distanceCm <= targetStopDistanceCm) {
                    consecutiveDistCount++;
                    if (consecutiveDistCount >= REQUIRED_CONSECUTIVE) break;
                } else {
                    consecutiveDistCount = 0;
                }
            }
        }

        pros::screen::print(pros::E_TEXT_MEDIUM, 1, "ever:%d curr:%d consec:%d",
            (int)visionEverTracked, (int)visionCurrentlyTracked, consecutiveDistCount);

        // 5. HEADING CALCULATION
        double rotationsDifference = std::round((currentGyroHeading - odometryTargetHeading) / 360.0);
        odometryTargetHeading += rotationsDifference * 360.0;

        double fusedTargetHeading;
        if (!visionEverTracked) {
            if (currentDistanceToTarget <= headingLockDistance) {
                if (!headingLocked) { lockedHeadingValue = odometryTargetHeading; headingLocked = true; }
                fusedTargetHeading = lockedHeadingValue;
            } else {
                headingLocked = false;
                fusedTargetHeading = odometryTargetHeading;
            }
        } else {
            // Object right (positive hOffset) → truth heading > current → positive correction → steers right
            double visualTruthHeading = currentGyroHeading + (lastVisionHorizontalOffset * AI_CAM_DEG_PER_UNIT);
            fusedTargetHeading = currentGyroHeading +
                                 ((visualTruthHeading - currentGyroHeading) * kp_distToHeadScaling);
            lastFusedHeading = fusedTargetHeading;
            if (visionCurrentlyTracked) {
                visionDropoutHandled = false;
            } else if (!visionDropoutHandled) {
                visionDropoutHandled = true;
            }
        }

        double headingCorrection = headingPID.calculate(fusedTargetHeading, currentGyroHeading);
        pros::screen::print(pros::E_TEXT_MEDIUM, 2, "odomH:%.1f fusedH:%.1f corr:%.3f",
            odometryTargetHeading, fusedTargetHeading, headingCorrection);

        // 6. MOTOR RPM READINGS
        double leftMotorRPM    = leftDrive.get_actual_velocity()   * DRIVE_MOTOR_RPM_ADJ;
        double rightMotorRPM   = rightDrive.get_actual_velocity()  * DRIVE_MOTOR_RPM_ADJ;
        double leftEncoderRPM  = passiveEncoderLeft.get_velocity()  * (encoderWheelCircumferenceCM / wheelCircumferenceCM);
        double rightEncoderRPM = passiveEncoderRight.get_velocity() * (encoderWheelCircumferenceCM / wheelCircumferenceCM);

        // 7. MOTION PHASES
        if (currentDistanceToTarget > breakDistance && !accelCompleted && !decel) {
            // LAUNCH
            double leftTractionVoltage  = tractionControlLeft.tractionControlSpeed(
                motorVoltageLeft,  leftMotorRPM,  leftEncoderRPM,  ACCEL_FACTOR_LAUNCH);
            double rightTractionVoltage = tractionControlRight.tractionControlSpeed(
                motorVoltageRight, rightMotorRPM, rightEncoderRPM, ACCEL_FACTOR_LAUNCH);
            motorVoltageLeft  = leftTractionVoltage  + (headingCorrection * accelHeadingScaling);
            motorVoltageRight = rightTractionVoltage - (headingCorrection * accelHeadingScaling);
            if (std::fabs(motorVoltageLeft)  >= std::fabs(maxSpeedVoltage) &&
                std::fabs(motorVoltageRight) >= std::fabs(maxSpeedVoltage))
                accelCompleted = true;
        } else if (currentDistanceToTarget > breakDistance && accelCompleted) {
            // CRUISE
            motorVoltageLeft  = maxSpeedVoltage + headingCorrection;
            motorVoltageRight = maxSpeedVoltage - headingCorrection;
        } else if (currentDistanceToTarget <= breakDistance && !decelCompleted) {
            // DECELERATION
            if (!decel) {
                decel = true;
                adaptiveABSLeft.initialize(std::fabs(motorVoltageLeft));
                adaptiveABSRight.initialize(std::fabs(motorVoltageRight));
            }
            adaptiveABSLeft.decelControlSpeed(leftMotorRPM,  leftEncoderRPM);
            adaptiveABSRight.decelControlSpeed(rightMotorRPM, rightEncoderRPM);
            pros::motor_brake_mode_e_t leftBrakeMode  = adaptiveABSLeft.getBrakeMode();
            pros::motor_brake_mode_e_t rightBrakeMode = adaptiveABSRight.getBrakeMode();
            double adjustedHeadingCorrection = headingCorrection * decelHeadingScaling * dir;
            leftDrive.set_brake_mode(leftBrakeMode);
            rightDrive.set_brake_mode(rightBrakeMode);
            motorVoltageLeft  = std::max(0.0,  adjustedHeadingCorrection);
            motorVoltageRight = std::max(0.0, -adjustedHeadingCorrection);
            leftEncoderRollingAverage  = rollingAverage(leftEncoderRPM,  leftEncoderRollingAverage,  3);
            rightEncoderRollingAverage = rollingAverage(rightEncoderRPM, rightEncoderRollingAverage, 3);
            if (std::fabs(leftEncoderRollingAverage)  <= std::fabs(minDriveMotorRPM) &&
                std::fabs(rightEncoderRollingAverage) <= std::fabs(minDriveMotorRPM))
                decelCompleted = true;
        } else if (decelCompleted) {
            // APPROACH
            leftDrive.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
            rightDrive.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
            motorVoltageLeft  = minSpeedVoltage + (headingCorrection * approachHeadingScaling);
            motorVoltageRight = minSpeedVoltage - (headingCorrection * approachHeadingScaling);
        }

        // 8. VOLTAGE SATURATION LIMITER
        double maxVoltage = std::max(std::fabs(motorVoltageLeft), std::fabs(motorVoltageRight));
        if (maxVoltage > absoluteMaxVoltage) {
            double scale = absoluteMaxVoltage / maxVoltage;
            motorVoltageLeft  *= scale;
            motorVoltageRight *= scale;
        }

        // 9. MOTOR OUTPUT
        leftDrive.move_voltage(static_cast<int32_t>(motorVoltageLeft  * 1000));
        rightDrive.move_voltage(static_cast<int32_t>(motorVoltageRight * 1000));

        pros::delay(10);
    }

    leftDrive.set_brake_mode(brakeMode);
    rightDrive.set_brake_mode(brakeMode);
    leftDrive.move(0);
    rightDrive.move(0);
}

// ============================================================================
// SECTION 3 — Three-phase navigation wrapper
// ============================================================================

struct BlindParams { double powerPct; double timeoutMs; bool useStall; };

static BlindParams getBlindParams(TargetType target) {
    switch (target) {
        case TargetType::LONG_GOAL:
        case TargetType::CENTER_GOAL:
        case TargetType::MATCH_LOADER: return { 35.0, BLIND_TIMEOUT_MS, true  };
        case TargetType::BLOCK:        return { 20.0, 600.0,            false };
        case TargetType::PARK_ZONE:    return {  0.0, 0.0,              false };
    }
    return { 35.0, BLIND_TIMEOUT_MS, true };
}

NavResult blindApproach(TargetType target) {
    BlindParams p = getBlindParams(target);
    if (p.powerPct <= 0.0) return NavResult::SUCCESS;

    // Convert % to millivolts for PROS move_voltage (max 12000mV)
    int32_t mv = static_cast<int32_t>(p.powerPct * 0.01 * absoluteMaxVoltage * 1000.0);

    uint32_t blindStart = pros::millis();
    uint32_t stallStart = 0;
    bool stallStarted   = false;

    leftDrive.move_voltage(mv);
    rightDrive.move_voltage(mv);

    while (true) {
        if (pros::millis() - blindStart >= static_cast<uint32_t>(p.timeoutMs)) {
            leftDrive.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
            rightDrive.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
            leftDrive.move(0);
            rightDrive.move(0);
            return NavResult::BLIND_TIMEOUT;
        }
        if (p.useStall) {
            double avgV = (fabs(leftDrive.get_actual_velocity()) +
                           fabs(rightDrive.get_actual_velocity())) / 2.0;
            avgV *= DRIVE_MOTOR_RPM_ADJ;
            if (avgV < STALL_DETECTION_RPM) {
                if (!stallStarted) { stallStart = pros::millis(); stallStarted = true; }
                else if (pros::millis() - stallStart >= static_cast<uint32_t>(STALL_CONFIRM_MS)) {
                    leftDrive.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
                    rightDrive.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
                    leftDrive.move(0);
                    rightDrive.move(0);
                    return NavResult::BLIND_CONTACT;
                }
            } else { stallStarted = false; }
        }
        pros::delay(10);
    }
}

NavResult navigateToTarget(double goalX, double goalY,
                           TargetType target,
                           const RoutePath& precomputedPath) {

    // ── Phase 1: Route planner + forwardToPoint ─────────────────────────────
    RoutePath path = (precomputedPath.count > 0)
                   ? precomputedPath
                   : routePlan(globalX, globalY, goalX, goalY);

    if (path.count == 0) return NavResult::BLOCKED_REROUTE;

    for (int i = 0; i < path.count; i++) {
        updateOdometry();
        double dx   = goalX - globalX;
        double dy   = goalY - globalY;
        double dist = sqrt(dx*dx + dy*dy);

        // Hand off to vision when close enough and Jetson has a lock
        if (dist <= VISION_HANDOFF_DISTANCE_CM && jetsonTargetTracked()) break;

        uint32_t wpStart      = pros::millis();
        const double wpTimeout = 4.0;

        // ── Phase 1: Route planner + forwardToPoint ─────────────────────────────
        StraightProfile wpProfile = DEFAULT_STRAIGHT;
        wpProfile.breakDistance          = 20.0;
        wpProfile.minSpeed               = 16.0;
        wpProfile.distanceTolerance      = 3.0;
        wpProfile.kp_heading             = 0.4;
        wpProfile.ki_heading             = 0.01;
        wpProfile.kd_heading             = 0.05;
        wpProfile.brakeMode              = pros::E_MOTOR_BRAKE_BRAKE;
        wpProfile.accelHeadingScaling    = 0.2;
        wpProfile.decelHeadingScaling    = 0.2;
        wpProfile.approachHeadingScaling = 0.2;
        wpProfile.maxSpeed               = ROBOT_CRUISE_POWER_PCT;
        wpProfile.headingLockDistance    = 8.0;
        wpProfile.timeout                = wpTimeout;

        forwardToPoint(path.x[i], path.y[i], wpProfile);

        if (pros::millis() - wpStart >= static_cast<uint32_t>(wpTimeout * 1000.0 - 50.0))
            return NavResult::BLOCKED_REROUTE;

        if (jetsonObstacleDetected()) return NavResult::BLOCKED_REROUTE;
    }

    if (target == TargetType::PARK_ZONE) return NavResult::SUCCESS;

    // ── Phase 2: Vision approach ───────────────────────────────────────────
    uint32_t visionWaitStart = pros::millis();
    while (!jetsonTargetTracked() && pros::millis() - visionWaitStart < 300)
        pros::delay(10);

    bool visionLostBeforeBlind = false;

    if (jetsonTargetTracked()) {
        moveVisionOdometryAI(CLASS_FWD_GOAL,
                             (float)BLIND_HANDOFF_DISTANCE_CM,
                             goalX, goalY,
                             15.0,
                             pros::E_MOTOR_BRAKE_COAST,
                             60.0);
        double jDist = jetsonTargetDistance();
        if (!jetsonTargetTracked() && (jDist < 0 || jDist > BLIND_HANDOFF_DISTANCE_CM))
            visionLostBeforeBlind = true;
    } else {
        visionLostBeforeBlind = true;
    }

    // ── Phase 3: Blind final push ──────────────────────────────────────────
    NavResult blindResult = blindApproach(target);

    if (visionLostBeforeBlind && blindResult == NavResult::BLIND_TIMEOUT)
        return NavResult::VISION_LOST;

    return blindResult == NavResult::BLIND_CONTACT ? NavResult::BLIND_CONTACT
         : blindResult == NavResult::BLIND_TIMEOUT  ? NavResult::BLIND_TIMEOUT
         : NavResult::SUCCESS;
}

// ============================================================================
// navigateTo — single-call navigation to a named field target
//
// Reads approach point, heading, and type from FIELD_TARGETS table.
// Waypoint loop uses 25° threshold: turns before forwardToPoint only when
// heading error exceeds what PID can safely correct in one 30cm cell.
// ============================================================================

NavResult navigateTo(TargetID id,
                     pros::AIVision::Color matchLoaderSig) {

    const NamedTarget& t = getTarget(id);

    // ── Phase 1: A* to approach point with 25° turn threshold ────────────────
    RoutePath path = routePlan(globalX, globalY, t.approachX, t.approachY);
    if (path.count == 0) {
        pros::lcd::print(2, "NO PATH to (%.0f,%.0f)", t.approachX, t.approachY);
        return NavResult::BLOCKED_REROUTE;
    }

    pros::lcd::print(2, "%d WPs → (%.0f,%.0f)", path.count, t.approachX, t.approachY);

    StraightProfile wpProfile = LOADED_MID_FWD_80;
    wpProfile.kp_heading             = 0.4;
    wpProfile.ki_heading             = 0.01;
    wpProfile.kd_heading             = 0.05;
    wpProfile.brakeMode              = pros::E_MOTOR_BRAKE_BRAKE;
    wpProfile.accelHeadingScaling    = 0.2;
    wpProfile.decelHeadingScaling    = 0.2;
    wpProfile.approachHeadingScaling = 0.2;
    wpProfile.headingLockDistance    = 8.0;
    wpProfile.timeout                = 8.0;  // per-waypoint timeout — forwardToPoint enforces this

    TurnProfile turnProfile = DEFAULT_TURN;
    turnProfile.exitTolerance = 5.0;
    turnProfile.maxSpeed      = 60.0;
    turnProfile.breakDistance = 20.0;
    turnProfile.maxCurrentA   = 5.0;

    int lcdLine = 3;  // start waypoint log at line 3, increment per waypoint
    for (int i = 0; i < path.count; i++) {
        updateOdometry();

        double wpDist, waypointHeading;
        calculatePathToTarget(globalX, globalY, path.x[i], path.y[i],
                              wpDist, waypointHeading);

        double currentH = fmod(getContinuousStandardHeading(), 360.0);
        if (currentH < 0) currentH += 360.0;

        double error = waypointHeading - currentH;
        if (error >  180.0) error -= 360.0;
        if (error < -180.0) error += 360.0;
        error = fabs(error);

        if (lcdLine <= 7) {
            pros::lcd::print(lcdLine++, "WP%d(%.0f,%.0f)e:%.0f %s",
                             i, path.x[i], path.y[i], error,
                             error > 25.0 ? "TRN" : "go");
        }

        if (error > 25.0)
            turnToPoint(path.x[i], path.y[i], turnProfile);

        uint32_t wpStart = pros::millis();
        forwardToPoint(path.x[i], path.y[i], wpProfile);

        // Timed out means forwardToPoint couldn't reach the waypoint — something blocked
        if (pros::millis() - wpStart >= static_cast<uint32_t>(wpProfile.timeout * 1000.0 - 50.0))
            return NavResult::BLOCKED_REROUTE;
    }

    // ── Parking: done ─────────────────────────────────────────────────────────
    if (t.type == TargetType::PARK_ZONE) return NavResult::SUCCESS;

    // ── Turn to face approach heading ─────────────────────────────────────────
    // Heading pulled directly from field_targets table — not computed from position.
    // currentHNorm normalized to [0,360) before delta: fmod preserves sign in C++
    // so negative continuous headings corrupt the calculation without this fix.
    {
        double currentH     = getContinuousStandardHeading();
        double currentHNorm = fmod(currentH, 360.0);
        if (currentHNorm < 0) currentHNorm += 360.0;
        double delta = fmod((t.approachHeading - currentHNorm) + 540.0, 360.0) - 180.0;
        pros::lcd::print(1, "Ph2:%s cH:%.0f tH:%.0f d:%.0f",
            t.type == TargetType::LONG_GOAL    ? "LG" :
            t.type == TargetType::MATCH_LOADER ? "ML" : "CG",
            currentH, t.approachHeading, delta);
        if (fabs(delta) > 5.0)
            turnOdometry(currentH + delta, turnProfile);  // continuous heading for turnOdometry
    }
    pros::delay(150);

    // ── DEBUG: stop here — uncomment Phase 2+3 below when approach is confirmed ─
    return NavResult::SUCCESS;

    // ── Phase 2+3: sensor approach by target type ─────────────────────────────
    // if (t.type == TargetType::LONG_GOAL) {
    //     StraightProfile backProfile = BACKWARD_STRAIGHT;
    //     backProfile.timeout = 3.0;
    //     backwardToPoint(t.targetX, t.targetY, backProfile);
    //     return blindApproach(t.type);
    // }
    // if (t.type == TargetType::MATCH_LOADER) {
    //     VisionProfile vp = DEFAULT_VISION;
    //     vp.drive.timeout = 3.0;
    //     visionOnly(matchLoaderSig, vp.minObjectWidth, 10.0, vp);
    //     return blindApproach(t.type);
    // }
    // if (t.type == TargetType::CENTER_GOAL) {
    // #if ACTIVE_BOT == BOT_24INCH
    //     uint32_t visionWaitStart = pros::millis();
    //     while (!jetsonTargetTracked() && pros::millis() - visionWaitStart < 300)
    //         pros::delay(10);
    //     if (jetsonTargetTracked())
    //         moveVisionOdometryAI(CLASS_FWD_GOAL, (float)BLIND_HANDOFF_DISTANCE_CM,
    //                              t.targetX, t.targetY, 15.0,
    //                              pros::E_MOTOR_BRAKE_COAST, 60.0);
    //     return blindApproach(t.type);
    // #else
    //     waitAndResetGPS(500);
    //     StraightProfile fwdProfile = DEFAULT_STRAIGHT;
    //     fwdProfile.timeout = 3.0;
    //     forwardToPoint(t.targetX, t.targetY, fwdProfile);
    //     return blindApproach(t.type);
    // #endif
    // }
    // return NavResult::BLOCKED_REROUTE;

    return NavResult::BLOCKED_REROUTE;
}

// ============================================================================
// SECTION 4 — Strategy functions
// ============================================================================

static bool g_isRedAlliance = true;
void setAllianceRed(bool isRed) { g_isRedAlliance = isRed; }
double getParkX() { return g_isRedAlliance ? RED_PARK_X : BLUE_PARK_X; }

// Replan helper — marks obstacle ahead, replans, retries once
static NavResult replanAndRetry(double goalX, double goalY, TargetType target) {
    updateOdometry();
    double dx = goalX - globalX, dy = goalY - globalY;
    double dist = sqrt(dx*dx + dy*dy);
    if (dist > 1.0)
        routeAddObstacle(globalX + (dx/dist)*30.48, globalY + (dy/dist)*30.48);

    RoutePath newPath = routePlan(globalX, globalY, goalX, goalY);
    routeClearObstacles();  // robots move — clear immediately after planning

    if (newPath.count == 0) return NavResult::BLOCKED_REROUTE;
    return navigateToTarget(goalX, goalY, target, newPath);
}

// Shared action handler — navigate, replan once on block, execute action
static void executeStrategy(double goalX, double goalY, TargetType target,
                             void (*mechanismAction)()) {
    NavResult result = navigateToTarget(goalX, goalY, target);
    if (result == NavResult::BLOCKED_REROUTE)
        result = replanAndRetry(goalX, goalY, target);

    if (result == NavResult::SUCCESS      ||
        result == NavResult::BLIND_CONTACT ||
        result == NavResult::BLIND_TIMEOUT) {
        if (mechanismAction) mechanismAction();
    }
}

// Mechanism stubs — replace with actual function calls when defined
static void doScore()   { score(1000, 100); }
static void doDescore() { outtake(1000, 80); }
static void doIntake()  { intakeHopperStart(3000, 80); }
static void doNothing() {}

void strategyScoreTopGoal() {
    const NamedTarget& t = getTarget(GOAL_NE);
    executeStrategy(t.approachX, t.approachY, TargetType::LONG_GOAL, doScore);
}
void strategyScoreBottomGoal() {
    const NamedTarget& t = getTarget(GOAL_SE);
    executeStrategy(t.approachX, t.approachY, TargetType::LONG_GOAL, doScore);
}
void strategyScoreCenterGoal() {
    const NamedTarget& t = getTarget(CENTER_NE);  // default arm — change per route
#if ACTIVE_BOT == BOT_15INCH
    RoutePath path = routePlan(globalX, globalY, t.approachX, t.approachY);
    if (path.count > 0) routeExecute(path);
    turnOdometry(t.approachHeading);
    waitAndResetGPS(500);
    executeStrategy(t.targetX, t.targetY, TargetType::CENTER_GOAL, doScore);
#else
    executeStrategy(t.approachX, t.approachY, TargetType::CENTER_GOAL, doScore);
#endif
}
void strategyDescoreTopGoal() {
    const NamedTarget& t = getTarget(GOAL_NE);
    executeStrategy(t.approachX, t.approachY, TargetType::LONG_GOAL, doDescore);
}
void strategyDescoreBottomGoal() {
    const NamedTarget& t = getTarget(GOAL_SE);
    executeStrategy(t.approachX, t.approachY, TargetType::LONG_GOAL, doDescore);
}
void strategyBlockTopGoal() {
    const NamedTarget& t = getTarget(GOAL_NE);
    executeStrategy(t.approachX, t.approachY, TargetType::PARK_ZONE, doNothing);
}
void strategyBlockBottomGoal() {
    const NamedTarget& t = getTarget(GOAL_SE);
    executeStrategy(t.approachX, t.approachY, TargetType::PARK_ZONE, doNothing);
}
void strategyUseMatchLoader() {
    updateOdometry();
    double bestDist = 1e9;
    TargetID bestID = LOADER_NE;
    const TargetID loaderIDs[4] = { LOADER_NE, LOADER_SE, LOADER_SW, LOADER_NW };
    for (int i = 0; i < 4; i++) {
        const NamedTarget& t = getTarget(loaderIDs[i]);
        double dx = t.targetX - globalX;
        double dy = t.targetY - globalY;
        double d  = sqrt(dx*dx + dy*dy);
        if (d < bestDist) { bestDist = d; bestID = loaderIDs[i]; }
    }
    const NamedTarget& best = getTarget(bestID);
    executeStrategy(best.approachX, best.approachY, TargetType::MATCH_LOADER, doIntake);
}
void strategyPark() {
    NavResult result = navigateToTarget(getParkX(), PARK_Y, TargetType::PARK_ZONE);
    if (result == NavResult::BLOCKED_REROUTE)
        replanAndRetry(getParkX(), PARK_Y, TargetType::PARK_ZONE);
}

// ============================================================================
// SECTION 5 — Match dispatch loop
// ============================================================================

static std::atomic<int> g_strategyCode(STRATEGY_IDLE);

void setStrategyCode(int code)  { g_strategyCode.store(code); }
int  getStrategyCode()          { return g_strategyCode.load(); }

static const double AUTON_DURATION_SEC = 60.0;

// Match start time — captured once when runAIMatch() is called
static uint32_t g_matchStartMs = 0;

static double timeRemainingSeconds() {
    double elapsed   = (pros::millis() - g_matchStartMs) / 1000.0;
    double remaining = AUTON_DURATION_SEC - elapsed;
    return remaining > 0.0 ? remaining : 0.0;
}

static bool shouldParkNow() {
    double timeLeft = timeRemainingSeconds();
    if (timeLeft <= PARK_TIME_BUFFER_SEC) return true;

    RoutePath pathToPark = routePlan(globalX, globalY, getParkX(), PARK_Y);
    if (pathToPark.count == 0) return timeLeft <= 8.0;

    return timeLeft <= (pathToPark.estimatedTimeSec + PARK_TIME_BUFFER_SEC);
}

static bool strategyIsLegal(int code) {
    // VAIG3/SG7: during isolation window (~first 15s), reject cross-field strategies
    if (timeRemainingSeconds() > AUTON_DURATION_SEC - 15.0) {
        if (code == STRATEGY_DESCORE_TOP || code == STRATEGY_BLOCK_TOP)
            return false;
    }
    return true;
}

void runAIMatch() {
    g_matchStartMs = pros::millis();  // capture match start for timer
    updateOdometry();
    int lastCode = STRATEGY_IDLE;

    while (true) {
        // Priority 1 — timer (always wins)
        if (shouldParkNow()) { strategyPark(); return; }

        // Priority 2 — safety/rules
        int currentCode = getStrategyCode();
        if (!strategyIsLegal(currentCode)) currentCode = lastCode;

        // Priority 3/4 — Jetson strategy or last known fallback
        // Note: legacy aliases map to same values as new codes — no duplicate cases
        switch (currentCode) {
            case STRATEGY_SCORE_TOP:        strategyScoreTopGoal();      break;  // 1
            case STRATEGY_SCORE_BOTTOM:     strategyScoreBottomGoal();   break;  // 2
            case STRATEGY_SCORE_CENTER:     strategyScoreCenterGoal();   break;  // 3
            case STRATEGY_DESCORE_TOP:      strategyDescoreTopGoal();    break;  // 4
            case STRATEGY_DESCORE_BOTTOM:   strategyDescoreBottomGoal(); break;  // 5
            case STRATEGY_BLOCK_TOP:        strategyBlockTopGoal();      break;  // 6
            case STRATEGY_BLOCK_BOTTOM:     strategyBlockBottomGoal();   break;  // 7
            case STRATEGY_USE_MATCH_LOADER: strategyUseMatchLoader();    break;  // 8
            case STRATEGY_PARK:             strategyPark(); return;              // 9
            case STRATEGY_IDLE:
            default:                        pros::delay(50);             break;  // 0
        }

        if (currentCode != STRATEGY_IDLE) lastCode = currentCode;
        pros::delay(10);
    }
}

// ============================================================================
// SECTION 6 — Behavior stubs
// ============================================================================

void visionSweepNorth() {
    // Not yet implemented
}