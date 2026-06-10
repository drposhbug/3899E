/*----------------------------------------------------------------------------
 * ai.cpp — VAIRC-specific AI functions for the V5 Brain (Team 3899E)
 *
 * Contains all VAIRC match logic:
 *   SECTION 1 — Jetson data accessors
 *   SECTION 2 — moveVisionOdometryAI (vision navigation)
 *   SECTION 3 — Navigation helper functions (blindApproach, navigateTo)
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
#include "motion_config.h"  // VISION_LONG_GOAL_FWD and all named motion profiles
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

        // Profile selected per segment — distance and angle determine which tier
        StraightProfile wpProfile   = selectFwdProfile(wpDist);
        TurnProfile     turnProfile = selectTurnProfile(error);
        wpProfile.timeout = 8.0;  // per-waypoint safety ceiling — not a tuning value

        if (lcdLine <= 7) {
            pros::lcd::print(lcdLine++, "WP%d(%.0f,%.0f)e:%.0f %s",
                             i, path.x[i], path.y[i], error,
                             error > 25.0 ? "TRN" : "go");
        }

        if (error > 25.0)
            turnToPoint(path.x[i], path.y[i], turnProfile);

        uint32_t wpStart = pros::millis();
        forwardToPoint(path.x[i], path.y[i], wpProfile);

        // Timeout on a transit waypoint = obstacle or position drift.
        // GPS reset and continue to the next waypoint — do not abort the route.
        if (pros::millis() - wpStart >= static_cast<uint32_t>(wpProfile.timeout * 1000.0 - 50.0)) {
            requestGpsReset();
        }
    }

    // ── Parking: turn to face wall, then done ────────────────────────────────
    if (t.type == TargetType::PARK_ZONE) {
        double currentH     = getContinuousStandardHeading();
        double currentHNorm = fmod(currentH, 360.0);
        if (currentHNorm < 0) currentHNorm += 360.0;
        double delta = fmod((t.approachHeading - currentHNorm) + 540.0, 360.0) - 180.0;
        if (fabs(delta) > 5.0)
            turnOdometry(currentH + delta, selectTurnProfile(fabs(delta)));
        return NavResult::SUCCESS;
    }

    // ── Turn to face target ───────────────────────────────────────────────────
    // Long goals and match loaders use turnToPoint — computes correct heading
    // from current position to target directly.
    // Center goals and park zones use approachHeading (diagonal/wall headings).
    if (t.type == TargetType::LONG_GOAL || t.type == TargetType::MATCH_LOADER) {
        turnToPoint(t.targetX, t.targetY, selectTurnProfile(180.0));
    } else {
        double currentH     = getContinuousStandardHeading();
        double currentHNorm = fmod(currentH, 360.0);
        if (currentHNorm < 0) currentHNorm += 360.0;
        double delta = fmod((t.approachHeading - currentHNorm) + 540.0, 360.0) - 180.0;
        pros::lcd::print(1, "Ph2:%s cH:%.0f tH:%.0f d:%.0f",
            t.type == TargetType::MATCH_LOADER ? "ML" : "CG",
            currentH, t.approachHeading, delta);
        if (fabs(delta) > 5.0)
            turnOdometry(currentH + delta, selectTurnProfile(fabs(delta)));
    }

    // ── Phase 2: final approach — target-type specific, 24" bot ──────────────
#if ACTIVE_BOT == BOT_24INCH
    if (t.type == TargetType::LONG_GOAL) {
        // 1s settle after approach turn before vision locks on
        pros::delay(1000);

        // Vision-guided approach — orangeBase signature, circuit breaker on contact
        visionForwardToPoint(aiVision_orangeBase,
                             220,              // stop when orangeBase reaches 220px wide
                             t.targetX, t.targetY,
                             VISION_LONG_GOAL_FWD);

        // GPS reset is non-blocking — task runs in parallel with scoring.
        requestGpsReset();

        return NavResult::SUCCESS;
    }

    if (t.type == TargetType::MATCH_LOADER) {
        // Final drive to post — wall contact (stall/overcurrent/timeout) = SUCCESS.
        // No settle delay needed — odometry approach, no vision lock.
        forwardToPoint(t.targetX, t.targetY, SHORT_FWD);
        requestGpsReset();
        return NavResult::SUCCESS;
    }
#endif

    // Non-long-goal targets and 15" bot: A* transit + approach turn is the
    // full Phase 1. Phase 2+3 for these targets added in a later pass.
    return NavResult::SUCCESS;

    // ── Phase 2+3 stubs — kept for reference, enable per target type later ────
    // Each target type has a 24" bot path and a 15" bot path.
    // 24" bot: Jetson vision (YOLO) or VEX AI Vision sensor.
    // 15" bot: GPS reset + odometry drive. Profile tier chosen from target distance.
    // Both paths end with blindApproach() for final contact confirmation.
    //
    // To enable: remove the debug return above and uncomment this block.
    // ─────────────────────────────────────────────────────────────────────────
    // updateOdometry();
    // double ph2Dist = hypot(t.targetX - globalX, t.targetY - globalY);
    //
    // if (t.type == TargetType::LONG_GOAL) {
    // #if ACTIVE_BOT == BOT_24INCH
    //     // 24" bot: Jetson vision — rear approach, back into goal
    //     uint32_t visionWaitStart = pros::millis();
    //     while (!jetsonTargetTracked() && pros::millis() - visionWaitStart < 300)
    //         pros::delay(10);
    //     if (jetsonTargetTracked())
    //         moveVisionOdometryAI(CLASS_FWD_GOAL, (float)BLIND_HANDOFF_DISTANCE_CM,
    //                              t.targetX, t.targetY, 15.0,
    //                              pros::E_MOTOR_BRAKE_COAST, 60.0);
    //     return blindApproach(t.type);
    // #else
    //     // 15" bot: GPS reset + backward odometry — rear faces goal
    //     waitAndResetGPS(500);
    //     backwardToPoint(t.targetX, t.targetY, selectBwdProfile(ph2Dist));
    //     return blindApproach(t.type);
    // #endif
    // }
    //
    // if (t.type == TargetType::MATCH_LOADER) {
    // #if ACTIVE_BOT == BOT_24INCH
    //     // 24" bot: VEX AI Vision sensor — front approach
    //     visionOnly(matchLoaderSig, DEFAULT_VISION.minObjectWidth, 10.0, DEFAULT_VISION);
    //     return blindApproach(t.type);
    // #else
    //     // 15" bot: GPS reset + forward odometry
    //     waitAndResetGPS(500);
    //     forwardToPoint(t.targetX, t.targetY, selectFwdProfile(ph2Dist));
    //     return blindApproach(t.type);
    // #endif
    // }
    //
    // if (t.type == TargetType::CENTER_GOAL) {
    // #if ACTIVE_BOT == BOT_24INCH
    //     // 24" bot: Jetson vision — forward approach
    //     uint32_t visionWaitStart = pros::millis();
    //     while (!jetsonTargetTracked() && pros::millis() - visionWaitStart < 300)
    //         pros::delay(10);
    //     if (jetsonTargetTracked())
    //         moveVisionOdometryAI(CLASS_FWD_GOAL, (float)BLIND_HANDOFF_DISTANCE_CM,
    //                              t.targetX, t.targetY, 15.0,
    //                              pros::E_MOTOR_BRAKE_COAST, 60.0);
    //     return blindApproach(t.type);
    // #else
    //     // 15" bot: GPS reset + forward odometry
    //     waitAndResetGPS(500);
    //     forwardToPoint(t.targetX, t.targetY, selectFwdProfile(ph2Dist));
    //     return blindApproach(t.type);
    // #endif
    // }
    //
    // return NavResult::BLOCKED_REROUTE;
}

// ============================================================================
// SECTION 4 — Strategy functions
// ============================================================================

static bool g_isRedAlliance = true;
void setAllianceRed(bool isRed) { g_isRedAlliance = isRed; }
double getParkX() { return g_isRedAlliance ? RED_PARK_X : BLUE_PARK_X; }

// Mechanism stubs — replace with actual function calls when defined
static void doScore()   { score(1000, 100); }
static void doDescore() { outtake(1000, 80); }
static void doIntake()  { intakeHopperStart(3000, 80); }
static void doNothing() {}

// Shared result check — mechanism fires on any successful arrival
static bool arrivedAt(NavResult r) {
    return r == NavResult::SUCCESS      ||
           r == NavResult::BLIND_CONTACT ||
           r == NavResult::BLIND_TIMEOUT;
}

void strategyScoreTopGoal() {
    if (arrivedAt(navigateTo(LONG_GOAL_NE))) doScore();
}

void strategyScoreBottomGoal() {
    if (arrivedAt(navigateTo(LONG_GOAL_SE))) doScore();
}

void strategyScoreCenterGoal() {
#if ACTIVE_BOT == BOT_15INCH
    // 15" bot: own approach sequence — navigateTo handles A* + turns via the
    // 15" field_targets table (rear-facing headings, GPS-reset Phase 2).
    if (arrivedAt(navigateTo(CENTER_GOAL_NE))) doScore();
#else
    // 24" bot: standard navigateTo — Jetson vision Phase 2 when uncommented
    if (arrivedAt(navigateTo(CENTER_GOAL_NE))) doScore();
#endif
}

void strategyDescoreTopGoal() {
    if (arrivedAt(navigateTo(LONG_GOAL_NE))) doDescore();
}

void strategyDescoreBottomGoal() {
    if (arrivedAt(navigateTo(LONG_GOAL_SE))) doDescore();
}

void strategyBlockTopGoal() {
    // Navigate to goal position and hold — no mechanism action
    navigateTo(LONG_GOAL_NE);
}

void strategyBlockBottomGoal() {
    navigateTo(LONG_GOAL_SE);
}

void strategyUseMatchLoader() {
    // Pick nearest match loader — compare approach points to current position
    updateOdometry();
    double bestDist = 1e9;
    TargetID bestID = LOADER_NE;
    const TargetID loaderIDs[4] = { LOADER_NE, LOADER_SE, LOADER_SW, LOADER_NW };
    for (int i = 0; i < 4; i++) {
        const NamedTarget& t = getTarget(loaderIDs[i]);
        double dx = t.approachX - globalX;
        double dy = t.approachY - globalY;
        double d  = sqrt(dx*dx + dy*dy);
        if (d < bestDist) { bestDist = d; bestID = loaderIDs[i]; }
    }
    if (arrivedAt(navigateTo(bestID))) doIntake();
}

void strategyPark() {
    navigateTo(g_isRedAlliance ? PARK_ALLIANCE : PARK_OPPONENT);
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
        if (shouldParkNow()) {
            routeOpenParkZones();  // clear D blocks so A* can route into park zone
            strategyPark();
            return;
        }

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

// ── visionSweepNorth ──────────────────────────────────────────────────────────
// Reactive block sweep using VEX AI Vision sensor.
//
// Loop:
//   1. Look for alliance-colour blocks via aiVision
//   2. Block visible → steer toward it and drive
//   3. No block visible → spin in place until one appears (max 360°)
//   4. Stall detected (hit wall/block pile) → back up 20cm, then scan again
//   5. After 30s → navigate to nearest long goal
//
// Alliance colour selected from g_isRedAlliance set by setAllianceRed().
// Intake runs continuously throughout.
//
// Tuning constants at top — do not inline-override elsewhere.
// ─────────────────────────────────────────────────────────────────────────────

void visionSweepNorth() {

    // ── Tuning constants ──────────────────────────────────────────────────────
    static constexpr double SWEEP_DURATION_MS  = 30000.0; // total sweep time (ms)
    static constexpr double SWEEP_TURN_SPEED   = 25.0;    // % speed while scanning
    static constexpr double SCAN_MAX_DEG       = 360.0;   // max scan before giving up
    static constexpr double SCAN_STEP_MS       = 20.0;    // scan loop interval (ms)
    static constexpr int    MIN_BLOCK_WIDTH_PX = 8;       // minimum detection width (px)
    static constexpr int    CHASE_PIXEL_WIDTH  = 60;      // stop chase when block this wide
    static constexpr double CHASE_DISTANCE_CM  = 50.0;    // safety distance cap per chase
    static constexpr double BACKUP_MS          = 500.0;   // ms to back up after each chase

    // ── visionOnly profile for block chasing ─────────────────────────────────
    // Seeded from DEFAULT_VISION — short timeout, low current trip for block contact.
    VisionProfile sweepProfile = DEFAULT_VISION;
    sweepProfile.drive.maxSpeed            = 40.0;   // % — slow enough to intake reliably
    sweepProfile.drive.minSpeed            = 15.0;   // %
    sweepProfile.drive.timeout             = 3.0;    // s per chase — move on if no contact
    sweepProfile.drive.maxCurrentA         = 4.0;    // A — trips on block/wall contact
    sweepProfile.drive.overcurrentDurationMs = 300;  // ms
    sweepProfile.minObjectWidth            = MIN_BLOCK_WIDTH_PX;
    sweepProfile.kp_distToHeadScaling      = 0.4;

    // ── Alliance colour ───────────────────────────────────────────────────────
    pros::AIVision::Color blockSig = g_isRedAlliance ? aiVision_redCube : aiVision_blueCube;

    // ── Intake runs for full sweep duration ───────────────────────────────────
    intakeHopperStart(SWEEP_DURATION_MS + 2000.0, 80.0, 0.0, true);

    uint32_t sweepStart = pros::millis();

    // ── Main sweep loop ───────────────────────────────────────────────────────
    while (pros::millis() - sweepStart < static_cast<uint32_t>(SWEEP_DURATION_MS)) {

        // 1. CHECK — is a block visible right now?
        bool blockVisible = false;
        {
            int cnt = aiVision.get_object_count();
            for (int i = 0; i < cnt; i++) {
                pros::AIVision::Object o = aiVision.get_object(i);
                if (pros::AIVision::is_type(o, pros::AivisionDetectType::color) &&
                    o.id == blockSig.id &&
                    o.object.color.width >= MIN_BLOCK_WIDTH_PX) {
                    blockVisible = true;
                    break;
                }
            }
        }

        if (blockVisible) {
            // 2. CHASE — visionOnly drives toward block, exits on:
            //    - block fills CHASE_PIXEL_WIDTH px (close enough to intake)
            //    - overcurrent trip (wall or block pile contact)
            //    - timeout (3s)
            //    - encoder distance cap (CHASE_DISTANCE_CM)
            visionOnly(blockSig, CHASE_PIXEL_WIDTH, CHASE_DISTANCE_CM, sweepProfile);

            // 3. BACK UP — clear contact point so we can turn and scan again
            leftDrive.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
            rightDrive.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
            int32_t backMv = static_cast<int32_t>(-sweepProfile.drive.minSpeed * 0.01 * absoluteMaxVoltage * 1000.0);
            leftDrive.move_voltage(backMv);
            rightDrive.move_voltage(backMv);
            pros::delay(static_cast<uint32_t>(BACKUP_MS));
            leftDrive.move(0);
            rightDrive.move(0);
            pros::delay(100);

        } else {
            // 4. SCAN — spin until block appears or 360° elapsed
            leftDrive.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
            rightDrive.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

            int32_t turnMv  = static_cast<int32_t>(SWEEP_TURN_SPEED * 0.01 * absoluteMaxVoltage * 1000.0);
            double  degTurned = 0.0;
            double  lastH     = getContinuousStandardHeading();

            while (degTurned < SCAN_MAX_DEG &&
                   pros::millis() - sweepStart < static_cast<uint32_t>(SWEEP_DURATION_MS)) {

                bool scanFound = false;
                {
                    int scnt = aiVision.get_object_count();
                    for (int si = 0; si < scnt; si++) {
                        pros::AIVision::Object so = aiVision.get_object(si);
                        if (pros::AIVision::is_type(so, pros::AivisionDetectType::color) &&
                            so.id == blockSig.id &&
                            so.object.color.width >= MIN_BLOCK_WIDTH_PX) {
                            scanFound = true;
                            break;
                        }
                    }
                }
                if (scanFound) break;

                leftDrive.move_voltage( turnMv);
                rightDrive.move_voltage(-turnMv);
                pros::delay(static_cast<uint32_t>(SCAN_STEP_MS));

                double newH = getContinuousStandardHeading();
                degTurned  += std::fabs(newH - lastH);
                lastH       = newH;
            }

            leftDrive.move(0);
            rightDrive.move(0);
            pros::delay(50);
        }
    }

    // ── Sweep done — stop everything ─────────────────────────────────────────
    leftDrive.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    rightDrive.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    leftDrive.move(0);
    rightDrive.move(0);
    intakeHopperStop();

    // ── Navigate to nearest long goal ─────────────────────────────────────────
    updateOdometry();

    const TargetID longGoals[4] = { LONG_GOAL_NE, LONG_GOAL_SE, LONG_GOAL_NW, LONG_GOAL_SW };
    TargetID nearestGoal = LONG_GOAL_NE;
    double   bestDist    = 1e9;

    for (int i = 0; i < 4; i++) {
        const NamedTarget& t = getTarget(longGoals[i]);
        double dx = t.approachX - globalX;
        double dy = t.approachY - globalY;
        double d  = std::hypot(dx, dy);
        if (d < bestDist) { bestDist = d; nearestGoal = longGoals[i]; }
    }

    navigateTo(nearestGoal);
}