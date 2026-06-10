#ifndef MOTION_CONFIG_H
#define MOTION_CONFIG_H

#include "main.h"  // PROS entry point — pros::motor_brake_mode_e_t

// ══════════════════════════════════════════════════════════════════════════════
// MOTION PROFILE STRUCTS
//
// Single source of truth for all tunable motion parameters.
// Each struct covers one motion family:
//   StraightProfile — straightDistance, forwardToPoint, backwardToPoint,
//                     driveForward, driveBackward
//   TurnProfile     — turnOdometry, pivotTurnOdometryV2, and all turn wrappers
//   VisionProfile   — visionForwardToPoint, visionBackwardToPoint,
//                     visionDriveForward, visionDriveBackward, visionOnly
//                     (NOT visionDriveMinimal or visionDriveV2 — legacy arch)
//
// Usage — use defaults:
//   driveForward(60.0);
//
// Usage — override one field:
//   StraightProfile fast = DEFAULT_STRAIGHT;
//   fast.maxSpeed = 60.0;
//   driveForward(60.0, fast);
//
// Do NOT modify the DEFAULT_* constants at runtime. Copy, then modify.
// ══════════════════════════════════════════════════════════════════════════════


// ──────────────────────────────────────────────────────────────────────────────
// StraightProfile
// Covers: straightDistance, forwardToPoint, backwardToPoint,
//         driveForward, driveBackward
// ──────────────────────────────────────────────────────────────────────────────
struct StraightProfile {
    // ── Motion shape ──────────────────────────────────────────────────────────
    double breakDistance;           // cm before target to begin decel
    double minSpeed;                // minimum speed % during approach phase
    double maxSpeed;                // peak cruise speed %
    double distanceTolerance;       // exit bubble radius (cm)
    double timeout;                 // maximum run time (seconds)
    pros::motor_brake_mode_e_t brakeMode;  // motor behavior at stop

    // ── Heading PID ───────────────────────────────────────────────────────────
    double kp_heading;              // proportional gain
    double ki_heading;              // integral gain
    double kd_heading;              // derivative gain

    // ── Phase heading scaling ─────────────────────────────────────────────────
    // Scales how aggressively heading correction is applied in each phase.
    // 0.0 = no correction that phase, 1.0 = full PID output applied.
    double accelHeadingScaling;     // correction weight during acceleration
    double decelHeadingScaling;     // correction weight during deceleration
    double approachHeadingScaling;  // correction weight during final approach

    // ── forwardToPoint / backwardToPoint-only fields ──────────────────────────
    // headingLockDistance: freeze heading target within this many cm of the
    // destination to prevent atan2 instability on the final approach.
    double headingLockDistance;

    // ── Internal motion constants ─────────────────────────────────────────────
    // Rarely changed; unified here so all straight functions stay in sync.
    double launchVoltage;           // initial kick voltage to overcome static friction (V)
    double accelFactor;             // traction control voltage ramp multiplier per tick
    double slipThreshold;           // motor vs encoder RPM difference before traction cuts in
    double decelStepPercent;        // ABS voltage reduction per step during decel
    double lockThreshold;           // encoder/motor RPM ratio that declares wheel lockup

    // ── Overcurrent protection ────────────────────────────────────────────────
    // Circuit breaker — exits the move if drive current stays above maxCurrentA
    // for overcurrentDurationMs milliseconds. Protects against stalls and jams.
    // Set maxCurrentA = 50.0 to disable.
    double   maxCurrentA;           // total drive current trip threshold (amps)
    uint32_t overcurrentDurationMs;  // ms current must stay high before tripping
};

// ──────────────────────────────────────────────────────────────────────────────
// TurnProfile
// Covers: turnOdometry, pivotTurnOdometryV2, turnLeft, turnRight,
//         pivotLeftMP, pivotRightMP, turnToPoint, turnLeftToPoint, turnRightToPoint
// ──────────────────────────────────────────────────────────────────────────────
struct TurnProfile {
    // ── Motion shape ──────────────────────────────────────────────────────────
    double breakDistance;           // degrees before target to begin decel
    double minSpeed;                // approach phase speed %
    double maxSpeed;                // peak turn speed %
    double exitTolerance;           // acceptable heading error to declare turn complete (degrees)
    double timeout;                 // maximum run time (seconds)

    // ── Internal motion constants ─────────────────────────────────────────────
    // Turn-specific values — deliberately different from StraightProfile.
    double accelFactor;             // traction control ramp multiplier during turn launch
    double slipThreshold;           // motor vs encoder RPM difference before traction cuts in
    double decelStepPercent;        // ABS voltage reduction per step during decel
    double lockThreshold;           // encoder/motor RPM ratio that declares wheel lockup

    // ── Overcurrent protection ────────────────────────────────────────────────
    // Circuit breaker — exits the move if drive current stays above maxCurrentA
    // for overcurrentDurationMs milliseconds. Protects against stalls and jams.
    // Set maxCurrentA = 50.0 to disable.
    double   maxCurrentA;           // total drive current trip threshold (amps)
    uint32_t overcurrentDurationMs;  // ms current must stay high before tripping
};

// ──────────────────────────────────────────────────────────────────────────────
// VisionProfile
// Covers: visionForwardToPoint, visionBackwardToPoint, visionDriveForward,
//         visionDriveBackward, visionOnly
// Does NOT cover visionDriveMinimal or visionDriveV2 — those are legacy
// simple architecture with individual params, left as-is.
//
// Motion params live in the embedded StraightProfile — single source of truth.
// Tune drive behaviour in `drive`; vision-only params follow below.
// ──────────────────────────────────────────────────────────────────────────────
struct VisionProfile {
    // ── Motion params — single source of truth ────────────────────────────────
    // All motion shape, heading PID, phase scaling, traction, ABS, and
    // overcurrent fields are inherited from StraightProfile.
    // Vision functions read p.drive.kp_heading, p.drive.maxSpeed etc —
    // identical field names to forwardToPoint / backwardToPoint.
    StraightProfile drive;

    // ── Vision heading fusion ─────────────────────────────────────────────────
    // kp_vision_heading: heading PID gain used AFTER vision first locks.
    // Replaces drive.kp_heading once a valid detection is acquired.
    // Tune independently — vision-guided correction typically needs different
    // gain than pure odometry. drive.kp_heading still governs pre-lock behaviour.
    double kp_vision_heading;

    // kp_distToHeadScaling: aggressiveness of vision correction.
    // 0.0 = hold odometry heading (ignores vision), 1.0 = full snap to object.
    double kp_distToHeadScaling;

    // ── Vision object filter ──────────────────────────────────────────────────
    // Bounding box and size filter — ignores detections outside this screen region.
    // Screen is 320×240 pixels.
    int    minObjectWidth;          // minimum pixel width for a valid detection
    int    minX;                    // left bound of valid detection zone (pixels)
    int    maxX;                    // right bound of valid detection zone (pixels)
    int    minY;                    // top bound of valid detection zone (pixels)
    int    maxY;                    // bottom bound of valid detection zone (pixels)
};


// ══════════════════════════════════════════════════════════════════════════════
// DEFAULT PROFILE INSTANCES
//
// Defined in motion_config.cpp. Import via extern — do not redefine.
// These are the calibrated field defaults. Copy before modifying.
// ══════════════════════════════════════════════════════════════════════════════
extern const StraightProfile DEFAULT_STRAIGHT;      // general forward drive
extern const StraightProfile BACKWARD_STRAIGHT;     // backward drive (softer gains)
extern const StraightProfile LOADED_MID_FWD_80;     // 24" bot loaded, forward, 80% speed
extern const TurnProfile     DEFAULT_TURN;           // point turn (both sides drive)
extern const TurnProfile     DEFAULT_PIVOT;          // pivot turn (one side brakes)
extern const VisionProfile   DEFAULT_VISION;         // vision-guided approach
extern const VisionProfile   VISION_LONG_GOAL_FWD;  // orangeBase forward approach, 24" bot long goal


// ══════════════════════════════════════════════════════════════════════════════
// TIERED NAVIGATION PROFILES
//
// Selected automatically by selectFwdProfile / selectBwdProfile / selectTurnProfile
// based on the distance or angle of each individual waypoint segment.
//
// ── Range thresholds — change ONLY here to reclassify segments ───────────────
// Straight: SHORT = 1–STRAIGHT_SHORT_MAX_CM, MID = up to STRAIGHT_MID_MAX_CM,
//           LONG = anything above.
// Turn:     SHORT = 1–TURN_SHORT_MAX_DEG, MID = up to TURN_MID_MAX_DEG,
//           LONG = anything above.
// ─────────────────────────────────────────────────────────────────────────────
constexpr double STRAIGHT_SHORT_MAX_CM = 50.0;   // cm  — SHORT/MID boundary
constexpr double STRAIGHT_MID_MAX_CM   = 80.0;   // cm  — MID/LONG boundary
constexpr double TURN_SHORT_MAX_DEG    = 50.0;   // deg — SHORT/MID boundary
constexpr double TURN_MID_MAX_DEG      = 100.0;  // deg — MID/LONG boundary

// ── Tiered forward straight profiles ─────────────────────────────────────────
extern const StraightProfile SHORT_FWD;   // <= STRAIGHT_SHORT_MAX_CM
extern const StraightProfile MID_FWD;     // <= STRAIGHT_MID_MAX_CM
extern const StraightProfile LONG_FWD;    // >  STRAIGHT_MID_MAX_CM

// ── Tiered backward straight profiles ────────────────────────────────────────
extern const StraightProfile SHORT_BWD;   // <= STRAIGHT_SHORT_MAX_CM
extern const StraightProfile MID_BWD;     // <= STRAIGHT_MID_MAX_CM
extern const StraightProfile LONG_BWD;    // >  STRAIGHT_MID_MAX_CM

// ── Tiered turn profiles ──────────────────────────────────────────────────────
extern const TurnProfile SHORT_TURN;      // <= TURN_SHORT_MAX_DEG
extern const TurnProfile MID_TURN;        // <= TURN_MID_MAX_DEG
extern const TurnProfile LONG_TURN;       // >  TURN_MID_MAX_DEG

// ── Selectors — return const ref to the correct named profile ─────────────────
// Call with the measured segment distance or turn angle; profile is chosen
// automatically from the thresholds above. No inline overrides at call sites.
const StraightProfile& selectFwdProfile(double distCm);
const StraightProfile& selectBwdProfile(double distCm);
const TurnProfile&     selectTurnProfile(double degrees);

#endif // MOTION_CONFIG_H