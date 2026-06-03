#ifndef MOTION_CONFIG_H
#define MOTION_CONFIG_H

#include "main.h"  // PROS entry point — pros::motor_brake_mode_e_t

// ══════════════════════════════════════════════════════════════════════════════
// MOTION PROFILE STRUCTS
//
// Single source of truth for all tunable motion parameters.
// Each struct covers one motion family:
//   StraightProfile — straightDistance, forwardToPoint, and all straight wrappers
//   TurnProfile     — turnOdometry, pivotTurnOdometryV2, and all turn wrappers
//   VisionProfile   — moveVisionOdometry, moveVisionOdometryOpen, visionOnly
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
};

// ──────────────────────────────────────────────────────────────────────────────
// VisionProfile
// Covers: moveVisionOdometry, moveVisionOdometryOpen, visionOnly
// Does NOT cover visionDriveMinimal or visionDriveV2 — those are a separate
// simpler architecture with different params and no motion phases.
// ──────────────────────────────────────────────────────────────────────────────
struct VisionProfile {
    // ── Motion shape ──────────────────────────────────────────────────────────
    double breakDistance;           // cm from target to begin decel
    double minSpeed;                // approach phase speed %
    double maxSpeed;                // peak cruise speed %
    double distanceTolerance;       // odometry exit bubble radius (cm)
    double timeout;                 // maximum run time (seconds)
    pros::motor_brake_mode_e_t brakeMode;  // motor behavior at stop

    // ── Heading PID ───────────────────────────────────────────────────────────
    double kp_head;                 // proportional gain
    double ki_head;                 // integral gain
    double kd_head;                 // derivative gain

    // ── Vision heading fusion ─────────────────────────────────────────────────
    // kp_distToHeadScaling: aggressiveness of vision correction.
    // 0.0 = hold odometry heading (ignores vision), 1.0 = full snap to object.
    double kp_distToHeadScaling;

    // ── Phase heading scaling ─────────────────────────────────────────────────
    double accelHeadingScaling;     // correction weight during acceleration
    double decelHeadingScaling;     // correction weight during deceleration
    double approachHeadingScaling;  // correction weight during final approach

    // ── Heading lock ──────────────────────────────────────────────────────────
    // Freeze heading target within this many cm of target to prevent atan2
    // instability. Only active before vision acquires.
    double headingLockDistance;

    // ── Vision object filter ──────────────────────────────────────────────────
    // Bounding box and size filter — ignores detections outside this screen region.
    int    minObjectWidth;          // minimum pixel width for a valid detection
    int    minX;                    // left bound of valid detection zone (pixels)
    int    maxX;                    // right bound of valid detection zone (pixels)
    int    minY;                    // top bound of valid detection zone (pixels)
    int    maxY;                    // bottom bound of valid detection zone (pixels)

    // ── Internal motion constants ─────────────────────────────────────────────
    double launchVoltage;           // initial kick voltage to overcome static friction (V)
    double accelFactor;             // traction control voltage ramp multiplier per tick
    double slipThreshold;           // motor vs encoder RPM difference before traction cuts in
    double decelStepPercent;        // ABS voltage reduction per step during decel
    double lockThreshold;           // encoder/motor RPM ratio that declares wheel lockup
};


// ══════════════════════════════════════════════════════════════════════════════
// DEFAULT PROFILE INSTANCES
//
// Defined in motion_config.cpp. Import via extern — do not redefine.
// These are the calibrated field defaults. Copy before modifying.
// ══════════════════════════════════════════════════════════════════════════════
extern const StraightProfile DEFAULT_STRAIGHT;   // general forward drive
extern const StraightProfile BACKWARD_STRAIGHT;  // backward drive (softer gains)
extern const TurnProfile     DEFAULT_TURN;        // point turn (both sides drive)
extern const TurnProfile     DEFAULT_PIVOT;       // pivot turn (one side brakes)
extern const VisionProfile   DEFAULT_VISION;      // vision-guided approach

#endif // MOTION_CONFIG_H
