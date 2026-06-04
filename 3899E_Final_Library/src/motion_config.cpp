#include "motion_config.h"
#include "main.h"

// ══════════════════════════════════════════════════════════════════════════════
// DEFAULT MOTION PROFILES
//
// These are the calibrated defaults for each motion family.
// Tune here — every wrapper and top-level function picks them up automatically.
//
// To deviate from defaults at a call site:
//   StraightProfile fast = DEFAULT_STRAIGHT;
//   fast.maxSpeed = 60.0;
//   driveForward(60.0, fast);
// ══════════════════════════════════════════════════════════════════════════════


// ──────────────────────────────────────────────────────────────────────────────
// DEFAULT_STRAIGHT — general forward drive
// Used by: driveForward, driveBackward, forwardToPoint, backwardToPoint, straightDistance
// ──────────────────────────────────────────────────────────────────────────────
const StraightProfile DEFAULT_STRAIGHT = {
    // ── Motion shape ──────────────────────────────────────────────────────────
    .breakDistance          = 35.0,                     // cm
    .minSpeed               = 16.0,                     // %
    .maxSpeed               = 100.0,                    // %
    .distanceTolerance      = 2.0,                      // cm
    .timeout                = 3.0,                      // seconds
    .brakeMode              = pros::E_MOTOR_BRAKE_BRAKE,

    // ── Heading PID ───────────────────────────────────────────────────────────
    .kp_heading             = 0.615,
    .ki_heading             = 0.0,
    .kd_heading             = 0.0,

    // ── Phase heading scaling ─────────────────────────────────────────────────
    .accelHeadingScaling    = 0.10,
    .decelHeadingScaling    = 0.05,
    .approachHeadingScaling = 0.05,

    // ── forwardToPoint / backwardToPoint heading lock ─────────────────────────────────────────────
    .headingLockDistance    = 8.0,                      // cm

    // ── Internal motion constants ─────────────────────────────────────────────
    .launchVoltage          = 6.0,                      // V
    .accelFactor            = 1.2,
    .slipThreshold          = 20.0,
    .decelStepPercent       = 0.45,
    .lockThreshold          = 0.25,
    .maxCurrentA            = 12.0,   // amps — trip if sustained above this
    .overcurrentDurationMs   = 500,    // ms current must stay high before tripping
};

// ──────────────────────────────────────────────────────────────────────────────
// BACKWARD_STRAIGHT — backward drive
// Softer heading gains and lower max speed for reverse stability.
// Used by: driveBackward, backwardToPoint
// ──────────────────────────────────────────────────────────────────────────────
const StraightProfile BACKWARD_STRAIGHT = {
    // ── Motion shape ──────────────────────────────────────────────────────────
    .breakDistance          = 30.0,                     // cm
    .minSpeed               = 16.0,                     // %
    .maxSpeed               = 80.0,                     // %
    .distanceTolerance      = 2.0,                      // cm
    .timeout                = 3.0,                      // seconds
    .brakeMode              = pros::E_MOTOR_BRAKE_BRAKE,

    // ── Heading PID ───────────────────────────────────────────────────────────
    .kp_heading             = 0.8,
    .ki_heading             = 0.0,
    .kd_heading             = 0.0,

    // ── Phase heading scaling ─────────────────────────────────────────────────
    .accelHeadingScaling    = 0.08,
    .decelHeadingScaling    = 0.06,
    .approachHeadingScaling = 0.06,

    // ── forwardToPoint / backwardToPoint heading lock ─────────────────────────────────────────────
    .headingLockDistance    = 8.0,                      // cm

    // ── Internal motion constants ─────────────────────────────────────────────
    .launchVoltage          = 6.0,                      // V
    .accelFactor            = 1.2,
    .slipThreshold          = 20.0,
    .decelStepPercent       = 0.45,
    .lockThreshold          = 0.25,
    .maxCurrentA            = 12.0,   // amps
    .overcurrentDurationMs   = 500,    // ms
};

// ──────────────────────────────────────────────────────────────────────────────
// DEFAULT_TURN — point turn (both sides drive, opposite direction)
// Used by: turnOdometry, turnLeft, turnRight, turnToPoint,
//          turnLeftToPoint, turnRightToPoint
// ──────────────────────────────────────────────────────────────────────────────
const TurnProfile DEFAULT_TURN = {
    // ── Motion shape ──────────────────────────────────────────────────────────
    .breakDistance    = 5.0,    // degrees
    .minSpeed         = 25.0,   // %
    .maxSpeed         = 100.0,  // %
    .exitTolerance    = 0.5,    // degrees — tight; 16° default in turnOdometry was a bug
    .timeout          = 3.0,    // seconds

    // ── Internal motion constants ─────────────────────────────────────────────
    // Deliberately different from straight — turns require faster traction response.
    .accelFactor      = 1.5,
    .slipThreshold    = 10.0,
    .decelStepPercent = 20.0,
    .lockThreshold    = 10.0,
    .maxCurrentA            = 12.0,   // amps
    .overcurrentDurationMs   = 500,    // ms
};

// ──────────────────────────────────────────────────────────────────────────────
// DEFAULT_PIVOT — pivot turn (one side brakes, one side drives)
// Used by: pivotTurnOdometryV2, pivotLeftMP, pivotRightMP
// ──────────────────────────────────────────────────────────────────────────────
const TurnProfile DEFAULT_PIVOT = {
    // ── Motion shape ──────────────────────────────────────────────────────────
    .breakDistance    = 5.0,    // degrees
    .minSpeed         = 20.0,   // %
    .maxSpeed         = 100.0,  // %
    .exitTolerance    = 2.0,    // degrees — wider tolerance suits pivot dynamics
    .timeout          = 3.0,    // seconds

    // ── Internal motion constants ─────────────────────────────────────────────
    // Pivot turns have no traction/ABS system — these fields are unused.
    // Set to 0 to make clear they are not applied.
    .accelFactor      = 0.0,
    .slipThreshold    = 0.0,
    .decelStepPercent = 0.0,
    .lockThreshold    = 0.0,
    .maxCurrentA            = 12.0,   // amps
    .overcurrentDurationMs   = 500,    // ms
};

// ──────────────────────────────────────────────────────────────────────────────
// DEFAULT_VISION — vision-guided approach
// Used by: moveVisionOdometry, moveVisionOdometryOpen, visionOnly
// ──────────────────────────────────────────────────────────────────────────────
const VisionProfile DEFAULT_VISION = {
    // ── Motion shape ──────────────────────────────────────────────────────────
    .breakDistance          = 30.0,                      // cm
    .minSpeed               = 16.0,                      // %
    .maxSpeed               = 100.0,                     // %
    .distanceTolerance      = 2.0,                       // cm
    .timeout                = 3.0,                       // seconds
    .brakeMode              = pros::E_MOTOR_BRAKE_COAST,

    // ── Heading PID ───────────────────────────────────────────────────────────
    .kp_head                = 0.1,
    .ki_head                = 0.0,
    .kd_head                = 0.0,

    // ── Vision heading fusion ─────────────────────────────────────────────────
    .kp_distToHeadScaling   = 0.3,

    // ── Phase heading scaling ─────────────────────────────────────────────────
    .accelHeadingScaling    = 0.2,
    .decelHeadingScaling    = 0.2,
    .approachHeadingScaling = 0.2,

    // ── Heading lock ──────────────────────────────────────────────────────────
    .headingLockDistance    = 15.0,                      // cm

    // ── Vision object filter ──────────────────────────────────────────────────
    .minObjectWidth         = 10,
    .minX                   = 0,
    .maxX                   = 320,
    .minY                   = 0,
    .maxY                   = 240,

    // ── Internal motion constants ─────────────────────────────────────────────
    .launchVoltage          = 6.0,                       // V
    .accelFactor            = 1.2,
    .slipThreshold          = 20.0,
    .decelStepPercent       = 0.45,
    .lockThreshold          = 0.25,
    .maxCurrentA            = 12.0,   // amps
    .overcurrentDurationMs   = 500,    // ms
};