#include "motion_config.h"
#include "main.h"

// ══════════════════════════════════════════════════════════════════════════════
// MOTION PROFILES
//
// Calibrated defaults for each motion family. Tune here — every call site
// picks them up automatically.
//
// To deviate from a default at a call site:
//   StraightProfile fast = DEFAULT_STRAIGHT;
//   fast.maxSpeed = 60.0;
//   driveForward(60.0, fast);
// ══════════════════════════════════════════════════════════════════════════════


// ──────────────────────────────────────────────────────────────────────────────
// DEFAULT_STRAIGHT — general forward drive
// Used by: driveForward, forwardToPoint, straightDistance
// ──────────────────────────────────────────────────────────────────────────────
const StraightProfile DEFAULT_STRAIGHT = {
    // ── Motion shape ──────────────────────────────────────────────────────────
    .breakDistance          = 30.0,   // % of leg distance — 30% means a 150cm leg breaks at 45cm
    .minSpeed               = 16.0,   // % — floor speed during approach phase
    .maxSpeed               = 100.0,  // % — peak cruise speed
    .distanceTolerance      = 2.0,    // cm — exit bubble radius
    .timeout                = 3.0,    // seconds
    .brakeMode              = pros::E_MOTOR_BRAKE_BRAKE,

    // ── Heading PID ───────────────────────────────────────────────────────────
    .kp_heading             = 0.615,
    .ki_heading             = 0.0,
    .kd_heading             = 0.0,

    // ── Phase heading scaling ─────────────────────────────────────────────────
    .accelHeadingScaling    = 0.10,
    .decelHeadingScaling    = 0.05,
    .approachHeadingScaling = 0.05,

    // ── Heading lock ──────────────────────────────────────────────────────────
    .headingLockDistance    = 8.0,    // cm

    // ── Internal motion constants ─────────────────────────────────────────────
    .launchVoltage          = 6.0,
    .accelFactor            = 1.2,
    .slipThreshold          = 20.0,
    .decelStepPercent       = 0.45,
    .lockThreshold          = 0.25,
    .maxCurrentA            = 12.0,
    .overcurrentDurationMs  = 500,
};

// ──────────────────────────────────────────────────────────────────────────────
// BACKWARD_STRAIGHT — general reverse drive
// Higher heading gain and lower max speed for reverse stability.
// Used by: driveBackward, backwardToPoint
// ──────────────────────────────────────────────────────────────────────────────
const StraightProfile BACKWARD_STRAIGHT = {
    // ── Motion shape ──────────────────────────────────────────────────────────
    .breakDistance          = 30.0,
    .minSpeed               = 16.0,
    .maxSpeed               = 80.0,   // derated — reverse is less stable at full power
    .distanceTolerance      = 2.0,
    .timeout                = 3.0,
    .brakeMode              = pros::E_MOTOR_BRAKE_BRAKE,

    // ── Heading PID ───────────────────────────────────────────────────────────
    // Higher kp than DEFAULT_STRAIGHT — rear-wheel lead amplifies yaw, needs faster correction.
    .kp_heading             = 0.8,
    .ki_heading             = 0.0,
    .kd_heading             = 0.0,

    // ── Phase heading scaling ─────────────────────────────────────────────────
    .accelHeadingScaling    = 0.08,
    .decelHeadingScaling    = 0.06,
    .approachHeadingScaling = 0.06,

    // ── Heading lock ──────────────────────────────────────────────────────────
    .headingLockDistance    = 8.0,

    // ── Internal motion constants ─────────────────────────────────────────────
    .launchVoltage          = 6.0,
    .accelFactor            = 1.2,
    .slipThreshold          = 20.0,
    .decelStepPercent       = 0.45,
    .lockThreshold          = 0.25,
    .maxCurrentA            = 12.0,
    .overcurrentDurationMs  = 500,
};

// ──────────────────────────────────────────────────────────────────────────────
// LOADED_MID_FWD_80 — 24" bot carrying game objects, forward, 80% speed
//
// Tuned for a heavier-than-normal robot: the long break distance gives extra
// runway to shed speed gracefully under load, coast brake prevents the motor
// from fighting inertia at the stop point, and the reduced current trip threshold
// avoids nuisance trips when the loaded drivetrain works harder than usual.
//
// Used by: driveForward, forwardToPoint
// ──────────────────────────────────────────────────────────────────────────────
const StraightProfile LOADED_MID_FWD_80 = {
    // ── Motion shape ──────────────────────────────────────────────────────────
    .breakDistance          = 30.0,
    .minSpeed               = 13.0,
    .maxSpeed               = 80.0,
    .distanceTolerance      = 1.0,
    .timeout                = 5.0,
    .brakeMode              = pros::E_MOTOR_BRAKE_BRAKE,

    // ── Heading PID ───────────────────────────────────────────────────────────
    .kp_heading             = 1.0,
    .ki_heading             = 0.0,
    .kd_heading             = 0.0,

    // ── Phase heading scaling ─────────────────────────────────────────────────
    .accelHeadingScaling    = 0.2,
    .decelHeadingScaling    = 0.1,
    .approachHeadingScaling = 0.1,

    // ── Heading lock ──────────────────────────────────────────────────────────
    .headingLockDistance    = 3.0,

    // ── Internal motion constants ─────────────────────────────────────────────
    .launchVoltage          = 6.0,
    .accelFactor            = 1.2,
    .slipThreshold          = 0.3,
    .decelStepPercent       = 2.0,
    .lockThreshold          = 0.3,
    .maxCurrentA            = 8.0,
    .overcurrentDurationMs  = 500,
};

// ──────────────────────────────────────────────────────────────────────────────
// DEFAULT_TURN — point turn (both sides drive in opposite directions)
// Used by: turnOdometry, turnLeft, turnRight, turnToPoint,
//          turnLeftToPoint, turnRightToPoint
// ──────────────────────────────────────────────────────────────────────────────
const TurnProfile DEFAULT_TURN = {
    // ── Motion shape ──────────────────────────────────────────────────────────
    .breakDistance    = 30.0,   // % of total turn angle
    .minSpeed         = 15.0,
    .maxSpeed         = 60.0,
    .exitTolerance    = 2.0,    // degrees
    .timeout          = 5.0,

    // ── Internal motion constants ─────────────────────────────────────────────
    // Traction control and ABS disabled — not appropriate for rotational motion.
    .accelFactor      = 1.0,
    .slipThreshold    = 1.0,    // never triggers
    .decelStepPercent = 20.0,   // irrelevant — ABS never fires
    .lockThreshold    = 1.0,    // never triggers
    .maxCurrentA      = 12.0,
    .overcurrentDurationMs = 500,
};

// ──────────────────────────────────────────────────────────────────────────────
// DEFAULT_PIVOT — pivot turn (one side brakes, one side drives)
// Used by: pivotTurnOdometryV2, pivotLeftMP, pivotRightMP
// ──────────────────────────────────────────────────────────────────────────────
const TurnProfile DEFAULT_PIVOT = {
    // ── Motion shape ──────────────────────────────────────────────────────────
    .breakDistance    = 5.0,
    .minSpeed         = 20.0,
    .maxSpeed         = 100.0,
    .exitTolerance    = 2.0,
    .timeout          = 3.0,

    // ── Internal motion constants ─────────────────────────────────────────────
    // Traction and ABS not applied to pivot turns.
    .accelFactor      = 0.0,
    .slipThreshold    = 0.0,
    .decelStepPercent = 0.0,
    .lockThreshold    = 0.0,
    .maxCurrentA      = 12.0,
    .overcurrentDurationMs = 500,
};

// ──────────────────────────────────────────────────────────────────────────────
// DEFAULT_VISION — vision-guided approach
// Used by: visionForwardToPoint, visionBackwardToPoint,
//          visionDriveForward, visionDriveBackward, visionOnly
// ──────────────────────────────────────────────────────────────────────────────
const VisionProfile DEFAULT_VISION = {
    .drive = {
        .breakDistance          = 30.0,
        .minSpeed               = 13.0,
        .maxSpeed               = 80.0,
        .distanceTolerance      = 1.0,
        .timeout                = 5.0,
        .brakeMode              = pros::E_MOTOR_BRAKE_COAST,

        .kp_heading             = 0.1,    // low gain — vision correction is noisy
        .ki_heading             = 0.0,
        .kd_heading             = 0.0,

        .accelHeadingScaling    = 0.2,
        .decelHeadingScaling    = 0.1,
        .approachHeadingScaling = 0.1,

        .headingLockDistance    = 15.0,   // wider — vision target may shift on approach

        .launchVoltage          = 6.0,
        .accelFactor            = 1.2,
        .slipThreshold          = 0.3,
        .decelStepPercent       = 2.0,
        .lockThreshold          = 0.3,
        .maxCurrentA            = 4.0,
        .overcurrentDurationMs  = 500,
    },

    .kp_distToHeadScaling   = 0.3,

    .minObjectWidth         = 10,
    .minX                   = 0,
    .maxX                   = 320,
    .minY                   = 0,
    .maxY                   = 240,
};


// ══════════════════════════════════════════════════════════════════════════════
// TIERED NAVIGATION PROFILES
//
// Selected automatically per waypoint segment by selectFwdProfile /
// selectBwdProfile / selectTurnProfile based on measured distance or angle.
//
// Seeding — initial values, tune per tier independently:
//   SHORT_FWD  — seeded from field-tested navigateTo inline values (short hops)
//   MID_FWD    — seeded from LOADED_MID_FWD_80
//   LONG_FWD   — seeded from LOADED_MID_FWD_80
//   *_BWD      — copies of matching _FWD tier; tune independently when needed
//   *_TURN     — copies of DEFAULT_TURN for all tiers
// ══════════════════════════════════════════════════════════════════════════════

// ──────────────────────────────────────────────────────────────────────────────
// SHORT_FWD — forward, segment <= STRAIGHT_SHORT_MAX_CM (50 cm)
// Seeded from field-tested navigateTo short-hop values.
// ──────────────────────────────────────────────────────────────────────────────
const StraightProfile SHORT_FWD = {
    .breakDistance          = 30.0,
    .minSpeed               = 13.0,
    .maxSpeed               = 80.0,
    .distanceTolerance      = 1.0,
    .timeout                = 8.0,
    .brakeMode              = pros::E_MOTOR_BRAKE_BRAKE,

    .kp_heading             = 0.4,
    .ki_heading             = 0.01,
    .kd_heading             = 0.05,

    .accelHeadingScaling    = 0.2,
    .decelHeadingScaling    = 0.2,
    .approachHeadingScaling = 0.2,

    .headingLockDistance    = 8.0,

    .launchVoltage          = 6.0,
    .accelFactor            = 1.2,
    .slipThreshold          = 0.3,
    .decelStepPercent       = 2.0,
    .lockThreshold          = 0.3,
    .maxCurrentA            = 8.0,
    .overcurrentDurationMs  = 500,
};

// ──────────────────────────────────────────────────────────────────────────────
// MID_FWD — forward, STRAIGHT_SHORT_MAX_CM < segment <= STRAIGHT_MID_MAX_CM
//           (50–80 cm)
// Seeded from LOADED_MID_FWD_80.
// ──────────────────────────────────────────────────────────────────────────────
const StraightProfile MID_FWD = {
    .breakDistance          = 30.0,
    .minSpeed               = 13.0,
    .maxSpeed               = 80.0,
    .distanceTolerance      = 1.0,
    .timeout                = 8.0,
    .brakeMode              = pros::E_MOTOR_BRAKE_BRAKE,

    .kp_heading             = 1.0,
    .ki_heading             = 0.0,
    .kd_heading             = 0.0,

    .accelHeadingScaling    = 0.2,
    .decelHeadingScaling    = 0.1,
    .approachHeadingScaling = 0.1,

    .headingLockDistance    = 3.0,

    .launchVoltage          = 6.0,
    .accelFactor            = 1.2,
    .slipThreshold          = 0.3,
    .decelStepPercent       = 2.0,
    .lockThreshold          = 0.3,
    .maxCurrentA            = 8.0,
    .overcurrentDurationMs  = 500,
};

// ──────────────────────────────────────────────────────────────────────────────
// LONG_FWD — forward, segment > STRAIGHT_MID_MAX_CM (80+ cm)
// Seeded from LOADED_MID_FWD_80.
// ──────────────────────────────────────────────────────────────────────────────
const StraightProfile LONG_FWD = {
    .breakDistance          = 30.0,
    .minSpeed               = 13.0,
    .maxSpeed               = 80.0,
    .distanceTolerance      = 1.0,
    .timeout                = 8.0,
    .brakeMode              = pros::E_MOTOR_BRAKE_BRAKE,

    .kp_heading             = 1.0,
    .ki_heading             = 0.0,
    .kd_heading             = 0.0,

    .accelHeadingScaling    = 0.2,
    .decelHeadingScaling    = 0.1,
    .approachHeadingScaling = 0.1,

    .headingLockDistance    = 3.0,

    .launchVoltage          = 6.0,
    .accelFactor            = 1.2,
    .slipThreshold          = 0.3,
    .decelStepPercent       = 2.0,
    .lockThreshold          = 0.3,
    .maxCurrentA            = 8.0,
    .overcurrentDurationMs  = 500,
};

// ──────────────────────────────────────────────────────────────────────────────
// SHORT_BWD — backward, segment <= STRAIGHT_SHORT_MAX_CM
// Seeded from SHORT_FWD — tune independently when needed.
// ──────────────────────────────────────────────────────────────────────────────
const StraightProfile SHORT_BWD = {
    .breakDistance          = 30.0,
    .minSpeed               = 13.0,
    .maxSpeed               = 80.0,
    .distanceTolerance      = 1.0,
    .timeout                = 8.0,
    .brakeMode              = pros::E_MOTOR_BRAKE_BRAKE,

    .kp_heading             = 0.4,
    .ki_heading             = 0.01,
    .kd_heading             = 0.05,

    .accelHeadingScaling    = 0.2,
    .decelHeadingScaling    = 0.2,
    .approachHeadingScaling = 0.2,

    .headingLockDistance    = 8.0,

    .launchVoltage          = 6.0,
    .accelFactor            = 1.2,
    .slipThreshold          = 0.3,
    .decelStepPercent       = 2.0,
    .lockThreshold          = 0.3,
    .maxCurrentA            = 8.0,
    .overcurrentDurationMs  = 500,
};

// ──────────────────────────────────────────────────────────────────────────────
// MID_BWD — backward, 50–80 cm
// Seeded from MID_FWD — tune independently when needed.
// ──────────────────────────────────────────────────────────────────────────────
const StraightProfile MID_BWD = {
    .breakDistance          = 30.0,
    .minSpeed               = 13.0,
    .maxSpeed               = 80.0,
    .distanceTolerance      = 1.0,
    .timeout                = 8.0,
    .brakeMode              = pros::E_MOTOR_BRAKE_BRAKE,

    .kp_heading             = 1.0,
    .ki_heading             = 0.0,
    .kd_heading             = 0.0,

    .accelHeadingScaling    = 0.2,
    .decelHeadingScaling    = 0.1,
    .approachHeadingScaling = 0.1,

    .headingLockDistance    = 3.0,

    .launchVoltage          = 6.0,
    .accelFactor            = 1.2,
    .slipThreshold          = 0.3,
    .decelStepPercent       = 2.0,
    .lockThreshold          = 0.3,
    .maxCurrentA            = 8.0,
    .overcurrentDurationMs  = 500,
};

// ──────────────────────────────────────────────────────────────────────────────
// LONG_BWD — backward, 80+ cm
// Seeded from LONG_FWD — tune independently when needed.
// ──────────────────────────────────────────────────────────────────────────────
const StraightProfile LONG_BWD = {
    .breakDistance          = 30.0,
    .minSpeed               = 13.0,
    .maxSpeed               = 80.0,
    .distanceTolerance      = 1.0,
    .timeout                = 8.0,
    .brakeMode              = pros::E_MOTOR_BRAKE_BRAKE,

    .kp_heading             = 1.0,
    .ki_heading             = 0.0,
    .kd_heading             = 0.0,

    .accelHeadingScaling    = 0.2,
    .decelHeadingScaling    = 0.1,
    .approachHeadingScaling = 0.1,

    .headingLockDistance    = 3.0,

    .launchVoltage          = 6.0,
    .accelFactor            = 1.2,
    .slipThreshold          = 0.3,
    .decelStepPercent       = 2.0,
    .lockThreshold          = 0.3,
    .maxCurrentA            = 8.0,
    .overcurrentDurationMs  = 500,
};

// ──────────────────────────────────────────────────────────────────────────────
// SHORT_TURN — point turn, <= TURN_SHORT_MAX_DEG (50°)
// Seeded from DEFAULT_TURN — tune independently when needed.
// ──────────────────────────────────────────────────────────────────────────────
const TurnProfile SHORT_TURN = {
    .breakDistance    = 30.0,
    .minSpeed         = 15.0,
    .maxSpeed         = 60.0,
    .exitTolerance    = 2.0,
    .timeout          = 5.0,

    .accelFactor      = 1.0,
    .slipThreshold    = 1.0,
    .decelStepPercent = 20.0,
    .lockThreshold    = 1.0,
    .maxCurrentA      = 12.0,
    .overcurrentDurationMs = 500,
};

// ──────────────────────────────────────────────────────────────────────────────
// MID_TURN — point turn, 50°–100°
// Seeded from DEFAULT_TURN — tune independently when needed.
// ──────────────────────────────────────────────────────────────────────────────
const TurnProfile MID_TURN = {
    .breakDistance    = 30.0,
    .minSpeed         = 15.0,
    .maxSpeed         = 60.0,
    .exitTolerance    = 2.0,
    .timeout          = 5.0,

    .accelFactor      = 1.0,
    .slipThreshold    = 1.0,
    .decelStepPercent = 20.0,
    .lockThreshold    = 1.0,
    .maxCurrentA      = 12.0,
    .overcurrentDurationMs = 500,
};

// ──────────────────────────────────────────────────────────────────────────────
// LONG_TURN — point turn, 100°+
// Seeded from DEFAULT_TURN — tune independently when needed.
// ──────────────────────────────────────────────────────────────────────────────
const TurnProfile LONG_TURN = {
    .breakDistance    = 30.0,
    .minSpeed         = 15.0,
    .maxSpeed         = 60.0,
    .exitTolerance    = 2.0,
    .timeout          = 5.0,

    .accelFactor      = 1.0,
    .slipThreshold    = 1.0,
    .decelStepPercent = 20.0,
    .lockThreshold    = 1.0,
    .maxCurrentA      = 12.0,
    .overcurrentDurationMs = 500,
};


// ══════════════════════════════════════════════════════════════════════════════
// PROFILE SELECTORS
//
// Each function reads the threshold constants from motion_config.h and returns
// a const reference to the appropriate named profile. No copies made.
// Change STRAIGHT_SHORT_MAX_CM / STRAIGHT_MID_MAX_CM / TURN_SHORT_MAX_DEG /
// TURN_MID_MAX_DEG in motion_config.h to reclassify segments globally.
// ══════════════════════════════════════════════════════════════════════════════

const StraightProfile& selectFwdProfile(double distCm) {
    if (distCm <= STRAIGHT_SHORT_MAX_CM) return SHORT_FWD;
    if (distCm <= STRAIGHT_MID_MAX_CM)   return MID_FWD;
    return LONG_FWD;
}

const StraightProfile& selectBwdProfile(double distCm) {
    if (distCm <= STRAIGHT_SHORT_MAX_CM) return SHORT_BWD;
    if (distCm <= STRAIGHT_MID_MAX_CM)   return MID_BWD;
    return LONG_BWD;
}

const TurnProfile& selectTurnProfile(double degrees) {
    if (degrees <= TURN_SHORT_MAX_DEG) return SHORT_TURN;
    if (degrees <= TURN_MID_MAX_DEG)   return MID_TURN;
    return LONG_TURN;
}
