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
    // Weight applied to heading correction output at each drive phase.
    // Lower during decel and approach to avoid overcorrection at low speed.
    .accelHeadingScaling    = 0.10,
    .decelHeadingScaling    = 0.05,
    .approachHeadingScaling = 0.05,

    // ── Heading lock ──────────────────────────────────────────────────────────
    // Freezes heading correction below this distance — prevents oscillation
    // on the final approach when the robot is nearly on target.
    .headingLockDistance    = 8.0,    // cm

    // ── Internal motion constants ─────────────────────────────────────────────
    .launchVoltage          = 6.0,    // V — initial kick before traction ramp takes over
    .accelFactor            = 1.2,    // traction ramp multiplier per control cycle
    .slipThreshold          = 20.0,   // RPM delta above which traction control intervenes
    .decelStepPercent       = 0.45,   // % of absoluteMaxVoltage removed per ABS step
    .lockThreshold          = 0.25,   // lockup ratio above which ABS switches to coast
    .maxCurrentA            = 12.0,   // amps — trip if sustained above this
    .overcurrentDurationMs  = 500,    // ms current must stay high before circuit trips
};

// ──────────────────────────────────────────────────────────────────────────────
// BACKWARD_STRAIGHT — general reverse drive
// Higher heading gain and lower max speed for reverse stability.
// Used by: driveBackward, backwardToPoint
// ──────────────────────────────────────────────────────────────────────────────
const StraightProfile BACKWARD_STRAIGHT = {
    // ── Motion shape ──────────────────────────────────────────────────────────
    .breakDistance          = 30.0,   // % of leg distance
    .minSpeed               = 16.0,   // %
    .maxSpeed               = 80.0,   // % — derated; reverse is less stable at full power
    .distanceTolerance      = 2.0,    // cm
    .timeout                = 3.0,    // seconds
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
    .headingLockDistance    = 8.0,    // cm

    // ── Internal motion constants ─────────────────────────────────────────────
    .launchVoltage          = 6.0,    // V
    .accelFactor            = 1.2,
    .slipThreshold          = 20.0,   // RPM delta
    .decelStepPercent       = 0.45,   // % of absoluteMaxVoltage per ABS step
    .lockThreshold          = 0.25,   // lockup ratio
    .maxCurrentA            = 12.0,   // amps
    .overcurrentDurationMs  = 500,    // ms
};

// ──────────────────────────────────────────────────────────────────────────────
// LOADED_MID_FWD_80 — 24" bot carrying game objects, forward, 80% speed
//
// Tuned for a heavier-than-normal robot: the long break distance gives extra
// runway to shed speed gracefully under load, coast brake prevents the motor
// from fighting inertia at the stop point, and the reduced current trip threshold
// avoids nuisance trips when the loaded drivetrain works harder than usual.
//
// slipThreshold, decelStepPercent, and lockThreshold are field-tuned values
// specific to the loaded configuration — do not copy them to other profiles
// without re-validating against DEFAULT_STRAIGHT's unit conventions.
//
// Used by: driveForward, forwardToPoint
// ──────────────────────────────────────────────────────────────────────────────
const StraightProfile LOADED_MID_FWD_80 = {
    // ── Motion shape ──────────────────────────────────────────────────────────
    .breakDistance          = 30.0,   // % of leg distance — was 85cm fixed; now scales with leg length
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
    .breakDistance    = 30.0,   // % of total turn angle — 30% means a 90° turn breaks at 27°
    .minSpeed         = 15.0,   // %
    .maxSpeed         = 60.0,   // %
    .exitTolerance    = 2.0,    // degrees
    .timeout          = 5.0,    // seconds

    // ── Internal motion constants ─────────────────────────────────────────────
    // Traction control and ABS disabled for turns — not appropriate for rotational
    // motion on carpet. Values set to never-trigger thresholds per profile convention.
    .accelFactor      = 1.0,    // no ramp — hold voltage as-is
    .slipThreshold    = 1.0,    // slip ratio max is 1.0 — never triggers
    .decelStepPercent = 20.0,   // irrelevant — ABS never fires (lockThreshold = 1.0)
    .lockThreshold    = 1.0,    // lockup ratio max is 1.0 — never triggers
    .maxCurrentA      = 12.0,   // amps
    .overcurrentDurationMs = 500, // ms
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
    .exitTolerance    = 2.0,    // degrees — wider than point turn; pivot dynamics are less precise
    .timeout          = 3.0,    // seconds

    // ── Internal motion constants ─────────────────────────────────────────────
    // Traction and ABS are not applied to pivot turns — the braked side handles
    // deceleration passively. Fields set to 0 to make the intent explicit.
    .accelFactor      = 0.0,
    .slipThreshold    = 0.0,
    .decelStepPercent = 0.0,
    .lockThreshold    = 0.0,
    .maxCurrentA      = 12.0,   // amps
    .overcurrentDurationMs = 500, // ms
};

// ──────────────────────────────────────────────────────────────────────────────
// DEFAULT_VISION — vision-guided approach
// Used by: visionForwardToPoint, visionBackwardToPoint,
//          visionDriveForward, visionDriveBackward, visionOnly
//
// Motion params populated from LOADED_MID_FWD_80 as the starting baseline.
// Vision-specific overrides:
//   kp_heading     = 0.1   — low gain; vision correction is noisy, aggressive
//                            kp causes oscillation when detection jitters
//   brakeMode      = COAST — vision approach releases cleanly at pixel target
//   headingLockDistance = 15.0 — wider than straight profiles; vision target
//                            may shift as robot closes in, freeze too early
//                            and the final approach misses
// ──────────────────────────────────────────────────────────────────────────────
const VisionProfile DEFAULT_VISION = {
    .drive = {
    .breakDistance          = 30.0,   // % of leg distance
        .minSpeed               = 13.0,
        .maxSpeed               = 80.0,
        .distanceTolerance      = 1.0,
        .timeout                = 5.0,
        .brakeMode              = pros::E_MOTOR_BRAKE_COAST,  // override

        .kp_heading             = 0.1,    // override — vision-appropriate low gain
        .ki_heading             = 0.0,
        .kd_heading             = 0.0,

        .accelHeadingScaling    = 0.2,
        .decelHeadingScaling    = 0.1,
        .approachHeadingScaling = 0.1,

        .headingLockDistance    = 15.0,   // override — wider for vision approach

        .launchVoltage          = 6.0,
        .accelFactor            = 1.2,
        .slipThreshold          = 0.3,
        .decelStepPercent       = 2.0,
        .lockThreshold          = 0.3,
        .maxCurrentA            = 4.0,
        .overcurrentDurationMs  = 500,
    },

    // ── Vision heading fusion ─────────────────────────────────────────────────
    .kp_distToHeadScaling   = 0.3,

    // ── Vision object filter ──────────────────────────────────────────────────
    .minObjectWidth         = 10,
    .minX                   = 0,
    .maxX                   = 320,
    .minY                   = 0,
    .maxY                   = 240,
};