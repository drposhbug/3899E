#include "main.h"
#include "robot_config.h"
#include "driver.h"
#include "auton.h"
#include "utils.h"
#include "navigation.h"
#include "autontasks.h"

// ── initialize ────────────────────────────────────────────────────────────────
// Runs once on power-on, before any competition mode begins.
// Sets encoder directions, calibrates the IMU, and registers vision signatures.
void initialize()
{
    // Tracking wheel direction — left and X encoders are physically reversed.
    passiveEncoderLeft.set_reversed(true);
    passiveEncoderRight.set_reversed(false);
    passiveEncoderX.set_reversed(true);

    // Full hardware init: IMU calibration, vision sig registration, encoder zero.
    robotInit();

    // If no field controller is connected (practice / home use), go straight to
    // driver control. At competition the field controller takes over automatically
    // and calls autonomous() or opcontrol() at the correct time — no code change needed.
    if (!pros::competition::is_connected()) {
        opcontrol();
    }
}

// ── disabled ──────────────────────────────────────────────────────────────────
// Called whenever the robot is disabled by the field controller.
// Motors are cut automatically by PROS; add any safe-state logic here if needed.
void disabled() {}

// ── competition_initialize ────────────────────────────────────────────────────
// Called after initialize() during the pre-autonomous phase of a competition.
// Use for auton selector UI or any last-minute pre-match setup.
void competition_initialize() {}

// ── autonomous ────────────────────────────────────────────────────────────────
// Called when the autonomous period begins (field controller signal).
void autonomous()
{
    pros::screen::erase();
    // wingPneumatics.set_value(false);  // ensure wings are retracted at auton start
    // test();
}

// ── opcontrol ─────────────────────────────────────────────────────────────────
// Called when the driver control period begins.
void opcontrol()
{
    // pros::screen::erase();
    Controller.clear();  // clear any residual controller display from init
    driverControl();
}