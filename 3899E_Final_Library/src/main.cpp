#include "main.h"
#include "robot_config.h"
#include "driver.h"
#include "auton.h"
#include "utils.h"
#include "navigation.h"
#include "autontasks.h"
#include "odometry.h"
#include "ai.h"

void initialize()
{
    pros::screen::print(pros::E_TEXT_MEDIUM, 0, "GPS diag start");
    pros::screen::print(pros::E_TEXT_MEDIUM, 1, "GPS err:%.3fm", gpsSensor.get_error());
    pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Raw X:%.3f Y:%.3fm",
        gpsSensor.get_position().x, gpsSensor.get_position().y);

    passiveEncoderLeft.set_reversed(true);
    passiveEncoderRight.set_reversed(false);
    passiveEncoderX.set_reversed(false);

    robotInit();
    pros::lcd::initialize();

    // Extend front hood immediately at power-on.
    frontHoodPneumatics.set_value(true);

    // ── Persistent background tasks — survive all period transitions ──────────
    // Odometry: runs at 100Hz, updates globalX/Y/heading continuously.
    // setStartPosition() must still be called in each routine to set origin.
    startOdometryTask();

    // Colour sort — NOT launched here. Must start after setAllianceRed() is called
    // in each auton routine so the flipper knows which colour to reject.
    // Launched in blueRightIsolation / redRightIsolation etc.

    // Position display: live X/Y/heading on brain screen line 0.
    pros::Task([]{
        while (true) {
            pros::screen::print(pros::E_TEXT_MEDIUM, 0,
                "X:%.1f Y:%.1f H:%.1f",
                globalX, globalY, getContinuousStandardHeading());
            pros::delay(100);
        }
    }, TASK_PRIORITY_MIN, TASK_STACK_DEPTH_DEFAULT, "PosDisplay");
    // ─────────────────────────────────────────────────────────────────────────

    // GPS live display — disabled for now
    // pros::screen::erase();
    // while (!Controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
    //     pros::screen::print(pros::E_TEXT_MEDIUM, 1, "GPS err:%.3fm", gpsSensor.get_error());
    //     pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Raw X:%.3f Y:%.3fm",
    //         gpsSensor.get_position().x, gpsSensor.get_position().y);
    //     requestGpsReset();
    //     pros::delay(200);
    //     pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Reset: %s  X:%.1f Y:%.1fcm",
    //         gpsResetSucceeded.load() ? "OK" : "FAIL", globalX, globalY);
    //     pros::delay(100);
    // }
}

void disabled() {}

void competition_initialize() {}

// ── VAIRC match state ─────────────────────────────────────────────────────────
// autonomous() is called TWICE by VAIRC field control:
//   First call  = Isolation Period  (15 seconds)
//   Second call = Interaction Period (105 seconds)
// Static flag persists between the two calls.
// NOTE: restart the Brain if a match is reset mid-match.
static bool firstAutoFlag = true;

void autonomous()
{
    if (firstAutoFlag) {
        // ══ ISOLATION PERIOD (first 15s) ══════════════════════════════════════
        firstAutoFlag = false;  // flip flag — next call = Interaction

        // Uncomment ONE routine:
        // ── Competition ───────────────────────────────────────────────────────
        // blueRightIsolation();
        // blueLeftIsolation();
        redRightIsolation();
        // redLeftIsolation();
        // ── Test / Dev ────────────────────────────────────────────────────────
        //coordinateFinder();
        // visionTest();
        // navTest();
        // routeTest();
        // fieldTargetsTest();
        // rightSideAuton();
        // setAllianceRed(true); sweepAndScore();
        // setAllianceRed(true); longGoalAuto15s(true);
        // runAIMatchRoute();
    //    testSweepIntake();
        // ─────────────────────────────────────────────────────────────────────

    } else {
        // ══ INTERACTION PERIOD (next 1m45s) ═══════════════════════════════════
        // Uncomment ONE routine (competition only — test slots leave this blank):
        // blueRightInteraction();
        // blueLeftInteraction();
        redRightInteraction();
        // redLeftInteraction();
    }

    while (true) pros::delay(100);
}

void opcontrol()
{
    pros::screen::erase();
    Controller.clear();
  //  driverControl();
}