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
// static flags persist between the two calls.
// NOTE: restart the Brain if a match is reset mid-match.
static bool isIsolationPeriod = true;
static bool tasksStarted      = false;

void autonomous()
{
    // ── Background tasks — launch ONCE, persist through both periods ──────────
    if (!tasksStarted) {
        tasksStarted = true;

        static ColorTaskParams sortParams;
        sortParams.isRunning = true;
        sortParams.delayMs   = 0;
        pros::Task(colorDetectionTask, &sortParams, "colourSort");

        pros::Task([]{
            while (true) {
                pros::screen::print(pros::E_TEXT_MEDIUM, 0,
                    "X:%.1f Y:%.1f H:%.1f",
                    globalX, globalY, getContinuousStandardHeading());
                pros::delay(100);
            }
        }, TASK_PRIORITY_MIN, TASK_STACK_DEPTH_DEFAULT, "PosDisplay");
    }
    // ─────────────────────────────────────────────────────────────────────────

    if (isIsolationPeriod) {
        // ══ ISOLATION PERIOD (first 15s) ══════════════════════════════════════
        isIsolationPeriod = false;  // flip flag — next call = Interaction

        // Uncomment ONE routine:
        // ── Competition ───────────────────────────────────────────────────────
        // blueRightIsolation();
        // blueLeftIsolation();
        // redRightIsolation();
        // redLeftIsolation();
        // ── Test / Dev ────────────────────────────────────────────────────────
        //visionTest();
        // navTest();
        // routeTest();
        // fieldTargetsTest();
        // rightSideAuton();
        //setAllianceRed(true); sweepAndScore();
        // setAllianceRed(true); longGoalAuto15s(true);
        // runAIMatchRoute();
        // ─────────────────────────────────────────────────────────────────────

    } else {
        // ══ INTERACTION PERIOD (next 1m45s) ═══════════════════════════════════
        // Uncomment ONE routine (competition only — test slots leave this blank):
        // blueRightInteraction();
        // blueLeftInteraction();
        // redRightInteraction();
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
