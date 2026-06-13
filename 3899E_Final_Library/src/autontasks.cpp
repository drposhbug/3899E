#include "autontasks.h"
#include "robot_config.h"
#include "utils.h"
#include "odometry.h"
#include <atomic>
#include <cmath>

// ──────────────────────────────────────────────────────────────────────────────
// GLOBALS
// ──────────────────────────────────────────────────────────────────────────────

HeadingDisplayParams headingDisplayParams = {false};

// Written by motion functions; read by the display tasks.
double g_targetDistance = 0.0;  // current nav target distance (cm)
double g_targetHeading  = 0.0;  // current nav target heading (degrees)

// ──────────────────────────────────────────────────────────────────────────────
// SHARED ASYNC TASK STATE
// Each async group stores its timing/power params plus a running flag.
// Tasks are self-terminating via the flag — no handle storage needed.
// ──────────────────────────────────────────────────────────────────────────────
struct AsyncTaskParams {
    std::atomic<bool> running{false};
    double timeMs  = 0;
    double delayMs = 0;
    double power   = 100;
};

// ══════════════════════════════════════════════════════════════════════════════
// INTAKE HOPPER TASK  (intake motors)
// ══════════════════════════════════════════════════════════════════════════════
static AsyncTaskParams intakeHopperParams;

void intakeHopperTask(void*) {
    intakeHopperParams.running.store(true);

    // Honor optional start delay before doing anything
    if (intakeHopperParams.delayMs > 0) {
        pros::delay(static_cast<uint32_t>(intakeHopperParams.delayMs));
    }

    // Convert power % to millivolts (100 % ≈ 12 V, so /8.34 gives volts × 1000 = mV)
    int32_t  voltage   = static_cast<int32_t>((intakeHopperParams.power / 8.34) * 1000);
    uint32_t startTime = pros::millis();

    // Spin intake motors for the requested duration, or until stopped externally
    while (intakeHopperParams.running.load() &&
           pros::millis() - startTime < static_cast<uint32_t>(intakeHopperParams.timeMs)) {
        intakeMotor1.move_voltage(voltage);
        intakeMotor2.move_voltage(voltage);
        pros::delay(10);
    }

    intakeMotor1.move(0);
    intakeMotor2.move(0);
    intakeHopperParams.running.store(false);
}

void intakeHopperStart(double timeMs, double power, double delayMs, bool async) {
    // Stop any previous instance before starting a new one
    if (intakeHopperParams.running.load()) {
        intakeHopperParams.running.store(false);
        pros::delay(20);
    }
    intakeHopperParams.timeMs  = timeMs;
    intakeHopperParams.power   = power;
    intakeHopperParams.delayMs = delayMs;

    if (async) {
        pros::Task(intakeHopperTask, nullptr, "intakeHopper");  // launch background task; self-terminates via flag
    } else {
        intakeHopperTask(nullptr);     // blocking: run on the calling task
    }
}

void intakeHopperStop() {
    intakeHopperParams.running.store(false);
    intakeMotor1.move(0);
    intakeMotor2.move(0);
}

// ══════════════════════════════════════════════════════════════════════════════
// MATCHLOAD TASK  (intake motors)
// ══════════════════════════════════════════════════════════════════════════════
static AsyncTaskParams matchloadParams;

void matchloadTask(void*) {
    matchloadParams.running.store(true);

    // Honor optional start delay
    if (matchloadParams.delayMs > 0) {
        pros::delay(static_cast<uint32_t>(matchloadParams.delayMs));
    }

    int32_t voltage = static_cast<int32_t>((matchloadParams.power / 8.34) * 1000);

    intakeMotor1.move_voltage(voltage);
    intakeMotor2.move_voltage(voltage);

    uint32_t startTime = pros::millis();
    while (matchloadParams.running.load() &&
           pros::millis() - startTime < static_cast<uint32_t>(matchloadParams.timeMs)) {
        pros::delay(10);
        // Continuously command voltage to guard against the motor's internal timeout
        intakeMotor1.move_voltage(voltage);
        intakeMotor2.move_voltage(voltage);
    }

    intakeMotor1.move(0);
    intakeMotor2.move(0);

    matchloadParams.running.store(false);
}

void matchloadStart(double timeMs, double power, double delayMs, bool async) {
    if (matchloadParams.running.load()) {
        matchloadParams.running.store(false);
        pros::delay(20);
    }
    matchloadParams.timeMs  = timeMs;
    matchloadParams.power   = power;
    matchloadParams.delayMs = delayMs;

    if (async) {
        pros::Task(matchloadTask, nullptr, "matchload");
    } else {
        matchloadTask(nullptr);
    }
}

// ══════════════════════════════════════════════════════════════════════════════
// LEGACY BLOCKING INTAKE
// ══════════════════════════════════════════════════════════════════════════════

// Spin intake in reverse for 'time' ms.
void intake(double time, bool pistonState) {
    (void)pistonState;  // pistonState unused — front hood removed

    uint32_t startTime = pros::millis();
    while (pros::millis() - startTime < static_cast<uint32_t>(time)) {
        intakeMotor1.move_voltage(-12000);  // full reverse (12 V)
        intakeMotor2.move_voltage(-12000);
        pros::delay(10);
    }
    intakeMotor1.move(0);
    intakeMotor2.move(0);
}

// ══════════════════════════════════════════════════════════════════════════════
// ASYNC INTAKE TASK  (shared state + optional colour-sort)
// ══════════════════════════════════════════════════════════════════════════════
static std::atomic<bool> g_intakeTaskRunning(false);
static double g_intakeTimeMs      = 0;
static double g_intakePct         = 100;
static bool   g_intakePistonState = false;
static bool   g_matchLoadState    = false;

void intakeTaskEntry(void*) {
    g_intakeTaskRunning.store(true);

    uint32_t startTime = pros::millis();
    int32_t  voltage   = static_cast<int32_t>((g_intakePct / 8.34) * 1000);

    // Colour sort is handled by the background colorDetectionTask (sortMotor + opticalSensor).
    // intakeTaskEntry just runs the intake motors for the requested duration.
    while (g_intakeTaskRunning.load() &&
           pros::millis() - startTime < static_cast<uint32_t>(g_intakeTimeMs)) {
        intakeMotor1.move_voltage(voltage);
        intakeMotor2.move_voltage(voltage);
        pros::delay(10);
    }

    intakeMotor1.move(0);
    intakeMotor2.move(0);
    g_intakeTaskRunning.store(false);
}

void intakeStart(double timeMs, double intakePct, bool pistonState, bool matchLoad) {
    (void)pistonState;  // unused — front hood removed
    (void)matchLoad;    // unused — match load pneumatic removed

    // Stop any running intake task before launching a new one
    if (g_intakeTaskRunning.load()) {
        g_intakeTaskRunning.store(false);
        pros::delay(20);
    }
    g_intakeTimeMs   = timeMs;
    g_intakePct      = intakePct;
    pros::Task(intakeTaskEntry, nullptr, "intakeTask");
}

// Colour sort is now always-on via the background colorDetectionTask.
// intakeColourStart forwards to intakeStart — kept for auton call-site compatibility.
void intakeColourStart(double timeMs, double intakePct, bool pistonState, bool matchLoad) {
    intakeStart(timeMs, intakePct, pistonState, matchLoad);
}

// Signal the running intake task to stop early
void intakeStop() {
    g_intakeTaskRunning.store(false);
    pros::delay(20);
}

// ══════════════════════════════════════════════════════════════════════════════
// BLOCKING SCORING HELPERS
// Run intake motors for 'time' ms.
// ══════════════════════════════════════════════════════════════════════════════

void score(double time, double power) {
    uint32_t startTime = pros::millis();
    int32_t voltage = static_cast<int32_t>((power / 8.34) * 1000);

    while (pros::millis() - startTime < static_cast<uint32_t>(time)) {
        intakeMotor1.move_voltage(voltage);
        intakeMotor2.move_voltage(voltage);
        pros::delay(10);
    }

    intakeMotor1.move(0);
    intakeMotor2.move(0);
}

// Score through the left lane only (right gate blocks the right lane)
void leftScore(double time, double power) {
    uint32_t startTime = pros::millis();
    rightGatePneumatics.set_value(true);   // block right lane
    leftGatePneumatics.set_value(false);   // open left lane

    int32_t voltage = static_cast<int32_t>((power / 8.34) * 1000);

    while (pros::millis() - startTime < static_cast<uint32_t>(time)) {
        intakeMotor1.move_voltage(voltage);
        intakeMotor2.move_voltage(voltage);
        pros::delay(10);
    }

    leftGatePneumatics.set_value(true);   // close left lane when done
    intakeMotor1.move(0);
    intakeMotor2.move(0);
}

// Score through the right lane only (left gate blocks the left lane)
void rightScore(double time, double power) {
    uint32_t startTime = pros::millis();
    rightGatePneumatics.set_value(false);  // open right lane
    leftGatePneumatics.set_value(true);    // block left lane

    int32_t voltage = static_cast<int32_t>((power / 8.34) * 1000);

    while (pros::millis() - startTime < static_cast<uint32_t>(time)) {
        intakeMotor1.move_voltage(voltage);
        intakeMotor2.move_voltage(voltage);
        pros::delay(10);
    }

    rightGatePneumatics.set_value(true);  // close right lane when done
    intakeMotor1.move(0);
    intakeMotor2.move(0);
}

void stopScore() {
    intakeMotor1.move(0);
    intakeMotor2.move(0);
}

// ══════════════════════════════════════════════════════════════════════════════
// BLOCKING OUTTAKE
// ══════════════════════════════════════════════════════════════════════════════

void outtake(double time, double power) {
    uint32_t startTime = pros::millis();

    while (pros::millis() - startTime < static_cast<uint32_t>(time)) {
        int32_t voltage = static_cast<int32_t>((power / 8.34) * 1000);
        intakeMotor1.move_voltage(-voltage);  // negative = reverse
        intakeMotor2.move_voltage(-voltage);
        pros::delay(10);
    }

    intakeMotor1.move(0);
    intakeMotor2.move(0);
}

void stopOuttake() {
    intakeMotor1.move(0);
    intakeMotor2.move(0);
}

// ══════════════════════════════════════════════════════════════════════════════
// DISPLAY TASKS
// ══════════════════════════════════════════════════════════════════════════════

// Heading display: shows encoder distances, IMU heading, and nav targets on the
// driver controller.  Updates every 500 ms — anything faster risks VEXnet drops.
void headingDisplayTask(void* params) {
    HeadingDisplayParams* p = static_cast<HeadingDisplayParams*>(params);

    while (p->isRunning) {
        // Read encoder positions: PROS returns centidegrees, divide by 100 for degrees
        double leftEnc   = passiveEncoderLeft.get_position()  / 100.0;
        double rightEnc  = passiveEncoderRight.get_position() / 100.0;
        double centerEnc = passiveEncoderX.get_position()     / 100.0;

        // Convert encoder degrees to centimeters travelled
        double leftCM   = leftEnc   * encoderWheelCircumferenceCM / 360.0;
        double rightCM  = rightEnc  * encoderWheelCircumferenceCM / 360.0;
        double centerCM = centerEnc * encoderWheelCircumferenceCM / 360.0;
        double heading  = getNormalizedHeading();

        Controller.print(0, 0, "L:%.0f R:%.0f X:%.0f  ", leftCM, rightCM, centerCM);
        Controller.print(1, 0, "Avg:%.0f  H:%.1f   ", (leftCM + rightCM) / 2.0, heading);
        Controller.print(2, 0, "Tgt D:%.0f H:%.0f   ", g_targetDistance, g_targetHeading);

        pros::delay(500);
    }

    Controller.clear();
}

// Driver display: shows LeftMotor3 diagnostics for troubleshooting.
void driverDisplayTask(void* params) {
    HeadingDisplayParams* p = static_cast<HeadingDisplayParams*>(params);

    while (p->isRunning) {
        double leftRpm3 = LeftMotor3.get_actual_velocity();

        Controller.clear();

        Controller.print(0, 0, "L3 RPM:%.0f OK:%d",
            leftRpm3,
            LeftMotor3.is_installed() ? 1 : 0);

        Controller.print(1, 0, "V:%.1f A:%.2f",
            LeftMotor3.get_voltage()      / 1000.0,
            LeftMotor3.get_current_draw() / 1000.0);

        Controller.print(2, 0, "Temp:%.0fC",
            LeftMotor3.get_temperature());

        pros::delay(500);
    }

    Controller.clear();
}

// ══════════════════════════════════════════════════════════════════════════════
// ASYNC SCORING TASK
// ══════════════════════════════════════════════════════════════════════════════
static std::atomic<bool> g_scoringTaskRunning(false);
static double g_scoringTimeMs = 0;
static double g_scoringPower  = 100;

void scoringTaskEntry(void*) {
    g_scoringTaskRunning.store(true);

    uint32_t startTime = pros::millis();
    int32_t  voltage   = static_cast<int32_t>((g_scoringPower / 8.34) * 1000);

    while (g_scoringTaskRunning.load() &&
           pros::millis() - startTime < static_cast<uint32_t>(g_scoringTimeMs)) {
        intakeMotor1.move_voltage(voltage);
        intakeMotor2.move_voltage(voltage);
        pros::delay(10);
    }

    intakeMotor1.move(0);
    intakeMotor2.move(0);
    g_scoringTaskRunning.store(false);
}

void scoreStart(double timeMs, double power) {
    if (g_scoringTaskRunning.load()) {
        g_scoringTaskRunning.store(false);
        pros::delay(20);
    }
    g_scoringTimeMs = timeMs;
    g_scoringPower  = power;
    pros::Task(scoringTaskEntry, nullptr, "scoringTask");
}

// scoreRedStart — async, left gate = red blocks
static double g_scoreRedTimeMs = 0;
void scoreRedTask(void*) { leftScore(g_scoreRedTimeMs, 80.0); }
void scoreRedStart(double timeMs) {
    g_scoreRedTimeMs = timeMs;
    pros::Task(scoreRedTask, nullptr, "scoreRed");
}

// scoreBlueStart — async, right gate = blue blocks
static double g_scoreBlueTimeMs = 0;
void scoreBlueTask(void*) { rightScore(g_scoreBlueTimeMs, 80.0); }
void scoreBlueStart(double timeMs) {
    g_scoreBlueTimeMs = timeMs;
    pros::Task(scoreBlueTask, nullptr, "scoreBlue");
}

void scoringStop() {
    g_scoringTaskRunning.store(false);
    intakeMotor1.move(0);
    intakeMotor2.move(0);
    leftGatePneumatics.set_value(true);
    rightGatePneumatics.set_value(true);
}

// ══════════════════════════════════════════════════════════════════════════════
// COORDINATE FINDER TASK
// ══════════════════════════════════════════════════════════════════════════════

struct CoordinateFinderParams { bool isRunning; };
static CoordinateFinderParams coordFinderParams = {false};

void coordinateFinderTask(void* params) {
    CoordinateFinderParams* p = static_cast<CoordinateFinderParams*>(params);

    while (p->isRunning) {
        pros::screen::erase();
        pros::screen::set_pen(0xFFFFFF);

        pros::screen::print(pros::E_TEXT_MEDIUM, 1, "=== COORDINATE FINDER ===");
        pros::screen::print(pros::E_TEXT_LARGE,  3, "X: %.1f cm", globalX);
        pros::screen::print(pros::E_TEXT_LARGE,  5, "Y: %.1f cm", globalY);
        pros::screen::print(pros::E_TEXT_LARGE,  7, "H: %.1f deg", getNormalizedHeading());

        pros::screen::set_pen(0xFFFF00);
        pros::screen::print(pros::E_TEXT_SMALL,  9, "Push robot to target position");
        pros::screen::print(pros::E_TEXT_SMALL, 10, "Record coordinates above");

        pros::delay(500);
    }
}

void startCoordinateFinder() {
    if (!coordFinderParams.isRunning) {
        coordFinderParams.isRunning = true;
        pros::Task(coordinateFinderTask, &coordFinderParams, "coordFinder");
    }
}

void stopCoordinateFinder() {
    coordFinderParams.isRunning = false;
    pros::delay(600);
    pros::screen::erase();
}

// ══════════════════════════════════════════════════════════════════════════════
// MATCHLOAD PNEUMATIC TASKS  — stubs kept for call-site compatibility
// matchLoadPneumatics has been physically removed; these functions are no-ops.
// ══════════════════════════════════════════════════════════════════════════════
static AsyncTaskParams matchloadPneumaticParams;

void matchloadPneumaticTask(void*) {
    matchloadPneumaticParams.running.store(true);

    if (matchloadPneumaticParams.delayMs > 0) {
        pros::delay(static_cast<uint32_t>(matchloadPneumaticParams.delayMs));
    }

    // matchLoadPneumatics removed — nothing to fire
    uint32_t startTime = pros::millis();
    while (matchloadPneumaticParams.running.load() &&
           pros::millis() - startTime < static_cast<uint32_t>(matchloadPneumaticParams.timeMs)) {
        pros::delay(10);
    }

    matchloadPneumaticParams.running.store(false);
}

void matchloadPneumaticStart(double timeMs, double delayMs, bool async) {
    if (matchloadPneumaticParams.running.load()) {
        matchloadPneumaticParams.running.store(false);
        pros::delay(20);
    }
    matchloadPneumaticParams.timeMs  = timeMs;
    matchloadPneumaticParams.delayMs = delayMs;

    if (async) {
        pros::Task(matchloadPneumaticTask, nullptr, "matchloadPneumatic");
    } else {
        matchloadPneumaticTask(nullptr);
    }
}

void matchloadPneumaticStop() {
    matchloadPneumaticParams.running.store(false);
    // matchLoadPneumatics removed — nothing to retract
}

// ══════════════════════════════════════════════════════════════════════════════
// MATCHLOAD PISTON TIMED TASK — stub, no-op
// ══════════════════════════════════════════════════════════════════════════════
static std::atomic<bool> g_matchloadPistonTaskRunning(false);
static double g_matchloadPistonTimeMs  = 0;
static double g_matchloadPistonDelayMs = 0;

void matchloadPistonTaskFn(void*) {
    // matchLoadPneumatics removed — task exits immediately
    g_matchloadPistonTaskRunning.store(false);
}

void matchloadPistonStart(double timeMs, double delayMs) {
    if (g_matchloadPistonTaskRunning.load()) {
        g_matchloadPistonTaskRunning.store(false);
        pros::delay(10);
    }
    g_matchloadPistonDelayMs = delayMs;
    g_matchloadPistonTimeMs  = timeMs;
    g_matchloadPistonTaskRunning.store(true);
    pros::Task(matchloadPistonTaskFn, nullptr, "matchloadPiston");
}

void matchloadPistonStop() {
    g_matchloadPistonTaskRunning.store(false);
    // matchLoadPneumatics removed — nothing to retract
}