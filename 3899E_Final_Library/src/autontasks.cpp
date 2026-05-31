// #include "autontasks.h"
// #include "robot_config.h"
// #include "utils.h"
// #include "odometry.h"
// #include <atomic>
// #include <cmath>

// // ──────────────────────────────────────────────────────────────────────────────
// // GLOBALS
// // ──────────────────────────────────────────────────────────────────────────────

// HeadingDisplayParams headingDisplayParams = {false};

// // Written by motion functions; read by the display tasks.
// double g_targetDistance = 0.0;  // current nav target distance (cm)
// double g_targetHeading  = 0.0;  // current nav target heading (degrees)

// // ──────────────────────────────────────────────────────────────────────────────
// // SHARED ASYNC TASK STATE
// // Each async group stores its timing/power params plus a running flag.
// // Tasks are self-terminating via the flag — no handle storage needed.
// // ──────────────────────────────────────────────────────────────────────────────
// struct AsyncTaskParams {
//     std::atomic<bool> running{false};
//     double timeMs  = 0;
//     double delayMs = 0;
//     double power   = 100;
// };

// // ══════════════════════════════════════════════════════════════════════════════
// // INTAKE HOPPER TASK  (intake motors + front hood)
// // ══════════════════════════════════════════════════════════════════════════════
// static AsyncTaskParams intakeHopperParams;

// void intakeHopperTask(void*) {
//     intakeHopperParams.running.store(true);

//     // Honor optional start delay before doing anything
//     if (intakeHopperParams.delayMs > 0) {
//         pros::delay(static_cast<uint32_t>(intakeHopperParams.delayMs));
//     }

//     frontHoodPneumatics.set_value(false);  // close front hood for intake

//     // Convert power % to millivolts (100 % ≈ 12 V, so /8.34 gives volts × 1000 = mV)
//     int32_t  voltage   = static_cast<int32_t>((intakeHopperParams.power / 8.34) * 1000);
//     uint32_t startTime = pros::millis();

//     // Spin intake motors for the requested duration, or until stopped externally
//     while (intakeHopperParams.running.load() &&
//            pros::millis() - startTime < static_cast<uint32_t>(intakeHopperParams.timeMs)) {
//         intakeMotor1.move_voltage(voltage);
//         intakeMotor2.move_voltage(voltage);
//         pros::delay(10);
//     }

//     intakeMotor1.move(0);
//     intakeMotor2.move(0);
//     intakeHopperParams.running.store(false);
// }

// void intakeHopperStart(double timeMs, double power, double delayMs, bool async) {
//     // Stop any previous instance before starting a new one
//     if (intakeHopperParams.running.load()) {
//         intakeHopperParams.running.store(false);
//         pros::delay(20);
//     }
//     intakeHopperParams.timeMs  = timeMs;
//     intakeHopperParams.power   = power;
//     intakeHopperParams.delayMs = delayMs;

//     if (async) {
//         pros::Task(intakeHopperTask, nullptr, "intakeHopper");  // launch background task; self-terminates via flag
//     } else {
//         intakeHopperTask(nullptr);     // blocking: run on the calling task
//     }
// }

// void intakeHopperStop() {
//     intakeHopperParams.running.store(false);
//     intakeMotor1.move(0);
//     intakeMotor2.move(0);
// }

// // ══════════════════════════════════════════════════════════════════════════════
// // MATCHLOAD TASK  (intake motors + front hood + match-load piston)
// // ══════════════════════════════════════════════════════════════════════════════
// static AsyncTaskParams matchloadParams;
// static double matchloadRetractDelay = 200;  // ms to hold piston before retracting

// void matchloadTask(void*) {
//     matchloadParams.running.store(true);

//     // Honor optional start delay
//     if (matchloadParams.delayMs > 0) {
//         pros::delay(static_cast<uint32_t>(matchloadParams.delayMs));
//     }

//     frontHoodPneumatics.set_value(false);  // close front hood for intake

//     int32_t voltage = static_cast<int32_t>((matchloadParams.power / 8.34) * 1000);

//     // Begin spinning the intake and drop the match-load piston simultaneously
//     intakeMotor1.move_voltage(voltage);
//     intakeMotor2.move_voltage(voltage);
//     matchLoadPneumatics.set_value(true);

//     uint32_t startTime = pros::millis();
//     while (matchloadParams.running.load() &&
//            pros::millis() - startTime < static_cast<uint32_t>(matchloadParams.timeMs)) {
//         pros::delay(10);
//         // Continuously command voltage to guard against the motor's internal timeout
//         intakeMotor1.move_voltage(voltage);
//         intakeMotor2.move_voltage(voltage);
//     }

//     // Stop intake first, then retract piston after a brief hold
//     intakeMotor1.move(0);
//     intakeMotor2.move(0);
//     pros::delay(static_cast<uint32_t>(matchloadRetractDelay));
//     matchLoadPneumatics.set_value(false);

//     matchloadParams.running.store(false);
// }

// void matchloadStart(double timeMs, double power, double delayMs, bool async) {
//     if (matchloadParams.running.load()) {
//         matchloadParams.running.store(false);
//         pros::delay(20);
//     }
//     matchloadParams.timeMs  = timeMs;
//     matchloadParams.power   = power;
//     matchloadParams.delayMs = delayMs;

//     if (async) {
//         pros::Task(matchloadTask, nullptr, "matchload");
//     } else {
//         matchloadTask(nullptr);
//     }
// }

// // ══════════════════════════════════════════════════════════════════════════════
// // LEGACY BLOCKING INTAKE
// // ══════════════════════════════════════════════════════════════════════════════

// // Spin intake in reverse for 'time' ms.  pistonState=true opens the front hood.
// void intake(double time, bool pistonState) {
//     frontHoodPneumatics.set_value(pistonState);  // open/close front hood per arg

//     uint32_t startTime = pros::millis();
//     while (pros::millis() - startTime < static_cast<uint32_t>(time)) {
//         intakeMotor1.move_voltage(-12000);  // full reverse (12 V)
//         intakeMotor2.move_voltage(-12000);
//         pros::delay(10);
//     }
//     intakeMotor1.move(0);
//     intakeMotor2.move(0);
// }

// // ══════════════════════════════════════════════════════════════════════════════
// // ASYNC INTAKE TASK  (shared state + optional colour-sort)
// // ══════════════════════════════════════════════════════════════════════════════
// static std::atomic<bool> g_intakeTaskRunning(false);
// static double g_intakeTimeMs          = 0;
// static double g_intakePct             = 100;
// static bool   g_intakePistonState     = false;
// static bool   g_matchLoadState        = false;
// static bool   g_enableColourDetection = false;

// void intakeTaskEntry(void*) {
//     g_intakeTaskRunning.store(true);

//     // Configure pneumatics per caller's request
//     matchLoadPneumatics.set_value(g_matchLoadState);
//     frontHoodPneumatics.set_value(g_intakePistonState);

//     uint32_t startTime = pros::millis();
//     int32_t  voltage   = static_cast<int32_t>((g_intakePct / 8.34) * 1000);

//     // Consecutive-read counters for Octoball colour sort (noise rejection).
//     // Octoball is 8-sided — a single hue read can land on a facet edge and
//     // return noise.  Requiring N consecutive reads in range before firing the
//     // rudder filters edge-reads without adding meaningful latency.
//     int redConsecutive  = 0;
//     int blueConsecutive = 0;
//     const int REQUIRED_CONSECUTIVE = 2;  // 2 × 10 ms ≈ 20 ms confirmation window

//     while (g_intakeTaskRunning.load() &&
//            pros::millis() - startTime < static_cast<uint32_t>(g_intakeTimeMs)) {
//         intakeMotor1.move_voltage(voltage);
//         intakeMotor2.move_voltage(voltage);

//         // Only run colour sort logic when explicitly enabled for this routine
//         if (g_enableColourDetection) {
//             double leftHue  = leftLaneOptical.get_hue();
//             double rightHue = rightLaneOptical.get_hue();

//             // Red wraps near 0°/360° on the hue wheel — requires two detection bands
//             bool redThisCycle =
//                 ((leftHue  >= RED_HUE_MIN_1 && leftHue  <= RED_HUE_MAX_1) ||
//                  (leftHue  >= RED_HUE_MIN_2 && leftHue  <= RED_HUE_MAX_2)) ||
//                 ((rightHue >= RED_HUE_MIN_1 && rightHue <= RED_HUE_MAX_1) ||
//                  (rightHue >= RED_HUE_MIN_2 && rightHue <= RED_HUE_MAX_2));

//             // Blue sits mid-wheel (~215–225°) — one band only
//             bool blueThisCycle =
//                 (leftHue  >= BLUE_HUE_MIN && leftHue  <= BLUE_HUE_MAX) ||
//                 (rightHue >= BLUE_HUE_MIN && rightHue <= BLUE_HUE_MAX);

//             // Increment the matching colour counter; reset the opposite
//             if (redThisCycle) {
//                 redConsecutive++;
//                 blueConsecutive = 0;
//             } else if (blueThisCycle) {
//                 blueConsecutive++;
//                 redConsecutive = 0;
//             } else {
//                 // Neither colour — only reset counters when the lane is confirmed empty
//                 // (avoids resetting mid-ball when a facet edge produces a momentary miss)
//                 bool nearLeft  = leftLaneOptical.get_proximity()  > 50;
//                 bool nearRight = rightLaneOptical.get_proximity() > 50;
//                 if (!nearLeft && !nearRight) {
//                     redConsecutive  = 0;
//                     blueConsecutive = 0;
//                 }
//             }

//             // Fire rudder once colour is confirmed by REQUIRED_CONSECUTIVE reads
//             if (redConsecutive >= REQUIRED_CONSECUTIVE) {
//                 rudderPneumatics.set_value(true);   // route to right lane
//                 redConsecutive = 0;
//             } else if (blueConsecutive >= REQUIRED_CONSECUTIVE) {
//                 rudderPneumatics.set_value(false);  // route to left lane
//                 blueConsecutive = 0;
//             }
//         }

//         pros::delay(10);
//     }

//     // Cleanup: stop motors and reset all mechanism pneumatics
//     intakeMotor1.move(0);
//     intakeMotor2.move(0);
//     frontHoodPneumatics.set_value(false);
//     matchLoadPneumatics.set_value(false);
//     ptoPneumatics.set_value(false);
//     g_enableColourDetection = false;
//     g_intakeTaskRunning.store(false);
// }

// void intakeStart(double timeMs, double intakePct, bool pistonState, bool matchLoad) {
//     // Stop any running intake task before launching a new one
//     if (g_intakeTaskRunning.load()) {
//         g_intakeTaskRunning.store(false);
//         pros::delay(20);
//     }
//     g_intakeTimeMs      = timeMs;
//     g_intakePistonState = pistonState;
//     g_intakePct         = intakePct;
//     g_matchLoadState    = matchLoad;
//     pros::Task(intakeTaskEntry, nullptr, "intakeTask");
// }

// // Same as intakeStart but enables the colour-sort rudder logic
// void intakeColourStart(double timeMs, double intakePct, bool pistonState, bool matchLoad) {
//     if (g_intakeTaskRunning.load()) {
//         g_intakeTaskRunning.store(false);
//         pros::delay(20);
//     }
//     g_intakeTimeMs          = timeMs;
//     g_intakePistonState     = pistonState;
//     g_intakePct             = intakePct;
//     g_matchLoadState        = matchLoad;
//     g_enableColourDetection = true;
//     pros::Task(intakeTaskEntry, nullptr, "intakeTask");
// }

// // Signal the running intake task to stop early
// void intakeStop() {
//     g_intakeTaskRunning.store(false);
//     pros::delay(20);
// }

// // ══════════════════════════════════════════════════════════════════════════════
// // BLOCKING SCORING HELPERS
// // Open hood, engage PTO, run intake motors for 'time' ms, then clean up.
// // ══════════════════════════════════════════════════════════════════════════════

// void score(double time, double power) {
//     uint32_t startTime = pros::millis();
//     frontHoodPneumatics.set_value(true);  // open front hood for scoring
//     ptoPneumatics.set_value(true);        // engage PTO

//     int32_t voltage = static_cast<int32_t>((power / 8.34) * 1000);

//     while (pros::millis() - startTime < static_cast<uint32_t>(time)) {
//         intakeMotor1.move_voltage(voltage);
//         intakeMotor2.move_voltage(voltage);
//         pros::delay(10);
//     }

//     frontHoodPneumatics.set_value(false);  // close hood after scoring
//     intakeMotor1.move(0);
//     intakeMotor2.move(0);
//     ptoPneumatics.set_value(false);        // disengage PTO
// }

// // Score through the left lane only (right gate blocks the right lane)
// void leftScore(double time, double power) {
//     uint32_t startTime = pros::millis();
//     frontHoodPneumatics.set_value(true);
//     ptoPneumatics.set_value(true);
//     rightGatePneumatics.set_value(true);   // block right lane
//     leftGatePneumatics.set_value(false);   // open left lane

//     int32_t voltage = static_cast<int32_t>((power / 8.34) * 1000);

//     while (pros::millis() - startTime < static_cast<uint32_t>(time)) {
//         intakeMotor1.move_voltage(voltage);
//         intakeMotor2.move_voltage(voltage);
//         pros::delay(10);
//     }

//     frontHoodPneumatics.set_value(false);
//     intakeMotor1.move(0);
//     intakeMotor2.move(0);
//     ptoPneumatics.set_value(false);
// }

// // Score through the right lane only (left gate blocks the left lane)
// void rightScore(double time, double power) {
//     uint32_t startTime = pros::millis();
//     frontHoodPneumatics.set_value(true);
//     ptoPneumatics.set_value(true);
//     rightGatePneumatics.set_value(false);  // open right lane
//     leftGatePneumatics.set_value(true);    // block left lane

//     int32_t voltage = static_cast<int32_t>((power / 8.34) * 1000);

//     while (pros::millis() - startTime < static_cast<uint32_t>(time)) {
//         intakeMotor1.move_voltage(voltage);
//         intakeMotor2.move_voltage(voltage);
//         pros::delay(10);
//     }

//     frontHoodPneumatics.set_value(false);
//     intakeMotor1.move(0);
//     intakeMotor2.move(0);
//     ptoPneumatics.set_value(false);
// }

// void stopScore() {
//     intakeMotor1.move(0);
//     intakeMotor2.move(0);
// }

// // ══════════════════════════════════════════════════════════════════════════════
// // BLOCKING OUTTAKE
// // ══════════════════════════════════════════════════════════════════════════════

// void outtake(double time, double power) {
//     uint32_t startTime = pros::millis();
//     ptoPneumatics.set_value(true);         // engage PTO
//     frontHoodPneumatics.set_value(false);  // close front hood for outtake

//     while (pros::millis() - startTime < static_cast<uint32_t>(time)) {
//         int32_t voltage = static_cast<int32_t>((power / 8.34) * 1000);
//         intakeMotor1.move_voltage(-voltage);  // negative = reverse
//         intakeMotor2.move_voltage(-voltage);
//         pros::delay(10);
//     }

//     ptoPneumatics.set_value(false);  // disengage PTO
//     intakeMotor1.move(0);
//     intakeMotor2.move(0);
// }

// void stopOuttake() {
//     intakeMotor1.move(0);
//     intakeMotor2.move(0);
// }

// // ══════════════════════════════════════════════════════════════════════════════
// // DISPLAY TASKS
// // ══════════════════════════════════════════════════════════════════════════════

// // Heading display: shows encoder distances, IMU heading, and nav targets on the
// // driver controller.  Updates every 500 ms — anything faster risks VEXnet drops.
// void headingDisplayTask(void* params) {
//     HeadingDisplayParams* p = static_cast<HeadingDisplayParams*>(params);

//     while (p->isRunning) {
//         // Read encoder positions: PROS returns centidegrees, divide by 100 for degrees
//         double leftEnc   = passiveEncoderLeft.get_position()  / 100.0;
//         double rightEnc  = passiveEncoderRight.get_position() / 100.0;
//         double centerEnc = passiveEncoderX.get_position()     / 100.0;

//         // Convert encoder degrees to centimeters travelled
//         double leftCM   = leftEnc   * encoderWheelCircumferenceCM / 360.0;
//         double rightCM  = rightEnc  * encoderWheelCircumferenceCM / 360.0;
//         double centerCM = centerEnc * encoderWheelCircumferenceCM / 360.0;
//         double avgCM    = (leftCM + rightCM) / 2.0;
//         double heading  = getNormalizedHeading();

//         // Line 0: encoder distances (L=left, R=right, X=lateral)
//         // %.0f strips decimals to fit within the controller's 15-char line width
//         Controller.print(0, 0, "L:%.0f R:%.0f X:%.0f  ", leftCM, rightCM, centerCM);

//         // Line 1: average distance and current heading
//         Controller.print(1, 0, "Avg:%.0f  H:%.1f   ", avgCM, heading);

//         // Line 2: current navigation targets (updated by motion functions)
//         Controller.print(2, 0, "Tgt D:%.0f H:%.0f   ", g_targetDistance, g_targetHeading);

//         pros::delay(500);
//     }

//     Controller.clear();
// }

// // Driver display: shows LeftMotor3 diagnostics for troubleshooting.
// // Reads all six drive motor RPMs, but only displays LeftMotor3 detail.
// void driverDisplayTask(void* params) {
//     HeadingDisplayParams* p = static_cast<HeadingDisplayParams*>(params);

//     while (p->isRunning) {
//         // Read current RPM from all six drivetrain motors
//         double leftRpm1  = LeftMotor1.get_actual_velocity();
//         double leftRpm2  = LeftMotor2.get_actual_velocity();
//         double leftRpm3  = LeftMotor3.get_actual_velocity();
//         double rightRpm1 = RightMotor1.get_actual_velocity();
//         double rightRpm2 = RightMotor2.get_actual_velocity();
//         double rightRpm3 = RightMotor3.get_actual_velocity();

//         Controller.clear();

//         // Line 0: LeftMotor3 RPM and installed status (1=connected, 0=missing)
//         Controller.print(0, 0, "L3 RPM:%.0f OK:%d",
//             leftRpm3,
//             LeftMotor3.is_installed() ? 1 : 0);

//         // Line 1: voltage (PROS returns mV → /1000 for V) and current (mA → /1000 for A)
//         Controller.print(1, 0, "V:%.1f A:%.2f",
//             LeftMotor3.get_voltage()       / 1000.0,
//             LeftMotor3.get_current_draw()  / 1000.0);

//         // Line 2: motor temperature in °C (PROS reports Celsius, not %)
//         Controller.print(2, 0, "Temp:%.0fC",
//             LeftMotor3.get_temperature());

//         pros::delay(500);
//     }

//     Controller.clear();
// }

// // ══════════════════════════════════════════════════════════════════════════════
// // ASYNC SCORING TASK
// // ══════════════════════════════════════════════════════════════════════════════
// static std::atomic<bool> g_scoringTaskRunning(false);
// static double g_scoringTimeMs = 0;
// static double g_scoringPower  = 100;

// void scoringTaskEntry(void*) {
//     g_scoringTaskRunning.store(true);

//     frontHoodPneumatics.set_value(false);  // ensure hood is closed before engaging PTO
//     ptoPneumatics.set_value(true);         // engage PTO for scoring

//     uint32_t startTime = pros::millis();
//     int32_t  voltage   = static_cast<int32_t>((g_scoringPower / 8.34) * 1000);

//     while (g_scoringTaskRunning.load() &&
//            pros::millis() - startTime < static_cast<uint32_t>(g_scoringTimeMs)) {
//         intakeMotor1.move_voltage(voltage);
//         intakeMotor2.move_voltage(voltage);
//         pros::delay(10);
//     }

//     frontHoodPneumatics.set_value(false);  // close hood after scoring
//     intakeMotor1.move(0);
//     intakeMotor2.move(0);
//     ptoPneumatics.set_value(false);        // disengage PTO
//     g_scoringTaskRunning.store(false);
// }

// void scoreStart(double timeMs, double power) {
//     if (g_scoringTaskRunning.load()) {
//         g_scoringTaskRunning.store(false);
//         pros::delay(20);
//     }
//     g_scoringTimeMs = timeMs;
//     g_scoringPower  = power;
//     pros::Task(scoringTaskEntry, nullptr, "scoringTask");
// }

// // ══════════════════════════════════════════════════════════════════════════════
// // COORDINATE FINDER TASK
// // Prints live (X, Y, heading) to the Brain screen so the programmer can push
// // the robot around the field and record coordinates for autonomous path planning.
// // ══════════════════════════════════════════════════════════════════════════════

// struct CoordinateFinderParams { bool isRunning; };
// static CoordinateFinderParams coordFinderParams = {false};

// void coordinateFinderTask(void* params) {
//     CoordinateFinderParams* p = static_cast<CoordinateFinderParams*>(params);

//     while (p->isRunning) {
//         updateOdometry();  // refresh globalX/Y from encoder and IMU readings

//         pros::screen::erase();
//         pros::screen::set_pen(0xFFFFFF);  // white

//         // Title bar (medium text, line 1)
//         pros::screen::print(pros::E_TEXT_MEDIUM, 1, "=== COORDINATE FINDER ===");

//         // Large position readout — easy to read from across the field
//         pros::screen::print(pros::E_TEXT_LARGE, 3, "X: %.1f cm", globalX);
//         pros::screen::print(pros::E_TEXT_LARGE, 5, "Y: %.1f cm", globalY);
//         pros::screen::print(pros::E_TEXT_LARGE, 7, "H: %.1f deg", getNormalizedHeading());

//         // Usage instructions in yellow (small text, lines 9-10)
//         pros::screen::set_pen(0xFFFF00);
//         pros::screen::print(pros::E_TEXT_SMALL, 9,  "Push robot to target position");
//         pros::screen::print(pros::E_TEXT_SMALL, 10, "Record coordinates above");

//         pros::delay(500);
//     }
// }

// void startCoordinateFinder() {
//     if (!coordFinderParams.isRunning) {
//         coordFinderParams.isRunning = true;
//         pros::Task(coordinateFinderTask, &coordFinderParams, "coordFinder");  // pass params struct by address
//     }
// }

// void stopCoordinateFinder() {
//     coordFinderParams.isRunning = false;
//     pros::delay(600);  // wait for the task to finish its current 500 ms cycle
//     pros::screen::erase();
// }

// // ══════════════════════════════════════════════════════════════════════════════
// // MATCHLOAD PNEUMATIC-ONLY TASK
// // Drops the match-load piston for a set duration, then retracts.
// // No motors — designed to run in parallel with intakeHopperTask.
// // ══════════════════════════════════════════════════════════════════════════════
// static AsyncTaskParams matchloadPneumaticParams;

// void matchloadPneumaticTask(void*) {
//     matchloadPneumaticParams.running.store(true);

//     // Honor optional start delay before firing the piston
//     if (matchloadPneumaticParams.delayMs > 0) {
//         pros::delay(static_cast<uint32_t>(matchloadPneumaticParams.delayMs));
//     }

//     matchLoadPneumatics.set_value(true);  // drop piston

//     // Hold piston down for the full requested duration
//     uint32_t startTime = pros::millis();
//     while (matchloadPneumaticParams.running.load() &&
//            pros::millis() - startTime < static_cast<uint32_t>(matchloadPneumaticParams.timeMs)) {
//         pros::delay(10);
//     }

//     matchLoadPneumatics.set_value(false);  // retract piston
//     matchloadPneumaticParams.running.store(false);
// }

// void matchloadPneumaticStart(double timeMs, double delayMs, bool async) {
//     // Stop any previous instance before starting a new one
//     if (matchloadPneumaticParams.running.load()) {
//         matchloadPneumaticParams.running.store(false);
//         pros::delay(20);
//     }
//     matchloadPneumaticParams.timeMs  = timeMs;
//     matchloadPneumaticParams.delayMs = delayMs;

//     if (async) {
//         pros::Task(matchloadPneumaticTask, nullptr, "matchloadPneumatic");
//     } else {
//         matchloadPneumaticTask(nullptr);
//     }
// }

// void matchloadPneumaticStop() {
//     matchloadPneumaticParams.running.store(false);
//     matchLoadPneumatics.set_value(false);  // immediately retract
// }

// // ══════════════════════════════════════════════════════════════════════════════
// // MATCHLOAD PISTON TIMED TASK
// // Extends the match-load piston for timeMs ms (with optional start delay),
// // then retracts.  Runs as a background PROS task.
// // ══════════════════════════════════════════════════════════════════════════════
// static std::atomic<bool> g_matchloadPistonTaskRunning(false);
// static double g_matchloadPistonTimeMs  = 0;
// static double g_matchloadPistonDelayMs = 0;

// // Internal PROS task function for matchloadPistonStart — timed piston extend/retract.
// void matchloadPistonTaskFn(void*) {
//     uint32_t startTime = pros::millis();
//     while (g_matchloadPistonTaskRunning.load() &&
//            pros::millis() - startTime < static_cast<uint32_t>(g_matchloadPistonTimeMs)) {
//         pros::delay(static_cast<uint32_t>(g_matchloadPistonDelayMs));
//         matchLoadPneumatics.set_value(true);  // extend piston
//         pros::delay(10);
//     }
//     matchLoadPneumatics.set_value(false);  // retract when time is up
//     pros::delay(10);
// }

// void matchloadPistonStart(double timeMs, double delayMs) {
//     if (g_matchloadPistonTaskRunning.load()) {
//         g_matchloadPistonTaskRunning.store(false);
//         pros::delay(10);
//     }
//     g_matchloadPistonDelayMs = delayMs;
//     g_matchloadPistonTaskRunning.store(true);
//     g_matchloadPistonTimeMs = timeMs;
//     pros::Task(matchloadPistonTaskFn, nullptr, "matchloadPiston");
// }

// void matchloadPistonStop() {
//     // Signal the task to stop, then ensure piston is retracted
//     if (g_matchloadPistonTaskRunning.load()) {
//         g_matchloadPistonTaskRunning.store(false);
//         pros::delay(10);  // brief wait for the task to exit cleanly
//     }
//     matchLoadPneumatics.set_value(false);
// }
