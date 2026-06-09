#ifndef AUTONTASKS_H
#define AUTONTASKS_H

#include "utils.h"   // Color enum, arm helpers, task param structs

// color sort tasks
void redColorSortStart();
void blueColorSortStart();

// ══════════════════════════════════════════════════════════════════════════════
// INTAKE / HOPPER TASKS
// These functions start a timed motor burst and optionally return immediately
// (async = true) so the main autonomous thread can continue to the next step.
// ══════════════════════════════════════════════════════════════════════════════

// Spin the intake/hopper for timeMs milliseconds at the given power%.
// delayMs: wait this long before starting (0 = start immediately).
// async:   true = launch as a PROS background task and return; false = block.
void intakeHopperStart(double timeMs, double power, double delayMs = 0.0, bool async = true);

// Same for the match-loader mechanism (motors + piston).
void matchloadStart(double timeMs, double power, double delayMs = 0.0, bool async = true);

// ══════════════════════════════════════════════════════════════════════════════
// LEGACY INTAKE HELPERS
// Kept for backwards compatibility with older autonomous routines.
// ══════════════════════════════════════════════════════════════════════════════
void intake(double time, bool pistonState);
void intakeStart(double timeMs, double intakePct, bool pistonState, bool matchLoad);
void intakeColourStart(double timeMs, double intakePct, bool pistonState, bool matchLoad);
void intakeStop();

// ══════════════════════════════════════════════════════════════════════════════
// SCORING HELPERS
// Run the scoring mechanism for a set time and power.
// ══════════════════════════════════════════════════════════════════════════════
void score(double time, double power);
void rightScore(double time, double power);  // right-lane only
void leftScore(double time, double power);   // left-lane only
void stopScore();

// ══════════════════════════════════════════════════════════════════════════════
// OUTTAKE HELPERS
// ══════════════════════════════════════════════════════════════════════════════
void outtake(double time, double power);
void stopOuttake();

// Start scoring for timeMs milliseconds (non-blocking).
void scoreStart(double timeMs, double power);

// ══════════════════════════════════════════════════════════════════════════════
// DISPLAY / TELEMETRY TASKS
// These tasks print live robot state to the driver controller each cycle.
// ══════════════════════════════════════════════════════════════════════════════
struct HeadingDisplayParams {
    bool isRunning;  // set false to stop the display task
};

extern HeadingDisplayParams headingDisplayParams;

// Shared variables written by motion functions and displayed by the tasks.
extern double g_targetDistance;
extern double g_targetHeading;

// Display loop: shows encoder distances, IMU heading, and nav targets.
void headingDisplayTask(void* params);   // PROS task function — void(void*)

// Driver-mode display loop: shows LeftMotor3 diagnostics (RPM, voltage, temp).
void driverDisplayTask(void* params);    // PROS task function — void(void*)

// ══════════════════════════════════════════════════════════════════════════════
// COORDINATE FINDER
// Prints the robot's (globalX, globalY, heading) to the Brain screen in a loop
// so the programmer can push the robot around and record field coordinates.
// ══════════════════════════════════════════════════════════════════════════════
void startCoordinateFinder();
void stopCoordinateFinder();

// ══════════════════════════════════════════════════════════════════════════════
// MATCH-LOAD PNEUMATIC HELPERS
// ══════════════════════════════════════════════════════════════════════════════

// Extend the match-load pneumatic for timeMs ms (optionally with a start delay).
// async = true: run as a background task.
void matchloadPneumaticStart(double timeMs, double delayMs = 0.0, bool async = true);

// Blocking version: extend piston for timeMs ms (with optional start delay), then retract.
void matchloadPistonStart(double timeMs, double delayMs = 0.0);

// Immediately retract the match-load piston.
void matchloadPistonStop();

// ══════════════════════════════════════════════════════════════════════════════
// STOP HELPERS
// ══════════════════════════════════════════════════════════════════════════════
void intakeHopperStop();
void matchloadPneumaticStop();

#endif // AUTONTASKS_H
