#ifndef AUTONTASKS_H
#define AUTONTASKS_H

#include "utils.h"

// New tasks
void intakeHopperStart(double timeMs, double power, double delayMs = 0, bool async = true);
void matchloadStart(double timeMs, double power, double delayMs = 0, bool async = true);

// Legacy functions
void intake(double time, bool pistonState);
void intakeStart(double timeMs, double intakePct, bool pistonState, bool matchLoad);
void intakeColourStart(double timeMs, double intakePct, bool pistonState, bool matchLoad);
void intakeStop();
void score(double time, double power);
void rightScore(double time, double power);
void leftScore(double time, double power);
void stopScore();
void outtake(double time, double power);
void stopOuttake();
void scoreStart(double timeMs, double power);

struct HeadingDisplayParams {
    bool isRunning;
};

extern HeadingDisplayParams headingDisplayParams;  // extern = declaration only
extern double g_targetDistance;
extern double g_targetHeading;
int headingDisplayTask(void *params);
int driverDisplayTask(void *params);

// Coordinate Finder - displays robot position for path planning
void startCoordinateFinder();
void stopCoordinateFinder();

void matchloadPneumaticStart(double timeMs, double delayMs = 0, bool async = true);
void matchloadPistonStart(double timeMs, double delayMs = 0);
void matchloadPistonStop();

void intakeHopperStop();
void matchloadPneumaticStop();

#endif // AUTONTASKS_H