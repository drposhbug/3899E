#ifndef AUTONTASKS_HPP
#define AUTONTASKS_HPP

#include "utils.hpp"

// New tasks
void intakeHopperStart(double timeMs, double power, double delayMs = 0, bool async = true);
void matchloadStart(double timeMs, double power, double delayMs = 0, bool async = true);

// Legacy functions
void intake(double time, bool pistonState);
void intakeStart(double timeMs, double intakePct, bool pistonState);
void intakeStart2(double timeMs, double intakePct, bool pistonState, bool matchLoad, Color targetColor);
void intakeStop();
void score(double time, double power);
void stopScore();
void outtake(double time, double power);
void stopOuttake();
void scoreStart(double timeMs, double power);

struct HeadingDisplayParams {
    bool isRunning;
};

extern HeadingDisplayParams headingDisplayParams;
extern double g_targetDistance;
extern double g_targetHeading;
void headingDisplayTask(void* params);

#endif // AUTONTASKS_HPP
