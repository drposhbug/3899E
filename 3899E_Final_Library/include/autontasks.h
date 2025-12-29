#ifndef AUTONTASKS_H
#define AUTONTASKS_H

#include "utils.h"

void intake2(double time, bool pistonState);
void intakeStart(double timeMs, double intakePct, bool pistonState, bool matchLoad);
void intakeStart(double timeMs, double intakePct, bool pistonState, bool matchLoad, Color targetColor);
void intakeStop();
void score(double time, double power);
void stopScore();
void stopIntake();
void outtake(double time);
void stopOuttake();

#endif // AUTONTASKS_H