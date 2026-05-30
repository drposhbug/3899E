/*----------------------------------------------------------------------------
 * pid.cpp — Generic PID controller implementation (Team 3899E)
 *
 * Platform-independent: no PROS or hardware calls. All state is per-instance.
 * Call pidReset() before reusing an instance for a new movement segment.
 *----------------------------------------------------------------------------*/

#include "pid.h"

// ── Constructor ───────────────────────────────────────────────────────────────
PID::PID(double kp, double ki, double kd)
    : kp(kp), ki(ki), kd(kd), prevError(0.0), integral(0.0) {}

// ── calculate ─────────────────────────────────────────────────────────────────
// Standard PID formula:
//   output = kp*error + ki*∫error + kd*(Δerror / Δt)
// Called once per control loop iteration.
double PID::calculate(double targetValue, double currentReading)
{
    double error = targetValue - currentReading;

    // Proportional: reacts to current error magnitude
    double pTerm = kp * error;

    // Integral: accumulates past error to eliminate steady-state offset
    integral += error;
    double iTerm = ki * integral;

    // Derivative: damps oscillation by reacting to rate of error change
    double dTerm = kd * (error - prevError);
    prevError = error;  // save for next iteration

    return pTerm + iTerm + dTerm;
}

// ── pidReset ──────────────────────────────────────────────────────────────────
// Clears state so the controller starts fresh (use between autonomous moves).
void PID::pidReset()
{
    prevError = 0.0;
    integral  = 0.0;
}

// ── setCoefficients ───────────────────────────────────────────────────────────
// Allows gain changes at runtime, e.g. for gain-scheduled straight/turn loops.
void PID::setCoefficients(double newKp, double newKi, double newKd)
{
    kp = newKp;
    ki = newKi;
    kd = newKd;
}
