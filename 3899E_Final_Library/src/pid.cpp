#include "pid.h"
#include "vex.h"
#include "robot_config.h"

using namespace vex;

/**
 * PID Constructor
 * Initialize controller with tuning coefficients and reset state variables
 */
PID::PID(double kp, double ki, double kd)
    : kp(kp), ki(ki), kd(kd), prevError(0), integral(0) {}

/**
 * Calculate PID Control Output
 * Computes the correction value based on error between target and current reading
 */
double PID::calculate(double targetValue, double currentReading)
{
    // Calculate current error
    double error = targetValue - currentReading;

    // Proportional term - immediate response to current error
    double pTerm = kp * error;

    // Integral term - response to accumulated error over time
    integral += error;
    double iTerm = ki * integral;

    // Derivative term - response to rate of error change
    double dTerm = kd * (error - prevError);
    prevError = error;

    // Return combined PID output
    return pTerm + iTerm + dTerm;
}

/**
 * Reset PID Controller State
 * Clears accumulated error and previous error to prevent unwanted behavior
 * when switching between different control contexts
 */
void PID::pidReset()
{
    prevError = 0;
    integral = 0;
}