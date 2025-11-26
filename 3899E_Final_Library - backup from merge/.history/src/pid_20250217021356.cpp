#include "pid.h" // Include the PID header file
#include "vex.h" // Include the VEX library
#include "robot-config.h" // Include the robot configuration to use the Brain object

using namespace vex; // Use the VEX namespace

// Constructor to initialize PID coefficients and internal variables
PID::PID(double kp, double ki, double kd)
    : kp(kp), ki(ki), kd(kd), prevError(0), integral(0) {}

// Get PID control value after given target value and current reading
double PID::calculate(double targetValue, double currentReading) {
    // Calculate the error
    double error = targetValue - currentReading;

    // Proportional term
    double pTerm = kp * error;

    // Integral term
    integral += error;
    double iTerm = ki * integral;

    // Derivative term
    double dTerm = kd * (error - prevError);
    prevError = error; // Update previous error

    // PID output
    return pTerm + iTerm + dTerm;
}

// Method to reset the PID controller (useful when switching contexts)
void PID::pidReset() {
    error = 0;
    prevError = 0; // Reset previous error to 0
    integral = 0;  // Reset integral term to 0
}

// Method to set PID coefficients dynamically
void PID::setCoefficients(double newKp, double newKi, double newKd) {
    kp = newKp; // Set new proportional coefficient
    ki = newKi; // Set new integral coefficient
    kd = newKd; // Set new derivative coefficient
}
