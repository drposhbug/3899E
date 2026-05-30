#ifndef PID_H
#define PID_H

// Generic PID controller — platform-independent, no hardware dependencies.
// One instance per control loop (heading, distance, vision, etc.).
class PID {
public:
    // Initialize with proportional, integral, and derivative gains.
    PID(double kp, double ki, double kd);

    // Compute the control output for one time step.
    // targetValue   – desired setpoint
    // currentReading – measured process variable
    // Returns the corrective output (positive = increase, negative = decrease).
    double calculate(double targetValue, double currentReading);

    // Reset accumulated state (integral, previous error).
    // Call this when reusing a PID instance for a new movement.
    void pidReset();

    // Update gains at runtime (useful for gain-scheduled controllers).
    void setCoefficients(double newKp, double newKi, double newKd);

private:
    double kp, ki, kd;  // proportional, integral, derivative gains
    double prevError;   // error from the previous time step (for derivative)
    double integral;    // running error sum (for integral)
};

#endif // PID_H
