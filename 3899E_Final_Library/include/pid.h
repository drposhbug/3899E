#ifndef PID_H
#define PID_H

/**
 * PID Controller Class
 * 
 * Implements a Proportional-Integral-Derivative controller for precise motor control.
 * Used primarily for heading correction during straight-line autonomous movements.
 */
class PID {
public:
    /**
     * Constructor - Initialize PID controller with tuning coefficients
     * @param kp Proportional gain - responds to current error
     * @param ki Integral gain - responds to accumulated error over time  
     * @param kd Derivative gain - responds to rate of error change
     */
    PID(double kp, double ki, double kd);

    /**
     * Calculate PID output based on target and current values
     * @param targetValue Desired setpoint value
     * @param currentReading Current sensor reading
     * @return PID correction value to apply to system
     */
    double calculate(double targetValue, double currentReading);

    /**
     * Reset PID controller state
     * Call this when switching between different control contexts
     * to prevent integral windup and derivative spikes
     */
    void pidReset();

private:
    // PID tuning coefficients
    double kp, ki, kd;
    
    // Controller state variables
    double prevError;  // Previous error for derivative calculation
    double integral;   // Accumulated error for integral calculation
};

#endif // PID_H