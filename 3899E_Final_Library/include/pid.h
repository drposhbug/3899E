#ifndef PID_H // Include guard to prevent multiple inclusions
#define PID_H

class PID {
public:
    // Constructor to initialize PID coefficients and internal variables
    PID(double kp, double ki, double kd);

    // Method to get PID control signal given a target value and current reading
    double calculate(double targetValue, double currentReading);

    // Method to reset the PID controller (useful when switching contexts)
    void pidReset();

    // Method to set PID coefficients dynamically
    void setCoefficients(double newKp, double newKi, double newKd);

private:
    double kp, ki, kd; // PID coefficients for proportional, integral, and derivative terms
    double prevError;  // Previous error for derivative calculation
    double integral;   // Sum of errors for integral calculation
    double error; 
};

#endif // PID_H