#include "vex.h"  // Make sure this is included to use vex:: types

#ifndef PID_TASKS_H // Include guard to prevent multiple inclusions
#define PID_TASKS_H

void move(double distanceCM, double maxSpeed, vex::directionType dir = vex::forward);
void pidStraight(double targetHeading, double targetDistanceCM, double speed, double kp_heading = 0.6, double ki_heading = 0, double kd_heading = 0, double distanceOffset = 5.0, vex::brakeType brakeMode = vex::brakeType::brake);
void turn(double targetHeading, 
        double breakDistanceInDegrees, 
        double minSpeed = 17, 
        double maxSpeed = 100);

void straight(double targetDistance, 
            double breakDistance, 
            double minSpeed = 17, 
            double targetHeading = 0, 
            double kp_heading = 0.2, 
            double ki_heading = 0.0, 
            double kd_heading = 0.0, 
            double accelHeadingScaling = 0.25, 
            double decelHeadingScaling = 0.25, 
            double approachHeadingScaling = 0.25, 
            double maxSpeed = 50);

void straightOdometry(double targetDistance, 
    double breakDistance, 
    double minSpeed = 17, 
    double targetHeading = 0, 
    double kp_heading = 0.8, 
    double ki_heading = 0.0, 
    double kd_heading = 0.0, 
    double accelHeadingScaling = 0.2, 
    double decelHeadingScaling = 0.2, 
    double approachHeadingScaling = 0.2, 
    double maxSpeed = 100);        
            
void backward(double targetDistance, 
            double breakDistance, 
            double minSpeed = 17, 
            double targetHeading = 0, 
            double kp_heading = 1.7, 
            double ki_heading = 0.0007, 
            double kd_heading = 0.0025, 
            double accelHeadingScaling = 0.275, 
            double decelHeadingScaling = 0.2, 
            double approachHeadingScaling = 0.2, 
            double maxSpeed = 100);     

void arcTurn(double targetDistance, 
             double breakDistance,
             double minSpeed,
             double maxSpeed,
             double turnRadius,    // Radius of turn in cm
             bool turnLeft);



//double targetDistance, double maxSpeed = 100, double targetHeading = 0, double breakDistance = 90, double kp_heading = 0.2, double ki_heading = 0.0, double kd_heading = 0.0, double accelHeadingScaling = 0.4, double decelHeadingScaling = 0.25, double approachHeadingScaling = 0.25, double minSpeed = 15 Pretty good for backwards
void turnOdometry(double targetHeading, 
            double breakDistanceInDegrees, 
            double minSpeed = 15, 
            double maxSpeed = 100);

double launchControl(double targetDriverSpeed, vex::motor& motor, vex::rotation& encoder);

// Define the LaunchControl class
class LaunchControl {
public:
    LaunchControl(vex::motor& motor, vex::rotation& encoder, double slipThresholdValue = 1.1); // Default value changed to 0.1

    double adjustSpeed(double targetPower);

private:
    vex::motor& motor;
    vex::rotation& encoder;

    const double slipThreshold; // Threshold for slip detection
    double motorRPM;      // Motor RPM value
    double encoderRPMScaled;    // Encoder RPM value
};


// Define the LaunchControl class
class tractionControl {
public:
    tractionControl(double minSpeedVoltage, double maxSpeedVoltage, double slipThreshold);
    double tractionControlSpeed(double tractionMotorVoltage, double motorSpeed, double robotSpeed, double accelFactor);

private:
    double minSpeedVoltage;
    double maxSpeedVoltage;
    double slipThreshold;
    //static constexpr double slipThreshold = 1.1; // Class-wide constant
    //static constexpr double accelFactor = 1.15;

}; 


// Revised ABSController to match:
class ABSController {
public:
    ABSController(double lockThreshold);
    vex::brakeType ABSSpeedReduction(double wheelSpeed, double robotSpeed);

private:
    double ABSLockThreshold;
};

// Forward/backward wrappers
void forwardMP(double targetDistance,
            double breakDistance = 35, 
            double minSpeed = 10,
            double cartesianAngle = 0,
            double kp_heading = 1.2, 
            double ki_heading = 0.0,
            double kd_heading = 0.0, 
            double accelHeadingScaling = 2.0,
            double decelHeadingScaling = 2.5, 
            double approachHeadingScaling = 2.2,
            double maxSpeed = 85);

void backwarMP(double targetDistance,
             double breakDistance = 35, 
             double minSpeed = 10,
             double cartesianAngle = 0,
             double kp_heading = 1.2, 
             double ki_heading = 0.0,
             double kd_heading = 0.0, 
             double accelHeadingScaling = 2.0,
             double decelHeadingScaling = 2.5, 
             double approachHeadingScaling = 2.2,
             double maxSpeed = 85);

// Turn wrappers
void leftMP(double targetAngle, 
    double breakDistance = 35, 
    double minSpeed = 10, 
    double maxSpeed = 85);

void rightMP(double targetAngle, 
     double breakDistance = 35, 
     double minSpeed = 10, 
     double maxSpeed = 85);



#endif // PID_TASKS_H;