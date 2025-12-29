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
    double targetHeading = 0, 
    double minSpeed = 16, 
    double kp_heading = 0.4, 
    double ki_heading = 0.01, 
    double kd_heading = 0.05, 
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
            double minSpeed = 17, 
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

// Adaptive ABS for deceleration phase - prevents wheel lockup while guaranteeing minimum braking
class adaptiveABS {
public:
    adaptiveABS(double decelStepPercent, double lockThreshold);
    void initialize(double startingVoltage);
    double decelControlSpeed(double motorVoltage, double wheelSpeed, double robotSpeed);

private:
    double decelStepVoltage;        // Voltage step size (based on 12V max)
    double lockThreshold;           // Lockup detection threshold (0-1)
    double minimumBrakingVoltage;   // Maximum voltage during rollback (guarantees minimum braking)
};

// Forward/backward wrappers
void forwardMP(double targetDistance,
            double breakDistance = 35, 
            double targetHeading = 0,
            double minSpeed = 16,
            double kp_heading = 0.615, 
            double ki_heading = 0,
            double kd_heading = 0, 
            double accelHeadingScaling = .10,
            double decelHeadingScaling = 0.05, 
            double approachHeadingScaling = 0.05,
            double maxSpeed = 100);

void backwardMP(double targetDistance,
             double breakDistance = 35, 
             double targetHeading = 0,
             double minSpeed = 16,
             double kp_heading = 0.615, 
             double ki_heading = 0,
             double kd_heading = 0, 
             double accelHeadingScaling = .10,
             double decelHeadingScaling = 0.05, 
             double approachHeadingScaling = 0.05,
             double maxSpeed = 100);

// Turn wrappers
void leftMP(double turnAmount, 
    double breakDistance = 35, 
    double minSpeed = 17, 
    double maxSpeed = 100);

void rightMP(double turnAmount,
             double breakDistance = 35,
             double minSpeed = 17,
             double maxSpeed = 100);

void pivotTurnOdometry(double targetHeading,
             double breakDistanceInDegrees,
             double minSpeed, double maxSpeed);

void pivotLeftMP(double turnAmount, double breakDistance, double minSpeed, double maxSpeed);

void pivotRightMP(double turnAmount, double breakDistance, double minSpeed, double maxSpeed);

void driveForward(double targetDistance,
             double breakDistance = 10, 
             double targetHeading = 0,
             double minSpeed = 20,
             double kp_heading = 0.615, 
             double ki_heading = 0,
             double kd_heading = 0, 
             double accelHeadingScaling = 0.10,
             double decelHeadingScaling = 0.05, 
             double approachHeadingScaling = 0.05,
             double maxSpeed = 100);

void driveBackward(double targetDistance,
              double breakDistance = 10, 
              double targetHeading = 0,
              double minSpeed = 20,
              double kp_heading = 0.615, 
              double ki_heading = 0,
              double kd_heading = 0, 
              double accelHeadingScaling = 0.10,
              double decelHeadingScaling = 0.05, 
              double approachHeadingScaling = 0.05,
              double maxSpeed = 100);

void turnRight(double absoluteTargetHeading, 
               double breakDistance = 5, 
               double minSpeed = 20,
               double maxSpeed = 100);

void turnLeft(double absoluteTargetHeading, 
              double breakDistance = 5, 
              double minSpeed = 20, 
              double maxSpeed = 100);
              
void intake(bool state, double speedPct);
void intake2(double time, bool pistonState);
void intakeStart(double timeMs, double intakePct, bool pistonState, bool matchLoad);
void intakeStop();
void score(double time, double power);
void stopScore();
void stopIntake();
void outtake(double time);
void stopOuttake();
void pidlessForward(double timeMs, double speedPct);

#endif // PID_TASKS_H;