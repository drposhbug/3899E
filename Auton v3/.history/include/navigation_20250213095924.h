#include "vex.h"  // Make sure this is included to use vex:: types

#ifndef PID_TASKS_H // Include guard to prevent multiple inclusions
#define PID_TASKS_H

void move(double distanceCM, double maxSpeed, vex::directionType dir = vex::forward);
void pidStraightTime(double targetHeading, bool (*exitCondition)());
void pidStraight(double targetHeading, double targetDistanceCM, double speed, double kp_heading = 0.6, double ki_heading = 0, double kd_heading = 0, double distanceOffset = 5.0, vex::brakeType brakeMode = vex::brakeType::brake);
void pidDistance(double targetDistance, double maxSpeed, bool (*exitCondition)());
void pidStraightDistance(double targetDistanceCm, double speed, double targetHeading, bool (*exitCondition)());
void spotTurn(double targetHeading, double maxSpeed, double minSpeed, double kp_heading = 1, double ki_heading = 0.0, double kd_heading = 0.0);
void pidBackwardsDistance(double targetDistanceCm, double speed, double targetHeading, bool (*exitCondition)());
void leftTurn(double targetAngle, double maxSpeed, double kP, double minSpeed);
void slipControlV1(double maxLeftSpeed, double maxRightSpeed, double startingSpeed);
void pidStraightDistanceLaunch(double targetHeading, double targetDistanceCm, double maxSpeed = 70, double kp_heading = 0.4, double ki_heading = 0, double kd_heading = 0, double kp_distance = 0.09, double ki_distance = 0, double kd_distance = 0, double minSpeed = 10, vex::brakeType brakeMode = vex::brakeType::brake); 
void pidStraightDistanceABS(double targetHeading, double targetDistance, double maxSpeed = 70, double kp_heading = 0.4, double ki_heading = 0, double kd_heading = 0, double kp_distance = 0.09, double ki_distance = 0, double kd_distance = 0, double minSpeed = 10, double breakDistance = 15); 
void pidStraightDistanceSlipABS(double targetHeading, double targetDistance, double maxSpeed = 70, double kp_heading = 0.4, double ki_heading = 0, double kd_heading = 0, double kp_distance = 0.09, double ki_distance = 0, double kd_distance = 0, double minSpeed = 10, double breakDistance = 15); 
void pidStraightDistanceLaunchABS(double targetHeading, double targetDistance, double maxSpeed = 70, double kp_heading = 0.3, double ki_heading = 0, double kd_heading = 0.0, double minSpeed = 10, double accelHeadingScaling = 0, double decelHeadingScaling = 1.25, double breakDistance = 25); 

void absControl(double targetLeftVoltage, double targetRightVoltage);
void turn(double targetHeading, 
        double breakDistanceInDegrees, 
        double minSpeed = 17, 
        double maxSpeed = 100);

void turn360(double targetHeading, 
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
            double maxSpeed = 100);

void straight360(double targetDistance, 
            double breakDistance, 
            double minSpeed = 17, 
            double targetHeading = 0, 
            double kp_heading = 0.2, 
            double ki_heading = 0.0, 
            double kd_heading = 0.0, 
            double accelHeadingScaling = 0.25, 
            double decelHeadingScaling = 0.25, 
            double approachHeadingScaling = 0.25, 
            double maxSpeed = 100);        

void straightOdometry(double targetDistance, 
            double breakDistance, 
            double minSpeed = 17, 
            double targetHeading = 0, 
            double kp_heading = 0.5, 
            double ki_heading = 0.0, 
            double kd_heading = 0.0, 
            double accelHeadingScaling = 0.0, 
            double decelHeadingScaling = 0.0, 
            double approachHeadingScaling = 0.0, 
            double maxSpeed = 100);            


void straightTemp(double targetDistance, 
            double breakDistance, 
            double minSpeed = 17, 
            double targetHeading = 0, 
            double kp_heading = 0.2, 
            double ki_heading = 0.0, 
            double kd_heading = 0.0, 
            double accelHeadingScaling = 0.25, 
            double decelHeadingScaling = 0.25, 
            double approachHeadingScaling = 0.25, 
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


// Define the ABS class
class antiLockBrake {
public:
    antiLockBrake(vex::motor& motor, vex::rotation& encoder, double minSpeedVoltage, vex::brakeType brakeMode); // Added brakeMode parameter
    double reduceSpeed();
    // Getter method for brakeMode
    vex::brakeType getBrakeMode() const;

private:
    vex::motor& motor;
    vex::rotation& encoder;
    double minSpeedVoltage;
    vex::brakeType brakeMode;  // Added brakeMode as a member variable

    static constexpr double lockThreshold = 0.25; // Class-wide constant
    double motorRPM;          // Motor RPM value
    double encoderRPMScaled;  // Encoder RPM value
};


// Define the slipControl class
class slipControl {
public:
    slipControl(vex::motor& motor, vex::rotation& encoder, double minSpeedVoltage, double maxSpeedVoltage); // Default value changed to 0.1

    double increaseSpeed(double slipMotorVoltage);

private:
    vex::motor& motor;
    vex::rotation& encoder;
    double minSpeedVoltage;
    double maxSpeedVoltage;


    static constexpr double slipThreshold = 1.25; // Class-wide constant
    double motorRPM;      // Motor RPM value
    double encoderRPMScaled;    // Encoder RPM value
};

/*
// Define the LaunchControl class
class spotTurnSlipControl {
public:
    spotTurnSlipControl(vex::motor& motor, vex::inertial& inertialSensor, double minSpeedVoltage, double maxSpeedVoltage);
    double spotTurnincreaseSpeed(double SpotTurnSlipMotorVoltage); 

private:
    vex::motor& motor;
    vex::inertial& inertialSensor;
    double minSpeedVoltage;
    double maxSpeedVoltage;
    double motorRPM;  
    static constexpr double slipThreshold = 1.25; // Class-wide constant
    
};
*/

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


// Define the ABS V3 class
struct ABSResult {
    double motorVoltage;
    vex::brakeType brakeMode;
};

class ABS {
public:
    ABS(vex::motor& motor, vex::rotation& encoder, double minSpeedVoltage, vex::brakeType brakeMode); // Added brakeMode parameter
    ABSResult reduceCurrentSpeed(double ABSMotorVoltage);

private:
    vex::motor& motor;
    vex::rotation& encoder;
    double minSpeedVoltage;
    vex::brakeType brakeMode;  // Added brakeMode as a member variable

    static constexpr double lockThreshold = 0.05; // Class-wide constant
    double motorRPM;          // Motor RPM value
    double encoderRPMScaled;  // Encoder RPM value
};

// Revised ABSController to match:
class ABSController {
public:
    ABSController(double lockThreshold);
    vex::brakeType ABSSpeedReduction(double wheelSpeed, double robotSpeed);

private:
    double ABSLockThreshold;
};



#endif // PID_TASKS_H


