#ifndef UTILS_H // Include guard to prevent multiple inclusions
#define UTILS_H
#include "vex.h"

// Function to normalize an angle to the range [-180, 180)
double normalizeHeading(double heading);

// v2 Function to normalize an angle to the range [-180, 180)
double normHeading(double heading);

// Function to get wheel properties
void getWheelProperties(double &wheelCircumferenceCm, double &gearRatio);

// Declaration of the spinToPosition function for armMotor
void spinArmToPosition(int position, int power);

// Declaration of the spinToPosition function for elbowMotor
void spinElbowToPosition(int position, int power);

// Declare the slip detection function
bool isSlipping(double motorSpeed, double encoderSpeed); 

// Implement lock-up detection logic
bool isLocking(double motorSpeed, double encoderSpeed); 

// Get encoder speed function
double getEncoderSpeed(vex::rotation& encoder); 

// Function to check for red or blue based on calibrated hue and brightness
void checkColor();

// Ring ejection function
void ringEjection();

// Initialize Colour Sensor
void initializeOpticalSensor(); 

bool isAccelerating(double targetDriverSpeed, double currentSpeed);

double getMotorSpeed(vex::motor& motor);

double getEncoderSpeed(vex::motor& motor);

double MotorPowerToSpeed(double motorPower); 

double SpeedToMotorPower(double speedCMperSec);

struct MotorControlParams {
    vex::motor* targetMotor;
    int DelayStart;
    int OnTime;
    vex::directionType dir;
};

// Function prototypes
void MotorControl(vex::motor& targetMotor, int DelayStart, int OnTime, vex::directionType dir);
int MotorControlThread(void* params);

// Declarations and definitions go here
typedef void (*ActionFunction)();
enum ColorType {
    RED,
    BLUE,
    ANY
};
void checkColor(ColorType colorToDetect, ActionFunction action);

void scaleVoltages(double& leftVoltage, double& rightVoltage, double absoluteMaxVoltage); 
void PIDVoltageCapCorrection(double& leftVoltage, double& rightVoltage, double absoluteMaxVoltage);

#endif // UTILS_H
