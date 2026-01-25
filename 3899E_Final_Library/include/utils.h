#ifndef UTILS_H 
#define UTILS_H

#include "vex.h"
#include "robot_config.h"

// Coordinate System Conversion Functions - ADDED
double modifiedToStandardCartesian(double modifiedAngle);
double standardToModifiedCartesian(double standardAngle);
double vexToStandardCartesian(double vexAngle);
double standardCartesianToVex(double standardAngle);

// Heading functions
double getAdjustedRotation();  // KEPT - this is the main one used

// Motor/Speed detection
bool isSlipping(double motorSpeed, double encoderSpeed);
bool isLocking(double motorSpeed, double encoderSpeed);
bool isAccelerating(double targetDriverSpeed, double currentSpeed);
double getMotorSpeed(vex::motor& motor);
double getEncoderSpeed(vex::rotation& encoder);
double calculateSlipRatio(double wheelSpeed, double robotSpeed);
double calculateLockupRatio(double wheelSpeed, double robotSpeed);
float rollingAverage(float newValue, float currentAverage, int n);
void PIDVoltageCapCorrection(double& leftVoltage, double& rightVoltage, double absoluteMaxVoltage);

// Color detection enum (must be defined before use)
enum class Color { RED, BLUE };

// Color detection
bool detectColor(Color targetColor);
bool detectRed(); 
bool detectBlue();
void resetColorDetection();
void initializeOpticalSensor();
void ringEjection();

// Motor control structs and functions
struct MotorControlParams {
    vex::motor* targetMotor;
    int DelayStart;
    int OnTime;
    vex::directionType dir;
};

void MotorControl(vex::motor& targetMotor, int DelayStart, int OnTime, vex::directionType dir);
int MotorControlThread(void* params);

// Arm control
struct ArmTaskParams {
    bool isRunning;
    int targetPosition;
    int delayMs;
    bool moveRequested;
};
int armTask(void* params);

// Color detection tasks
struct ColorTaskParams {
    bool isRunning;
    Color targetColor;
    int delayMs;
};
int colorDetectionTask(void* params);

void waitForButtonPress();

// Struct for Arm Reset Task Parameters
struct ArmResetTaskParams {
    bool isRunning;
    bool isResetComplete;
};

// Function declaration
int armResetTask(void *params);

// Structure for intake stall detection
struct IntakeStallTaskParams {
    bool isRunning;
    double stallThreshold;
    int reverseRotation;
    int reverseSpeed;
};

// Task function for monitoring intake stalls
int intakeStallTask(void *params);

// Function to start monitoring intake for stalls
void startIntakeStallDetection();

// Global task parameters
extern IntakeStallTaskParams intakeStallParams;

void waitForButton();

// Function: smartStop
void smartStop(double linearThreshold = 5.0, double angularThreshold = 5.0, int timeoutMsec = 250, bool brakeLock = true);

// Enhanced arm task parameters
struct SimpleArmTaskParams {
    bool isRunning;
    ArmPosition position;
    int adjustment;
    int delayMs;
    bool isComplete;
};

// Function declarations
int simpleArmTask(void *params);
void moveArm(ArmPosition position, int adjustment = 0, int delayMs = 0);

#endif // UTILS_H