#ifndef UTILS_H 
#define UTILS_H

#include "vex.h"
#include "robot_config.h"

// Heading functions
double getRotation();
double getAdjustedRotation();

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
    bool isRunning;         // Flag to control task execution
    double stallThreshold;  // Velocity threshold for stall detection (in percent)
    int reverseRotation;    // How much to reverse the intake in degrees
    int reverseSpeed;       // Speed to use when reversing (in percent)
};

// Task function for monitoring intake stalls
int intakeStallTask(void *params);

// Function to start monitoring intake for stalls
void startIntakeStallDetection();

// Global task parameters
extern IntakeStallTaskParams intakeStallParams;

void waitForButton();


// Enhanced arm task parameters
struct SimpleArmTaskParams {
    bool isRunning;          // Flag to control task execution
    ArmPosition position;    // Target position enum
    int adjustment;          // Adjustment value to add/subtract from position
    int delayMs;             // Delay before moving to position
    bool isComplete;         // Flag to indicate if the task has completed
};

// Function declarations
int simpleArmTask(void *params);
void moveArm(ArmPosition position, int adjustment = 0, int delayMs = 0);

#endif // UTILS_H