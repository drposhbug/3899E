#ifndef UTILS_H 
#define UTILS_H

#include "vex.h"
#include "robot_config.h"

// Heading functions
double normalizeHeading(double heading);
double normHeading(double heading);
double normHeading360(double heading);
double normalizeHeading180(double heading);
double getHeadingError360(double targetHeading, double currentHeading);
double getHeadingError(double targetHeading, double currentHeading);
double convertHeading(double currentHeading, double offset);
double getAdjustedHeading();
double convertToVEXHeading(double euclideanHeading);
double convertEuclideanToVEX(double euclideanHeading);

// Motor/Speed detection
bool isSlipping(double motorSpeed, double encoderSpeed);
bool isLocking(double motorSpeed, double encoderSpeed);
bool isAccelerating(double targetDriverSpeed, double currentSpeed);
double getMotorSpeed(vex::motor& motor);
double getEncoderSpeed(vex::rotation& encoder);
double calculateSlipRatio(double wheelSpeed, double robotSpeed);
float rollingAverage(float newValue, float currentAverage, int n);
void PIDVoltageCapCorrection(double& leftVoltage, double& rightVoltage, double absoluteMaxVoltage);

// Color detection
bool detectColor();
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
enum class Color { RED, BLUE };
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

struct IntakeStallTaskParams {
    bool isRunning;
    double stallThreshold;
    double reverseRotation;
    double reverseSpeed;
    int delayMs;               // Added for delay support
};

IntakeStallTaskParams intakeStallParams;

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
