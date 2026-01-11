#ifndef UTILS_HPP
#define UTILS_HPP

#include "robot_config.hpp"

// Heading functions
double getRotation();
double getAdjustedRotation();

// Motor/Speed detection
bool isSlipping(double motorSpeed, double encoderSpeed);
bool isLocking(double motorSpeed, double encoderSpeed);
bool isAccelerating(double targetDriverSpeed, double currentSpeed);
double getMotorSpeed(pros::Motor& motor);
double getEncoderSpeed(pros::Rotation& encoder);
double calculateSlipRatio(double wheelSpeed, double robotSpeed);
double calculateLockupRatio(double wheelSpeed, double robotSpeed);
float rollingAverage(float newValue, float currentAverage, int n);
void PIDVoltageCapCorrection(double& leftVoltage, double& rightVoltage, double absoluteMaxVoltage);

// Color detection
bool detectColor(Color targetColor);
bool detectRed();
bool detectBlue();
void resetColorDetection();
void initializeOpticalSensor();
void ringEjection();

// Motor control structs and functions
struct MotorControlParams {
    pros::Motor* targetMotor;
    int DelayStart;
    int OnTime;
    int dir; // 1 for forward, -1 for reverse
};

void MotorControl(pros::Motor& targetMotor, int DelayStart, int OnTime, int dir);
void MotorControlThread(void* params);

// Arm control
struct ArmTaskParams {
    bool isRunning;
    int targetPosition;
    int delayMs;
    bool moveRequested;
};
void armTask(void* params);

// Color detection tasks
struct ColorTaskParams {
    bool isRunning;
    Color targetColor;
    int delayMs;
};
void colorDetectionTask(void* params);

void waitForButtonPress();

// Struct for Arm Reset Task Parameters
struct ArmResetTaskParams {
    bool isRunning;
    bool isResetComplete;
};

// Function declaration
void armResetTask(void* params);

// Structure for intake stall detection
struct IntakeStallTaskParams {
    bool isRunning;
    double stallThreshold;
    int reverseRotation;
    int reverseSpeed;
};

// Task function for monitoring intake stalls
void intakeStallTask(void* params);

// Function to start monitoring intake for stalls
void startIntakeStallDetection();

// Global task parameters
extern IntakeStallTaskParams intakeStallParams;

void waitForButton();


#endif // UTILS_HPP
