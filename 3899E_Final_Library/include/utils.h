// ============================================================================
// UTILS.H - Utility Functions and Helper Classes
// ============================================================================
#ifndef UTILS_H 
#define UTILS_H

#include "vex.h"
#include "robot_config.h"

// ======================== MOTOR SPEED & DETECTION ===========================

bool isSlipping(double motorSpeed, double encoderSpeed);
bool isLocking(double motorSpeed, double encoderSpeed);
bool isAccelerating(double targetDriverSpeed, double currentSpeed);
double getMotorSpeed(vex::motor& motor);
double getEncoderSpeed(vex::rotation& encoder);
double calculateSlipRatio(double wheelSpeed, double robotSpeed);
float rollingAverage(float newValue, float currentAverage, int n);
void PIDVoltageCapCorrection(double& leftVoltage, double& rightVoltage, double absoluteMaxVoltage);

// ======================== COLOR DETECTION ===================================

bool detectColor();
void resetColorDetection();
void initializeOpticalSensor();

// ======================== MOTOR CONTROL TASKS ===============================

struct MotorControlParams {
    vex::motor* targetMotor;
    int DelayStart;
    int OnTime;
    vex::directionType dir;
};

void MotorControl(vex::motor& targetMotor, int DelayStart, int OnTime, vex::directionType dir);
int MotorControlThread(void* params);

// ======================== COLOR DETECTION TASK ==============================

enum class Color { RED, BLUE };

struct ColorTaskParams {
    bool isRunning;
    Color targetColor;
    int delayMs;
};

int colorDetectionTask(void* params);

// ======================== INTAKE STALL DETECTION ============================

struct IntakeStallTaskParams {
    bool isRunning;
    double stallThreshold;
    int reverseRotation;
    int reverseSpeed;
};

extern IntakeStallTaskParams intakeStallParams;

int intakeStallTask(void* params);
void startIntakeStallDetection();

// ======================== WAIT UTILITIES ====================================

void waitForButtonPress();  // Wait for controller R1 button
void waitForButton();        // Wait for auton bumper button

// ======================== TRACTION CONTROL ==================================

class tractionControl {
public:
    tractionControl(double minSpeedVoltage, double maxSpeedVoltage, double slipThreshold);
    double tractionControlSpeed(double tractionMotorVoltage, double motorSpeed, 
                               double robotSpeed, double accelFactor);

private:
    double minSpeedVoltage;
    double maxSpeedVoltage;
    double slipThreshold;
};

// ======================== ABS BRAKING =======================================

class ABSController {
public:
    ABSController(double lockThreshold);
    vex::brakeType ABSSpeedReduction(double wheelSpeed, double robotSpeed);

private:
    double ABSLockThreshold;
};

#endif // UTILS_H