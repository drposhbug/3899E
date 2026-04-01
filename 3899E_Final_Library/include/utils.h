#ifndef UTILS_H
#define UTILS_H

#include "main.h"          // PROS entry point
#include "robot_config.h"

// ══════════════════════════════════════════════════════════════════════════════
// HEADING UTILITIES
// All heading functions return Standard Cartesian convention:
//   East = 0°, Counter-Clockwise positive.
// The IMU natively reports VEX convention (CW+); these functions convert it.
// ══════════════════════════════════════════════════════════════════════════════

// Convert a VEX heading (CW+, North=0°) to Standard Cartesian (CCW+, East=0°).
double vexToStandardCartesian(double vexAngle);

// Continuous (unbounded) Standard Cartesian heading — best for PID math and
// odometry because it doesn't wrap, so deltas are always meaningful.
double getContinuousStandardHeading();

// Normalized Standard Cartesian heading wrapped to –180..+180°.
double getNormalizedStandardHeading();

// Alias for getNormalizedStandardHeading() — used for UI display and printing.
double getNormalizedHeading();

// Legacy alias maintained for backwards compatibility with existing call sites.
double getAdjustedRotation();

// ══════════════════════════════════════════════════════════════════════════════
// MOTOR / SPEED DETECTION
// Helpers for traction control and ABS — compare wheel speed to chassis speed.
// ══════════════════════════════════════════════════════════════════════════════
bool   isSlipping(double motorSpeed, double encoderSpeed);    // wheel faster than chassis
bool   isLocking(double motorSpeed, double encoderSpeed);     // wheel slower than chassis (brake lockup)
bool   isAccelerating(double targetDriverSpeed, double currentSpeed);
double getMotorSpeed(pros::Motor& motor);                     // returns RPM from motor internal encoder
double getEncoderSpeed(pros::Rotation& encoder);              // returns RPM from external tracking encoder
double calculateSlipRatio(double wheelSpeed, double robotSpeed);
double calculateLockupRatio(double wheelSpeed, double robotSpeed);

// Exponential rolling average: blends newValue into currentAverage over n samples.
float rollingAverage(float newValue, float currentAverage, int n);

// Clamp left/right voltages so neither exceeds absoluteMaxVoltage while
// preserving their ratio (prevents clipping one side of a turn).
void PIDVoltageCapCorrection(double& leftVoltage, double& rightVoltage, double absoluteMaxVoltage);

// ══════════════════════════════════════════════════════════════════════════════
// COLOR DETECTION
// Uses the optical sensor hue + brightness to identify ring/ball color.
// ══════════════════════════════════════════════════════════════════════════════

// Target color for ejection or sorting.
enum class Color { RED, BLUE };

bool detectColor(Color targetColor);  // returns true if sensor sees the target color
bool detectRed();
bool detectBlue();
void resetColorDetection();
void initializeOpticalSensor();
void ringEjection();   // immediately reverses intake to eject a detected wrong color

// Hue thresholds — tuned for the optical sensor's lighting conditions.
// Two red ranges are needed because red wraps across 0°/360° on the hue wheel.
extern const double RED_HUE_MIN_1;
extern const double RED_HUE_MAX_1;
extern const double RED_HUE_MIN_2;
extern const double RED_HUE_MAX_2;
extern const double BLUE_HUE_MIN;
extern const double BLUE_HUE_MAX;
extern const double MIN_BRIGHTNESS;  // ignore detections below this brightness

// ══════════════════════════════════════════════════════════════════════════════
// MOTOR CONTROL TASK
// Runs a timed motor burst asynchronously so the main thread can continue.
// ══════════════════════════════════════════════════════════════════════════════
struct MotorControlParams {
    int8_t             motorPort;   // V5 Smart Port number (avoids storing a pointer)
    int                DelayStart;  // ms to wait before spinning
    int                OnTime;      // ms to keep motor running
    bool               reversed;    // false = forward, true = reverse
};

// Blocking helper called from the main thread.
void MotorControl(pros::Motor& targetMotor, int DelayStart, int OnTime, bool reversed);

// PROS task function — signature must be void(void*).
void MotorControlThread(void* params);

// ══════════════════════════════════════════════════════════════════════════════
// ARM CONTROL TASK
// Moves the arm to an encoder position asynchronously.
// ══════════════════════════════════════════════════════════════════════════════
struct ArmTaskParams {
    bool isRunning;      // set false to stop the task loop
    int  targetPosition; // encoder position to move to (degrees)
    int  delayMs;        // delay before starting the move
    bool moveRequested;  // signal from caller that a new move should begin
};

void armTask(void* params);  // PROS task function

// ══════════════════════════════════════════════════════════════════════════════
// COLOR DETECTION TASK
// Continuously monitors the optical sensor and triggers ejection on match.
// ══════════════════════════════════════════════════════════════════════════════
struct ColorTaskParams {
    bool  isRunning;    // set false to stop the task loop
    Color targetColor;  // color to eject (e.g. BLUE on red alliance)
    int   delayMs;      // ms to wait after detection before stopping intake
};

void colorDetectionTask(void* params);  // PROS task function

// Waits until the user presses a button on the controller (used in pre-match setup).
void waitForButtonPress();

// ══════════════════════════════════════════════════════════════════════════════
// ARM RESET TASK
// Runs after auton to bring the arm back to its starting encoder position.
// ══════════════════════════════════════════════════════════════════════════════
struct ArmResetTaskParams {
    bool isRunning;
    bool isResetComplete;  // set true by task when the reset move finishes
};

void armResetTask(void* params);  // PROS task function

// ══════════════════════════════════════════════════════════════════════════════
// INTAKE STALL DETECTION TASK
// Monitors intake motor current; reverses briefly if a stall is detected.
// ══════════════════════════════════════════════════════════════════════════════
struct IntakeStallTaskParams {
    bool   isRunning;
    double stallThreshold;   // current draw (amps) above which a stall is declared
    int    reverseRotation;  // encoder degrees to back up when stall detected
    int    reverseSpeed;     // speed% for the backup move
};

void intakeStallTask(void* params);  // PROS task function
void startIntakeStallDetection();    // convenience wrapper that creates the task

extern IntakeStallTaskParams intakeStallParams;

// Wait for a controller button press (alias for waitForButtonPress).
void waitForButton();

// ══════════════════════════════════════════════════════════════════════════════
// SMART STOP
// Waits until the robot has decelerated below both linear and angular thresholds
// before returning, preventing the next command from starting during drift.
// ══════════════════════════════════════════════════════════════════════════════
void smartStop(double linearThreshold  = 5.0,
               double angularThreshold = 5.0,
               int    timeoutMsec      = 250,
               bool   brakeLock        = true);

// ══════════════════════════════════════════════════════════════════════════════
// SIMPLE ARM TASK
// Lightweight arm mover: handles one position request at a time.
// ══════════════════════════════════════════════════════════════════════════════
struct SimpleArmTaskParams {
    bool        isRunning;
    ArmPosition position;    // target arm position (from ArmPosition enum)
    int         adjustment;  // fine-tune offset added to the enum value
    int         delayMs;     // delay before starting the move
    bool        isComplete;  // set true by task when move finishes
};

void simpleArmTask(void* params);  // PROS task function

// Convenience: queues an arm move by updating SimpleArmTaskParams and signaling the task.
// adjustment – optional encoder offset (positive = higher, negative = lower)
// delayMs    – optional start delay in milliseconds
void moveArm(ArmPosition position, int adjustment = 0, int delayMs = 0);

#endif // UTILS_H
