#include "main.h"
#include "robot_config.h"
#include "utils.h"
#include <cmath>
#include <algorithm>

// ══════════════════════════════════════════════════════════════════════════════
// HEADING CONVERSION
// ══════════════════════════════════════════════════════════════════════════════

// ── Gyro scalar ───────────────────────────────────────────────────────────────
// Corrects for accumulated sensor error over full rotations.
// Calibration method: spin the robot exactly 10 full turns; measure the
// reported angle.  GYRO_SCALE = 3600 / measured_value.
// If you get +32° after 10 turns → decrease GYRO_SCALE (< 1.0).
// If you get -328° after 10 turns → increase GYRO_SCALE (> 1.0).
static const double GYRO_SCALE = 1.009017;

// Returns continuous (unbounded) heading in degrees.
// Does NOT wrap — can return 720°, -450°, etc. Use this for PID and odometry.
// Convention: North = 0°, CW positive (same as VEX inertial sensor output).
double getContinuousStandardHeading() {
    // PROS get_rotation() returns unbounded degrees, CW positive for VEX IMU.
    double currentSensorRotation = InertialSensor.get_rotation();

    // Compute delta from the snapshot taken at setStartPosition, then apply
    // the gyro scale correction for accumulated sensor error.
    double scaledDelta = (currentSensorRotation - gyroReadingAtStart) * GYRO_SCALE;

    // Add delta to starting heading — CW+ convention is preserved throughout.
    return robotStartingHeading + scaledDelta;
}

// Returns heading normalized to −180…+180° (VEX Coordinates, North = 0°, CW+).
double getNormalizedStandardHeading() {
    return fmod(getContinuousStandardHeading() + 540.0, 360.0) - 180.0;
}

// Alias for getNormalizedStandardHeading() — use for display / telemetry.
double getNormalizedHeading() {
    return getNormalizedStandardHeading();
}

// Legacy alias — kept so existing call sites compile without changes.
double getAdjustedRotation() {
    return getNormalizedHeading();
}

// ══════════════════════════════════════════════════════════════════════════════
// MOTOR / ENCODER SPEED HELPERS
// ══════════════════════════════════════════════════════════════════════════════

// Small denominator guard used throughout this file.
static const double DIV_BY_ZERO_THRESHOLD = 0.001;

// Returns wheel surface speed in cm/s from the drive motor's internal encoder.
double getMotorSpeed(pros::Motor& motor) {
    // get_actual_velocity() → RPM at the motor shaft.
    // Divide by gearRatio to get wheel RPM, then convert RPM → cm/s.
    return motor.get_actual_velocity() / gearRatio * wheelCircumferenceCM / 60.0;
}

// Returns wheel surface speed in cm/s from a passive tracking-wheel encoder.
// get_velocity() returns centidegrees/s; divide by 100 for deg/s, then
// convert deg/s → RPM and multiply by circumference.
double getEncoderSpeed(pros::Rotation& encoder) {
    double degPerSec = encoder.get_velocity() / 100.0;  // centideg/s → deg/s
    double rpm = degPerSec / 360.0 * 60.0;
    return rpm * encoderWheelCircumferenceCM / 60.0;    // cm/s
}

// True when the drive wheel is spinning faster than the chassis is moving
// (loss of traction / wheelspin).
bool isSlipping(double motorSpeed, double encoderSpeed) {
    const double slipThreshold = 0.1;
    return motorSpeed > encoderSpeed * (1.0 + slipThreshold);
}

// True when the wheel is rotating much slower than the chassis
// (wheel lock-up under heavy braking).
bool isLocking(double motorSpeed, double encoderSpeed) {
    const double lockThreshold = 0.85;
    return motorSpeed < encoderSpeed * (1.0 - lockThreshold);
}

// True when the requested speed is higher than the current speed in the same
// direction, or when the direction is reversing (both count as accelerating).
bool isAccelerating(double targetDriverSpeed, double currentSpeed) {
    if (targetDriverSpeed * currentSpeed > 0)
        return std::fabs(targetDriverSpeed) > std::fabs(currentSpeed);
    if (targetDriverSpeed * currentSpeed < 0)
        return true;   // direction reversal = accelerating
    return false;
}

// Slip ratio: how much the drive wheel outruns the chassis.
// Range: 0.0 (perfect traction) to 1.0 (full spin).
double calculateSlipRatio(double wheelSpeed, double robotSpeed) {
    double ref = std::max(std::fabs(wheelSpeed), std::fabs(robotSpeed));
    if (ref < DIV_BY_ZERO_THRESHOLD) return 0.0;
    return std::fabs((wheelSpeed - robotSpeed) / ref);
}

// Lockup ratio: how much the chassis outruns the wheel during braking.
// Range: 0.0 (no lockup) to 1.0 (full lock).
double calculateLockupRatio(double wheelSpeed, double robotSpeed) {
    if (std::fabs(robotSpeed) < DIV_BY_ZERO_THRESHOLD)
        return (std::fabs(wheelSpeed) < DIV_BY_ZERO_THRESHOLD) ? 0.0 : 1.0;
    return std::fabs((robotSpeed - wheelSpeed) / robotSpeed);
}

// Exponential rolling average over n samples.
// Keeps recent values weighted more heavily without storing a history buffer.
float rollingAverage(float newValue, float currentAverage, int n) {
    return currentAverage * (n - 1) / n + newValue / n;
}

// Voltage cap that preserves the left/right differential (PID correction term).
// If one side exceeds absoluteMaxVoltage, both sides are scaled so the
// over-limit side is clamped and the other side is reduced by the same amount,
// keeping the turn intent intact.
void PIDVoltageCapCorrection(double& leftVoltage, double& rightVoltage,
                             double absoluteMaxVoltage) {
    double diff = std::fabs(leftVoltage - rightVoltage);
    if (std::fabs(leftVoltage) > absoluteMaxVoltage) {
        leftVoltage  = std::copysign(absoluteMaxVoltage, leftVoltage);
        rightVoltage = std::copysign(absoluteMaxVoltage - diff, rightVoltage);
    } else if (std::fabs(rightVoltage) > absoluteMaxVoltage) {
        leftVoltage  = std::copysign(absoluteMaxVoltage - diff, leftVoltage);
        rightVoltage = std::copysign(absoluteMaxVoltage, rightVoltage);
    }
}

// ══════════════════════════════════════════════════════════════════════════════
// COLOR DETECTION
// ══════════════════════════════════════════════════════════════════════════════

// Hue thresholds — calibrated under competition lighting.
// Red wraps across 0°/360° on the hue wheel, so two ranges are needed.
const double RED_HUE_MIN_1 = 325.0;
const double RED_HUE_MAX_1 = 360.0;
const double RED_HUE_MIN_2 =   0.0;
const double RED_HUE_MAX_2 =  15.0;
const double BLUE_HUE_MIN  = 218.0;
const double BLUE_HUE_MAX  = 245.0;
const double MIN_BRIGHTNESS =  5.0;  // ignore readings below this brightness

// Turn on the optical sensor's illumination LED at full power.
void initializeOpticalSensor() {
    opticalSensor.set_led_pwm(100);  // 0–100% LED brightness
}

// Debounce state — require 3 consecutive matching readings before reporting.
static int  consecutiveDetections = 0;
static bool lastDetectedColor = false;

// Returns true when targetColor has been seen for 3 consecutive calls.
// Filters single-frame noise from reflections or fast-moving objects.
bool detectColor(Color targetColor) {
    double hue        = opticalSensor.get_hue();
    double brightness = opticalSensor.get_brightness();

    if (brightness < MIN_BRIGHTNESS) {
        consecutiveDetections = 0;
        lastDetectedColor = false;
        return false;
    }

    bool colorDetected = false;
    if (targetColor == Color::RED) {
        colorDetected = (hue >= RED_HUE_MIN_1 && hue <= RED_HUE_MAX_1) ||
                        (hue >= RED_HUE_MIN_2 && hue <= RED_HUE_MAX_2);
    } else if (targetColor == Color::BLUE) {
        colorDetected = (hue >= BLUE_HUE_MIN && hue <= BLUE_HUE_MAX);
    }

    if (colorDetected == lastDetectedColor && colorDetected)
        consecutiveDetections++;
    else
        consecutiveDetections = 1;

    lastDetectedColor = colorDetected;
    return (consecutiveDetections >= 3);
}

bool detectRed()  { return detectColor(Color::RED);  }
bool detectBlue() { return detectColor(Color::BLUE); }

void resetColorDetection() {
    consecutiveDetections = 0;
    lastDetectedColor = false;
}

// ringEjection: commented out — re-implement when arm/intake hardware is confirmed.
// void ringEjection() { ... }

// ══════════════════════════════════════════════════════════════════════════════
// MOTOR CONTROL TASK
// Runs a timed motor burst. Called directly or launched as a PROS background task.
// ══════════════════════════════════════════════════════════════════════════════

// Blocking version: waits DelayStart ms, spins for OnTime ms, then stops.
// reversed = true → negative voltage (reverse direction).
void MotorControl(pros::Motor& targetMotor, int DelayStart, int OnTime, bool reversed) {
    pros::delay(DelayStart);
    targetMotor.move_voltage(reversed ? -12000 : 12000);
    pros::delay(OnTime);
    targetMotor.move(0);
}

// PROS task wrapper — casts void* to MotorControlParams, builds a temporary
// Motor from the stored port number, then calls the blocking helper.
void MotorControlThread(void* params) {
    MotorControlParams* p = static_cast<MotorControlParams*>(params);
    pros::Motor motor(p->motorPort, pros::MotorGears::blue);
    MotorControl(motor, p->DelayStart, p->OnTime, p->reversed);
}

// ══════════════════════════════════════════════════════════════════════════════
// COLOR DETECTION TASK
// Monitors the optical sensor continuously and ejects a wrong-color ring.
// ══════════════════════════════════════════════════════════════════════════════
// void colorDetectionTask(void* params) {
//     ColorTaskParams* p = static_cast<ColorTaskParams*>(params);

//     while (p->isRunning) {
//         double hue = opticalSensor.get_hue();

//         bool redSeen  = (hue >= RED_HUE_MIN_1 && hue <= RED_HUE_MAX_1) ||
//                         (hue >= RED_HUE_MIN_2 && hue <= RED_HUE_MAX_2);
//         bool blueSeen = (hue >= BLUE_HUE_MIN  && hue <= BLUE_HUE_MAX);

//         if (redSeen  && p->targetColor == Color::RED) {
//             pros::lcd::set_text(0, "DETECT: RED");
//             pros::delay(p->delayMs);
//             intakeMotor1.move(0);         // stop
//             pros::delay(50);
//             intakeMotor1.move(-100);      // reverse to eject (negative = reverse in PROS)
//         } else if (blueSeen && p->targetColor == Color::BLUE) {
//             pros::lcd::set_text(0, "DETECT: BLUE");
//             pros::delay(p->delayMs);
//             intakeMotor1.move(0);
//             pros::delay(50);
//             intakeMotor1.move(-100);
//         }

//         pros::delay(10);  // ~100 Hz loop
//     }
// }

// ══════════════════════════════════════════════════════════════════════════════
// BUTTON WAIT HELPERS
// ══════════════════════════════════════════════════════════════════════════════

// Blocks until the driver presses and releases R1 on the controller.
// Used during pre-match setup to confirm alliance color selection.
void waitForButtonPress() {
    pros::lcd::set_text(1, "Press R1 to continue");

    // Drain any pre-existing press so we wait for a fresh one.
    while (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
        pros::delay(20);

    while (!Controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
        pros::delay(20);

    while (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
        pros::delay(20);

    pros::lcd::clear();
}

// Alias used by older code paths.
void waitForButton() {
    waitForButtonPress();
}

// ══════════════════════════════════════════════════════════════════════════════
// INTAKE STALL DETECTION TASK
// Detects a jammed intake and briefly reverses it to clear the blockage.
// ══════════════════════════════════════════════════════════════════════════════
IntakeStallTaskParams intakeStallParams;

// void intakeStallTask(void* params) {
//     IntakeStallTaskParams* p = static_cast<IntakeStallTaskParams*>(params);
//     const int REQUIRED_CONSECUTIVE_STALLS = 10;  // ~200 ms at 20 ms loop rate
//     int stallCounter = 0;

//     while (p->isRunning) {
//         // Measure absolute velocity as a percentage of maximum.
//         double velPct = std::fabs(intakeMotor1.get_actual_velocity())
//                         / absoluteMaxRPM * 100.0;

//         if (velPct < p->stallThreshold) {
//             stallCounter++;
//             if (stallCounter >= REQUIRED_CONSECUTIVE_STALLS) {
//                 // Reverse the intake to clear the jam.
//                 intakeMotor1.move_relative(p->reverseRotation,
//                                            p->reverseSpeed * absoluteMaxRPM / 100.0);

//                 // Wait for the move to complete (poll velocity ≈ 0).
//                 while (std::fabs(intakeMotor1.get_actual_velocity()) > 5.0)
//                     pros::delay(10);

//                 // Coast to a stop.
//                 intakeMotor1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
//                 intakeMotor1.move(0);

//                 p->isRunning = false;
//                 break;
//             }
//         } else {
//             stallCounter = 0;  // reset if intake is spinning freely again
//         }

//         pros::delay(20);
//     }
// }

// Starts the intake and launches the stall-detection task.
// void startIntakeStallDetection() {
//     intakeStallParams.isRunning       = true;
//     intakeStallParams.stallThreshold  = 1.0;   // % velocity below which a stall is declared
//     intakeStallParams.reverseRotation = 210;   // degrees to reverse to clear jam
//     intakeStallParams.reverseSpeed    = 60;    // % speed for the reversal

//     intakeMotor1.move(-100);  // run intake in intake direction (negative = forward intake)
//     pros::Task stall_task(intakeStallTask, &intakeStallParams, "IntakeStall");
// }

// ══════════════════════════════════════════════════════════════════════════════
// SIMPLE ARM TASK
// Moves the arm to an encoder position asynchronously.
// ══════════════════════════════════════════════════════════════════════════════
// static SimpleArmTaskParams simpleArmParams;

// void simpleArmTask(void* params) {
//     SimpleArmTaskParams* p = static_cast<SimpleArmTaskParams*>(params);
//     p->isComplete = false;

//     if (p->delayMs > 0)
//         pros::delay(p->delayMs);

//     // targetPosition = enum value + optional fine-tune offset.
//     // Arm motor move_absolute calls go here once arm motors are re-added.
//     // double targetPos = static_cast<double>(p->position) + p->adjustment;
//     // armMotor1.move_absolute(targetPos, 100);
//     // armMotor2.move_absolute(targetPos, 100);

//     p->isComplete = true;
//     p->isRunning  = false;
// }

// // Queues an arm move by filling simpleArmParams and launching the task.
// void moveArm(ArmPosition position, int adjustment, int delayMs) {
//     simpleArmParams.isRunning   = true;
//     simpleArmParams.position    = position;
//     simpleArmParams.adjustment  = adjustment;
//     simpleArmParams.delayMs     = delayMs;
//     simpleArmParams.isComplete  = false;
//     pros::Task arm_task(simpleArmTask, &simpleArmParams, "ArmTask");
// }

// // ══════════════════════════════════════════════════════════════════════════════
// // SMART STOP
// // Applies brakes and waits until both linear and angular speeds drop below
// // their thresholds (or the timeout expires), then optionally locks in HOLD mode.
// // ══════════════════════════════════════════════════════════════════════════════
// void smartStop(double linearThreshold, double angularThreshold,
//                int timeoutMsec, bool brakeLock) {
//     // Immediately command both groups to brake.
//     leftDrive.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
//     rightDrive.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
//     leftDrive.brake();
//     rightDrive.brake();

//     // Poll until the robot settles or the timeout expires.
//     int elapsed = 0;
//     while (elapsed < timeoutMsec) {
//         // Linear speed from passive encoder (centideg/s → deg/s → RPM → cm/s).
//         double linearSpeed  = std::fabs(passiveEncoderLeft.get_velocity() / 100.0
//                                         / 360.0 * 60.0 * encoderWheelCircumferenceCM / 60.0);
//         // Angular speed from IMU z-axis gyro (degrees/s).
//         double angularSpeed = std::fabs(InertialSensor.get_gyro_rate().z);

//         if (linearSpeed < linearThreshold && angularSpeed < angularThreshold)
//             break;

//         pros::delay(10);
//         elapsed += 10;
//     }

    // Optionally lock the robot in place with HOLD mode.
    if (brakeLock) {
        leftDrive.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        rightDrive.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        leftDrive.brake();
        rightDrive.brake();
    }
}
