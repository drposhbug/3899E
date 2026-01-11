#pragma once
#include "api.h"

// ==========================================
// ENUMS
// ==========================================
enum class Color { RED, BLUE };

// ==========================================
// PHYSICAL CONSTANTS
// ==========================================
// These remain const (not extern) because their values are fixed at compile time
const double numberDriveMotor = 6;
const double accelerationFactor = 1.05;
const double absoluteMaxRPM = 600;
const double absoluteMaxVoltage = 12000; // PROS uses millivolts (12000 = 12V)
const double gearRatio = 6;
const double VOLTAGE_TOLERANCE = 0.1;
const double minLaunchPower = 20;
const double DRIVE_MOTOR_RPM_ADJ = 400.0 / 600.0;
const double TRACK_WIDTH = 31.0;
const double ENCODER_OFFSET_X = 4.0;
const double LEFT_ENCODER_OFFSET_Y = 5.3;
const double RIGHT_ENCODER_OFFSET_Y = 5.3;
const double wheelCircumferenceCM = 32.0;
const double encoderWheelCircumferenceCM = 15.96;
const double DISTANCE_TO_WHEEL = 15.25;
const double DISTANCE_TO_ENCODER = 8.3;
const double ENCODER_RADIUS_RATIO = DISTANCE_TO_WHEEL / DISTANCE_TO_ENCODER;

// ==========================================
// EXTERNAL HARDWARE DECLARATIONS
// ==========================================
// These tell other files "these objects exist in robot_config.cpp"

// Motor Groups (For bulk movements)
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;
extern pros::MotorGroup intakeMotors;

// Individual Motors (For specific algorithms/traction control)
extern pros::Motor leftMotor1;
extern pros::Motor leftMotor2;
extern pros::Motor leftMotor3;

extern pros::Motor rightMotor1;
extern pros::Motor rightMotor2;
extern pros::Motor rightMotor3;

extern pros::Motor intakeMotor1;
extern pros::Motor intakeMotor2;

// Controller
extern pros::Controller controller;

// Sensors
extern pros::Imu inertialSensor;
extern pros::Rotation passiveEncoderLeft;
extern pros::Rotation passiveEncoderRight;
extern pros::Rotation passiveEncoderX;
extern pros::Optical opticalSensor;
extern pros::adi::DigitalIn autonBumper;

// Pneumatics (using ADI Digital Out)
extern pros::adi::DigitalOut frontHoodPneumatics;
extern pros::adi::DigitalOut backHoodPneumatics;
extern pros::adi::DigitalOut matchLoadPneumatics;
extern pros::adi::DigitalOut ptoPneumatics;
extern pros::adi::DigitalOut wingPneumatics;

// ==========================================
// GLOBAL VARIABLES
// ==========================================
extern double headingOffset;
extern double targetDriverSpeedLeft;
extern double targetDriverSpeedRight;

// Arrays must be declared with size in extern if used directly, 
// or pointers if passed around. Keeping as arrays here:
extern bool isAcceleratingLeft[3];
extern bool isAcceleratingRight[3];

// ==========================================
// FUNCTION PROTOTYPES
// ==========================================
void vexcodeInit(void);
void resetMotorPositions();