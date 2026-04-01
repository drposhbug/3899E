#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include "main.h"   // PROS entry point — pulls in pros/pros.hpp and kernel headers

// ══════════════════════════════════════════════════════════════════════════════
// CONTROLLER
// ══════════════════════════════════════════════════════════════════════════════
// Primary driver controller (port MASTER).  Access buttons/joysticks via
// Controller.get_analog() / Controller.get_digital().
extern pros::Controller Controller;

// ══════════════════════════════════════════════════════════════════════════════
// DRIVETRAIN MOTORS
// Individual motor objects — used when a single motor needs direct control
// (e.g. reading velocity for traction-control or slip detection).
// ══════════════════════════════════════════════════════════════════════════════
extern pros::Motor LeftMotor1;
extern pros::Motor LeftMotor2;
extern pros::Motor LeftMotor3;
extern pros::Motor RightMotor1;
extern pros::Motor RightMotor2;
extern pros::Motor RightMotor3;

// Drive motor groups — preferred for synchronized drive commands.
// pros::MotorGroup forwards move/brake calls to all member motors at once.
extern pros::MotorGroup leftDrive;   // LeftMotor1-3
extern pros::MotorGroup rightDrive;  // RightMotor1-3

// ══════════════════════════════════════════════════════════════════════════════
// MECHANISM MOTORS
// ══════════════════════════════════════════════════════════════════════════════
extern pros::Motor intakeMotor1;
extern pros::Motor intakeMotor2;

// ══════════════════════════════════════════════════════════════════════════════
// PNEUMATICS
// pros::adi::DigitalOut drives a solenoid valve: set(true) = extended.
// ══════════════════════════════════════════════════════════════════════════════
extern pros::adi::DigitalOut frontHoodPneumatics;
extern pros::adi::DigitalOut backHoodPneumatics;
extern pros::adi::DigitalOut matchLoadPneumatics;
extern pros::adi::DigitalOut ptoPneumatics;
extern pros::adi::DigitalOut wingPneumatics;
extern pros::adi::DigitalOut indexPneumatics;
extern pros::adi::DigitalOut leftGatePneumatics;   // left-lane scoring gate
extern pros::adi::DigitalOut rightGatePneumatics;  // right-lane scoring gate
extern pros::adi::DigitalOut rudderPneumatics;     // rudder for ball control in intake

// ══════════════════════════════════════════════════════════════════════════════
// SENSORS
// ══════════════════════════════════════════════════════════════════════════════

// IMU — provides heading and cumulative rotation.
// get_heading() returns 0-360°; get_rotation() returns unbounded degrees.
extern pros::Imu InertialSensor;

// Odometry rotation encoders (passive tracking wheels).
// get_position() returns centidegrees; divide by 100.0 for degrees.
extern pros::Rotation passiveEncoderLeft;
extern pros::Rotation passiveEncoderRight;
extern pros::Rotation passiveEncoderX;  // lateral (strafing) encoder

// Optical sensor for ring/ball color detection.
extern pros::Optical opticalSensor;
extern pros::Optical leftLaneOptical;   // left-lane ball detection
extern pros::Optical rightLaneOptical;  // right-lane ball detection


// ══════════════════════════════════════════════════════════════════════════════
// AI VISION SENSOR
// NOTE: PROS 4 exposes the V5 AI Vision Sensor through pros::Vision.
// Color descriptor  → pros::vision_signature_s_t
// Code descriptor   → pros::vision_code_s_t
// Tag descriptor    → no standard PROS C++ wrapper; use VEX C SDK if needed.
// Verify exact type names against your installed PROS kernel version.
// ══════════════════════════════════════════════════════════════════════════════
extern pros::vision_signature_s_t aiVision_blueCube;
extern pros::vision_signature_s_t aiVision_orangeGoal;
extern pros::vision_signature_s_t aiVision_redCube;

extern pros::vision_color_code_t aiVision_redLoad;
extern pros::vision_color_code_t aiVision_blueLoad;
extern pros::vision_color_code_t aiVision_blueRedBlue;
extern pros::vision_color_code_t aiVision_redBlue;

// AprilTag detection objects (tag descriptors).
// Use pros::vision_object_s_t or the VEX C API for tag data if needed.
extern pros::vision_object_s_t aiVision_blueBlock;
extern pros::vision_object_s_t aiVision_redBlock;

// The AI Vision sensor itself.
extern pros::Vision aiVision;

// ══════════════════════════════════════════════════════════════════════════════
// GLOBAL ROBOT STATE VARIABLES
// ══════════════════════════════════════════════════════════════════════════════
extern double robotStartingHeading;          // heading the robot faces at match start (degrees)
extern double robotStartingHeadingStandard;  // same heading in Standard Cartesian convention
extern double gyroReadingAtStart;            // IMU "tare" value recorded at init time
extern double headingOffset;                 // applied to IMU readings to set alliance-relative reference (e.g. 240° for red-side autos)

extern double targetDriverSpeedLeft;   // current speed setpoint for left drive (driver loop)
extern double targetDriverSpeedRight;  // current speed setpoint for right drive (driver loop)

// Per-motor acceleration state flags (one entry per drive motor).
extern bool isAcceleratingLeft[3];
extern bool isAcceleratingRight[3];

// ── Drive constants ───────────────────────────────────────────────────────────
extern const double numberDriveMotor;    // number of drive motors per side
extern const double accelerationFactor; // ramp rate for velocity smoothing
extern const double absoluteMaxRPM;     // free-spin RPM of the drive motor cartridge
extern const double absoluteMaxVoltage; // motor voltage cap (mV, typically 12000)
extern const double gearRatio;          // external drive gear ratio (motor:wheel)
extern const double minLaunchPower;     // minimum power% to overcome static friction at launch

extern const double DRIVE_MOTOR_RPM_ADJ;   // RPM scaling factor for drive motor measurement
extern const double ENCODER_RADIUS_RATIO;  // ratio of tracking wheel radius to drive wheel radius

// Odometry geometry constants (all in centimeters).
extern const double TRACK_WIDTH;            // distance between left and right tracking wheels
extern const double ENCODER_OFFSET_X;       // lateral offset of the X (strafing) encoder
extern const double LEFT_ENCODER_OFFSET_Y;  // longitudinal offset of the left encoder from robot center
extern const double RIGHT_ENCODER_OFFSET_Y; // longitudinal offset of the right encoder from robot center

// ── Wheel/encoder dimensions ──────────────────────────────────────────────────
extern const double wheelCircumferenceCM;         // drive wheel circumference (cm)
extern const double encoderWheelCircumferenceCM;  // tracking wheel circumference (cm)
extern const double VOLTAGE_TOLERANCE;            // minimum voltage difference worth correcting

// ══════════════════════════════════════════════════════════════════════════════
// ARM POSITION ENUM
// Encoder tick positions for each named arm state.
// These values are compared against pros::Motor::get_position().
// ══════════════════════════════════════════════════════════════════════════════
enum ArmPosition {
    Starting     = 0,
    Load1        = 83,
    Load2        = 128,
    Hover        = 580,
    Side         = 700,
    Alliance     = 550,
    ScoringSide  = 514,
    ScoringAlliance = 334,
    Descore      = 437
};

// Tracks the arm's last commanded position (used to avoid redundant moves).
extern ArmPosition armstat;

// ══════════════════════════════════════════════════════════════════════════════
// MOTION PROFILE DEFAULT PARAMETERS
// Tuned values for each motion type. Pass these as defaults in navigation
// function signatures so call sites only need to override what they change.
// ══════════════════════════════════════════════════════════════════════════════
namespace MotionDefaults {

    namespace StraightForward {
        constexpr double BREAK_DISTANCE          = 35.0;  // cm before target to start decel
        constexpr double MIN_SPEED               = 20.0;  // minimum speed% during approach
        constexpr double MAX_SPEED               = 100.0; // peak speed%
        constexpr double KP_HEADING              = 0.615; // proportional heading correction gain
        constexpr double KI_HEADING              = 0.0;
        constexpr double KD_HEADING              = 0.0;
        constexpr double ACCEL_HEADING_SCALING   = 0.10;  // heading correction weight during accel
        constexpr double DECEL_HEADING_SCALING   = 0.05;  // heading correction weight during decel
        constexpr double APPROACH_HEADING_SCALING= 0.05;  // heading correction weight in final approach
    }

    namespace StraightBackward {
        constexpr double BREAK_DISTANCE          = 30.0;
        constexpr double MIN_SPEED               = 20.0;
        constexpr double MAX_SPEED               = 80.0;
        constexpr double KP_HEADING              = 0.8;
        constexpr double KI_HEADING              = 0.0;
        constexpr double KD_HEADING              = 0.0;
        constexpr double ACCEL_HEADING_SCALING   = 0.08;
        constexpr double DECEL_HEADING_SCALING   = 0.06;
        constexpr double APPROACH_HEADING_SCALING= 0.06;
    }

    namespace TurningLeft {
        constexpr double BREAK_DISTANCE = 5.0;   // degrees before target to start decel
        constexpr double MIN_SPEED      = 20.0;
        constexpr double MAX_SPEED      = 100.0;
    }

    namespace TurningRight {
        constexpr double BREAK_DISTANCE = 5.0;
        constexpr double MIN_SPEED      = 20.0;
        constexpr double MAX_SPEED      = 100.0;
    }
}

// ══════════════════════════════════════════════════════════════════════════════
// ROBOT INITIALIZATION
// ══════════════════════════════════════════════════════════════════════════════
// Call once in PROS initialize() to configure motor directions, brake modes,
// encoder resets, and IMU calibration.
void robotInit();

#endif // ROBOT_CONFIG_H
