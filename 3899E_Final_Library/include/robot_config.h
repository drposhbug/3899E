#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include "main.h"   // PROS entry point — pulls in pros/pros.hpp and kernel headers

// ══════════════════════════════════════════════════════════════════════════════
// CONTROLLER
// ══════════════════════════════════════════════════════════════════════════════
// Primary driver controller (port MASTER).
// Access axes via Controller.get_analog(), buttons via Controller.get_digital().
extern pros::Controller Controller;

// ══════════════════════════════════════════════════════════════════════════════
// DRIVETRAIN MOTORS
// ══════════════════════════════════════════════════════════════════════════════
// Individual motor objects — use when a single motor needs direct access
// (e.g. reading velocity for traction-control or slip detection).
extern pros::Motor LeftMotor1;
extern pros::Motor LeftMotor2;
extern pros::Motor LeftMotor3;
extern pros::Motor RightMotor1;
extern pros::Motor RightMotor2;
extern pros::Motor RightMotor3;

// Motor groups — preferred for all synchronized drive commands.
// pros::MotorGroup forwards move/brake calls to all member motors at once.
extern pros::MotorGroup leftDrive;   // LeftMotor1-3
extern pros::MotorGroup rightDrive;  // RightMotor1-3

// ══════════════════════════════════════════════════════════════════════════════
// MECHANISM MOTORS
// ══════════════════════════════════════════════════════════════════════════════
extern pros::Motor intakeMotor;
extern pros::Motor lever;
extern pros::Motor colorSortMotor;

// ══════════════════════════════════════════════════════════════════════════════
// PNEUMATICS
// ══════════════════════════════════════════════════════════════════════════════
// set_value(true) = solenoid extended, set_value(false) = retracted.
extern pros::adi::DigitalOut matchloader;  
extern pros::adi::DigitalOut scoreFlap;  
extern pros::adi::DigitalOut colorSortFlap;        
extern pros::adi::DigitalOut scorePiston;    

// ══════════════════════════════════════════════════════════════════════════════
// SENSORS
// ══════════════════════════════════════════════════════════════════════════════

// IMU — provides heading (0–360°) and unbounded cumulative rotation.
extern pros::Imu InertialSensor;

// Passive odometry tracking wheels.
// get_position() returns centidegrees; divide by 100.0 for degrees.
extern pros::Rotation passiveEncoderLeft;
extern pros::Rotation passiveEncoderRight;
extern pros::Rotation passiveEncoderX;   // lateral (strafing) encoder

// Optical sensors — ring/ball color detection and lane tracking.
extern pros::Optical opticalSensor;

// Radio - for communication
extern std::int8_t receiverPort;
extern pros::Link* receiver;
void getMessageReceived(float out[5]);
void setMessageToSend(float newMessage[5]);
extern std::string LINK_ID;
extern bool matchloaderState;
extern bool scoreFlapState;
extern bool scorePistonState;

// ══════════════════════════════════════════════════════════════════════════════
// AI VISION SENSOR
// ══════════════════════════════════════════════════════════════════════════════
// Color signatures use PROS YCbCr / UV color space — NOT HSV.
// Re-run the PROS Vision Sensor utility to calibrate U/V ranges for your lighting.
extern pros::vision_signature_s_t aiVision_blueCube;
extern pros::vision_signature_s_t aiVision_orangeGoal;
extern pros::vision_signature_s_t aiVision_redCube;

// Color codes — multi-signature patterns detected together.
extern pros::vision_color_code_t aiVision_redLoad;
extern pros::vision_color_code_t aiVision_blueLoad;
extern pros::vision_color_code_t aiVision_blueRedBlue;
extern pros::vision_color_code_t aiVision_redBlue;

// Detection result objects — populated at runtime by aiVision.get_by_sig().
extern pros::vision_object_s_t aiVision_blueBlock;
extern pros::vision_object_s_t aiVision_redBlock;

// The AI Vision sensor itself (port 14).
extern pros::Vision aiVision;

// ══════════════════════════════════════════════════════════════════════════════
// GLOBAL ROBOT STATE
// ══════════════════════════════════════════════════════════════════════════════
extern double robotStartingHeading;          // heading at match start (degrees)
extern double robotStartingHeadingStandard;  // same heading in Standard Cartesian convention
extern double gyroReadingAtStart;            // IMU value recorded at init (tare reference)

// Applied to IMU readings to set alliance-relative reference.
// e.g. 240° for a red-side auto that starts 240° from field east.
extern double headingOffset;

extern double targetDriverSpeedLeft;   // current left drive speed setpoint (driver loop)
extern double targetDriverSpeedRight;  // current right drive speed setpoint (driver loop)

// Per-motor acceleration state flags (one entry per drive motor per side).
extern bool isAcceleratingLeft[3];
extern bool isAcceleratingRight[3];

// ── Drive constants ───────────────────────────────────────────────────────────
extern const double numberDriveMotor;    // number of drive motors per side
extern const double accelerationFactor; // velocity ramp multiplier per loop tick
extern const double absoluteMaxRPM;     // free-spin RPM of the blue (600 RPM) cartridge
extern const double absoluteMaxVoltage; // motor voltage cap (V)
extern const double gearRatio;          // external gear ratio (motor shaft to wheel)
extern const double minLaunchPower;     // minimum power % to overcome static friction
extern const double VOLTAGE_TOLERANCE;  // minimum voltage delta worth correcting

// RPM scaling: drivetrain is geared to 400 RPM output; scale raw 600 RPM readings.
extern const double DRIVE_MOTOR_RPM_ADJ;

// ── Odometry geometry (centimeters) ──────────────────────────────────────────
extern const double TRACK_WIDTH;             // left-to-right tracking wheel span
extern const double ENCODER_OFFSET_X;        // lateral encoder offset from robot center
extern const double LEFT_ENCODER_OFFSET_Y;   // longitudinal offset, left encoder
extern const double RIGHT_ENCODER_OFFSET_Y;  // longitudinal offset, right encoder

// ── Wheel dimensions ──────────────────────────────────────────────────────────
extern const double wheelCircumferenceCM;         // drive wheel circumference (cm)
extern const double encoderWheelCircumferenceCM;  // tracking wheel circumference (cm)

// Ratio of drive half-track to encoder half-track — used in turning-radius math.
extern const double ENCODER_RADIUS_RATIO;

// ══════════════════════════════════════════════════════════════════════════════
// ARM POSITION ENUM
// ══════════════════════════════════════════════════════════════════════════════
// Named encoder tick positions for each arm state.
// Compared against pros::Motor::get_position() to detect current arm state.
enum ArmPosition {
    Starting        = 0,
    Load1           = 83,
    Load2           = 128,
    Hover           = 580,
    Side            = 700,
    Alliance        = 550,
    ScoringSide     = 514,
    ScoringAlliance = 334,
    Descore         = 437
};

// Tracks the arm's last commanded position (avoids redundant moves).
extern ArmPosition armstat;

// ══════════════════════════════════════════════════════════════════════════════
// MOTION PROFILE DEFAULT PARAMETERS
// ══════════════════════════════════════════════════════════════════════════════
// Tuned defaults for each motion type. Navigation functions use these as
// default argument values — call sites only need to pass what they change.
namespace MotionDefaults {

    namespace StraightForward {
        constexpr double BREAK_DISTANCE           = 35.0;  // cm before target to begin decel
        constexpr double MIN_SPEED                = 20.0;  // minimum speed % during approach
        constexpr double MAX_SPEED                = 100.0; // peak speed %
        constexpr double KP_HEADING               = 0.615; // proportional heading correction gain
        constexpr double KI_HEADING               = 0.0;
        constexpr double KD_HEADING               = 0.0;
        constexpr double ACCEL_HEADING_SCALING    = 0.10;  // heading correction weight during accel
        constexpr double DECEL_HEADING_SCALING    = 0.05;  // heading correction weight during decel
        constexpr double APPROACH_HEADING_SCALING = 0.05;  // heading correction weight in final approach
    }

    namespace StraightBackward {
        constexpr double BREAK_DISTANCE           = 30.0;
        constexpr double MIN_SPEED                = 20.0;
        constexpr double MAX_SPEED                = 80.0;
        constexpr double KP_HEADING               = 0.8;
        constexpr double KI_HEADING               = 0.0;
        constexpr double KD_HEADING               = 0.0;
        constexpr double ACCEL_HEADING_SCALING    = 0.08;
        constexpr double DECEL_HEADING_SCALING    = 0.06;
        constexpr double APPROACH_HEADING_SCALING = 0.06;
    }

    namespace TurningLeft {
        constexpr double BREAK_DISTANCE = 5.0;   // degrees before target to begin decel
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
// Call once from PROS initialize() before any competition mode begins.
// Configures motor directions, brake modes, encoder zeros, and IMU calibration.
void robotInit();

#endif // ROBOT_CONFIG_H