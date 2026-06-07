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
// intakeMotor1/2: 11W V5 Smart Motor, blue cartridge (600 RPM).
extern pros::Motor intakeMotor1;  // port 10, reversed
extern pros::Motor intakeMotor2;  // port  9, forward

// hoodMotor: 5.5W V5 Smart Motor (port 8).
// Fixed at 200 RPM — no swappable cartridge.  Declared with pros::MotorGears::green
// (18:1 enum) so velocity-API scaling matches the hardware's fixed internal ratio.
// Always commanded in lock-step with intakeMotor1/2 via move_voltage(±12000).
extern pros::Motor hoodMotor;     // port  8, forward

// upperIndexerMotor: 5.5W V5 Smart Motor (port 4).
// Same hardware config as hoodMotor — fixed 200 RPM, pros::MotorGears::green.
// Runs opposite direction to hoodMotor so both 5.5W motors pull together through
// the indexer path.  Always commanded in lock-step with the other intake motors.
extern pros::Motor upperIndexerMotor;  // port  4, reversed

// ══════════════════════════════════════════════════════════════════════════════
// PNEUMATICS
// ══════════════════════════════════════════════════════════════════════════════
// set_value(true) = solenoid extended, set_value(false) = retracted.
extern pros::adi::DigitalOut frontHoodPneumatics;  // ADI port G — front hood
extern pros::adi::DigitalOut matchLoadPneumatics;  // ADI port E — match loader / auton bumper
extern pros::adi::DigitalOut ptoPneumatics;        // ADI port H — PTO engagement
extern pros::adi::DigitalOut wingPneumatics;       // ADI port C — wing extension
extern pros::adi::DigitalOut leftGatePneumatics;   // ADI port A — left scoring gate
extern pros::adi::DigitalOut rightGatePneumatics;  // ADI port F — right scoring gate
extern pros::adi::DigitalOut rudderPneumatics;     // ADI port D — intake rudder

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
extern pros::Optical leftLaneOptical;
extern pros::Optical rightLaneOptical;

// GPS Sensor — port 3, left side mount
// Offset from tracking center: x = -0.1524m (6" left), y = 0.0m (centered)
// Mount optical window at 9.5" (24.1cm) height — same as field GPS strips.
extern pros::Gps gpsSensor;

// Maximum GPS position error (meters) to accept a reset.
// get_error() returns estimated RMS error — lower = higher confidence.
// 0.10m = 10cm; tighten after field calibration.
#define GPS_MAX_ERROR_M  0.10

// ══════════════════════════════════════════════════════════════════════════════
// AI VISION SENSOR
// ══════════════════════════════════════════════════════════════════════════════
// Uses pros::AIVision — NOT the older pros::Vision sensor API.
// Colors defined from VEX aivision::colordesc values and pushed to sensor in robotInit().
// Detection type: AivisionModeType::colors (color blob detection).
//
// COLOR1 (id=1): red cube   — r=146 g=27  b=79  hue_range=21  sat=0.6
// COLOR2 (id=2): blue cube  — r=59  g=91  b=170 hue_range=19  sat=0.28
// COLOR3 (id=3): orange cap  — match loader cap; placeholder, tune on field
// COLOR4 (id=4): orange base — long goal base; different shade, tune separately
extern pros::AIVision::Color aiVision_redCube;
extern pros::AIVision::Color aiVision_blueCube;
extern pros::AIVision::Color aiVision_orangeCap;   // match loader cap
extern pros::AIVision::Color aiVision_orangeBase;  // long goal base

// The AI Vision sensor itself (port 14).
extern pros::AIVision aiVision;

// ══════════════════════════════════════════════════════════════════════════════
// GLOBAL ROBOT STATE
// ══════════════════════════════════════════════════════════════════════════════
extern double robotStartingHeading;          // heading at match start (degrees)
extern double robotStartingHeadingStandard;  // heading at match start — VEX Coordinates (name kept for compatibility)
extern double gyroReadingAtStart;            // IMU value recorded at init (tare reference)

// Applied to IMU readings to set alliance-relative reference.
// e.g. 90° for a red-side auto starting with the robot facing East (90° from North).
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

// pros::Rotation::get_velocity() returns centidegrees/second, not RPM.
// Multiply by this to convert to RPM: cdeg/s ÷ 100 ÷ 360 × 60 = cdeg/s ÷ 600.
extern const double ROTATION_CDEG_TO_RPM;

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
// Moved to motion_config.h — single source of truth for all motion defaults.
// ══════════════════════════════════════════════════════════════════════════════

// ══════════════════════════════════════════════════════════════════════════════
// ROBOT INITIALIZATION
// ══════════════════════════════════════════════════════════════════════════════
// Call once from PROS initialize() before any competition mode begins.
// Configures motor directions, brake modes, encoder zeros, and IMU calibration.
void robotInit();

#endif // ROBOT_CONFIG_H