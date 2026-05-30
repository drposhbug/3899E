#include "robot_config.h"

// ══════════════════════════════════════════════════════════════════════════════
// PORT MAP  (update whenever hardware changes)
//
//  Smart Ports:
//     7   Left  Motor 1     (600 RPM, reversed)
//    19   Left  Motor 2     (600 RPM, forward)
//     6   Left  Motor 3     (600 RPM, reversed)
//     1   Right Motor 1     (600 RPM, forward)
//     5   Right Motor 2     (600 RPM, reversed)
//     2   Right Motor 3     (600 RPM, forward)
//    10   Intake Motor 1    (600 RPM, 11W, reversed)
//     9   Intake Motor 2    (600 RPM, 11W, forward)
//    11   Hood Motor        (200 RPM, 5.5W — fixed speed, no cartridge, forward)
//    15   Upper Indexer     (200 RPM, 5.5W — fixed speed, no cartridge, reversed)
//     3   GPS Sensor        (left side mount)
//    17   IMU
//    20   Left  Encoder     (reversed)
//    13   Right Encoder     (not reversed)
//    12   X-axis Encoder    (reversed)
//     4   Optical Sensor    (sorting)
//    16   Left  Lane Optical
//    11   Right Lane Optical
//    14   AI Vision Sensor
//
//  ADI (3-wire) Ports:
//    A   Left Gate Pneumatic
//    C   Wing Pneumatic
//    D   Rudder Pneumatic
//    E   Match-Load Pneumatic / Auton Bumper
//    F   Right Gate Pneumatic
//    G   Front Hood Pneumatic
//    H   PTO Pneumatic
//
// PROS sign convention: NEGATIVE port number = motor direction reversed.
// Individual pros::Motor objects and pros::MotorGroup entries must use the
// same signs so they agree on "forward."
// ══════════════════════════════════════════════════════════════════════════════

// ── Controller ────────────────────────────────────────────────────────────────
pros::Controller Controller(pros::E_CONTROLLER_MASTER);

// ── Drive motors (600 RPM blue cartridge) ─────────────────────────────────────
// Negative port = physically reversed motor; matches mounting orientation.
pros::Motor LeftMotor1 (-7,  pros::MotorGears::blue);
pros::Motor LeftMotor2 (19,  pros::MotorGears::blue);
pros::Motor LeftMotor3 (-6,  pros::MotorGears::blue);
pros::Motor RightMotor1( 1,  pros::MotorGears::blue);
pros::Motor RightMotor2(-5,  pros::MotorGears::blue);
pros::Motor RightMotor3( 2,  pros::MotorGears::blue);

// Drive motor groups — port signs must mirror individual motor definitions above.
pros::MotorGroup leftDrive ({-7, 19, -6}, pros::MotorGears::blue);
pros::MotorGroup rightDrive({ 1, -5,  2}, pros::MotorGears::blue);

// ── Mechanism motors ──────────────────────────────────────────────────────────
// intakeMotor1/2: 11W V5 Smart Motor, blue cartridge (600 RPM).
pros::Motor intakeMotor1(-10, pros::MotorGears::blue);  // port 10, reversed
pros::Motor intakeMotor2( 9,  pros::MotorGears::blue);  // port  9, forward

// hoodMotor: 5.5W V5 Smart Motor (port 11).
// This motor has NO swappable cartridge — speed is fixed at 200 RPM by hardware.
// pros::MotorGears::green (18:1 enum) matches the motor's internal fixed ratio so
// that velocity-based API calls scale correctly.  move_voltage() is unaffected by
// gearset and drives the motor at full power regardless.
pros::Motor hoodMotor(11, pros::MotorGears::green);      // port 11, forward (negate to reverse)

// upperIndexerMotor: 5.5W V5 Smart Motor (port 15).
// Same hardware config as hoodMotor — fixed 200 RPM, pros::MotorGears::green.
// Runs in the OPPOSITE direction to hoodMotor (port negated) so both motors
// pull game objects through the indexer path together.
pros::Motor upperIndexerMotor(-15, pros::MotorGears::green); // port 15, reversed

// ── Pneumatics ────────────────────────────────────────────────────────────────
// set_value(true) = solenoid extended, set_value(false) = retracted.
pros::adi::DigitalOut frontHoodPneumatics ('G');
pros::adi::DigitalOut matchLoadPneumatics ('E');
pros::adi::DigitalOut ptoPneumatics       ('H');
pros::adi::DigitalOut wingPneumatics      ('C');
pros::adi::DigitalOut leftGatePneumatics  ('A');
pros::adi::DigitalOut rightGatePneumatics ('F');
pros::adi::DigitalOut rudderPneumatics    ('D');

// ── Sensors ───────────────────────────────────────────────────────────────────

// IMU — reset(true) blocks until calibration completes (~2 s).
pros::Imu InertialSensor(17);

// GPS Sensor — port 3
// x_offset: -0.1524m = 6 inches left of tracking center
// y_offset:  0.0m    = centered lengthwise
pros::Gps gpsSensor(3, -0.1524, 0.0);

// Passive odometry tracking wheels.
// Reversal is applied via set_reversed() in initialize() — the pros::Rotation
// constructor only accepts a port number in PROS 4.
pros::Rotation passiveEncoderLeft (20);
pros::Rotation passiveEncoderRight(13);
pros::Rotation passiveEncoderX    (12);

// Optical sensors for ring color sorting and lane detection.
pros::Optical opticalSensor   (15);
pros::Optical leftLaneOptical (16);
pros::Optical rightLaneOptical(11);

// ── AI Vision Sensor ──────────────────────────────────────────────────────────
// PROS Vision signatures use YCbCr / UV color space — NOT HSV.
// Values below are PLACEHOLDERS; re-run the PROS Vision Sensor utility to
// calibrate U/V ranges for your specific field lighting conditions.
//
// Signature format: id, u_min, u_max, u_mean, v_min, v_max, v_mean, range, type
pros::vision_signature_s_t aiVision_blueCube   = pros::Vision::signature_from_utility(
    1, -3600, -2800, -3200,  7000,  9000,  8000, 3.0, 0);  // TODO: recalibrate

pros::vision_signature_s_t aiVision_orangeGoal = pros::Vision::signature_from_utility(
    2,  3000,  5000,  4000, -1200,  -600,  -900, 3.0, 0);  // TODO: recalibrate

pros::vision_signature_s_t aiVision_redCube    = pros::Vision::signature_from_utility(
    3,  3500,  5500,  4500,  1000,  2500,  1750, 3.0, 0);  // TODO: recalibrate

// AI Vision sensor (port 14) — must be defined before color codes.
pros::Vision aiVision(14);

// Color codes — multi-signature patterns detected together.
pros::vision_color_code_t aiVision_redLoad     = aiVision.create_color_code(1, 3, 0, 0, 0);
pros::vision_color_code_t aiVision_blueLoad    = aiVision.create_color_code(1, 1, 0, 0, 0);
pros::vision_color_code_t aiVision_blueRedBlue = aiVision.create_color_code(1, 3, 1, 0, 0);
pros::vision_color_code_t aiVision_redBlue     = aiVision.create_color_code(3, 1, 0, 0, 0);

// Detection result objects — written to by aiVision.get_by_sig() at runtime.
pros::vision_object_s_t aiVision_blueBlock = {};
pros::vision_object_s_t aiVision_redBlock  = {};

// ══════════════════════════════════════════════════════════════════════════════
// GLOBAL STATE VARIABLES
// ══════════════════════════════════════════════════════════════════════════════
double robotStartingHeading         = 0.0;
double robotStartingHeadingStandard = 0.0;
double gyroReadingAtStart           = 0.0;

// Set in each auton to the alliance-relative starting angle.
// e.g. 90° for a red-side auto starting with the robot facing East (90° from North).
double headingOffset = 0.0;

double targetDriverSpeedLeft  = 0.0;
double targetDriverSpeedRight = 0.0;

bool isAcceleratingLeft [3] = {false, false, false};
bool isAcceleratingRight[3] = {false, false, false};

// ── Drive constants ───────────────────────────────────────────────────────────
const double numberDriveMotor   = 6.0;
const double accelerationFactor = 1.05;   // speed ramp multiplier per loop tick
const double absoluteMaxRPM     = 600.0;  // free-spin RPM of the blue cartridge
const double absoluteMaxVoltage = 12.0;   // voltage cap — motion code works in 0..12 V
const double gearRatio          = 6.0;    // external gear ratio (motor shaft to wheel)
const double minLaunchPower     = 20.0;   // minimum % to overcome static friction
const double VOLTAGE_TOLERANCE  =  0.1;   // V delta too small to act on

// Drivetrain is geared to 400 RPM output; scale raw 600 RPM encoder readings.
const double DRIVE_MOTOR_RPM_ADJ = 400.0 / 600.0;

// ── Odometry geometry (centimeters) ──────────────────────────────────────────
const double TRACK_WIDTH            = 11.30;   // left-to-right tracking wheel span
const double ENCODER_OFFSET_X       = -0.023;  // lateral encoder offset from robot center
const double LEFT_ENCODER_OFFSET_Y  =  0.0;    // longitudinal offset, left encoder
const double RIGHT_ENCODER_OFFSET_Y =  0.0;    // longitudinal offset, right encoder

// ── Wheel dimensions ──────────────────────────────────────────────────────────
const double wheelCircumferenceCM        = 32.00;  // drive wheel circumference (cm)
const double encoderWheelCircumferenceCM = 15.96;  // tracking wheel circumference (cm)

// Half-track distances used in turning-radius calculations.
static const double DISTANCE_TO_WHEEL   = 15.25;  // half-track of drive wheels (cm)
static const double DISTANCE_TO_ENCODER =  8.30;  // half-track of tracking wheels (cm)

// Derived ratio — drive half-track to encoder half-track.
const double ENCODER_RADIUS_RATIO = DISTANCE_TO_WHEEL / DISTANCE_TO_ENCODER;

// ── Arm state tracker ─────────────────────────────────────────────────────────
ArmPosition armstat = ArmPosition::Starting;

// ══════════════════════════════════════════════════════════════════════════════
// INTERNAL HELPERS
// ══════════════════════════════════════════════════════════════════════════════

// Zero all drive motor encoder positions.
static void resetMotorPositions()
{
    leftDrive .tare_position();
    rightDrive.tare_position();
}

// ══════════════════════════════════════════════════════════════════════════════
// ROBOT INITIALIZATION
// ══════════════════════════════════════════════════════════════════════════════
// Call once from PROS initialize() before any competition mode begins.
void robotInit()
{
    // Retract wings so they don't interfere during IMU calibration.
    wingPneumatics.set_value(false);

    // Calibrate the IMU — reset(true) blocks until finished (~2 s).
    pros::lcd::set_text(1, "Calibrating IMU...");
    InertialSensor.reset(true);
    InertialSensor.tare_rotation();
    pros::delay(500);  // wait for tare to settle
    gyroReadingAtStart = InertialSensor.get_rotation();     // capture baseline
    pros::lcd::set_text(1, "IMU ready");

    // Register color signatures with the AI Vision sensor.
    aiVision.set_signature(1, &aiVision_blueCube);
    aiVision.set_signature(2, &aiVision_orangeGoal);
    aiVision.set_signature(3, &aiVision_redCube);

    pros::delay(500);
    pros::lcd::clear();

    // Zero all drive motor encoders after calibration.
    resetMotorPositions();
}