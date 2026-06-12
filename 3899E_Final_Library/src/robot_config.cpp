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
//     4   GPS Sensor        (right side mount, 13.5cm right of center, centered lengthwise)
//    16   Sort Motor        (200 RPM, 5.5W — colour sort flipper, no cartridge, forward)
//    17   IMU
//    20   Left  Encoder     (reversed)
//    13   Right Encoder     (not reversed)
//    12   X-axis Encoder    (not reversed — set_reversed(false) in initialize())
//     4   Optical Sensor    (colour sort — port 3, single sensor for testing)
//    16   Left  Lane Optical  [REMOVED — port now used by sort motor]
//    11   Right Lane Optical  [REMOVED — port conflict with hood motor]
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
pros::Motor intakeMotor1(10, pros::MotorGears::blue);  // port 10, reversed
pros::Motor intakeMotor2( -9,  pros::MotorGears::blue);  // port  9, forward

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
pros::Motor lowerIndexerMotor(16, pros::MotorGears::green); // port 16, reversed
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

// GPS Sensor — port 4
// x_offset: -0.135m = 13.5cm right of tracking center (right = negative in PROS robot frame)
// y_offset:  0.0m   = centered lengthwise
pros::Gps gpsSensor(4, -0.135, 0.0);

// Passive odometry tracking wheels.
// Left/Right reversal set via set_reversed() in initialize().
// X encoder reversal set here in constructor — reversed=true corrects physical mounting direction.
pros::Rotation passiveEncoderLeft (20);
pros::Rotation passiveEncoderRight(13);
pros::Rotation passiveEncoderX    (12);  

// Colour sort optical sensor — port 3.
// Single-sensor configuration for initial field testing.
// Second sensor (failsafe) added later once primary is validated.
pros::Optical opticalSensor(3);

// Colour sort flipper motor — port 16, 5.5W, no swappable cartridge (fixed 200 RPM).
// pros::MotorGears::green matches the internal fixed ratio for velocity API scaling.
// Held in brake HOLD after each sort so the flipper stays at its last position.
pros::Motor sortMotor(16, pros::MotorGears::green);

// ── AI Vision Sensor ──────────────────────────────────────────────────────────
// Color descriptors converted from VEX aivision::colordesc format:
//   aivision::colordesc(id, r, g, b, hue_range, saturation)
// PROS AIVision::Color uses identical fields in the same order.
pros::AIVision::Color aiVision_redCube  = {.id=1, .red=146, .green=27,  .blue=79,  .hue_range=21.0, .saturation_range=0.6};
pros::AIVision::Color aiVision_blueCube = {.id=2, .red=59,  .green=91,  .blue=170, .hue_range=19.0, .saturation_range=0.28};
// Orange cap on match loader post — placeholder values, tune on field.
// Typical orange: high red, mid green, low blue. Narrow hue_range avoids
// false matches on other field elements. Adjust all values after testing.
pros::AIVision::Color aiVision_orangeCap  = {.id=3, .red=210, .green=100, .blue=10,  .hue_range=15.0, .saturation_range=0.7};
// Long goal base — different shade of orange from the cap; tune RGB separately on field.
pros::AIVision::Color aiVision_orangeBase = {.id=4, .red=198, .green=151,  .blue=133,   .hue_range=40, .saturation_range=0.29};

// AI Vision sensor — port 14.
// Colors pushed to sensor and detection enabled in robotInit().
pros::AIVision aiVision(14);

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

// pros::Rotation::get_velocity() returns centidegrees/second.
// Convert: cdeg/s ÷ 100 (→ deg/s) ÷ 360 (→ rev/s) × 60 (→ RPM) = cdeg/s ÷ 600.
const double ROTATION_CDEG_TO_RPM = 1.0 / 600.0;

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

    // Colour sort motor — hold position after each sort move.
    sortMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    sortMotor.tare_position();

    // Colour sort optical — LED on at full power for reliable hue reads.
    opticalSensor.set_led_pwm(100);

    // Push color descriptors to the AI Vision sensor and enable color detection.
    aiVision.set_color(aiVision_redCube);
    aiVision.set_color(aiVision_blueCube);
    aiVision.set_color(aiVision_orangeCap);
    aiVision.set_color(aiVision_orangeBase);
    aiVision.enable_detection_types(pros::AivisionModeType::colors);

    pros::delay(500);
    pros::lcd::clear();

    // Zero all drive motor encoders after calibration.
    resetMotorPositions();
}