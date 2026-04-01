#include "robot_config.h"

// ══════════════════════════════════════════════════════════════════════════════
// PORT MAP  (update this whenever hardware changes)
//
//  Smart Ports:
//     7   Left  Motor 1     (600 RPM, reversed)
//    19   Left  Motor 2     (600 RPM, forward)
//     6   Left  Motor 3     (600 RPM, reversed)
//     1   Right Motor 1     (600 RPM, forward)
//     5   Right Motor 2     (600 RPM, reversed)
//     2   Right Motor 3     (600 RPM, forward)
//    10   Intake Motor 1    (600 RPM, reversed)
//     9   Intake Motor 2    (600 RPM, forward)
//    17   IMU
//    20   Left  Encoder     (reversed)
//    13   Right Encoder     (not reversed)
//    12   X-axis Encoder    (reversed)
//    15   Optical Sensor    (sorting)
//    16   Left  Lane Optical
//    11   Right Lane Optical
//    14   AI Vision Sensor
//
//  ADI (3-wire) Ports:
//    A   Left Gate Pneumatic
//    C   Wing Pneumatic
//    D   Rudder Pneumatic
//    E   Match-Load Pneumatic / Auton Bumper  ← check physical wiring
//    F   Right Gate Pneumatic
//    G   Front Hood Pneumatic
//    H   PTO Pneumatic
//
// PROS sign convention: NEGATIVE port number = motor direction reversed.
// Both the individual pros::Motor objects AND the pros::MotorGroup entries
// must use the same signs so they agree on "forward."
// ══════════════════════════════════════════════════════════════════════════════

// ── Controller ────────────────────────────────────────────────────────────────
pros::Controller Controller(pros::E_CONTROLLER_MASTER);

// ── Drive motors (600 RPM blue cartridge) ─────────────────────────────────────
// Reversal per motor matches the physical mounting orientation on this robot.
pros::Motor LeftMotor1 (-7,  pros::MotorGears::blue);  // physically reversed
pros::Motor LeftMotor2 (19,  pros::MotorGears::blue);  // forward
pros::Motor LeftMotor3 (-6,  pros::MotorGears::blue);  // physically reversed
pros::Motor RightMotor1( 1,  pros::MotorGears::blue);  // forward
pros::Motor RightMotor2(-5,  pros::MotorGears::blue);  // physically reversed
pros::Motor RightMotor3( 2,  pros::MotorGears::blue);  // forward

// Drive motor groups — use these for synchronized drive commands.
// Port signs must mirror the individual motor definitions above.
pros::MotorGroup leftDrive ({-7, 19, -6}, pros::MotorGears::blue);
pros::MotorGroup rightDrive({ 1, -5,  2}, pros::MotorGears::blue);

// ── Mechanism motors (600 RPM blue cartridge) ─────────────────────────────────
pros::Motor intakeMotor1(-10, pros::MotorGears::blue);  // reversed
pros::Motor intakeMotor2( 9,  pros::MotorGears::blue);  // forward

// ── Pneumatics ────────────────────────────────────────────────────────────────
// pros::adi::DigitalOut::set_value(true) = extend solenoid,
//                         set_value(false) = retract.
pros::adi::DigitalOut frontHoodPneumatics ('G');
pros::adi::DigitalOut backHoodPneumatics  ('G');  // TODO: assign dedicated ADI port if re-added
pros::adi::DigitalOut matchLoadPneumatics ('E');
pros::adi::DigitalOut ptoPneumatics       ('H');
pros::adi::DigitalOut wingPneumatics      ('C');
pros::adi::DigitalOut indexPneumatics     ('C');  // TODO: assign dedicated ADI port if re-added
pros::adi::DigitalOut leftGatePneumatics  ('A');
pros::adi::DigitalOut rightGatePneumatics ('F');
pros::adi::DigitalOut rudderPneumatics    ('D');

// ── Sensors ───────────────────────────────────────────────────────────────────

// IMU — reset() with argument true blocks until calibration finishes.
pros::Imu InertialSensor(17);

// Passive odometry tracking wheels.
// Reversal is applied via set_reversed() in initialize() in main.cpp —
// pros::Rotation constructor in PROS 4 only accepts a port number.
pros::Rotation passiveEncoderLeft (20);
pros::Rotation passiveEncoderRight(13);
pros::Rotation passiveEncoderX    (12);

// Optical sensors for ring color sorting.
pros::Optical opticalSensor   (15);
pros::Optical leftLaneOptical (16);
pros::Optical rightLaneOptical(11);

// ── AI Vision Sensor ──────────────────────────────────────────────────────────
// NOTE: the original VEXcode code used vex::aivision with colordesc values in
// (hue, saturation, brightness) format.  PROS Vision signatures use a different
// YCbCr / UV color space.  The values below are PLACEHOLDERS — re-run the
// PROS Vision Sensor utility to calibrate U/V ranges for your field lighting.
//
// Signature format: id, u_min, u_max, u_mean, v_min, v_max, v_mean, range, type
pros::vision_signature_s_t aiVision_blueCube   = pros::Vision::signature_from_utility(
    1, -3600, -2800, -3200,  7000,  9000,  8000, 3.0, 0);  // TODO: recalibrate

pros::vision_signature_s_t aiVision_orangeGoal = pros::Vision::signature_from_utility(
    2,  3000,  5000,  4000, -1200,  -600,  -900, 3.0, 0);  // TODO: recalibrate

pros::vision_signature_s_t aiVision_redCube    = pros::Vision::signature_from_utility(
    3,  3500,  5500,  4500,  1000,  2500,  1750, 3.0, 0);  // TODO: recalibrate

// The AI Vision sensor itself (must be defined before color codes).
pros::Vision aiVision(14);

// Color codes — combinations of two color signatures detected together.
pros::vision_color_code_t aiVision_redLoad     = aiVision.create_color_code(1, 3, 0, 0, 0);
pros::vision_color_code_t aiVision_blueLoad    = aiVision.create_color_code(1, 1, 0, 0, 0);
pros::vision_color_code_t aiVision_blueRedBlue = aiVision.create_color_code(1, 3, 1, 0, 0);
pros::vision_color_code_t aiVision_redBlue     = aiVision.create_color_code(3, 1, 0, 0, 0);

// Tag detection objects — populated at runtime by aiVision.get_by_sig().
pros::vision_object_s_t aiVision_blueBlock = {};
pros::vision_object_s_t aiVision_redBlock  = {};

// ══════════════════════════════════════════════════════════════════════════════
// GLOBAL STATE VARIABLES
// ══════════════════════════════════════════════════════════════════════════════
double robotStartingHeading         = 0.0;
double robotStartingHeadingStandard = 0.0;
double gyroReadingAtStart           = 0.0;

// headingOffset: set in each auton to the alliance-relative starting angle.
// e.g. 240° for a red-side auto that starts 240° from field east.
double headingOffset = 0.0;

double targetDriverSpeedLeft  = 0.0;
double targetDriverSpeedRight = 0.0;

bool isAcceleratingLeft [3] = {false, false, false};
bool isAcceleratingRight[3] = {false, false, false};

// ── Drive constants ───────────────────────────────────────────────────────────
const double numberDriveMotor   = 6.0;
const double accelerationFactor = 1.05;  // speed ramp multiplier per loop tick
const double absoluteMaxRPM     = 600.0; // free-spin RPM of blue (600 RPM) cartridge
const double absoluteMaxVoltage = 12.0;  // volts — motion code works in 0..12 V units
const double gearRatio          = 6.0;   // external gear ratio (motor shaft to wheel)
const double minLaunchPower     = 20.0;  // minimum % to overcome static friction
const double VOLTAGE_TOLERANCE  =  0.1;  // V delta too small to act on

// DRIVE_MOTOR_RPM_ADJ: physical drivetrain is geared to 400 RPM output,
// so scale raw 600 RPM motor encoder readings by 400/600.
const double DRIVE_MOTOR_RPM_ADJ = 400.0 / 600.0;

// ── Odometry geometry (centimeters) ──────────────────────────────────────────
const double TRACK_WIDTH            = 11.30;   // left-to-right tracking wheel span
const double ENCODER_OFFSET_X       = -0.023;  // lateral encoder offset from robot center
const double LEFT_ENCODER_OFFSET_Y  =  0.0;    // longitudinal offset, left encoder
const double RIGHT_ENCODER_OFFSET_Y =  0.0;    // longitudinal offset, right encoder

// ── Wheel dimensions ──────────────────────────────────────────────────────────
const double wheelCircumferenceCM        = 32.00;  // drive wheel circumference
const double encoderWheelCircumferenceCM = 15.96;  // tracking wheel circumference

// Derived: ratio used in turning-radius calculations.
static const double DISTANCE_TO_WHEEL   = 15.25;  // half-track of drive wheels (cm)
static const double DISTANCE_TO_ENCODER =  8.30;  // half-track of tracking wheels (cm)
const double ENCODER_RADIUS_RATIO = DISTANCE_TO_WHEEL / DISTANCE_TO_ENCODER;

// ── Arm state tracker ─────────────────────────────────────────────────────────
ArmPosition armstat = ArmPosition::Starting;

// ══════════════════════════════════════════════════════════════════════════════
// ROBOT INITIALIZATION
// Call once from PROS initialize() before any competition mode starts.
// ══════════════════════════════════════════════════════════════════════════════

// Reset all drive encoder positions to zero.
static void resetMotorPositions()
{
    leftDrive .tare_position();
    rightDrive.tare_position();
}

void robotInit()
{
    // Retract wing pneumatics so they don't interfere during calibration.
    wingPneumatics.set_value(false);

    // Calibrate the IMU — reset(true) blocks until finished (~2 s).
    pros::lcd::set_text(1, "Calibrating IMU...");
    InertialSensor.reset(true);
    InertialSensor.tare_rotation();
    pros::lcd::set_text(1, "IMU ready");

    // Register AI Vision color signatures with the sensor.
    aiVision.set_signature(1, &aiVision_blueCube);
    aiVision.set_signature(2, &aiVision_orangeGoal);
    aiVision.set_signature(3, &aiVision_redCube);

    pros::delay(500);
    pros::lcd::clear();

    // Zero all drive motor encoders.
    resetMotorPositions();
}
