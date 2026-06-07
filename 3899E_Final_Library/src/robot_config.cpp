#include "robot_config.h"
#include <cstring>

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

std::int8_t rightMotor1Port = 5; 
std::int8_t rightMotor2Port = 9;
std::int8_t rightMotor3Port = 11;

std::int8_t leftMotor1Port = -18;
std::int8_t leftMotor2Port = -14;
std::int8_t leftMotor3Port = -20;

std::int8_t intakeMotorPort = -19;
std::int8_t leverPort = -10; 
std::int8_t colorSortMotorPort = 3;

std::int8_t horizontalEncoderPort = 2;
std::int8_t verticalEncoderPort = 4;
std::int8_t imuPort = 17; 
std::int8_t colorSensorPort = 15;
std::int8_t gpsSensorPort = 17; // not real
std::int8_t aiVisionFrontPort = 12;
std::int8_t aiVisionBackPort = 8;
std::int8_t aiVisionPort = 21;

std::int8_t receiverPort = 6;
pros::Link* receiver;
float messageReceived[5] = {0,0,0,0,0};
float messageToSend[5] = {0,0,0,0,0};
std::string LINK_ID = "theThingThatMustBeTheSameAcrossBothBigBotAndSmallBot2055A"; // don't change this
pros::Mutex radioMutex;
bool matchloaderState = false, scoreFlapState = false, scorePistonState = false;

char matchloaderPort = 'A';
char scoreFlapPort = 'H';
char colorSortFlapPort = 'B';
char scorePistonPort = 'B';


// ── Controller ────────────────────────────────────────────────────────────────
pros::Controller Controller(pros::E_CONTROLLER_MASTER);

// ── Drive motors (600 RPM blue cartridge) ─────────────────────────────────────
// Negative port = physically reversed motor; matches mounting orientation.
pros::Motor LeftMotor1 (leftMotor1Port,  pros::MotorGears::blue);
pros::Motor LeftMotor2 (leftMotor2Port,  pros::MotorGears::blue);
pros::Motor LeftMotor3 (leftMotor3Port,  pros::MotorGears::blue);
pros::Motor RightMotor1(rightMotor1Port,  pros::MotorGears::blue);
pros::Motor RightMotor2(rightMotor2Port,  pros::MotorGears::blue);
pros::Motor RightMotor3(rightMotor3Port,  pros::MotorGears::blue);

// Drive motor groups — port signs must mirror individual motor definitions above.
pros::MotorGroup leftDrive ({leftMotor1Port, leftMotor2Port, leftMotor3Port}, pros::MotorGears::blue);
pros::MotorGroup rightDrive({ rightMotor1Port, rightMotor2Port,  rightMotor3Port}, pros::MotorGears::blue);

// ── Mechanism motors (600 RPM blue cartridge) ─────────────────────────────────
pros::Motor intakeMotor(intakeMotorPort, pros::MotorGears::blue);  // reversed
pros::Motor lever(leverPort, pros::MotorGears::blue);
pros::Motor colorSortMotor(colorSortMotorPort,  pros::MotorGears::blue);  // forward

// ── Pneumatics ────────────────────────────────────────────────────────────────
// set_value(true) = solenoid extended, set_value(false) = retracted.
pros::adi::DigitalOut matchloader(matchloaderPort);

pros::adi::DigitalOut scoreFlap(scoreFlapPort);

pros::adi::DigitalOut colorSortFlap(colorSortFlapPort);

pros::adi::DigitalOut scorePiston(scorePistonPort);

// ── Sensors ───────────────────────────────────────────────────────────────────

// IMU — reset(true) blocks until calibration completes (~2 s).
pros::Imu InertialSensor(imuPort);

// Passive odometry tracking wheels.
// Reversal is applied via set_reversed() in initialize() — the pros::Rotation
// constructor only accepts a port number in PROS 4.
pros::Rotation passiveEncoderLeft   (verticalEncoderPort);
pros::Rotation passiveEncoderRight  (verticalEncoderPort);
pros::Rotation passiveEncoderX      (horizontalEncoderPort);

// Optical sensors for ring color sorting and lane detection.
pros::Optical opticalSensor   (colorSensorPort);

// GPS Sensor -- doesn't actually exist (yet).
pros::GPS gpsSensor (gpsSensorPort);

pros::AIVision aiVision_Front(aiVisionFrontPort);
pros::AIVision aiVision_Back(aiVisionBackPort);
pros::AIVision aiVision(aiVisionPort);

// ── AI Vision Sensor ──────────────────────────────────────────────────────────
// Color descriptors converted from VEX aivision::colordesc format:
//   aivision::colordesc(id, r, g, b, hue_range, saturation)
// PROS AIVision::Color uses identical fields in the same order.
pros::AIVision::Color aiVision_redCube  = {.id=1, .red=167, .green=29,  .blue=70,  .hue_range=16.0, .saturation_range=0.38};
pros::AIVision::Color aiVision_blueCube = {.id=2, .red=31,  .green=69,  .blue=115, .hue_range=17.0, .saturation_range=0.42};

// AI Vision sensor — port 14.
// Colors pushed to sensor and detection enabled in robotInit().
// pros::AIVision aiVision(14);

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
const double DRIVE_MOTOR_RPM_ADJ = 400.0 / 600.0; // ????

// pros::Rotation::get_velocity() returns centidegrees/second.
// Convert: cdeg/s ÷ 100 (→ deg/s) ÷ 360 (→ rev/s) × 60 (→ RPM) = cdeg/s ÷ 600.
const double ROTATION_CDEG_TO_RPM = 1.0 / 600.0;

// ── Odometry geometry (centimeters) ──────────────────────────────────────────
const double TRACK_WIDTH            = 25.40;   // left-to-right tracking wheel span
const double ENCODER_OFFSET_X       = -13.573125;  // lateral encoder offset from robot center
const double LEFT_ENCODER_OFFSET_Y  =  1.031875;    // longitudinal offset, left encoder
const double RIGHT_ENCODER_OFFSET_Y =  1.031875;    // longitudinal offset, right encoder

// ── Wheel dimensions ──────────────────────────────────────────────────────────
const double wheelCircumferenceCM        = 25.93;  // drive wheel circumference (cm)
const double encoderWheelCircumferenceCM = 15.96;  // tracking wheel circumference (cm)

// Half-track distances used in turning-radius calculations.
static const double DISTANCE_TO_WHEEL   = 12.70;  // half-track of drive wheels (cm)
static const double DISTANCE_TO_ENCODER =  0.00;  // half-track of tracking wheels (cm)

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

// Radio function
void runRadio(void* param) {
    while (true) {
        if (receiver != nullptr) {
            radioMutex.take(TIMEOUT_MAX);
			receiver->receive(&messageReceived, sizeof(messageReceived));
			receiver->clear_receive_buf();
			receiver->transmit(&messageToSend, sizeof(messageToSend));
            radioMutex.give();
		}
        pros::delay(50);
    }
}

void getReceivedMessage(float out[5]) { // mutexes
    radioMutex.take(TIMEOUT_MAX);
    memcpy(out, messageReceived, sizeof(messageReceived));
    radioMutex.give();
}

void setMessageToSend(float newMessage[5]) {
    radioMutex.take(TIMEOUT_MAX);
    memcpy(messageToSend, newMessage, sizeof(newMessage));
    radioMutex.give();
}


// ══════════════════════════════════════════════════════════════════════════════
// ROBOT INITIALIZATION
// ══════════════════════════════════════════════════════════════════════════════
// Call once from PROS initialize() before any competition mode begins.
void robotInit()
{

    // Calibrate the IMU — reset(true) blocks until finished (~2 s).
    pros::lcd::set_text(1, "Calibrating IMU...");
    InertialSensor.reset(true);
    InertialSensor.tare_rotation();
    pros::delay(500);  // wait for tare to settle
    gyroReadingAtStart = InertialSensor.get_rotation();     // capture baseline
    pros::lcd::set_text(1, "IMU ready");

    // Push color descriptors to the AI Vision sensor and enable color detection.
    aiVision_Front.reset();
    aiVision_Front.set_color(aiVision_redCube);
    aiVision_Front.set_color(aiVision_blueCube);
    aiVision_Front.enable_detection_types(pros::AivisionModeType::colors);

    pros::delay(500);
    pros::lcd::clear();

    // Zero all drive motor encoders after calibration.
    resetMotorPositions();

    // start radio link
    receiver = new pros::Link(receiverPort, LINK_ID, pros::E_LINK_RECIEVER, true);
    pros::Task radioTask(runRadio);
}

// ── Dual AI Vision Sensors ───────────────────────────────────────────────────
// We use pros::AiVision (not pros::Vision) and pass your exact VEXcode values.
// Format: (port, signature 1, signature 2, signature 3...)
