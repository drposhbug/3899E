#include "robot_config.hpp"

// Define controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// ==========================================
// MOTOR DEFINITIONS (Port Only to avoid errors)
// ==========================================

// Left Drive
pros::Motor LeftMotor1(7);
pros::Motor LeftMotor2(19);
pros::Motor LeftMotor3(8);

// Right Drive
pros::Motor RightMotor1(1);
pros::Motor RightMotor2(11);
pros::Motor RightMotor3(2);

// Intake
pros::Motor intakeMotor1(10);
pros::Motor intakeMotor2(9);

// ==========================================
// PNEUMATICS & SENSORS
// ==========================================

// Pneumatics
pros::adi::DigitalOut frontHoodPneumatics('G');
pros::adi::DigitalOut backHoodPneumatics('F');
pros::adi::DigitalOut matchLoadPneumatics('E');
pros::adi::DigitalOut ptoPneumatics('H');
pros::adi::DigitalOut wingPneumatics('C');

// Sensors
pros::Imu InertialSensor(18);

// Rotation Sensors (Port only)
pros::Rotation passiveEncoderLeft(6);
pros::Rotation passiveEncoderRight(17);
pros::Rotation passiveEncoderX(5);

pros::Optical opticalSensor(12);
pros::adi::DigitalIn autonBumper('A');

// ==========================================
// GLOBALS
// ==========================================

double targetDriverSpeedLeft = 0.0;
double targetDriverSpeedRight = 0.0;
bool isAcceleratingLeft[3] = {false, false, false};
bool isAcceleratingRight[3] = {false, false, false};

// Define Separate Motor Arrays (pointers to motors)
pros::Motor* leftMotor[3] = {&LeftMotor3, &LeftMotor2, &LeftMotor1};
pros::Motor* rightMotor[3] = {&RightMotor3, &RightMotor2, &RightMotor1};

// Constants
const double numberDriveMotor = 6;
const double accelerationFactor = 1.05;
const double absoluteMaxRPM = 600;
const double absoluteMaxVoltage = 12;
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

double headingOffset = 0.0;

// ==========================================
// FUNCTIONS
// ==========================================

void resetMotorPositions()
{
    leftMotor[0]->tare_position();
    leftMotor[1]->tare_position();
    leftMotor[2]->tare_position();
    rightMotor[0]->tare_position();
    rightMotor[1]->tare_position();
    rightMotor[2]->tare_position();
}

void vexcodeInit(void)
{
    // 1. Configure Motor Gearing (Blue Cartridge = 600 RPM)
    LeftMotor1.set_gearing(pros::E_MOTOR_GEAR_600);
    LeftMotor2.set_gearing(pros::E_MOTOR_GEAR_600);
    LeftMotor3.set_gearing(pros::E_MOTOR_GEAR_600);
    RightMotor1.set_gearing(pros::E_MOTOR_GEAR_600);
    RightMotor2.set_gearing(pros::E_MOTOR_GEAR_600);
    RightMotor3.set_gearing(pros::E_MOTOR_GEAR_600);
    intakeMotor1.set_gearing(pros::E_MOTOR_GEAR_600);
    intakeMotor2.set_gearing(pros::E_MOTOR_GEAR_600);

    // 2. Configure Motor Directions
    LeftMotor1.set_reversed(true);
    LeftMotor2.set_reversed(false);
    LeftMotor3.set_reversed(true);

    RightMotor1.set_reversed(false);
    RightMotor2.set_reversed(true);
    RightMotor3.set_reversed(false);

    intakeMotor1.set_reversed(true);
    intakeMotor2.set_reversed(false);

    // 3. Configure Sensor Directions
    passiveEncoderLeft.set_reversed(true);
    passiveEncoderRight.set_reversed(true);
    passiveEncoderX.set_reversed(true);

    // 4. Initialize Hardware
    wingPneumatics.set_value(false);
    
    InertialSensor.reset();
    pros::lcd::print(0, "Calibrating Inertial Sensor...");
    
    while (InertialSensor.is_calibrating())
    {
        pros::delay(100);
    }
    
    InertialSensor.set_heading(0);
    pros::lcd::print(1, "Calibration Complete");
    pros::delay(500);
    pros::lcd::clear();

    resetMotorPositions();
}