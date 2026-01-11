#include "robot_config.hpp"

// Define controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// ==========================================
// MOTOR DEFINITIONS (Port Only to avoid errors)
// ==========================================

// Left Drive
pros::Motor leftMotor1(7);
pros::Motor leftMotor2(19);
pros::Motor leftMotor3(8);

// Right Drive
pros::Motor rightMotor1(1);
pros::Motor rightMotor2(11);
pros::Motor rightMotor3(2);

// Intake
pros::Motor intakeMotor1(10);
pros::Motor intakeMotor2(9);

// ==========================================
// PNEUMATICS & SENSORS
// ==========================================

// Pneumatics
pros::adi::DigitalOut frontHoodPneumatics(7);
pros::adi::DigitalOut backHoodPneumatics(6);
pros::adi::DigitalOut matchLoadPneumatics(5);
pros::adi::DigitalOut ptoPneumatics(8);
pros::adi::DigitalOut wingPneumatics(3);

// Sensors
pros::Imu inertialSensor(18);

// Rotation Sensors (Port only)
pros::Rotation passiveEncoderLeft(6);
pros::Rotation passiveEncoderRight(17);
pros::Rotation passiveEncoderX(5);

pros::Optical opticalSensor(12);
pros::adi::DigitalIn autonBumper(1);

// ==========================================
// GLOBALS
// ==========================================

double targetDriverSpeedLeft = 0.0;
double targetDriverSpeedRight = 0.0;
bool isAcceleratingLeft[3] = {false, false, false};
bool isAcceleratingRight[3] = {false, false, false};

// Define Separate Motor Arrays (pointers to motors)
pros::Motor* leftMotor[3] = {&leftMotor3, &leftMotor2, &leftMotor1};
pros::Motor* rightMotor[3] = {&rightMotor3, &rightMotor2, &rightMotor1};

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

/*
void vexcodeInit(void)
{
    // 1. Configure Motor Gearing (Blue Cartridge = 600 RPM)
    leftMotor1.set_gearing(pros::E_MOTOR_GEAR_600);
    leftMotor2.set_gearing(pros::E_MOTOR_GEAR_600);
    leftMotor3.set_gearing(pros::E_MOTOR_GEAR_600);
    rightMotor1.set_gearing(pros::E_MOTOR_GEAR_600);
    rightMotor2.set_gearing(pros::E_MOTOR_GEAR_600);
    rightMotor3.set_gearing(pros::E_MOTOR_GEAR_600);
    intakeMotor1.set_gearing(pros::E_MOTOR_GEAR_600);
    intakeMotor2.set_gearing(pros::E_MOTOR_GEAR_600);

    // 2. Configure Motor Directions
    leftMotor1.set_reversed(true);
    leftMotor2.set_reversed(false);
    leftMotor3.set_reversed(true);

    rightMotor1.set_reversed(false);
    rightMotor2.set_reversed(true);
    rightMotor3.set_reversed(false);

    intakeMotor1.set_reversed(true);
    intakeMotor2.set_reversed(false);

    // 3. Configure Sensor Directions
    passiveEncoderLeft.set_reversed(true);
    passiveEncoderRight.set_reversed(true);
    passiveEncoderX.set_reversed(true);

    // 4. Initialize Hardware
    wingPneumatics.set_value(false);
    
    inertialSensor.reset();
    pros::lcd::print(0, "Calibrating Inertial Sensor...");
    
    while (inertialSensor.is_calibrating())
    {
        pros::delay(100);
    }
    
    inertialSensor.set_heading(0);
    pros::lcd::print(1, "Calibration Complete");
    pros::delay(500);
    pros::lcd::clear();

    resetMotorPositions();
}
    */

  void vexcodeInit(void)
{
    pros::lcd::print(0, "Init: Motors...");
    pros::delay(200);
    
    // 1. Configure Motor Gearing
    leftMotor1.set_gearing(pros::E_MOTOR_GEAR_600);
    leftMotor2.set_gearing(pros::E_MOTOR_GEAR_600);
    leftMotor3.set_gearing(pros::E_MOTOR_GEAR_600);
    rightMotor1.set_gearing(pros::E_MOTOR_GEAR_600);
    rightMotor2.set_gearing(pros::E_MOTOR_GEAR_600);
    rightMotor3.set_gearing(pros::E_MOTOR_GEAR_600);
    intakeMotor1.set_gearing(pros::E_MOTOR_GEAR_600);
    intakeMotor2.set_gearing(pros::E_MOTOR_GEAR_600);

    pros::lcd::print(0, "Init: Directions...");
    pros::delay(200);
    
    // 2. Configure Motor Directions
    leftMotor1.set_reversed(true);
    leftMotor2.set_reversed(false);
    leftMotor3.set_reversed(true);
    rightMotor1.set_reversed(false);
    rightMotor2.set_reversed(true);
    rightMotor3.set_reversed(false);
    intakeMotor1.set_reversed(true);
    intakeMotor2.set_reversed(false);

    pros::lcd::print(0, "Init: Encoders...");
    pros::delay(200);
    
    // 3. Configure Sensor Directions
    passiveEncoderLeft.set_reversed(true);
    passiveEncoderRight.set_reversed(true);
    passiveEncoderX.set_reversed(true);

    pros::lcd::print(0, "Init: Pneumatics...");
    pros::delay(200);
    
    // 4. Initialize Hardware
    wingPneumatics.set_value(false);
    
    pros::lcd::print(0, "Init: IMU Reset...");
    pros::delay(200);
    
    // 5. IMU
    inertialSensor.reset();
    
    pros::lcd::print(0, "Init: IMU Calibrating...");
    
    while (inertialSensor.is_calibrating()) {
        pros::delay(100);
    }
    
    pros::lcd::print(0, "Init: IMU Set Heading...");
    pros::delay(200);
    
    inertialSensor.set_heading(0);
    
    pros::lcd::print(0, "Init: Reset Positions...");
    pros::delay(200);
    
    resetMotorPositions();
    
    pros::lcd::print(0, "Init: COMPLETE!");
    pros::delay(500);
    pros::lcd::clear();
}