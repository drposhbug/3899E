#include "robot_config.hpp"

// ==========================================
// PHYSICAL CONSTANTS (Definitions)
// ==========================================
const double numberDriveMotor = 6;
const double accelerationFactor = 1.05;
const double absoluteMaxRPM = 600;
const double absoluteMaxVoltage = 12000; // PROS uses millivolts
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
// MOTOR DEFINITIONS (PROS 4 Syntax)
// ==========================================

// NOTE: In PROS 4, gearsets are pros::v5::MotorGears::blue (600), green (200), red (100)
// Negative port numbers indicate reversed motors.

// LEFT MOTORS
// Ports: 8 (reversed), 19 (normal), 7 (reversed)
pros::Motor leftMotor1(-8, pros::v5::MotorGears::blue);
pros::Motor leftMotor2(19, pros::v5::MotorGears::blue);
pros::Motor leftMotor3(-7, pros::v5::MotorGears::blue);

// We recreate the group using ports to avoid object copy issues
pros::MotorGroup leftMotors({-8, 19, -7}, pros::v5::MotorGears::blue);

// RIGHT MOTORS
// Ports: 2 (normal), 11 (reversed), 1 (normal)
pros::Motor rightMotor1(2, pros::v5::MotorGears::blue);
pros::Motor rightMotor2(-11, pros::v5::MotorGears::blue);
pros::Motor rightMotor3(1, pros::v5::MotorGears::blue);

pros::MotorGroup rightMotors({2, -11, 1}, pros::v5::MotorGears::blue);

// INTAKE MOTORS
// Ports: 10 (reversed), 9 (normal)
pros::Motor intakeMotor1(-10, pros::v5::MotorGears::blue);
pros::Motor intakeMotor2(9, pros::v5::MotorGears::blue);

pros::MotorGroup intakeMotors({-10, 9}, pros::v5::MotorGears::blue);

// ==========================================
// PNEUMATICS (ADI)
// ==========================================
// Use pros::adi::DigitalOut instead of ADIDigitalOut
pros::adi::DigitalOut frontHoodPneumatics('G');
pros::adi::DigitalOut backHoodPneumatics('F');
pros::adi::DigitalOut matchLoadPneumatics('E');
pros::adi::DigitalOut ptoPneumatics('H');
pros::adi::DigitalOut wingPneumatics('C');

// ==========================================
// SENSORS & CONTROLLER
// ==========================================
pros::Imu inertialSensor(18);
pros::Optical opticalSensor(12);
pros::adi::DigitalIn autonBumper('A');

pros::Rotation passiveEncoderLeft(6);
pros::Rotation passiveEncoderRight(17);
pros::Rotation passiveEncoderX(5);

pros::Controller controller(pros::E_CONTROLLER_MASTER);

// ==========================================
// GLOBAL VARIABLES
// ==========================================
double headingOffset = 0.0;
double targetDriverSpeedLeft = 0.0;
double targetDriverSpeedRight = 0.0;
bool isAcceleratingLeft[3] = {false, false, false};
bool isAcceleratingRight[3] = {false, false, false};

// ==========================================
// INITIALIZATION FUNCTIONS
// ==========================================
void resetMotorPositions() {
    leftMotors.tare_position();
    rightMotors.tare_position();
}

void vexcodeInit(void) {
    // PROS 4 IMU Reset
    inertialSensor.reset();
    
    // Blocking wait for calibration
    int time = 0;
    while (inertialSensor.is_calibrating()) {
        pros::lcd::print(1, "Calibrating IMU... %d ms", time);
        pros::delay(20);
        time += 20;
    }
    pros::lcd::print(1, "IMU Calibrated");
    
    resetMotorPositions();
    passiveEncoderLeft.reset_position();
    passiveEncoderRight.reset_position();
    
    wingPneumatics.set_value(false);
}