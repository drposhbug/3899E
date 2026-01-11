#ifndef ROBOT_CONFIG_HPP
#define ROBOT_CONFIG_HPP

#include "pros/apix.h"
#include "pros/motors.hpp"
#include "pros/imu.hpp"
#include "pros/rotation.hpp"
#include "pros/adi.hpp"
#include "pros/optical.hpp"
#include "pros/misc.hpp"

// Declare external instances of controller
extern pros::Controller controller;

// Declare arrays for the left and right motors
extern pros::Motor leftMotor1;
extern pros::Motor leftMotor2;
extern pros::Motor leftMotor3;
extern pros::Motor rightMotor1;
extern pros::Motor rightMotor2;
extern pros::Motor rightMotor3;
extern pros::Motor* leftMotor[3];
extern pros::Motor* rightMotor[3];

// Declare intake motors
extern pros::Motor intakeMotor1;
extern pros::Motor intakeMotor2;

// Declare Pneumatics (Using modern pros::adi namespace)
extern pros::adi::DigitalOut frontHoodPneumatics;
extern pros::adi::DigitalOut backHoodPneumatics;
extern pros::adi::DigitalOut matchLoadPneumatics;
extern pros::adi::DigitalOut ptoPneumatics;
extern pros::adi::DigitalOut wingPneumatics;

// Declare Sensors
extern pros::Imu inertialSensor;
extern pros::Rotation passiveEncoderLeft;
extern pros::Rotation passiveEncoderRight;
extern pros::Rotation passiveEncoderX;
extern pros::Optical opticalSensor;
extern pros::adi::DigitalIn autonBumper;

// Declare Global Variables
extern double targetDriverSpeedLeft;
extern double targetDriverSpeedRight;
extern bool isAcceleratingLeft[3];
extern bool isAcceleratingRight[3];
extern const double numberDriveMotor;
extern const double accelerationFactor;
extern const double absoluteMaxRPM;
extern const double absoluteMaxVoltage;
extern const double gearRatio;
extern const double minLaunchPower;
extern double headingOffset;
extern const double DRIVE_MOTOR_RPM_ADJ;
extern const double ENCODER_RADIUS_RATIO;
extern const double TRACK_WIDTH;
extern const double ENCODER_OFFSET_X;
extern const double LEFT_ENCODER_OFFSET_Y;
extern const double RIGHT_ENCODER_OFFSET_Y;

// Declare Constants
extern const double wheelCircumferenceCM;
extern const double encoderWheelCircumferenceCM;
extern const double VOLTAGE_TOLERANCE;

// Arm Position Enum
enum ArmPosition {
    Starting = 0,
    Load1 = 83,
    Load2 = 128,
    Hover = 580,
    Side = 700,
    Alliance = 550,
    ScoringSide = 514,
    ScoringAlliance = 334,
    Descore = 437
};

// Declare a variable to keep track of the arm's current position
extern ArmPosition armstat;

// Function to initialize the robot configuration
void vexcodeInit(void);

// Function to reset motor positions
void resetMotorPositions();

// Motion Profile Default Parameters
namespace MotionDefaults {
    namespace StraightForward {
        constexpr double BREAK_DISTANCE = 35.0;
        constexpr double MIN_SPEED = 20.0;
        constexpr double MAX_SPEED = 100.0;
        constexpr double KP_HEADING = 0.615;
        constexpr double KI_HEADING = 0.0;
        constexpr double KD_HEADING = 0.0;
        constexpr double ACCEL_HEADING_SCALING = 0.10;
        constexpr double DECEL_HEADING_SCALING = 0.05;
        constexpr double APPROACH_HEADING_SCALING = 0.05;
    }

    namespace StraightBackward {
        constexpr double BREAK_DISTANCE = 30.0;
        constexpr double MIN_SPEED = 20.0;
        constexpr double MAX_SPEED = 80.0;
        constexpr double KP_HEADING = 0.8;
        constexpr double KI_HEADING = 0.0;
        constexpr double KD_HEADING = 0.0;
        constexpr double ACCEL_HEADING_SCALING = 0.08;
        constexpr double DECEL_HEADING_SCALING = 0.06;
        constexpr double APPROACH_HEADING_SCALING = 0.06;
    }

    namespace TurningLeft {
        constexpr double BREAK_DISTANCE = 5.0;
        constexpr double MIN_SPEED = 20.0;
        constexpr double MAX_SPEED = 100.0;
    }

    namespace TurningRight {
        constexpr double BREAK_DISTANCE = 5.0;
        constexpr double MIN_SPEED = 20.0;
        constexpr double MAX_SPEED = 100.0;
    }
}

#endif