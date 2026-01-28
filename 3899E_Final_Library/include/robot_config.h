#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include "vex.h"

// Declare external instances of brain, controller, and motors
extern vex::brain Brain;
extern vex::controller Controller;

// Declare arrays for the left and right motors
extern vex::motor LeftMotor1;
extern vex::motor LeftMotor2;
extern vex::motor LeftMotor3;
extern vex::motor RightMotor1;
extern vex::motor RightMotor2;
extern vex::motor RightMotor3;
extern vex::motor leftMotor[3];
extern vex::motor rightMotor[3];
// Declare motors as extern so they can be accessed globally
extern vex::motor intakeMotor1;
extern vex::motor intakeMotor2;
// Declare Pneumatics
extern vex::pneumatics frontHoodPneumatics;
extern vex::pneumatics backHoodPneumatics;
extern vex::pneumatics matchLoadPneumatics;
extern vex::pneumatics ptoPneumatics;
extern vex::pneumatics wingPneumatics;
// Declare Sensors
extern vex::inertial InertialSensor;

extern vex::rotation passiveEncoderLeft;
extern vex::rotation passiveEncoderRight;
extern vex::rotation passiveEncoderX;
extern vex::optical opticalSensor;
extern vex::bumper autonBumper;

// Declare Global Variables
extern double robotStartingHeading; //
extern double robotStartingHeadingStandard; 
extern double gyroReadingAtStart;   // The "Tare" value of the sensor
extern double targetDriverSpeedLeft;
extern double targetDriverSpeedRight;
extern bool isAcceleratingLeft[3];
extern bool isAcceleratingRight[3];
// Separate Motor Arrays
extern vex::motor leftMotors[3];
extern vex::motor rightMotors[3];
extern const double numberDriveMotor; 
extern const double accelerationFactor;
extern const double absoluteMaxRPM;
extern const double absoluteMaxVoltage;
extern const double gearRatio;
extern const double minLaunchPower;
extern double robotStartingHeading;
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

// Arm positions
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

// AI Vision Sensor Configuration
extern vex::aivision::colordesc AIVision20__blueCube;
extern vex::aivision::colordesc AIVision20__orangeGoal;
extern vex::aivision::colordesc AIVision20__redCube;

extern vex::aivision::codedesc AIVision20__redLoad;
extern vex::aivision::codedesc AIVision20__blueLoad;

extern vex::aivision AIVision20;

#endif