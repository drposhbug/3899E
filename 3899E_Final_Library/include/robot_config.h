#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include "vex.h"

// ========================================
// CORE SYSTEM COMPONENTS
// ========================================

// Brain and controller instances
extern vex::brain Brain;
extern vex::controller Controller;

// ========================================
// DRIVE MOTORS (6-Motor Tank Drive)
// ========================================

// Individual drive motors
extern vex::motor LeftMotor1;
extern vex::motor LeftMotor2;
extern vex::motor LeftMotor3;
extern vex::motor RightMotor1;
extern vex::motor RightMotor2;
extern vex::motor RightMotor3;

// Motor arrays for easy iteration
extern vex::motor leftMotor[3];
extern vex::motor rightMotor[3];

// ========================================
// INTAKE SYSTEM
// ========================================

extern vex::motor intakeMotor1;
extern vex::motor intakeMotor2;

// ========================================
// PNEUMATICS SYSTEM
// ========================================

extern vex::pneumatics frontHoodPneumatics;
extern vex::pneumatics backHoodPneumatics;
extern vex::pneumatics matchLoadPneumatics;
extern vex::pneumatics ptoPneumatics;

// ========================================
// SENSORS
// ========================================

extern vex::inertial InertialSensor;
extern vex::rotation passiveEncoderLeft;
extern vex::rotation passiveEncoderRight;
extern vex::rotation passiveEncoderX;
extern vex::optical opticalSensor;
extern vex::bumper autonBumper;

// Vision system
extern vex::aivision::colordesc red1;
extern vex::aivision visionSensor;

// ========================================
// ROBOT CONFIGURATION CONSTANTS
// ========================================

// Drive system specifications
extern const double numberDriveMotor;
extern const double absoluteMaxRPM;
extern const double absoluteMaxVoltage;
extern const double gearRatio;
extern const double DRIVE_MOTOR_RPM_ADJ;

// Physical dimensions (in cm)
extern const double TRACK_WIDTH;
extern const double wheelCircumferenceCM;
extern const double encoderWheelCircumferenceCM;
extern const double ENCODER_RADIUS_RATIO;

// Control parameters
extern const double VOLTAGE_TOLERANCE;
extern double headingOffset;

// ========================================
// ARM POSITION DEFINITIONS
// ========================================

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

// ========================================
// INITIALIZATION FUNCTION
// ========================================

void vexcodeInit(void);

#endif // ROBOT_CONFIG_H
