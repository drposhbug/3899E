#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include "vex.h" // Include the VEX library

// Declare external instances of brain, controller, and motors
extern vex::brain Brain;
extern vex::controller Controller;
extern vex::motor armMotor;

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
extern vex::motor intakeMotor;
//extern vex::pneumatics Pneumatics;
extern vex::pneumatics goalPneumatics;
extern vex::pneumatics elbow1Pneumatics;
extern vex::pneumatics elbow2Pneumatics;
extern vex::pneumatics doinkerPneumatics;
extern vex::pneumatics armPneumatics;
extern vex::limit armBumper; 
extern vex::inertial InertialSensor;
extern vex::aivision visionSensor;
extern vex::aivision::colordesc red1;  // Declare the red descriptor
extern vex::rotation passiveEncoderLeft; // Declare the passive encoder sensor
extern vex::rotation passiveEncoderRight; // Declare the passive encoder sensor
extern vex::rotation passiveEncoderX; // Declare the passive encoder sensor
extern vex::optical opticalSensor;

//Declare Global Variable
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
extern const double trackWidth;
extern const double DRIVE_MOTOR_RPM_ADJ;

//Declare Constants
extern const double wheelCircumferenceCM;
extern const double encoderWheelCircumferenceCM;

/*
enum ArmPosition {
    Starting = -65,     // Position 0
    Load = 20,  //85 original
    Ready = 115,
    Side = 300,   // Position -770 (Y button)
    Alliance = 405, // Position -550 (Right button)
    ScoringSide = 405,
    ScoringAlliance =525 
};
*/




//original
enum ArmPosition {
    Starting = 0,     // Position 0
    Load1 = 220,  //85 original
    Load2 = 280,
    Ready = 300,
    Side = 385,   // Position -770 (Y button)
    Alliance = 470, // Position -550 (Right button)
    ScoringSide = 570,
    ScoringAlliance =670 
};


/*
//compensate for auton
enum ArmPosition {
    Starting = -30,     // Position 0
    Load = 35,   // Position -770 (Y button)
    Alliance = 450, // Position -550 (Right button)
    Side = 300,   // Position -770 (Y button)
    ScoringSide = 440,
    ScoringAlliance =570 
};
*/

// Declare a variable to keep track of the arm's current position
extern ArmPosition armstat;

//need to delete and cleanup
// Enum definitions
//enum ArmMotorPosition {
//    ARM_STARTING = 0,
//    ARM_SCORE_BOT = -300 // Example value, adjust as necessary
//};

enum ElbowMotorPosition {
    ELBOW_STARTING = 0,
    ELBOW_SCORE_BOT = -250 // Example value, adjust as necessary
};

// Function to initialize the robot configuration
void vexcodeInit(void);

#endif



