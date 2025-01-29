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
extern vex::bumper armBumper;
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
extern const double DRIVE_MOTOR_RPM_ADJ;
extern const double ENCODER_RADIUS_RATIO;
extern const double TRACK_WIDTH;
extern const double ENCODER_OFFSET_X;  
extern const double LEFT_ENCODER_OFFSET_Y;  
extern const double RIGHT_ENCODER_OFFSET_Y;

//Declare Constants
extern const double wheelCircumferenceCM;
extern const double encoderWheelCircumferenceCM;

//original
enum ArmPosition {
    Starting = 0,     // Position 0
    Load1 = 189,  //85 original
    Load2 = 220,
    Ready = 420,
    Side = 590,   // Position -770 (Y button)
    Alliance = 770, // Position -550 (Right button)
    ScoringSide = 650,
    ScoringAlliance =470 
};


// Declare a variable to keep track of the arm's current position
extern ArmPosition armstat;


// Function to initialize the robot configuration
void vexcodeInit(void);

#endif



