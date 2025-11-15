#include "robot_config.h"

// ========================================
// CORE SYSTEM COMPONENTS
// ========================================

vex::brain Brain;
vex::controller Controller;

// ========================================
// DRIVE MOTORS (6-Motor Tank Drive)
// ========================================

// Left side drive motors
vex::motor LeftMotor1 = vex::motor(vex::PORT7, vex::gearSetting::ratio6_1, true);
vex::motor LeftMotor2 = vex::motor(vex::PORT20, vex::gearSetting::ratio6_1);
vex::motor LeftMotor3 = vex::motor(vex::PORT8, vex::gearSetting::ratio6_1, true);

// Right side drive motors
vex::motor RightMotor1 = vex::motor(vex::PORT1, vex::gearSetting::ratio6_1);
vex::motor RightMotor2 = vex::motor(vex::PORT11, vex::gearSetting::ratio6_1, true);
vex::motor RightMotor3 = vex::motor(vex::PORT2, vex::gearSetting::ratio6_1);

// Motor arrays for convenient iteration and control
vex::motor leftMotor[] = {LeftMotor3, LeftMotor2, LeftMotor1};
vex::motor rightMotor[] = {RightMotor3, RightMotor2, RightMotor1};

// ========================================
// INTAKE SYSTEM
// ========================================

vex::motor intakeMotor1 = vex::motor(vex::PORT10, vex::gearSetting::ratio6_1);
vex::motor intakeMotor2 = vex::motor(vex::PORT9, vex::gearSetting::ratio6_1, true);

// ========================================
// PNEUMATICS SYSTEM
// ========================================

vex::pneumatics frontHoodPneumatics = vex::pneumatics(Brain.ThreeWirePort.G);
vex::pneumatics backHoodPneumatics = vex::pneumatics(Brain.ThreeWirePort.F);
vex::pneumatics matchLoadPneumatics = vex::pneumatics(Brain.ThreeWirePort.E);
vex::pneumatics ptoPneumatics = vex::pneumatics(Brain.ThreeWirePort.H);

// ========================================
// SENSORS
// ========================================

vex::inertial InertialSensor = vex::inertial(vex::PORT6);
vex::rotation passiveEncoderLeft = vex::rotation(vex::PORT13, true);
vex::rotation passiveEncoderRight = vex::rotation(vex::PORT18, true);
vex::rotation passiveEncoderX = vex::rotation(vex::PORT5, true);
vex::optical opticalSensor = vex::optical(vex::PORT12);
vex::bumper autonBumper = vex::bumper(Brain.ThreeWirePort.A);

// Vision system
vex::aivision::colordesc red1(1, 238, 44, 125, 20, 0.3);
vex::aivision visionSensor(vex::PORT6, red1);

// ========================================
// ROBOT CONFIGURATION CONSTANTS
// ========================================

// Drive system specifications
const double numberDriveMotor = 6;
const double absoluteMaxRPM = 600;
const double absoluteMaxVoltage = 12;
const double gearRatio = 6;
const double DRIVE_MOTOR_RPM_ADJ = 400.0 / 600.0;  // Actual drivetrain RPM vs motor cartridge RPM

// Physical dimensions (in cm)
const double TRACK_WIDTH = 31.0;                    // Distance between left and right wheel centers
const double wheelCircumferenceCM = 32.0;           // Drive wheel circumference
const double encoderWheelCircumferenceCM = 15.96;   // Tracking wheel circumference
const double ENCODER_RADIUS_RATIO = 15.25 / 8.3;    // Ratio for encoder radius correction

// Control parameters
const double VOLTAGE_TOLERANCE = 0.1;               // Voltage comparison tolerance
double headingOffset = 0.0;                         // Heading calibration offset

// ========================================
// INITIALIZATION FUNCTION
// ========================================

/**
 * Initialize robot configuration and calibrate sensors
 * Call this once at the start of main()
 */
void vexcodeInit(void)
{
    // Calibrate the inertial sensor
    InertialSensor.calibrate();
    Brain.Screen.printAt(10, 20, "Calibrating Inertial Sensor...");
    
    // Wait for calibration to complete
    while (InertialSensor.isCalibrating()) {
        vex::task::sleep(100);
    }
    
    // Reset heading to 0 degrees
    InertialSensor.resetHeading();
    Brain.Screen.printAt(10, 40, "Calibration Complete");
}
