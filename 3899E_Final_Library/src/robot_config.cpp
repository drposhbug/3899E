#include "robot_config.h" // Include the robot configuration header

// Define global instances of brain, controller, and motors
vex::brain Brain;
vex::controller Controller;
// Define Drive Motors
vex::motor LeftMotor1 = vex::motor(vex::PORT7, vex::gearSetting::ratio6_1,true);
vex::motor LeftMotor2 = vex::motor(vex::PORT19, vex::gearSetting::ratio6_1);
vex::motor LeftMotor3 = vex::motor(vex::PORT8, vex::gearSetting::ratio6_1, true);
vex::motor RightMotor1 = vex::motor(vex::PORT1, vex::gearSetting::ratio6_1);  // reversed
vex::motor RightMotor2 = vex::motor(vex::PORT11, vex::gearSetting::ratio6_1, true);  // reversed
vex::motor RightMotor3 = vex::motor(vex::PORT2, vex::gearSetting::ratio6_1); // reversed
// Define Intatke Motors
vex::motor intakeMotor1 = vex::motor(vex::PORT10, vex::gearSetting::ratio6_1, true);// reversed
vex::motor intakeMotor2 = vex::motor(vex::PORT9, vex::gearSetting::ratio6_1); 
// Define Pneumatics
vex::pneumatics frontHoodPneumatics = vex::pneumatics(Brain.ThreeWirePort.G);
vex::pneumatics backHoodPneumatics = vex::pneumatics(Brain.ThreeWirePort.F);
vex::pneumatics matchLoadPneumatics = vex::pneumatics(Brain.ThreeWirePort.E);
vex::pneumatics ptoPneumatics = vex::pneumatics(Brain.ThreeWirePort.H);
vex::pneumatics wingPneumatics = vex::pneumatics(Brain.ThreeWirePort.C);
vex::pneumatics indexPneumatics = vex::pneumatics(Brain.ThreeWirePort.B);
// Define Sensors
vex::inertial InertialSensor = vex::inertial(vex::PORT16);
vex::rotation passiveEncoderLeft = vex::rotation(vex::PORT20, true);  // Initialize the encoder on PORT10
vex::rotation passiveEncoderRight = vex::rotation(vex::PORT13, true); // Initialize the encoder on PORT10
vex::rotation passiveEncoderX = vex::rotation(vex::PORT12, true);     // Initialize the encoder on PORT10
vex::optical opticalSensor = vex::optical(vex::PORT15);
vex::bumper autonBumper = vex::bumper(Brain.ThreeWirePort.A);
// Global Variables
double robotStartingHeading = 0.0; 
double robotStartingHeadingStandard = 0.0;
double gyroReadingAtStart = 0.0;
double targetDriverSpeedLeft = 0.0;                  // Target speed for left motors (-100 to +100)
double targetDriverSpeedRight = 0.0;                 // Target speed for right motors (-100 to +100)
bool isAcceleratingLeft[3] = {false, false, false};  // Acceleration flags for left motors
bool isAcceleratingRight[3] = {false, false, false}; // Acceleration flags for right motors
// Define Separate Motor Arrays
vex::motor leftMotor[] = {LeftMotor3, LeftMotor2, LeftMotor1};
vex::motor rightMotor[] = {RightMotor3, RightMotor2, RightMotor1};
// Define Constants
// Coordinate system constants
const double numberDriveMotor = 6;
const double accelerationFactor = 1.05; // Adjust this factor globally
const double absoluteMaxRPM = 600;
const double absoluteMaxVoltage = 12;
const double gearRatio = 6;
const double VOLTAGE_TOLERANCE = 0.1;
const double minLaunchPower = 20;
const double DRIVE_MOTOR_RPM_ADJ = 400.0 / 600.0;     // Drivetrain geared to 400 RPM over 600 RPM motor cartridge
const double TRACK_WIDTH = 11.3;          // Distance between left/right encoders in cm
const double ENCODER_OFFSET_X = -0.023;      // X offset of tracking wheels from center (if not centered)
const double LEFT_ENCODER_OFFSET_Y = 0; // Y offset of tracking wheels from center
const double RIGHT_ENCODER_OFFSET_Y = 0;
const double wheelCircumferenceCM = 32.0;        // circumference of the motorized wheel in cm
const double encoderWheelCircumferenceCM = 15.96; // Circumference of the encoder wheel in cm
const double DISTANCE_TO_WHEEL = 15.25;           // distance between left and right wheels in cm
const double DISTANCE_TO_ENCODER = 8.3;
const double ENCODER_RADIUS_RATIO = DISTANCE_TO_WHEEL / DISTANCE_TO_ENCODER;    



// ========================================
// AI Vision Sensor Configuration
// ========================================

// Color signatures calibrated for our field conditions
vex::aivision::colordesc AIVision20__blueCube(1, 63, 130, 192, 20, 0.25);
//vex::aivision::colordesc AIVision20__orangeGoal(2, 151, 90, 35, 18, 0.81);
vex::aivision::colordesc AIVision20__orangeGoal(2, 213, 135, 100, 16, 0.31);
vex::aivision::colordesc AIVision20__redCube(3, 188, 17, 56, 40, 1);

// Push Back AI classification objects
vex::aivision::tagdesc AIVision20__blueBlock(1);  
vex::aivision::tagdesc AIVision20__redBlock(2);   

// Color codes for detecting combinations
vex::aivision::codedesc AIVision20__redLoad(1, AIVision20__orangeGoal, AIVision20__blueCube, AIVision20__redCube);
vex::aivision::codedesc AIVision20__blueLoad(2, AIVision20__orangeGoal, AIVision20__redCube, AIVision20__blueCube);

// Initialize sensor with all descriptors
vex::aivision AIVision20(vex::PORT14, 
                         AIVision20__blueCube,      
                         AIVision20__orangeGoal,    
                         AIVision20__redCube,       
                         AIVision20__redLoad,       
                         AIVision20__blueLoad,      
                         AIVision20__blueBlock,     
                         AIVision20__redBlock);

double headingOffset = 0.0;

// Function to reset motor positions
void resetMotorPositions()
{
    leftMotor[0].resetPosition();
    leftMotor[1].resetPosition();
    leftMotor[2].resetPosition();
    rightMotor[0].resetPosition();
    rightMotor[1].resetPosition();
    rightMotor[2].resetPosition();
}

// Initialization function
void vexcodeInit(void)
{
    // Calibrate the inertial sensor
    wingPneumatics.set(false);
    InertialSensor.calibrate();
    Brain.Screen.printAt(10, 20, "Calibrating Inertial Sensor...");
    while (InertialSensor.isCalibrating())
    {
        vex::task::sleep(100);
    }
    InertialSensor.resetHeading();
    Brain.Screen.printAt(10, 40, "Calibration Complete");
       
    wait(500, vex::msec);
    Brain.Screen.clearScreen();

    // Reset motor positions
    resetMotorPositions();
}
