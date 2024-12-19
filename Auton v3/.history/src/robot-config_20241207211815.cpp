#include "robot-config.h" // Include the robot configuration header

// Define global instances of brain, controller, and motors
vex::brain Brain;
vex::controller Controller;
vex::motor LeftMotor1 = vex::motor(vex::PORT18, vex::gearSetting::ratio6_1);
vex::motor LeftMotor2 = vex::motor(vex::PORT19, vex::gearSetting::ratio6_1);
vex::motor LeftMotor3 = vex::motor(vex::PORT20, vex::gearSetting::ratio6_1);
vex::motor RightMotor1 = vex::motor(vex::PORT8, vex::gearSetting::ratio6_1, true); // reversed
vex::motor RightMotor2 = vex::motor(vex::PORT9, vex::gearSetting::ratio6_1, true); // reversed
vex::motor RightMotor3 = vex::motor(vex::PORT10, vex::gearSetting::ratio6_1, true); // reversed
vex::motor intakeMotor = vex::motor(vex::PORT7, vex::gearSetting::ratio6_1, false);
vex::motor armMotor = vex::motor(vex::PORT17);
vex::pneumatics goalPneumatics = vex::pneumatics(Brain.ThreeWirePort.B);
vex::pneumatics elbow1Pneumatics = vex::pneumatics(Brain.ThreeWirePort.B);
vex::pneumatics elbow2Pneumatics = vex::pneumatics(Brain.ThreeWirePort.D);
vex::pneumatics doinkerPneumatics = vex::pneumatics(Brain.ThreeWirePort.A);
vex::pneumatics intakePneumatics = vex::pneumatics(Brain.ThreeWirePort.H);
vex::inertial InertialSensor = vex::inertial(vex::PORT15);
vex::rotation passiveEncoderLeft = vex::rotation(vex::PORT16, true); // Initialize the encoder on PORT10
vex::rotation passiveEncoderRight = vex::rotation(vex::PORT6, true); // Initialize the encoder on PORT10
vex::optical opticalSensor = vex::optical(vex::PORT11);

// Global Variables
double targetDriverSpeedLeft = 0.0;   // Target speed for left motors (-100 to +100)
double targetDriverSpeedRight = 0.0;  // Target speed for right motors (-100 to +100)
bool isAcceleratingLeft[3] = {false, false, false};    // Acceleration flags for left motors
bool isAcceleratingRight[3] = {false, false, false};   // Acceleration flags for right motors
// Separate Motor Arrays
//vex::motor leftMotor[] = {LeftMotor1, LeftMotor2, LeftMotor3};
//vex::motor rightMotor[] = {RightMotor1, RightMotor2, RightMotor3};

vex::motor leftMotor[] = {LeftMotor3, LeftMotor2, LeftMotor1};
vex::motor rightMotor[] = {RightMotor3, RightMotor2, RightMotor1};
const double accelerationFactor = 1.1;  // Adjust this factor globally
const double maxRPM = 600;
const double maxVoltage = 12;
const double gearRatio = 6;
const double minLaunchPower = 20;

// Define the vision sensor signatures
vex::aivision::colordesc red1(1, 238, 44, 125, 20, 0.3);
vex::aivision visionSensor(vex::PORT6, red1);

//Define Constants
const double wheelCircumferenceCM = 15.9593; // circumference of the motorized wheel in cm
const double encoderWheelCircumferenceCM = 15.9593; // Circumference of the encoder wheel in cm

ArmPosition armstat = ArmPosition::Starting;

void resetMotorPositions() {
    //armMotor.setPosition(ARM_STARTING, vex::rotationUnits::deg);
    //elbowMotor.setPosition(ELBOW_STARTING, vex::rotationUnits::deg);
}

// Initialization function
void vexcodeInit(void) {
  // Calibrate the inertial sensor
  InertialSensor.calibrate();
  Brain.Screen.printAt(10, 20, "Calibrating Inertial Sensor...");
  while (InertialSensor.isCalibrating()) {
    vex::task::sleep(100); // Wait for calibration to complete
  }
  InertialSensor.resetHeading(); // Reset heading to 0
  Brain.Screen.printAt(10, 40, "Calibration Complete");

    // Reset motor positions
  resetMotorPositions();
}

