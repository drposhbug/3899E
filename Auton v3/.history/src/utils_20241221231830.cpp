#include "vex.h"
#include "utils.h"
#include <cmath> // Include the header for fmod function
#include "robot-config.h" 

using namespace vex;


/*
double normalizeHeading(double heading) {
    heading = fmod(heading + 180.0, 360.0) - 180.0; // Wrap the angle within [-180, 180)
    return heading;
}
*/

double normalizeHeading(double heading) {
    // Add 180 to the heading and take the modulus with 360 to wrap the angle.
    // This ensures the result is between [0, 360) degrees.
    heading = fmod(heading + 180.0, 360.0);

    // If the result is negative (e.g., due to fmod returning a negative value),
    // add 360 to shift it into the [0, 360) range.
    if (heading < 0)
        heading += 360.0;

    // Subtract 180 to convert the range from [0, 360) to [-180, 180).
    heading -= 180.0;
    
    // Ensure 180 stays as 180, and -180 stays as -180.
    if (heading == -180.0)
        heading = 180.0;

    // Return the normalized heading value.
    return heading;
}

double normaHeading(double currentHeading, double normTargetHeading) {
    // Add 180 to the heading and take the modulus with 360 to wrap the angle.
    // This ensures the result is between [0, 360) degrees.
    currentHeading = fmod(currentHeading + 180.0, 360.0);

    // If the result is negative (e.g., due to fmod returning a negative value),
    // add 360 to shift it into the [0, 360) range.
    if (currentHeading < 0)
        currentHeading += 360.0;

    // Subtract 180 to convert the range from [0, 360) to [-180, 180).
    currentHeading -= 180.0;
    
    // Ensure 180 stays as 180, and -180 stays as -180.
// Calculate the difference
    double difference = normTargetHeading - normCurrentHeading;

    // Normalize the difference to [-180, 180)
    if (difference <= -180.0) {
        difference += 360.0;
    }
    if (difference > 180.0) {
        difference -= 360.0;
    }

    // Return the normalized heading value.
    return heading;
}

// Function to calculate wheel diameter in centimeters and wheel circumference
void getWheelProperties(double &wheelCircumferenceCm, double &gearRatio) {
    double wheelDiameterInches = 3.25; // in inches
    double wheelDiameterCm = wheelDiameterInches * 2.54; // convert to cm
    wheelCircumferenceCm = wheelDiameterCm * M_PI; // in cm
    gearRatio = 6.0; // 6:1 gear ratio
}

// Implementation of the spinToPosition function for armMotor
//void spinArmToPosition(int position, int power) {
//    armMotor.spinToPosition(position, rotationUnits::deg, power, velocityUnits::pct);
//}

// Implementation of the spinToPosition function for elbowMotor
void spinElbowToPosition(int position, int power) {
 //   elbowMotor.spinToPosition(position, rotationUnits::deg, power, velocityUnits::pct);
}

 // Implement slip detection logic
bool isSlipping(double motorSpeed, double encoderSpeed) {
    const double slipThreshold = 0.1;
    return (motorSpeed > encoderSpeed * (1 + slipThreshold)); 
}

// Implement lock-up detection logic
bool isLocking(double motorSpeed, double encoderSpeed) {
    const double lockThreshold = .85; // 10% threshold
    return (motorSpeed < encoderSpeed * (1.0 - lockThreshold));
}

// Get encoder speed
//double getEncoderSpeed(vex::rotation& encoder) {
    // The number 60 is used to convert RPM (revolutions per minute) to RPS (revolutions per second)
  //  return (encoder.velocity(vex::velocityUnits::rpm) / 60.0) * wheelCircumferenceCM;
//}


//Colour Detection Function

// Calibration Constants for Color Detection
const double RED_HUE_MIN_1 = 344.0;  // First part of the red range (344°-360°)
const double RED_HUE_MAX_1 = 360.0;
const double RED_HUE_MIN_2 = 0.0;    // Second part of the red range (0°-10°)
const double RED_HUE_MAX_2 = 10.0;
const double BLUE_HUE_MIN = 217.0;
const double BLUE_HUE_MAX = 222.0;

// Brightness Thresholds for Red and Blue Detection (Adjust if needed)
const double RED_BRIGHTNESS_THRESHOLD = 10.0;   // Minimum brightness to detect red
const double BLUE_BRIGHTNESS_THRESHOLD = 10.0;  // Minimum brightness to detect blue

// Function to initialize the Optical Sensor
void initializeOpticalSensor() {
  opticalSensor.setLightPower(100, percent);  // Turn on the sensor light at 100% power
  opticalSensor.setLight(ledState::on);       // Ensure the light is on
}

typedef void (*ActionFunction)();  // Define a type for the action function pointer

// Function to check for the specified color and perform the action
void checkColor(ColorType colorToDetect, ActionFunction action) {
  static bool redDetected = false;
  static bool blueDetected = false;

  // Get the current hue and brightness from the Optical Sensor
  double hue = opticalSensor.hue();
  double brightness = opticalSensor.brightness();

  // Define detection conditions for red and blue with hue and brightness
  bool isRed = ((hue >= RED_HUE_MIN_1 && hue <= RED_HUE_MAX_1) || 
                (hue >= RED_HUE_MIN_2 && hue <= RED_HUE_MAX_2)) && 
                brightness >= RED_BRIGHTNESS_THRESHOLD;
  bool isBlue = (hue >= BLUE_HUE_MIN && hue <= BLUE_HUE_MAX) && 
                brightness >= BLUE_BRIGHTNESS_THRESHOLD;

  // Detect and perform action based on the specified color
  if ((colorToDetect == RED && isRed && !redDetected) || 
      (colorToDetect == BLUE && isBlue && !blueDetected) || 
      (colorToDetect == ANY && (isRed || isBlue))) {

    if (colorToDetect == RED) {
      Brain.Screen.print("Red detected");
      redDetected = true;
      blueDetected = false;
    }
    else if (colorToDetect == BLUE) {
      Brain.Screen.print("Blue detected");
      blueDetected = true;
      redDetected = false;
    }
    else if (colorToDetect == ANY) {
      Brain.Screen.print(isRed ? "Red detected" : "Blue detected");
      redDetected = isRed;
      blueDetected = isBlue;
    }

    Brain.Screen.newLine();
    
    // Call the specified action
    if (action != nullptr) {
      action();
    }
  }
  // No Color Detected
  else if (!isRed && !isBlue) {
    if (redDetected || blueDetected) {
      Brain.Screen.clearLine();
      Brain.Screen.print("No color detected");
      Brain.Screen.newLine();
    }
    redDetected = false;
    blueDetected = false;
  }
}


// Handle the ejection process
void ringEjection() {
    // Spin forward by 720 degrees (2 rotations) at 100% velocity
    intakeMotor.spinFor(forward, 720, rotationUnits::deg, 100, velocityUnits::pct);
    
    // Spin in reverse by 180 degrees to eject the ring
    intakeMotor.spinFor(reverse, 180, rotationUnits::deg, 100, velocityUnits::pct);
    
    // Resume forward intake at 12 volts
    intakeMotor.spin(forward, 12, voltageUnits::volt);
}

void armLoadConveyor() {
    // Spin forward by 720 degrees (2 rotations) at 100% velocity
    intakeMotor.spinFor(forward, 300, rotationUnits::deg, 20, velocityUnits::pct);
    
    // Spin in reverse by 180 degrees to eject the ring
    intakeMotor.spinFor(reverse, 200, rotationUnits::deg, 50, velocityUnits::pct);
    
    // Resume forward intake at 12 volts
    intakeMotor.spin(forward, 12, voltageUnits::volt);
}

// Generic function to control any motor
// Function to control any motor
void MotorControl(motor& targetMotor, int DelayStart, int OnTime, directionType dir) {
    task::sleep(DelayStart); // Wait before starting
    targetMotor.spin(dir, 12, voltageUnits::volt); // Spin motor
    task::sleep(OnTime); // Keep spinning
    targetMotor.stop(); // Stop motor
}

// Wrapper function matching the expected thread signature
int MotorControlThread(void* params) {
    MotorControlParams* mcParams = static_cast<MotorControlParams*>(params);
    MotorControl(*mcParams->targetMotor, mcParams->DelayStart, mcParams->OnTime, mcParams->dir);
    return 0; // Return value as required by thread signature
}


bool isAccelerating(double targetDriverSpeed, double currentSpeed) {
    // Clear and set the cursor for printing target and current speeds
    //Brain.Screen.clearLine(6);
    //Brain.Screen.setCursor(6, 1);
    //Brain.Screen.print("Target Speed: %.2f, Current Speed: %.2f", targetDriverSpeed, currentSpeed);
    
    // If both speeds are in the same direction
    if ((targetDriverSpeed * currentSpeed) > 0) {
       // Brain.Screen.clearLine(8);
        //Brain.Screen.setCursor(8, 1);
        // Added parentheses to ensure correct evaluation of the ternary operator
        //Brain.Screen.print("Same Direction. Target > Current: %s", 
        //                   (fabs(targetDriverSpeed) > fabs(currentSpeed)) ? "true" : "false");

        // Check if the target speed is greater than the current speed
        return fabs(targetDriverSpeed) > fabs(currentSpeed);
    }
    // If the speeds are in opposite directions
    else if ((targetDriverSpeed * currentSpeed) < 0) {

        // Moving from positive to negative or vice versa is still a sign of acceleration
        return true;
    }

    // If both are zero, or no acceleration
    return false;
}



// Function to calculate motor speed in cm per second using a constant circumference
double getMotorSpeed(vex::motor& motor) {
    // Get motor velocity in RPM and convert to cm/s using the constant circumference
    return motor.velocity(vex::velocityUnits::rpm) / gearRatio * wheelCircumferenceCM / 60.0;
}

// Function to calculate motor encoder speed in cm per second
double getEncoderSpeed(vex::rotation& encoder) {
    return encoder.velocity(vex::velocityUnits::rpm) * encoderWheelCircumferenceCM / 60.0;
}

// Function to convert motor power from joystick (-100 to 100) into speed, cm per second
double MotorPowerToSpeed(double motorPower) {
    // Convert percentage power to RPM
    double rpm = (motorPower / 100.0) * absoluteMaxRPM;
    
    // Convert RPM to cm/s (RPM * circumference / 60)
    double speedCMperSec = rpm * wheelCircumferenceCM / 60.0;
    
    return speedCMperSec;
}

double SpeedToMotorPower(double speedCMperSec) {
    // Convert cm/s to RPM (speedCMperSec * 60 / circumference)
    double rpm = (speedCMperSec * 60.0) / wheelCircumferenceCM;
    
    // Convert RPM to percentage motor power
    double motorPower = (rpm / absoluteMaxRPM) * 100.0;
    
    return motorPower;
}