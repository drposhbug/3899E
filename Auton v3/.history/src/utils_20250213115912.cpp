#include "vex.h"
#include "robot-config.h"  // Move this before utils.h since utils.h needs ArmPosition
#include "utils.h"
#include <cmath>
#include <algorithm>

using namespace vex;


// Minimum threshold for division operations to prevent divide by zero errors
const double DIV_BY_ZERO_THRESHOLD = 0.001;  

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

/*
double normHeading(double heading) {
    // Add 180 to the heading and take the modulus with 360 to wrap the angle.
    // This ensures the result is between [0, 360) degrees.
    double normHeading = fmod(heading + 180.0, 360.0);

    // If the result is negative (e.g., due to fmod returning a negative value),
    // add 360 to shift it into the [0, 360) range.
    if (normHeading < 0) {
        normHeading += 360.0;
    }

    // Subtract 180 to convert the range from [0, 360) to [-180, 180).
    normHeading -= 180.0;
    
    if ((heading + 180.0) > 0 && normHeading == -180) {
      // Ensure 180 stays as 180, and -180 stays as -180.
      normHeading = 180.0;
    }

    // Return the normalized heading value.
    return normHeading;
}
*/

double normHeading(double heading) {
    // Add 180 to the heading and take the modulus with 360 to wrap the angle.
    // This ensures the result is between [0, 360) degrees.
    double normHeading = fmod(heading + 180.0, 360.0);

    // If the result is negative (e.g., due to fmod returning a negative value),
    // add 360 to shift it into the [0, 360) range.
    if (normHeading < 0) {
        normHeading += 360.0;
    }

    // Subtract 180 to convert the range from [0, 360) to [-180, 180).
    normHeading -= 180.0;
    
    if ((heading + 180.0) > 0 && normHeading == -180) {
      // Ensure 180 stays as 180, and -180 stays as -180.
      normHeading = 180.0;
    }

    // Return the normalized heading value.
    return normHeading;
}

double normHeading360(double heading) {
    // Take the modulus with 360 to wrap the angle.
    // This ensures the result is between [0, 360) degrees.
    double normHeading = fmod(heading, 360.0);

    // If the result is negative (e.g., due to fmod returning a negative value),
    // add 360 to shift it into the [0, 360) range.
    if (normHeading < 0) {
        normHeading += 360.0;
    }
    
    // Return the normalized heading value in [0, 360) range
    return normHeading;
}


/**
* Calculate the shortest path error between target and current heading.
* Ensures the error is always in the range of -180 to +180 degrees for smooth PID control.
* 
* This ensures:
* 1. PID gets continuous error values with no discontinuities at 0/360
* 2. Robot always takes the shortest path to target heading
* 3. Error magnitude never exceeds 180 degrees
*/
double getHeadingError360(double targetHeading, double currentHeading) {
   // First normalize both headings to 0-360 range
   double error = normHeading360(targetHeading) - normHeading360(currentHeading);
   
   // Convert error to -180 to +180 range for shortest path
   if(error > 180) {
       error -= 360;  // If error > 180, shorter to turn CCW
   } else if(error < -180) {
       error += 360;  // If error < -180, shorter to turn CW
   }
   
   return error;  // Returns error in range -180 to +180 degrees
}

double getHeadingError(double targetHeading, double currentHeading) {
   double error = targetHeading - currentHeading;
   
   // Convert error to -180 to +180 range for shortest path
   if(error > 180) {
       error -= 360;  
   } else if(error < -180) {
       error += 360;  
   }
   
   return error;
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


// Color Detection Constants for utils.cpp
//const double RED_HUE_MIN_1 = 340.0;  // First red range (340°-360°)
//const double RED_HUE_MAX_1 = 360.0;
const double RED_HUE_MIN_1 = 335.0;  // First red range (340°-360°)
const double RED_HUE_MAX_1 = 365.0;
const double RED_HUE_MIN_2 = 0.0;    // Second red range (0°-15°)
const double RED_HUE_MAX_2 = 15.0;
//const double BLUE_HUE_MIN = 215.0;   // Blue range
//const double BLUE_HUE_MAX = 225.0;
const double BLUE_HUE_MIN = 210.0;   // Blue range
const double BLUE_HUE_MAX = 230.0;
const double MIN_BRIGHTNESS = 15.0;   // Minimum brightness threshold

// Function to initialize the Optical Sensor
void initializeOpticalSensor() {
  opticalSensor.setLightPower(100, percent);  // Turn on the sensor light at 100% power
  opticalSensor.setLight(ledState::on);       // Ensure the light is on
}

// Track consecutive detections to prevent false positives
static int consecutiveDetections = 0;
static bool lastDetectedColor = false;  // false = no color, true = color detected

bool detectColor() {
    double hue = opticalSensor.hue();
    double brightness = opticalSensor.brightness();
    bool colorDetected = false;

    // Check brightness threshold
    if (brightness < MIN_BRIGHTNESS) {
        consecutiveDetections = 0;
        lastDetectedColor = false;
        return false;
    }

    // Check for red or blue
    if (((hue >= RED_HUE_MIN_1 && hue <= RED_HUE_MAX_1) || 
         (hue >= RED_HUE_MIN_2 && hue <= RED_HUE_MAX_2)) ||
        (hue >= BLUE_HUE_MIN && hue <= BLUE_HUE_MAX)) {
        colorDetected = true;
    }

    // Handle consecutive detections
    if (colorDetected == lastDetectedColor && colorDetected) {
        consecutiveDetections++;
    } else {
        consecutiveDetections = 1;
    }

    lastDetectedColor = colorDetected;

    // Return true if we have enough consecutive detections
    return (consecutiveDetections >= 3);  // Require 3 consecutive detections
}

// Reset detection state if needed
void resetColorDetection() {
    consecutiveDetections = 0;
    lastDetectedColor = false;
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

// Scale voltages proportionally if either exceeds max voltage
void scaleVoltages(double& leftVoltage, double& rightVoltage, double absoluteMaxVoltage) {
    if (std::abs(leftVoltage) > absoluteMaxVoltage || std::abs(rightVoltage) > absoluteMaxVoltage) {
        double scale = absoluteMaxVoltage / std::max(std::abs(leftVoltage), std::abs(rightVoltage));
        leftVoltage *= scale;
        rightVoltage *= scale;
    }
}

double convertHeading(double currentHeading, double offset) {
    return currentHeading - offset;
}

void PIDVoltageCapCorrection(double& leftVoltage, double& rightVoltage, double absoluteMaxVoltage) {
    double pidCorrectionDiff = fabs(leftVoltage - rightVoltage);
    
    if (std::abs(leftVoltage) > absoluteMaxVoltage) {
        leftVoltage = std::copysign(absoluteMaxVoltage, leftVoltage);
        rightVoltage = std::copysign((absoluteMaxVoltage - pidCorrectionDiff), rightVoltage);
    }
    else if (std::abs(rightVoltage) > absoluteMaxVoltage) {
        leftVoltage = std::copysign((absoluteMaxVoltage - pidCorrectionDiff), leftVoltage);
        rightVoltage = std::copysign(absoluteMaxVoltage, rightVoltage);
    }

}

/**
* Calculates slip ratio between wheel and robot speeds
* @param wheelSpeed Individual wheel speed in RPM
* @param robotSpeed Robot's ground speed in RPM
* @return Slip ratio from 0 to 1: 0 = no slip, 1 = full slip
*/
double calculateSlipRatio(double wheelSpeed, double robotSpeed) {
   double maxSpeed = std::max(std::fabs(wheelSpeed), std::fabs(robotSpeed));
   if (maxSpeed < DIV_BY_ZERO_THRESHOLD) {
       return 0.0;
   }
   return std::fabs((wheelSpeed - robotSpeed) / maxSpeed);
}
/*
struct ArmTaskParams {
    ArmPosition position;
    int speed;
    int delayMs;
};

int armTask(void* params) {
    ArmTaskParams* p = (ArmTaskParams*)params;
    int startTime = Brain.Timer.time(msec);
    while((Brain.Timer.time(msec) - startTime) < p->delayMs) {
        this_thread::sleep_for(10);
    }
    //harmMotor.spinToPosition(p->position, rotationUnits::deg, p->speed, velocityUnits::pct, false);
    armMotor.spinToPosition(Load2, rotationUnits::deg, 100, velocityUnits::pct, false);
    return 0;
}
*/
/*
void armTask(ArmPosition position, int speed, int delayMs) {
    static ArmTaskParams params;
    params.position = position;
    params.speed = speed;
    params.delayMs = delayMs;
    thread(armTask, (void*)&params).detach();
}
*/

/**
* Calculates rolling average of a value over N samples
* @param newValue Latest measurement to include in average
* @param currentAverage Previous rolling average value
* @param n Number of samples to average over (typical: 5-10 for 50-100ms window at 10ms rate) 
* @return Updated rolling average
*/
float rollingAverage(float newValue, float currentAverage, int n) {
   return currentAverage * (n-1)/n + newValue/n;
}



int armTask(void* params) {
    ArmTaskParams* p = static_cast<ArmTaskParams*>(params);

    while(p->isRunning) {
        if(p->moveRequested && p->delayMs > 0) {
            wait(p->delayMs, msec);  // Wait for specified delay
            armMotor.spinToPosition(p->targetPosition, rotationUnits::deg, 100, velocityUnits::pct, false);
            p->moveRequested = false;  // Reset the move request
        }
        wait(10, msec);  // Small delay to prevent CPU overload
    }
    return 0;
}


int colorDetectionTask(void* params) {
    ColorTaskParams* p = static_cast<ColorTaskParams*>(params);
    
    while (p->isRunning) {
        double hue = opticalSensor.hue();
        Brain.Screen.clearLine(1);  // Clear line 1 before printing
        Brain.Screen.setCursor(1, 1);  // Set cursor to beginning of line 1
        
        if (((hue >= RED_HUE_MIN_1 && hue <= RED_HUE_MAX_1) || 
            (hue >= RED_HUE_MIN_2 && hue <= RED_HUE_MAX_2)) && p->targetColor == Color::RED) {
            Brain.Screen.print("RED");
            wait(p->delayMs, msec); 
            intakeMotor.stop(); // Stop the motor
            wait(50, msec); 
            intakeMotor.spin(reverse, 100, velocityUnits::pct);  // Spins continuously until stopped
        } 
        else if ((hue >= BLUE_HUE_MIN && hue <= BLUE_HUE_MAX) && p->targetColor == Color::BLUE) {
            Brain.Screen.print("BLUE");
            wait(p->delayMs, msec); 
            intakeMotor.stop(); // Stop the motor
            wait(50, msec); 
            intakeMotor.spin(reverse, 100, velocityUnits::pct);  // Spins continuously until stopped
        }
        
        wait(10, msec);  // Small delay to prevent CPU overload
    }
    return 0;
}

// Gets heading in counterclockwise degrees
// Converts raw clockwise sensor reading & applies calibration offset 
double getAdjustedHeading() {
   // Convert clockwise sensor to counterclockwise & apply offset
   return normHeading(InertialSensor.heading() + headingOffset);
}

double convertToVEXHeading(double euclideanHeading) {
    // Convert counterclockwise to clockwise
    double vexHeading = fmod(360.0 - euclideanHeading, 360.0);
    if (vexHeading < 0) {
        vexHeading += 360.0;
    }
    return vexHeading;
}

