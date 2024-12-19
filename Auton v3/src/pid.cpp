#include "pid.h" // Include the PID header file
#include "vex.h" // Include the VEX library
#include "robot-config.h" // Include the robot configuration to use the Brain object

using namespace vex; // Use the VEX namespace

// Constructor to initialize PID coefficients and internal variables
PID::PID(double kp, double ki, double kd)
    : kp(kp), ki(ki), kd(kd), prevError(0), integral(0) {}

// Get PID control value after given target value and current reading
double PID::calculate(double targetValue, double currentReading) {
    // Calculate the error
    double error = targetValue - currentReading;

    // Proportional term
    double pTerm = kp * error;

    // Integral term
    integral += error;
    double iTerm = ki * integral;

    // Derivative term
    double dTerm = kd * (error - prevError);
    prevError = error; // Update previous error

    // PID output
    return pTerm + iTerm + dTerm;
}

// Method to reset the PID controller (useful when switching contexts)
void PID::pidReset() {
    error = 0;
    prevError = 0; // Reset previous error to 0
    integral = 0;  // Reset integral term to 0
}

// Method to set PID coefficients dynamically
void PID::setCoefficients(double newKp, double newKi, double newKd) {
    kp = newKp; // Set new proportional coefficient
    ki = newKi; // Set new integral coefficient
    kd = newKd; // Set new derivative coefficient
}
/*
void pidHeadingController(double normTargetHeading, double kp_heading, double ki_heading, double kd_heading) {
 
    // Initialize PID controller for heading 
    PID headingPID(kp_heading, ki_heading, kd_heading); 
    headingPID.pidReset();

      // while (currentDistance < (targetDistanceCM)) {       
        // Get the current heading and normalize it 
double normCurrentHeading = normalizeHeading(InertialSensor.heading());


        // Calculate the heading correction using the PID controller 
        double headingCorrection = headingPID.calculate(normTargetHeading, normCurrentHeading);

        // Adjust motor speeds based on the heading correction 
        double leftMotorSpeed = speedVoltage + headingCorrection; // Base speed + heading correction
        double rightMotorSpeed = speedVoltage - headingCorrection; // Base speed + heading correction

        // Small delay 
        task::sleep(20); 
    }




void pidStraight2(double targetHeading, double targetDistanceCM, double speed, double kp_heading, double ki_heading, double kd_heading, double distanceOffset, brakeType brakeMode) {
    // Reset the encoders to start counting from zero
    passiveEncoderLeft.resetPosition();
    passiveEncoderRight.resetPosition();  

    //Convert % Speed input to voltage with max voltage of 12
    double speedVoltage = speed * 0.01 * 12;   

    // Initialize PID controller for heading 
    PID headingPID(kp_heading, ki_heading, kd_heading); 
    headingPID.pidReset();

        // Normalize the target heading 
        double normTargetHeading = normalizeHeading(targetHeading);

    // Start debug info
    Brain.Screen.clearScreen();
    Brain.Screen.print("Starting pidStraight function...");
     double currentDistance = 0;


    // Loop to continuously adjust motor power based on PID control 
      while (currentDistance < (targetDistanceCM - distanceOffset)) {       
      // while (currentDistance < (targetDistanceCM)) {       
        // Get the current heading and normalize it 
        double currentHeading = InertialSensor.heading();
        double normCurrentHeading = normalizeHeading(currentHeading);

        // Debug print the current heading
        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("Current Heading: %f", currentHeading);
        Brain.Screen.setCursor(3, 1);
        Brain.Screen.print("Normalized Heading: %f", normCurrentHeading);

        // Debug print the target heading
        Brain.Screen.setCursor(4, 1);
        Brain.Screen.print("Target Heading: %f", targetHeading);
        Brain.Screen.setCursor(5, 1);
        Brain.Screen.print("Normalized Target: %f", normTargetHeading);

        // Calculate the heading correction using the PID controller 
        double headingCorrection = headingPID.calculate(normTargetHeading, normCurrentHeading);

        // Adjust motor speeds based on the heading correction 
        double leftMotorSpeed = speedVoltage + headingCorrection; // Base speed + heading correction
        double rightMotorSpeed = speedVoltage - headingCorrection; // Base speed + heading correction

        // Debug print motor speeds
        Brain.Screen.setCursor(6, 1);
        Brain.Screen.print("Left Motor Speed: %f", leftMotorSpeed);
        Brain.Screen.setCursor(7, 1);
        Brain.Screen.print("Right Motor Speed: %f", rightMotorSpeed);

        // Set motor speeds 
        LeftMotor1.spin(forward, leftMotorSpeed, voltageUnits::volt);
        LeftMotor2.spin(forward, leftMotorSpeed, voltageUnits::volt);
        LeftMotor3.spin(forward, leftMotorSpeed, voltageUnits::volt); 
        RightMotor1.spin(forward, rightMotorSpeed, voltageUnits::volt); 
        RightMotor2.spin(forward, rightMotorSpeed, voltageUnits::volt); 
        RightMotor3.spin(forward, rightMotorSpeed, voltageUnits::volt); 

        // Print encoder value and distance covered
        currentDistance = (passiveEncoderLeft.position(degrees) + passiveEncoderRight.position(degrees) ) / 2 * (encoderWheelCircumferenceCM / 360.0);
        Brain.Screen.setCursor(8, 1);
        Brain.Screen.print("Target Distance: %f cm", targetDistanceCM);
        Brain.Screen.setCursor(9, 1);
        Brain.Screen.print("Distance: %f cm", currentDistance);
    
        // Small delay 
        task::sleep(20); 
    }

    // Stop the motors with braking 
// Stop the motors with the specified brake mode
LeftMotor1.stop(brakeMode); 
LeftMotor2.stop(brakeMode); 
LeftMotor3.stop(brakeMode); 
RightMotor1.stop(brakeMode); 
RightMotor2.stop(brakeMode); 
RightMotor3.stop(brakeMode); 

    // Final debug message
    Brain.Screen.setCursor(10, 1);
    Brain.Screen.print("pidStraight finished.");
}
*/