#include "navigation.h"
#include "robot-config.h"
#include "vex.h"
#include "utils.h"
#include "pid.h"  // Ensure this line is included
#include <cmath>
#include <algorithm>
#include <iterator>
#include <cstring>  // Include the cstring library for strcmp
#include "vex.h"  // Make sure this is included to use vex:: types


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace vex;

enum MotionPhase {
    READY,
    LAUNCH,
    CRUISE,
    DECELERATE,
    APPROACH,
    STOP
    
};

// Function to move the six wheel motors based on a given distance (in cm), max speed, and direction (default is forward)
void move(double distanceCM, double maxSpeed, vex::directionType dir) {
    // Use the globally declared wheel circumference to calculate the number of rotations needed
    double targetRotations = distanceCM / wheelCircumferenceCM;

    // Set brake mode to brake for all motors
        // Stop all left and right motors using the array and a loop
                for (int i = 0; i < 3; i++) {
                leftMotor[i].setBrake(brakeType::coast);  // Stop left motors
                rightMotor[i].setBrake(brakeType::coast); // Stop right motors
                }
    
    // Set motor velocities and move them for the calculated number of rotations
    for (int i = 0; i < 3; i++) {        
    leftMotor[i].spinFor(dir, targetRotations, rotationUnits::rev, maxSpeed, velocityUnits::pct, false);  
    rightMotor[i].spinFor(dir, targetRotations, rotationUnits::rev, maxSpeed, velocityUnits::pct, false);  
    }
}

void pidStraight(double targetHeading, double targetDistanceCM, double speed, double kp_heading, double ki_heading, double kd_heading, double distanceOffset, brakeType brakeMode) {
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
        for (int i = 0; i < 3; i++) {      
        leftMotor[i].spin(forward, leftMotorSpeed, voltageUnits::volt);
        rightMotor[i].spin(forward, leftMotorSpeed, voltageUnits::volt);
        }

        // Print encoder value and distance covered
        currentDistance = (passiveEncoderLeft.position(degrees) + passiveEncoderRight.position(degrees) ) / 2 * (encoderWheelCircumferenceCM / 360.0);
        Brain.Screen.setCursor(8, 1);
        Brain.Screen.print("Target Distance: %f cm", targetDistanceCM);
        Brain.Screen.setCursor(9, 1);
        Brain.Screen.print("Distance: %f cm", currentDistance);
    
        // Small delay 
        task::sleep(20); 
    }

    // Stop the motors with the specified brake mode
    for (int i = 0; i < 3; i++) {      
    leftMotor[i].stop(brakeMode); 
    rightMotor[i].stop(brakeMode); 
    }

    // Final debug message
    Brain.Screen.setCursor(10, 1);
    Brain.Screen.print("pidStraight finished.");
}


void pidStraightTime(double targetHeading, bool (*exitCondition)()) { 
    // PID coefficients for heading correction 
    double kp_heading = .6; // Proportional coefficient (.2 optimal)
    double ki_heading = 0.0; // Integral coefficient 
    double kd_heading = 0.0; // Derivative coefficient 
    // Initialize PID controller for heading 
    PID headingPID(kp_heading, ki_heading, kd_heading); 
    headingPID.pidReset();

int counter = 0;

// Set brake mode to brake 
    for (int i = 0; i < 3; i++) {      
    leftMotor[i].setBrake(brakeType::brake); 
    rightMotor[i].setBrake(brakeType::brake); 
    }

        
    // Loop to continuously adjust motor power based on PID control 
    //while (!exitCondition()) { 
    while (counter < 20) {
        // Get the current heading and normalize it 
        double currentHeading = InertialSensor.heading(); 
        double normalizedCurrentHeading = normalizeHeading(currentHeading); 
        // Normalize the target heading 
        double normalizedTargetHeading = normalizeHeading(targetHeading); 
        // Calculate the heading correction using the PID controller 
        double headingCorrection = headingPID.calculate(normalizedTargetHeading, normalizedCurrentHeading); 
        // Adjust motor speeds based on the heading correction 
        double leftMotorSpeed = 90 + headingCorrection; // Base speed + heading correction
        double rightMotorSpeed = 90 - headingCorrection; // Base speed + heading correction
        // Set motor speeds 
        // Set motor speeds 
        for (int i = 0; i < 3; i++) {      
        leftMotor[i].spin(forward, leftMotorSpeed, percent); 
        rightMotor[i].spin(forward, rightMotorSpeed, percent); 
        }
        // Small delay 
        task::sleep(20); 
        // Add to the counter at the end of each loop
        counter++;
    } 

    // Stop the motors 
    for (int i = 0; i < 3; i++) {      
    leftMotor[i].stop(); 
    rightMotor[i].stop(); 
    }
} 



/*
//PID Distance, v1 works
void pidDistance(double targetDistanceCm, double maxSpeed, bool (*exitCondition)()) {
    //PID pid(0.6, 0.0, 0.0); // Example PID coefficients, adjust as necessary
    // PID coefficients for heading correction 
    double kp_distance = .06;// Proportional coefficient 
    double ki_distance = 0.0; // Integral coefficient 
    double kd_distance = 0.0; // Derivative coefficient 
    double deadzone = 2;
    double leftMotorSpeed = 0; 
    double rightMotorSpeed = 0;
    double minSpeed = 7.5;
    // Initialize PID controller for heading 
    PID distancePID(kp_distance, ki_distance, kd_distance); 
    distancePID.pidReset();


//float target_distance_in_inches = 10.0;  // Example target distance
//float target_rotation_degrees = target_distance_in_inches / drive_in_to_deg_ratio;
//drive_in_to_deg_ratio = wheel_ratio / 360.0 * M_PI * wheel_diameter;

*/
/*
    // Convert wheel diameter to centimeters
    double wheelDiameterInches = 3.25; // in inches
    double wheelDiameterCm = wheelDiameterInches * 2.54; // correct conversion to cm
    double wheelCircumferenceCm = wheelDiameterCm * M_PI; // in cm

    // Calculate the number of wheel rotations needed to travel the target distance in cm
    double targetRotations = targetDistanceCm / wheelCircumferenceCm;
    double motorTargetRotations = targetRotations / 6.0; // Convert target rotations to motor rotations
*/
/*
    double wheelCircumferenceCm;
    double gearRatio;
    //gearRatio = 6.0; // 6:1 gear ratio

    // Get wheel properties from utils
    getWheelProperties(wheelCircumferenceCm,gearRatio);

    //double distancePerDegreeMotorRotation = gearRatio * wheelCircumferenceCm / 360.0;
     double distancePerMotorTick = wheelCircumferenceCm / 300.0;

    // Calculate the number of motor rotations needed to travel the target distance in cm
    double targetRotations = targetDistanceCm / distancePerMotorTick;



    // Reset motor positions
    for (int i = 0; i < 3; i++) {      
    leftMotor[i].resetPosition();
    rightMotor[i].resetPosition();
    }


    // Set brake mode to brake
    for (int i = 0; i < 3; i++) {      
    leftMotor[i].setBrake(brakeType::brake);
    rightMotor[i].setBrake(brakeType::brake);
    }

    while (!exitCondition()) {
        double averageMotorPosition = (
                                       LeftMotor2.position(rotationUnits::raw) + 
                                       LeftMotor3.position(rotationUnits::raw) + 
                                       RightMotor2.position(rotationUnits::raw) + 
                                       RightMotor3.position(rotationUnits::raw)) / 4.0;
                                                                           
         // Calculate the PID output for distance control
        double distanceCorrection = distancePID.calculate(targetRotations, averageMotorPosition);
        // If the PID output is greater than maxSpeed, clamp it to maxSpeed
    if (distanceCorrection > maxSpeed) {
        distanceCorrection = maxSpeed;
    } 
    // If the PID output is less than -maxSpeed, clamp it to -maxSpeed (for reverse direction)
    else if (distanceCorrection < -maxSpeed) { 
        distanceCorrection = -maxSpeed;
    }
    // If the PID output is positive but less than minSpeed, set it to minSpeed to ensure movement
    if (distanceCorrection >= 0 && distanceCorrection < minSpeed) {
        distanceCorrection = minSpeed;
    } 
    // If the PID output is negative but greater than -minSpeed, set it to -minSpeed (for reverse movement)
    else if (distanceCorrection <= 0 && distanceCorrection > -minSpeed) {
        distanceCorrection = -minSpeed;
    }
        leftMotorSpeed = distanceCorrection; 
        rightMotorSpeed = distanceCorrection;
        // Apply the PID output to the motors
        LeftMotor1.spin(forward, leftMotorSpeed, percent); 
        LeftMotor2.spin(forward, leftMotorSpeed, percent); 
        LeftMotor3.spin(forward, leftMotorSpeed, percent); 
        RightMotor1.spin(forward, rightMotorSpeed, percent); 
        RightMotor2.spin(forward, rightMotorSpeed, percent); 
        RightMotor3.spin(forward, rightMotorSpeed, percent); 

        // Calculate the distance traveled in cm
        double distanceTravelledCm = averageMotorPosition * distancePerMotorTick;

        // Check if the target distance has been reached
if ((targetRotations - averageMotorPosition) <= deadzone) {
//if ((targetRotations - averageMotorPosition) <= 1) {
    break; // Exit the loop if the target distance is reached within the dead zone
}

        vex::task::sleep(20); // Sleep the task for a short amount of time to prevent wasted resources
    }

    // Stop the motors with braking
    LeftMotor1.stop();
    LeftMotor2.stop();
    LeftMotor3.stop();
    RightMotor1.stop();
    RightMotor2.stop();
    RightMotor3.stop();

}
*/

void pidDistance(double targetDistanceCm, double maxSpeed, bool (*exitCondition)()) {
    //PID pid(0.6, 0.0, 0.0); // Example PID coefficients, adjust as necessary
    // PID coefficients for heading correction 
    double kp_distance = .06;// Proportional coefficient 
    double ki_distance = 0.0; // Integral coefficient 
    double kd_distance = 0.0; // Derivative coefficient 
    double deadzone = 2;
    double leftMotorSpeed = 0; 
    double rightMotorSpeed = 0;
    double minSpeed = 7.5;
    // Initialize PID controller for heading 
    PID distancePID(kp_distance, ki_distance, kd_distance); 
    distancePID.pidReset();


//float target_distance_in_inches = 10.0;  // Example target distance
//float target_rotation_degrees = target_distance_in_inches / drive_in_to_deg_ratio;
//drive_in_to_deg_ratio = wheel_ratio / 360.0 * M_PI * wheel_diameter;


    double wheelCircumferenceCm;
    double gearRatio;
    //gearRatio = 6.0; // 6:1 gear ratio

    // Get wheel properties from utils
    getWheelProperties(wheelCircumferenceCm,gearRatio);

    //double distancePerDegreeMotorRotation = gearRatio * wheelCircumferenceCm / 360.0;
     double distancePerMotorTick = wheelCircumferenceCm / 300.0;

    // Calculate the number of motor rotations needed to travel the target distance in cm
    double targetRotations = targetDistanceCm / distancePerMotorTick;



    // Reset motor positions
        // Stop the motors with the specified brake mode
    for (int i = 0; i < 3; i++) {      
    leftMotor[i].resetPosition();
    rightMotor[i].resetPosition();
    }

    // Set brake mode to brake
    for (int i = 0; i < 3; i++) {      
    leftMotor[i].setBrake(brakeType::brake);
    rightMotor[i].setBrake(brakeType::brake);
    }

    while (!exitCondition()) {
        double averageMotorPosition = (
                                       leftMotor[2].position(rotationUnits::raw) + 
                                       leftMotor[3].position(rotationUnits::raw) + 
                                       rightMotor[2].position(rotationUnits::raw) + 
                                       rightMotor[3].position(rotationUnits::raw)) / 4.0;
                                                                           
         // Calculate the PID output for distance control
        double distanceCorrection = distancePID.calculate(targetRotations, averageMotorPosition);
        // If the PID output is greater than maxSpeed, clamp it to maxSpeed
    if (distanceCorrection > maxSpeed) {
        distanceCorrection = maxSpeed;
    } 
    // If the PID output is less than -maxSpeed, clamp it to -maxSpeed (for reverse direction)
    else if (distanceCorrection < -maxSpeed) { 
        distanceCorrection = -maxSpeed;
    }
    // If the PID output is positive but less than minSpeed, set it to minSpeed to ensure movement
    if (distanceCorrection >= 0 && distanceCorrection < minSpeed) {
        distanceCorrection = minSpeed;
    } 
    // If the PID output is negative but greater than -minSpeed, set it to -minSpeed (for reverse movement)
    else if (distanceCorrection <= 0 && distanceCorrection > -minSpeed) {
        distanceCorrection = -minSpeed;
    }
        leftMotorSpeed = distanceCorrection; 
        rightMotorSpeed = distanceCorrection;
        // Apply the PID output to the motors
        LeftMotor1.spin(forward, leftMotorSpeed, percent); 
        LeftMotor2.spin(forward, leftMotorSpeed, percent); 
        LeftMotor3.spin(forward, leftMotorSpeed, percent); 
        RightMotor1.spin(forward, rightMotorSpeed, percent); 
        RightMotor2.spin(forward, rightMotorSpeed, percent); 
        RightMotor3.spin(forward, rightMotorSpeed, percent); 

        // Calculate the distance traveled in cm
       // double distanceTravelledCm = averageMotorPosition * distancePerMotorTick;

        // Check if the target distance has been reached
if ((targetRotations - averageMotorPosition) <= deadzone) {
//if ((targetRotations - averageMotorPosition) <= 1) {
    break; // Exit the loop if the target distance is reached within the dead zone
}

        vex::task::sleep(20); // Sleep the task for a short amount of time to prevent wasted resources
    }

    // Stop the motors 
    for (int i = 0; i < 3; i++) {      
    leftMotor[i].stop(); 
    rightMotor[i].stop(); 
    }

}




/*
void pidStraightDistance(double targetDistanceCm, double speed, double targetHeading, bool (*exitCondition)()) {
    PID pid(0.0, 0.0, 0.0); // Example PID coefficients, adjust as necessary
    // PID coefficients for heading correction
    double kp_heading = 0.0; // Proportional coefficient
    double ki_heading = 0.0; // Integral coefficient
    double kd_heading = 0.0; // Derivative coefficient

    // Initialize PID controller for heading
    PID headingPID(kp_heading, ki_heading, kd_heading);

    double wheelCircumferenceCm;
    double gearRatio;
    //gearRatio = 6.0; // 6:1 gear ratio

    // Get wheel properties from utils
    getWheelProperties(wheelCircumferenceCm,gearRatio);

    // Calculate the number of motor rotations needed to travel the target distance in cm;
    double targetRotations = targetDistanceCm / wheelCircumferenceCm / gearRatio * 360;

    // Reset motor positions
    for (int i = 0; i < 3; i++) {      
    leftMotor[i].resetPosition();
    rightMotor[i].resetPosition();
    }


    while (!exitCondition()) {
        // Get the average position of the motors in rotations
        double averageMotorPosition = (LeftMotor1.position(degrees) + 
                                       LeftMotor2.position(degrees) + 
                                       LeftMotor3.position(degrees) + 
                                       RightMotor1.position(degrees) + 
                                       RightMotor2.position(degrees) + 
                                       RightMotor3.position(degrees)) / 6.0;
                                       
        // Get the current heading and normalize it
        double currentHeading = InertialSensor.heading();
        double normalizedCurrentHeading = normalizeHeading(currentHeading);

        // Normalize the target heading
        double normalizedTargetHeading = normalizeHeading(targetHeading);

        // Calculate the PID output for distance control
        double output = pid.calculate(targetRotations, averageMotorPosition);

        // Calculate the heading correction using the PID controller
        double headingCorrection = headingPID.calculate(normalizedTargetHeading, normalizedCurrentHeading);        

        // Set motor speeds using motors declared in robot-config.cpp
        LeftMotor1.spin(forward, speed + output - headingCorrection, percent);
        LeftMotor2.spin(forward, speed + output - headingCorrection, percent);
        LeftMotor3.spin(forward, speed + output - headingCorrection, percent);
        RightMotor1.spin(forward, speed + output + headingCorrection, percent);
        RightMotor2.spin(forward, speed + output + headingCorrection, percent);
        RightMotor3.spin(forward, speed + output + headingCorrection, percent);

        // Calculate the distance traveled in cm
        double distanceTravelledCm = averageMotorPosition * wheelCircumferenceCm / gearRatio;
        // Print the distance traveled to the Brain screen for debugging
        Brain.Screen.printAt(10, 60, "targetRotations: %.2f rev", targetRotations);
        Brain.Screen.printAt(10, 80, "averageMotorPosition: %.2f rev", averageMotorPosition);
        Brain.Screen.printAt(10, 100, "Distance travelled: %.2f cm", distanceTravelledCm);
        Brain.Screen.printAt(10, 120, "Wheel Circumference: %.2f", wheelCircumferenceCm);
        Brain.Screen.printAt(10, 140, "Gear Ratio: %.2f", gearRatio);

        // Check if the target distance has been reached
        //if (fabs(averageMotorPosition - targetRotations) < 0.1) {
        if ((averageMotorPosition - targetRotations) >= 0) {
            break; // Exit the loop if the target distance is reached
        }

        vex::task::sleep(20); // Sleep the task for a short amount of time to prevent wasted resources
         
    }

    // Set brake mode to brake
    for (int i = 0; i < 3; i++) {      
    leftMotor[i].setBrake(brakeType::brake);
    rightMotor[i].setBrake(brakeType::brake);
    }

    // Stop the motors with braking
    LeftMotor1.stop();
    LeftMotor2.stop();
    LeftMotor3.stop();
    RightMotor1.stop();
    RightMotor2.stop();
    RightMotor3.stop();
}

*/

void pidStraightDistance(double targetDistanceCm, double maxSpeed, double targetHeading, bool (*exitCondition)()) { 
    // PID coefficients for heading correction 
    double kp_heading = .2; // Proportional coefficient 
    double ki_heading = 0.0; // Integral coefficient 
    double kd_heading = 0.0; // Derivative coefficient 

    // PID coefficients for distance correction
    double kp_distance = .1; // Proportional coefficient 
    double ki_distance = 0.0; // Integral coefficient 
    double kd_distance = 0.0; // Derivative coefficient 

    double deadzone = 1.2;
    double minSpeed = 7.5;

    double wheelCircumferenceCm;
    double gearRatio;
    //gearRatio = 6.0; // 6:1 gear ratio

    // Initialize PID controllers
    PID headingPID(kp_heading, ki_heading, kd_heading);
    PID distancePID(kp_distance, ki_distance, kd_distance);
    
    headingPID.pidReset();
    distancePID.pidReset();

    // Get wheel properties from utils
    getWheelProperties(wheelCircumferenceCm,gearRatio);

    //double distancePerDegreeMotorRotation = gearRatio * wheelCircumferenceCm / 360.0;
     double distancePerMotorTick = wheelCircumferenceCm / 300.0;

    // Calculate the number of motor rotations needed to travel the target distance in cm
    double targetRotations = targetDistanceCm / distancePerMotorTick;
    
    // Set brake mode to brake
    for (int i = 0; i < 3; i++) {      
    leftMotor[i].setBrake(brakeType::brake);
    rightMotor[i].setBrake(brakeType::brake);
    }

    // Loop to continuously adjust motor power based on PID control
    while (!exitCondition()) {
        // Get the current heading and normalize it 
        double currentHeading = InertialSensor.heading(); 
        double normalizedCurrentHeading = normalizeHeading(currentHeading); 
        double normalizedTargetHeading = normalizeHeading(targetHeading); 
        
        // Calculate the heading correction using the PID controller 
        double headingCorrection = headingPID.calculate(normalizedTargetHeading, normalizedCurrentHeading);
        
        // Get the current distance
        double averageMotorPosition = (
                                       LeftMotor2.position(rotationUnits::raw) + 
                                       LeftMotor3.position(rotationUnits::raw) + 
                                       RightMotor2.position(rotationUnits::raw) + 
                                       RightMotor3.position(rotationUnits::raw)) / 4.0;
        
         // Calculate the PID output for distance control
        double distanceCorrection = distancePID.calculate(targetRotations, averageMotorPosition);

        // If the PID output is greater than maxSpeed, clamp it to maxSpeed
    if (distanceCorrection > maxSpeed) {
        distanceCorrection = maxSpeed;
    } 
    // If the PID output is less than -maxSpeed, clamp it to -maxSpeed (for reverse direction)
    else if (distanceCorrection < -maxSpeed) { 
        distanceCorrection = -maxSpeed;
    }

    // If the PID output is positive but less than minSpeed, set it to minSpeed to ensure movement
    if (distanceCorrection >= 0 && distanceCorrection < minSpeed) {
        distanceCorrection = minSpeed;
    } 
    // If the PID output is negative but greater than -minSpeed, set it to -minSpeed (for reverse movement)
    else if (distanceCorrection <= 0 && distanceCorrection > -minSpeed) {
        distanceCorrection = -minSpeed;
    }


        // Adjust motor speeds based on the heading and distance corrections
        double leftMotorSpeed = distanceCorrection + headingCorrection; 
        double rightMotorSpeed = distanceCorrection - headingCorrection;
        
        LeftMotor1.spin(directionType::fwd, leftMotorSpeed, velocityUnits::pct);
        LeftMotor2.spin(directionType::fwd, leftMotorSpeed, velocityUnits::pct);
        LeftMotor3.spin(directionType::fwd, leftMotorSpeed, velocityUnits::pct);
        RightMotor1.spin(directionType::fwd, rightMotorSpeed, velocityUnits::pct);
        RightMotor2.spin(directionType::fwd, rightMotorSpeed, velocityUnits::pct);
        RightMotor3.spin(directionType::fwd, rightMotorSpeed, velocityUnits::pct);
        
        double rotationDegrees = passiveEncoderLeft.position(vex::rotationUnits::deg);
        Brain.Screen.printAt(10, 20, "Rotation Sensor: %.2f degrees", rotationDegrees);
                // Calculate the distance traveled in cm
        double distanceTravelledCm = averageMotorPosition * distancePerMotorTick;

        // Print the distance traveled to the Brain screen for debugging

        Brain.Screen.printAt(10, 40, "Target Distance: %.4f", targetDistanceCm);
        Brain.Screen.printAt(10, 60, "Distance travelled: %.2f", distanceTravelledCm);
        Brain.Screen.printAt(10, 80, "Target Rotation: %.2f", targetRotations);
        Brain.Screen.printAt(10, 100, "avg motor position: %.2f", averageMotorPosition);
        Brain.Screen.printAt(10, 120, "Lt Motor Speed: %.2f", leftMotorSpeed);
        Brain.Screen.printAt(10, 140, "Rt Motor Speed: %.2f", rightMotorSpeed);

if ((targetRotations - averageMotorPosition) <= deadzone) {
//if ((targetRotations - averageMotorPosition) <= 1) {
    break; // Exit the loop if the target distance is reached within the dead zone
}


        vex::task::sleep(20);
    }
    
    // Stop the motors 
    for (int i = 0; i < 3; i++) {      
    leftMotor[i].stop(); 
    rightMotor[i].stop(); 
    }
}

void pidBackwardsDistance(double targetDistanceCm, double speed, double targetHeading, bool (*exitCondition)()) {
    PID pid(0.0, 0.0, 0.0); // Example PID coefficients, adjust as necessary
    // PID coefficients for heading correction
    double kp_heading = 0.0; // Proportional coefficient
    double ki_heading = 0.0; // Integral coefficient
    double kd_heading = 0.0; // Derivative coefficient

    // Initialize PID controller for heading
    PID headingPID(kp_heading, ki_heading, kd_heading);

    double wheelCircumferenceCm;
    double gearRatio;
    //gearRatio = 6.0; // 6:1 gear ratio

    // Get wheel properties from utils
    getWheelProperties(wheelCircumferenceCm,gearRatio);

    // Calculate the number of motor rotations needed to travel the target distance in cm;
    double targetRotations = targetDistanceCm / wheelCircumferenceCm / gearRatio * 360;

    // Reset motor positions
    for (int i = 0; i < 3; i++) {      
    leftMotor[i].resetPosition();
    rightMotor[i].resetPosition();
    }


    while (!exitCondition()) {
        // Get the average position of the motors in rotations
        double averageMotorPosition = (leftMotor[1].position(degrees) - 
                                       leftMotor[2].position(degrees) - 
                                       leftMotor[3].position(degrees) - 
                                       rightMotor[1].position(degrees) - 
                                       rightMotor[2].position(degrees) - 
                                       rightMotor[3].position(degrees)) / 6.0;
                                       
        // Get the current heading and normalize it
        double currentHeading = InertialSensor.heading();
        double normalizedCurrentHeading = normalizeHeading(currentHeading);

        // Normalize the target heading
        double normalizedTargetHeading = normalizeHeading(targetHeading);

        // Calculate the PID output for distance control
        double output = pid.calculate(targetRotations, averageMotorPosition);

        // Calculate the heading correction using the PID controller
        double headingCorrection = headingPID.calculate(normalizedTargetHeading, normalizedCurrentHeading);        

        // Set motor speeds using motors declared in robot-config.cpp
        for (int i = 0; i < 3; i++) {   
        leftMotor[i].spin(reverse, speed + output - headingCorrection, percent);
        rightMotor[i].spin(reverse, speed + output + headingCorrection, percent);
        }


        // Calculate the distance traveled in cm
        double distanceTravelledCm = averageMotorPosition * wheelCircumferenceCm / gearRatio;
        // Print the distance traveled to the Brain screen for debugging
        Brain.Screen.printAt(10, 60, "targetRotations: %.2f rev", targetRotations);
        Brain.Screen.printAt(10, 80, "averageMotorPosition: %.2f rev", averageMotorPosition);
        Brain.Screen.printAt(10, 100, "Distance travelled: %.2f cm", distanceTravelledCm);
        Brain.Screen.printAt(10, 120, "Wheel Circumference: %.2f", wheelCircumferenceCm);
        Brain.Screen.printAt(10, 140, "Gear Ratio: %.2f", gearRatio);

        // Check if the target distance has been reached
        //if (fabs(averageMotorPosition - targetRotations) < 0.1) {
        if ((averageMotorPosition - targetRotations) >= 0) {
            break; // Exit the loop if the target distance is reached
        }

        vex::task::sleep(20); // Sleep the task for a short amount of time to prevent wasted resources
         
    }

    // Set brake mode to brake
    for (int i = 0; i < 3; i++) {      
    leftMotor[i].setBrake(brakeType::brake);
    rightMotor[i].setBrake(brakeType::brake);
    }

    // Stop the motors 
    for (int i = 0; i < 3; i++) {      
    leftMotor[i].stop(); 
    rightMotor[i].stop(); 
    }
}

/*

void spotTurn(double targetAngle, double maxSpeed, double kP, double minSpeed) {
    double error = targetAngle - InertialSensor.heading(degrees);
    double speed;

    while (fabs(error) > 1) { // Continue until the error is within 1 degree
        double currentAngle = InertialSensor.heading(degrees);
        error = targetAngle - currentAngle;

        // Calculate speed using proportional control
        speed = kP * error;

        // Limit the speed to maxSpeed and minSpeed
        if (fabs(speed) > maxSpeed) {
            speed = (speed > 0) ? maxSpeed : -maxSpeed;
        } else if (fabs(speed) < minSpeed) {
            speed = (speed > 0) ? minSpeed : -minSpeed;
        }

        // Set motors to run in opposite directions
        LeftMotor1.spin(forward, speed, pct);
        LeftMotor2.spin(forward, speed, pct);
        LeftMotor3.spin(forward, speed, pct);
        RightMotor1.spin(reverse, speed, pct);
        RightMotor2.spin(reverse, speed, pct);
        RightMotor3.spin(reverse, speed, pct);

        // Optional: Print sensor values to the brain screen for debugging
        Brain.Screen.printAt(10, 30, "Target Angle: %.2f", targetAngle);
        Brain.Screen.printAt(10, 50, "Current Angle: %.2f", currentAngle);
        Brain.Screen.printAt(10, 70, "Error: %.2f", error);
        Brain.Screen.printAt(10, 90, "Speed: %.2f", speed);

        task::sleep(20); // Small delay to prevent overwhelming the CPU
    }

    // Stop the motors
    LeftMotor1.stop(brake);
    LeftMotor2.stop(brake);
    LeftMotor3.stop(brake);
    RightMotor1.stop(brake);
    RightMotor2.stop(brake);
    RightMotor3.stop(brake);
}

void leftTurn(double targetAngle, double maxSpeed, double kP, double minSpeed) {
    double error = targetAngle - InertialSensor.heading(degrees);
    double speed;

    while (fabs(error) > 1) { // Continue until the error is within 1 degree
        double currentAngle = InertialSensor.heading(degrees);
        error = targetAngle - currentAngle;

        // Calculate speed using proportional control
        speed = kP * error;

        // Limit the speed to maxSpeed and minSpeed
        if (fabs(speed) > maxSpeed) {
            speed = (speed > 0) ? maxSpeed : -maxSpeed;
        } else if (fabs(speed) < minSpeed) {
            speed = (speed > 0) ? minSpeed : -minSpeed;
        }

        // Set motors to run in opposite directions
        LeftMotor1.spin(reverse, speed, pct);
        LeftMotor2.spin(reverse, speed, pct);
        LeftMotor3.spin(reverse, speed, pct);
        RightMotor1.spin(forward, speed, pct);
        RightMotor2.spin(forward, speed, pct);
        RightMotor3.spin(forward, speed, pct);

        // Optional: Print sensor values to the brain screen for debugging
        Brain.Screen.printAt(10, 30, "Target Angle: %.2f", targetAngle);
        Brain.Screen.printAt(10, 50, "Current Angle: %.2f", currentAngle);
        Brain.Screen.printAt(10, 70, "Error: %.2f", error);
        Brain.Screen.printAt(10, 90, "Speed: %.2f", speed);

        task::sleep(20); // Small delay to prevent overwhelming the CPU
    }

    // Stop the motors
    LeftMotor1.stop(brake);
    LeftMotor2.stop(brake);
    LeftMotor3.stop(brake);
    RightMotor1.stop(brake);
    RightMotor2.stop(brake);
    RightMotor3.stop(brake);
}

*/



//Launch Control

// Slip control acceleration function for all motors using arrays
void slipControlV1(double maxLeftSpeed, double maxRightSpeed, double startingSpeed) {
    // Arrays to hold the left and right motors
    //vex::motor leftMotors[] = {LeftMotor1, LeftMotor2, LeftMotor3};
    //vex::motor rightMotors[] = {RightMotor1, RightMotor2, RightMotor3};

    // Arrays to hold the current speeds of the motors
    double currentSpeedLeft[3] = {startingSpeed, startingSpeed, startingSpeed}; // Start at 20% power instead of 0
    double currentSpeedRight[3] = {startingSpeed, startingSpeed, startingSpeed}; // Start at 20% power instead of 0

    // Flags to check if each motor has reached max speed
    bool isAtMaxSpeedLeft[3] = {false, false, false};
    bool isAtMaxSpeedRight[3] = {false, false, false};

    //wheel circumference
   // double wheelCircumferenceCM = 25.93385; // Get the circumference of the motorized wheel
// Constants
//const double encoderWheelCircumferenceCM = 15.9593; // Circumference of the encoder wheel in cm


 //   const double accelerationFactor = 1.05; // Adjust this factor for faster or slower exponential acceleration
   // int loopCounter = 0;
    //const int maxLoops = 100; // Limit for the control loop

    // Initialize all motors to the minimum starting power and start spinning
    for (int i = 0; i < 3; i++) {
        leftMotors[i].setVelocity(currentSpeedLeft[i], vex::velocityUnits::pct);
        rightMotors[i].setVelocity(currentSpeedRight[i], vex::velocityUnits::pct);
        leftMotors[i].spin(vex::directionType::fwd);
        rightMotors[i].spin(vex::directionType::fwd);
    }

    // Main control loop with exponential acceleration
    while (!(isAtMaxSpeedLeft[0] && isAtMaxSpeedLeft[1] && isAtMaxSpeedLeft[2] &&
         isAtMaxSpeedRight[0] && isAtMaxSpeedRight[1] && isAtMaxSpeedRight[2])) {

        // Retrieve the encoder speed in cm/s based on its circumference
        double encoderSpeed = passiveEncoderLeft.velocity(vex::velocityUnits::rpm) * encoderWheelCircumferenceCM / 60.0;

        // Loop through all left motors
        for (int i = 0; i < 3; i++) {
            double motorSpeed = leftMotors[i].velocity(vex::velocityUnits::rpm) * wheelCircumferenceCM / 60.0;

            if (!isAtMaxSpeedLeft[i]) {
                if (isSlipping(motorSpeed, encoderSpeed)) {
                    currentSpeedLeft[i] *= (1.0 / accelerationFactor); // Decrease speed if slipping
                } else {
                    currentSpeedLeft[i] *= accelerationFactor; // Increase speed exponentially
                }

                if (currentSpeedLeft[i] >= maxLeftSpeed) {
                    currentSpeedLeft[i] = maxLeftSpeed;
                    isAtMaxSpeedLeft[i] = true;
                }

                leftMotors[i].setVelocity(currentSpeedLeft[i], vex::velocityUnits::pct);
            }
        }

        // Loop through all right motors
        for (int i = 0; i < 3; i++) {
            double motorSpeed = rightMotors[i].velocity(vex::velocityUnits::rpm) * wheelCircumferenceCM / 60.0;

            if (!isAtMaxSpeedRight[i]) {
                if (isSlipping(motorSpeed, encoderSpeed)) {
                    currentSpeedRight[i] *= (1.0 / accelerationFactor); // Decrease speed if slipping
                } else {
                    currentSpeedRight[i] *= accelerationFactor; // Increase speed exponentially
                }

                if (currentSpeedRight[i] >= maxRightSpeed) {
                    currentSpeedRight[i] = maxRightSpeed;
                    isAtMaxSpeedRight[i] = true;
                }

                rightMotors[i].setVelocity(currentSpeedRight[i], vex::velocityUnits::pct);
            }
        }

        //loopCounter++;
        vex::task::sleep(20); // Small delay for stability
    }

    //    vex::task::sleep(500); 

    // Stop all motors with brake mode applied after reaching max speed
    // Stop the motors
// Set brake mode to brake 
/*
    LeftMotor1.setBrake(brakeType::brake); 
    LeftMotor2.setBrake(brakeType::brake); 
    LeftMotor3.setBrake(brakeType::brake); 
    RightMotor1.setBrake(brakeType::brake); 
    RightMotor2.setBrake(brakeType::brake); 
    RightMotor3.setBrake(brakeType::brake); 
 */   

}


 void pidStraightDistanceLaunch(double targetHeading, double targetDistanceCm, double maxSpeed, double kp_heading, double ki_heading, double kd_heading, double kp_distance, double ki_distance, double kd_distance, double minSpeed, brakeType brakeMode) { 
    
    // Reset the encoders to start counting from zero
    passiveEncoderLeft.resetPosition();
    passiveEncoderRight.resetPosition();  

    double deadzone = 1;
    double normTargetHeading = normalizeHeading(targetHeading);

    // Initialize PID controllers
    PID headingPID(kp_heading, ki_heading, kd_heading);
    PID distancePID(kp_distance, ki_distance, kd_distance);
    
    headingPID.pidReset();
    distancePID.pidReset();
    
    //Convert % Speed input to voltage with max voltage of 12
    double maxSpeedVoltage = maxSpeed * 0.01 * 12; //max speed is entered as percentage, multiply by .01 to convert it to decimals and multiply with max 12 volts
    double minSpeedVoltage = minSpeed * 0.01 * 12; 

    int stabilityCounter = 0;                      // Counts consecutive iterations within target zone
    const int stabilityThreshold = 4;              // Number of consecutive stable iterations required to exit loop
    bool hasStabilizedOnce = false; // Flag to track if stabilityCounter has been incremented at least once

    double currentDistance = 0;

    //launchControl (60, 60, 20);
        
    // Loop to continuously adjust motor power based on PID control 
while (stabilityCounter < stabilityThreshold) {
        
        // Calculate the heading correction using the PID controller 
        double headingCorrection = headingPID.calculate(normTargetHeading, normalizeHeading(InertialSensor.heading()));

        currentDistance = (passiveEncoderLeft.position(degrees) + passiveEncoderRight.position(degrees) ) / 2 * (encoderWheelCircumferenceCM / 360.0);
        
         // Calculate the PID output for distance control
        double distanceCorrection = distancePID.calculate(targetDistanceCm, currentDistance);

        // **Check if Distance Error is Within Target Zone and Update Stability Counter**
        double distanceError = targetDistanceCm - currentDistance;

        if (fabs(distanceError) <= deadzone) {                     // Within target zone
        stabilityCounter++;                                    // Increment counter
        hasStabilizedOnce = true;                  // Set flag indicating at least one stable iteration
} 

 if (hasStabilizedOnce) {
            // Set the speed to minSpeed after the first stabilization
            distanceCorrection = (distanceCorrection > 0) ? minSpeedVoltage : -minSpeedVoltage;
            headingCorrection = headingCorrection * minSpeed / maxSpeed;
        } else {
            // Clamp the speed to maxSpeed before stabilization occurs
            if (distanceCorrection > maxSpeedVoltage) {
                distanceCorrection = maxSpeedVoltage;
            } else if (distanceCorrection < -maxSpeedVoltage) {
                distanceCorrection = -maxSpeedVoltage;
            }

            // Ensure a minimum speed to maintain movement before stabilization
            if (distanceCorrection >= 0 && distanceCorrection < minSpeedVoltage) {
                distanceCorrection = minSpeedVoltage;
            } else if (distanceCorrection <= 0 && distanceCorrection > -minSpeedVoltage) {
                distanceCorrection = -minSpeedVoltage;
            }
        }

                // Reverse heading correction when moving backwards
       // headingCorrection *= (distanceCorrection >= 0) ? 1 : -1;
     /* 
        if (targetDistanceCm < 0) {
        headingCorrection = -headingCorrection;
        }
    */

     
        // Adjust motor speeds based on the heading and distance corrections
        double leftMotorSpeed = distanceCorrection + headingCorrection; 
        double rightMotorSpeed = distanceCorrection - headingCorrection;
        
        for (int i = 0; i < 3; i++) {   
        leftMotor[i].spin(forward, leftMotorSpeed, voltageUnits::volt);
        rightMotor[i].spin(forward, rightMotorSpeed, voltageUnits::volt); 
        }
        

       // double LeftDistance = (passiveEncoderLeft.position(degrees) * (encoderWheelCircumferenceCM / 360.0));
       // double RightDistance = (passiveEncoderRight.position(degrees) * (encoderWheelCircumferenceCM / 360.0));
        double rotationDegrees = passiveEncoderLeft.position(vex::rotationUnits::deg);
        Brain.Screen.printAt(10, 20, "Rotation Sensor: %.2f degrees", rotationDegrees);
        Brain.Screen.printAt(10, 40, "Heading Correction: %.4f", headingCorrection);
        Brain.Screen.printAt(10, 80, "Norm Target Heading: %.2f", normTargetHeading);
        Brain.Screen.printAt(10, 100, "Norm Heading: %.2f", normalizeHeading(InertialSensor.heading()));
        Brain.Screen.printAt(10, 120, "Lt Motor Speed: %.2f", leftMotorSpeed);
        Brain.Screen.printAt(10, 140, "Rt Motor Speed: %.2f", rightMotorSpeed);
       // Brain.Screen.printAt(10, 140, "Rt Motor Speed: %.2f", rightMotorSpeed);
       // Brain.Screen.printAt(10, 170, "Max Speed Voltage: %.2f", maxSpeedVoltage);
       // Brain.Screen.printAt(10, 190, "Min Speed Voltage: %.2f", minSpeedVoltage);

        vex::task::sleep(20);
    }
    
    // Stop the motors
    LeftMotor1.stop(brakeMode);
    LeftMotor2.stop(brakeMode);
    LeftMotor3.stop(brakeMode);
    RightMotor1.stop(brakeMode);
    RightMotor2.stop(brakeMode);
    RightMotor3.stop(brakeMode);

    //task::sleep(800);  // Small delay to prevent overwhelming the CPU
    currentDistance = (passiveEncoderLeft.position(degrees) + passiveEncoderRight.position(degrees) ) / 2 * (encoderWheelCircumferenceCM / 360.0);
    // Debug print: Stopping motors
    Brain.Screen.clearLine(8);
    Brain.Screen.setCursor(8, 1);
    Brain.Screen.print("Distance Complete");
    Brain.Screen.print("Current Distance: %.2f", currentDistance);

}
     
/*
//PID Straight Distance Launch, everyworked but did not test after insertion of launchMode void
 void pidStraightDistanceLaunch(double targetDistanceCm, double maxSpeed, double targetHeading, bool (*exitCondition)()) { 
    // PID coefficients for heading correction 
    double kp_heading = .2; // Proportional coefficient 
    double ki_heading = 0.0; // Integral coefficient 
    double kd_heading = 0.0; // Derivative coefficient 

    // PID coefficients for distance correction
    double kp_distance = .1; // Proportional coefficient 
    double ki_distance = 0.0; // Integral coefficient 
    double kd_distance = 0.0; // Derivative coefficient 

    double deadzone = 1.2;
    double minSpeed = 7.5;

    double wheelCircumferenceCm;
    double gearRatio;
    //gearRatio = 6.0; // 6:1 gear ratio

    // Initialize PID controllers
    PID headingPID(kp_heading, ki_heading, kd_heading);
    PID distancePID(kp_distance, ki_distance, kd_distance);
    
    headingPID.pidReset();
    distancePID.pidReset();

    // Get wheel properties from utils
    getWheelProperties(wheelCircumferenceCm,gearRatio);

    //double distancePerDegreeMotorRotation = gearRatio * wheelCircumferenceCm / 360.0;
     double distancePerMotorTick = wheelCircumferenceCm / 300.0;

    // Calculate the number of motor rotations needed to travel the target distance in cm
    double targetRotations = targetDistanceCm / distancePerMotorTick;

    // Set brake mode to brake 
    LeftMotor1.setBrake(brakeType::brake); 
    LeftMotor2.setBrake(brakeType::brake); 
    LeftMotor3.setBrake(brakeType::brake); 
    RightMotor1.setBrake(brakeType::brake); 
    RightMotor2.setBrake(brakeType::brake); 
    RightMotor3.setBrake(brakeType::brake); 

    // *** Reset motor positions before entering the main loop ***
    for (int i = 0; i < 3; i++) {      
    leftMotor[i].resetPosition();
    rightMotor[i].resetPosition();
    }


    launchControl (60, 60, 20);
        
    // Loop to continuously adjust motor power based on PID control
    while (!exitCondition()) {
        // Get the current heading and normalize it 
        double currentHeading = InertialSensor.heading(); 
        double normalizedCurrentHeading = normalizeHeading(currentHeading); 
        double normalizedTargetHeading = normalizeHeading(targetHeading); 
        
        // Calculate the heading correction using the PID controller 
        double headingCorrection = headingPID.calculate(normalizedTargetHeading, normalizedCurrentHeading);
        
        // Get the current distance
        double averageMotorPosition = (
                                       LeftMotor2.position(rotationUnits::raw) + 
                                       LeftMotor3.position(rotationUnits::raw) + 
                                       RightMotor2.position(rotationUnits::raw) + 
                                       RightMotor3.position(rotationUnits::raw)) / 4.0;
        
         // Calculate the PID output for distance control
        double distanceCorrection = distancePID.calculate(targetRotations, averageMotorPosition);

        // If the PID output is greater than maxSpeed, clamp it to maxSpeed
    if (distanceCorrection > maxSpeed) {
        distanceCorrection = maxSpeed;
    } 
    // If the PID output is less than -maxSpeed, clamp it to -maxSpeed (for reverse direction)
    else if (distanceCorrection < -maxSpeed) { 
        distanceCorrection = -maxSpeed;
    }

    // If the PID output is positive but less than minSpeed, set it to minSpeed to ensure movement
    if (distanceCorrection >= 0 && distanceCorrection < minSpeed) {
        distanceCorrection = minSpeed;
    } 
    // If the PID output is negative but greater than -minSpeed, set it to -minSpeed (for reverse movement)
    else if (distanceCorrection <= 0 && distanceCorrection > -minSpeed) {
        distanceCorrection = -minSpeed;
    }


        // Adjust motor speeds based on the heading and distance corrections
        double leftMotorSpeed = distanceCorrection + headingCorrection; 
        double rightMotorSpeed = distanceCorrection - headingCorrection;
        
        LeftMotor1.spin(directionType::fwd, leftMotorSpeed, velocityUnits::pct);
        LeftMotor2.spin(directionType::fwd, leftMotorSpeed, velocityUnits::pct);
        LeftMotor3.spin(directionType::fwd, leftMotorSpeed, velocityUnits::pct);
        RightMotor1.spin(directionType::fwd, rightMotorSpeed, velocityUnits::pct);
        RightMotor2.spin(directionType::fwd, rightMotorSpeed, velocityUnits::pct);
        RightMotor3.spin(directionType::fwd, rightMotorSpeed, velocityUnits::pct);
        
        double rotationDegrees = passiveEncoderLeft.position(vex::rotationUnits::deg);
        Brain.Screen.printAt(10, 20, "Rotation Sensor: %.2f degrees", rotationDegrees);
                // Calculate the distance traveled in cm
        double distanceTravelledCm = averageMotorPosition * distancePerMotorTick;

        // Print the distance traveled to the Brain screen for debugging

        Brain.Screen.printAt(10, 40, "Target Distance: %.4f", targetDistanceCm);
        Brain.Screen.printAt(10, 60, "Distance travelled: %.2f", distanceTravelledCm);
        Brain.Screen.printAt(10, 80, "Target Rotation: %.2f", targetRotations);
        Brain.Screen.printAt(10, 100, "avg motor position: %.2f", averageMotorPosition);
        Brain.Screen.printAt(10, 120, "Lt Motor Speed: %.2f", leftMotorSpeed);
        Brain.Screen.printAt(10, 140, "Rt Motor Speed: %.2f", rightMotorSpeed);

if ((targetRotations - averageMotorPosition) <= deadzone) {
//if ((targetRotations - averageMotorPosition) <= 1) {
    break; // Exit the loop if the target distance is reached within the dead zone
}


        vex::task::sleep(20);
    }
    
    // Stop the motors
    LeftMotor1.stop();
    LeftMotor2.stop();
    LeftMotor3.stop();
    RightMotor1.stop();
    RightMotor2.stop();
    RightMotor3.stop();
}
/*
void spotTurn(double turnDegrees, double maxSpeed, double minSpeed, double kp_heading, double ki_heading, double kd_heading) {
    // Create a local PID controller for heading correction
    PID turnHeadingPID(kp_heading, ki_heading, kd_heading);

    // Get the current heading
    double currentHeading = InertialSensor.heading(degrees);

    // Calculate the target heading by adding the relative turn degrees to the current heading
    double targetHeading = currentHeading + turnDegrees;
    
    double error = targetHeading - currentHeading;
    double turnSpeed;

    // Reset the PID controller before starting the turn
    turnHeadingPID.pidReset();

    // Debug print: Initial values
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Starting Turn");
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("Initial Heading: %.2f", currentHeading);
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("Target Heading: %.2f", targetHeading);

    while (fabs(error) > 1) {  // Continue until the error is within 1 degree
        // Update current heading
        currentHeading = InertialSensor.heading(degrees);
        error = targetHeading - currentHeading;

        // Use the PID controller to calculate the motor speed based on the error
        turnSpeed = turnHeadingPID.calculate(targetHeading, currentHeading);

        // Limit the speed to maxSpeed and minSpeed
        if (fabs(turnSpeed) > maxSpeed) {
            turnSpeed = (turnSpeed > 0) ? maxSpeed : -maxSpeed;
        } else if (fabs(turnSpeed) < minSpeed) {
            turnSpeed = (turnSpeed > 0) ? minSpeed : -minSpeed;
        }

        // Debug print: Current values during turn
        Brain.Screen.clearLine(4);
        Brain.Screen.setCursor(4, 1);
        Brain.Screen.print("Current Heading: %.2f", currentHeading);
        Brain.Screen.clearLine(5);
        Brain.Screen.setCursor(5, 1);
        Brain.Screen.print("Error: %.2f", error);
        Brain.Screen.clearLine(6);
        Brain.Screen.setCursor(6, 1);
        Brain.Screen.print("Turn Speed: %.2f", turnSpeed);

        // Set motors to spin based on the sign of the error
        if (error > 0) {  // Right turn (clockwise)
            LeftMotor1.spin(forward, turnSpeed, pct);
            LeftMotor2.spin(forward, turnSpeed, pct);
            LeftMotor3.spin(forward, turnSpeed, pct);
            RightMotor1.spin(reverse, turnSpeed, pct);
            RightMotor2.spin(reverse, turnSpeed, pct);
            RightMotor3.spin(reverse, turnSpeed, pct);
        } else {  // Left turn (counterclockwise)
            LeftMotor1.spin(reverse, turnSpeed, pct);
            LeftMotor2.spin(reverse, turnSpeed, pct);
            LeftMotor3.spin(reverse, turnSpeed, pct);
            RightMotor1.spin(forward, turnSpeed, pct);
            RightMotor2.spin(forward, turnSpeed, pct);
            RightMotor3.spin(forward, turnSpeed, pct);
        }

        task::sleep(20);  // Small delay to prevent overwhelming the CPU
    }

    // Debug print: Stopping motors
    Brain.Screen.clearLine(8);
    Brain.Screen.setCursor(8, 1);
    Brain.Screen.print("Turn Complete");

    // Stop the motors with brake mode to ensure a quick stop
    LeftMotor1.stop(brakeType::brake);
    LeftMotor2.stop(brakeType::brake);
    LeftMotor3.stop(brakeType::brake);
    RightMotor1.stop(brakeType::brake);
    RightMotor2.stop(brakeType::brake);
    RightMotor3.stop(brakeType::brake);
}
*/

// Helper function to normalize angle between 0 and 360 degrees using fmod
/*
double normalizeHeading360(double heading) {
    heading = fmod(heading, 360);  // Get the modulus of the heading
    if (heading < 0) {
        heading += 360;  // If the result is negative, add 360 to make it positive
    }
    return heading;
}
*/
/*
//Spot Turn - works but can overshoot and not correct
void spotTurn(double targetHeading, double maxSpeed, double minSpeed, double kp_heading, double ki_heading, double kd_heading) {
    // Create a local PID controller for heading correction
    PID turnHeadingPID(kp_heading, ki_heading, kd_heading);

    // Get the current heading and normalize it
    double normTargetHeading = normalizeHeading(targetHeading);
    double currentHeading = normalizeHeading(InertialSensor.heading(degrees));
   
    // Calculate the initial error
    double error = normTargetHeading - currentHeading;

    // Determine whether the initial error is positive or negative
    bool isPositiveError = error > 0;

   // double turnSpeed;

    // Reset the PID controller before starting the turn
    turnHeadingPID.pidReset();

    // Debug print: Initial values
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Starting Turn");
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("Initial Heading: %.2f", currentHeading);
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("Target Heading: %.2f", targetHeading);

while (isPositiveError ? (error > 1 || error < -1) : (error < -1 || error > 1)) {  // Use the ternary operator to switch between conditions
        // Update current heading and normalize it using fmod
        currentHeading = normalizeHeading(InertialSensor.heading(degrees));
        error = normTargetHeading - currentHeading;

        // Use the PID controller to calculate the motor speed based on the error
        double turnSpeed = (turnHeadingPID.calculate(normTargetHeading, currentHeading));


        // Limit the speed to maxSpeed and minSpeed
        if (fabs(turnSpeed) > maxSpeed) {
            turnSpeed = (turnSpeed > 0) ? maxSpeed : -maxSpeed;
        } else if (fabs(turnSpeed) < minSpeed) {
            turnSpeed = (turnSpeed > 0) ? minSpeed : -minSpeed;
        }

 double gyroHeading = InertialSensor.heading(degrees);
        // Debug print: Current values during turn
        Brain.Screen.clearLine(4);
        Brain.Screen.setCursor(4, 1);
        Brain.Screen.print("Current Heading: %.2f", currentHeading);
        Brain.Screen.clearLine(5);
        Brain.Screen.setCursor(5, 1);
        Brain.Screen.print("Gyro Heading: %.2f", gyroHeading);
        Brain.Screen.clearLine(6);
        Brain.Screen.setCursor(6, 1);
        Brain.Screen.print("Turn Speed: %.2f", turnSpeed);

        // Set motors to spin based on the direction specified
            LeftMotor1.spin(forward, turnSpeed, pct);
            LeftMotor2.spin(forward, turnSpeed, pct);
            LeftMotor3.spin(forward, turnSpeed, pct);
            RightMotor1.spin(forward, -turnSpeed, pct);
            RightMotor2.spin(forward, -turnSpeed, pct);
            RightMotor3.spin(forward, -turnSpeed, pct);
    

        task::sleep(20);  // Small delay to prevent overwhelming the CPU
    }

    // Debug print: Stopping motors
    Brain.Screen.clearLine(8);
    Brain.Screen.setCursor(8, 1);
    Brain.Screen.print("Turn Complete");

    // Stop the motors with brake mode to ensure a quick stop
    LeftMotor1.stop(brakeType::brake);
    LeftMotor2.stop(brakeType::brake);
    LeftMotor3.stop(brakeType::brake);
    RightMotor1.stop(brakeType::brake);
    RightMotor2.stop(brakeType::brake);
    RightMotor3.stop(brakeType::brake);
}
*/

void spotTurn(double targetHeading, double maxSpeed, double minSpeed, double kp_heading, double ki_heading, double kd_heading) { 
    // Create a local PID controller for heading correction
    PID turnHeadingPID(kp_heading, ki_heading, kd_heading);

    // Get the current heading and normalize it
    double normTargetHeading = normalizeHeading(targetHeading);
    double currentHeading = normalizeHeading(InertialSensor.heading(degrees));
   
    // Calculate the initial error
    double error = normTargetHeading - currentHeading;

        // Handle the special case where error is exactly 180°
    if (error == 180.0) {
        // Force the robot to turn right (clockwise)
        error = -180.0; // Alternatively, set to 180.0 to force left turn
    }

    // Reset the PID controller before starting the turn
    turnHeadingPID.pidReset();

    // **Initialize Stability Counter Variables**
    int stabilityCounter = 0;                      // Counts consecutive iterations within target zone
    const int stabilityThreshold = 8;              // Number of consecutive stable iterations required to exit loop
    int resetCounter = 0;                           // Counts the number of stability counter resets
    //const int maxResets = 10;                        // Maximum number of allowed resets to prevent infinite loop
    bool hasStabilizedOnce = false; // Flag to track if stabilityCounter has been incremented at least once

    // Debug print: Initial values
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Starting Turn");
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("Initial Heading: %.2f", currentHeading);
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("Target Heading: %.2f", targetHeading);

    // **Start Control Loop with Stability Counter**
    while (stabilityCounter < stabilityThreshold) {  // Changed loop condition
        // Update current heading and normalize it
        currentHeading = normalizeHeading(InertialSensor.heading(degrees));
        error = normTargetHeading - currentHeading;

        // **Normalize the error to [-180°, 180°)**
        if (error > 180.0) {
            error -= 360.0;
        } else if (error < -180.0) {
            error += 360.0;
        }

        // **Check if Error is Within Target Zone and Update Stability Counter**
if (fabs(error) <= 1.0) {                     // Within target zone
    stabilityCounter++;                        // Increment counter
    hasStabilizedOnce = true;                  // Set flag indicating at least one stable iteration
/*
} else {
    if (hasStabilizedOnce) {                    // Only reset if stabilityCounter has been incremented before
        stabilityCounter = 0;                  // Reset counter if outside target zone
        resetCounter++;                         // Increment reset counter
        if (resetCounter >= maxResets) {        // Check if max resets reached
            break;                              // Exit loop to prevent infinite loop
        }
    }
    // If hasStabilizedOnce is false, do not count resets
    */
}

        // Use the PID controller to calculate the motor speed based on the error
        double turnSpeed = turnHeadingPID.calculate(normTargetHeading, currentHeading);

        // Limit the speed to maxSpeed and minSpeed
        if (fabs(turnSpeed) > maxSpeed) {
            turnSpeed = (turnSpeed > 0) ? maxSpeed : -maxSpeed;
        } else if (fabs(turnSpeed) < minSpeed) {
            turnSpeed = (turnSpeed > 0) ? minSpeed : -minSpeed;
        }

        double gyroHeading = InertialSensor.heading(degrees);
        // Debug print: Current values during turn
        Brain.Screen.clearLine(4);
        Brain.Screen.setCursor(4, 1);
        Brain.Screen.print("Current Heading: %.2f", currentHeading);
        Brain.Screen.clearLine(5);
        Brain.Screen.setCursor(5, 1);
        Brain.Screen.print("Gyro Heading: %.2f", gyroHeading);
        Brain.Screen.clearLine(6);
        Brain.Screen.setCursor(6, 1);
        Brain.Screen.print("Turn Speed: %.2f", turnSpeed);
        Brain.Screen.clearLine(7);
        Brain.Screen.setCursor(7, 1);
        Brain.Screen.print("Stability Count: %d", stabilityCounter);  // **Added Line**
        Brain.Screen.clearLine(8);
        Brain.Screen.setCursor(8, 1);
        Brain.Screen.print("Reset Counter: %d", resetCounter);          // **Added Line**

        // Set motors to spin based on the direction specified
        LeftMotor1.spin(forward, turnSpeed, pct);
        LeftMotor2.spin(forward, turnSpeed, pct);
        LeftMotor3.spin(forward, turnSpeed, pct);
        RightMotor1.spin(forward, -turnSpeed, pct);
        RightMotor2.spin(forward, -turnSpeed, pct);
        RightMotor3.spin(forward, -turnSpeed, pct);

        task::sleep(20);  // Small delay to prevent overwhelming the CPU
    }

     // Stop the motors with brake mode to ensure a quick stop
    LeftMotor1.stop(brakeType::brake);
    LeftMotor2.stop(brakeType::brake);
    LeftMotor3.stop(brakeType::brake);
    RightMotor1.stop(brakeType::brake);
    RightMotor2.stop(brakeType::brake);
    RightMotor3.stop(brakeType::brake); 
    
     // task::sleep(500);  // Small delay to prevent overwhelming the CPU
    currentHeading = normalizeHeading(InertialSensor.heading(degrees));
    // Debug print: Stopping motors
    Brain.Screen.clearLine(8);
    Brain.Screen.setCursor(8, 1);
    Brain.Screen.print("Turn Complete");
    Brain.Screen.print("Current Heading: %.2f", currentHeading);

}


// ABS braking function for all motors using arrays and lock-up detection
void absControl(double targetLeftVoltage, double targetRightVoltage) {
    // Arrays to hold the left and right motors
    vex::motor leftMotors[] = {LeftMotor1, LeftMotor2, LeftMotor3};
    vex::motor rightMotors[] = {RightMotor1, RightMotor2, RightMotor3};

    // Flags to check if each motor has reached the target voltage
    bool isAtTargetLeft[3] = {false, false, false};
    bool isAtTargetRight[3] = {false, false, false};

    // Initialize individual counters for each motor
    int leftMotorTargetCounter[3] = {0, 0, 0};
    int rightMotorTargetCounter[3] = {0, 0, 0};

    // **NEW:** Initialize individual lock-up counters for each motor
    int leftMotorLockupCounter[3] = {0, 0, 0};  // For left motors
    int rightMotorLockupCounter[3] = {0, 0, 0}; // For right motors

    // **NEW:** Initialize individual brake counters for each motor
    int leftMotorBrakeCounter[3] = {0, 0, 0};  // For left motors
    int rightMotorBrakeCounter[3] = {0, 0, 0}; // For right motors

    // Constants (ensure these are defined appropriately in your context)
    //const double wheelCircumferenceCM = 25.93385;        // Circumference of the motorized wheel in cm
    //const double encoderWheelCircumferenceCM = 15.9593;  // Circumference of the encoder wheel in cm

    // Initialize all motors to the target voltage and start spinning
    for (int i = 0; i < 3; i++) {
        leftMotors[i].spin(vex::directionType::fwd, targetLeftVoltage, vex::voltageUnits::volt);
        rightMotors[i].spin(vex::directionType::fwd, targetRightVoltage, vex::voltageUnits::volt);
    }


    // Main control loop
    while (!(isAtTargetLeft[0] && isAtTargetLeft[1] && isAtTargetLeft[2] &&
             isAtTargetRight[0] && isAtTargetRight[1] && isAtTargetRight[2])) {

        // Increment the loop counter
        // Retrieve the encoder speeds using the utility function
        double encoderSpeedLeft = getEncoderSpeed(passiveEncoderLeft);
        double encoderSpeedRight = getEncoderSpeed(passiveEncoderRight);

        // Loop through all motors using a single loop for both left and right motors
        for (int i = 0; i < 3; i++) {
            // **Check Left Motor i**
            if (!isAtTargetLeft[i]) {
                // Calculate the current motor speed in cm/s
                double motorSpeedLeft = leftMotors[i].velocity(vex::velocityUnits::rpm) * wheelCircumferenceCM / 60.0;

                // Calculate target speed equivalent based on voltage (assuming linear relationship)
                double targetSpeedLeft = targetLeftVoltage * wheelCircumferenceCM / 60.0;

                // Condition 1: Encoder speed has reached or is below the target speed equivalent
                if (encoderSpeedLeft <= targetSpeedLeft) {
                    leftMotors[i].setBrake(brakeType::coast);
                    leftMotors[i].spin(vex::directionType::fwd, targetLeftVoltage, vex::voltageUnits::volt);
                    isAtTargetLeft[i] = true;  
                    leftMotorTargetCounter[i]++;                  
                    //Brain.Screen.clearLine(8);
                    Brain.Screen.setCursor(6, 1); // Set cursor position (line 8, column 1)
                    Brain.Screen.print("L-Motor Target %d, %d, %d, Encoder: %.2f", leftMotorTargetCounter[0], leftMotorTargetCounter[1], leftMotorTargetCounter[2], encoderSpeedLeft);
                }
        
                // Condition 2: Potential lock-up detected
                else if (isLocking(motorSpeedLeft, encoderSpeedLeft)) {
                    // Prevent lock-up by maintaining target voltage
                    leftMotors[i].setBrake(brakeType::coast);
                    leftMotors[i].spin(vex::directionType::fwd, targetLeftVoltage, vex::voltageUnits::volt);
                    isAtTargetLeft[i] = false; // Ensure flag remains false
                                        //Brain.Screen.clearLine(8);
                    leftMotorLockupCounter[i]++;                       
                    Brain.Screen.setCursor(7, 1); // Set cursor position (line 8, column 1)
                    Brain.Screen.print("L-Motor Lockup %d, %d, %d, Encoder: %.2f", leftMotorLockupCounter[0], leftMotorLockupCounter[1], leftMotorLockupCounter[2], encoderSpeedLeft);
                }
                // Condition 3: Safe to stop motor with brake
                else {
                    leftMotors[i].stop(vex::brakeType::brake);
                    isAtTargetLeft[i] = false; // Ensure flag remains false
                                // Debug statement to check if code reaches Left Motor control
            leftMotorBrakeCounter[i]++;                       
            //Brain.Screen.clearLine(8);
            Brain.Screen.setCursor(8, 1); // Set cursor position (line 8, column 1)
            Brain.Screen.print("L-Motor Brake %d, %d, %d, Encoder: %.2f", leftMotorBrakeCounter[0], leftMotorBrakeCounter[1], leftMotorBrakeCounter[2], encoderSpeedLeft);
                }
            }

            // **Check Right Motor i**
            if (!isAtTargetRight[i]) {
                // Calculate the current motor speed in cm/s
                double motorSpeedRight = rightMotors[i].velocity(vex::velocityUnits::rpm) * wheelCircumferenceCM / 60.0;

                // Calculate target speed equivalent based on voltage (assuming linear relationship)
                double targetSpeedRight = targetRightVoltage * wheelCircumferenceCM / 60.0;

                // Condition 1: Encoder speed has reached or is below the target speed equivalent
                if (encoderSpeedRight <= targetSpeedRight) {
                    rightMotors[i].setBrake(brakeType::coast);
                    rightMotors[i].spin(vex::directionType::fwd, targetRightVoltage, vex::voltageUnits::volt);
                    isAtTargetRight[i] = true;
                    rightMotorTargetCounter[i]++;    
                    //Brain.Screen.clearLine(10);
                    Brain.Screen.setCursor(10, 1); // Set cursor position (line 8, column 1)
                    Brain.Screen.print("R-Motor Target %d, %d, %d, Encoder: %.2f", rightMotorTargetCounter[0], rightMotorTargetCounter[1], rightMotorTargetCounter[2], encoderSpeedRight);
                }
                // Condition 2: Potential lock-up detected
                else if (isLocking(motorSpeedRight, encoderSpeedRight)) {
                    // Prevent lock-up by maintaining target voltage
                    rightMotors[i].setBrake(brakeType::coast);
                    rightMotors[i].spin(vex::directionType::fwd, targetRightVoltage, vex::voltageUnits::volt);
                    isAtTargetRight[i] = false; // Ensure flag remains false
                    rightMotorLockupCounter[i]++;   
                                        //Brain.Screen.clearLine(10);
                    Brain.Screen.setCursor(11, 1); // Set cursor position (line 8, column 1)
                    Brain.Screen.print("R-Motor Lockup %d, %d, %d, Encoder: %.2f", rightMotorLockupCounter[0], rightMotorLockupCounter[1], rightMotorLockupCounter[2], encoderSpeedRight);
                }
                // Condition 3: Safe to stop motor with brake
                else {
                    rightMotors[i].stop(vex::brakeType::brake);
                    isAtTargetRight[i] = false; // Ensure flag remains false
                                // Debug statement to check if code reaches Right Motor control
            rightMotorBrakeCounter[i]++;                       
            //Brain.Screen.clearLine(10);
            Brain.Screen.setCursor(12, 1); // Set cursor position (line 8, column 1)
            Brain.Screen.print("R-Motor Brake %d, %d, %d, Encoder: %.2f", rightMotorBrakeCounter[0], rightMotorBrakeCounter[1], rightMotorBrakeCounter[2], encoderSpeedRight);
                }
                
            }
        }

        // Small delay for stability and to prevent CPU hogging
        vex::task::sleep(20); // 20 ms delay
    }

    // **Mandatory:** Set brake mode for all motors after reaching target voltages
    for (int i = 0; i < 3; i++) {
        leftMotors[i].setBrake(vex::brakeType::brake);
        rightMotors[i].setBrake(vex::brakeType::brake);
    }
}
/*
//Launch Control 2.5
// Launch Control Function: Adjusts speeds based on acceleration and slipping
double launchControl(double targetSpeed, vex::motor& motor, vex::rotation& encoder) {
       double minSpeed = 40;   
       double currentSpeed = getMotorSpeed(motor);// Function to calculate motor encoder speed in cm per second    
        //double currentSpeed = 20;// Function to calculate motor encoder speed in cm per second    
    
    // Adjust left motors
      //  if (isAccelerating(targetSpeed, currentSpeed)) {
            // Retrieve the encoder speed in cm/s based on its circumference
           double encoderSpeed = getEncoderSpeed(encoder);
            //double encoderSpeed = 100;
     
            // Check if slipping
            if (isSlipping(currentSpeed, encoderSpeed)) {
                // Decrease speed if slipping
                currentSpeed /= accelerationFactor;
        
            } else {
                // Increase speed towards target smoothly
                currentSpeed *= accelerationFactor;

               // Clamp currentSpeed between minSpeed and targetSpeed, applying the sign of targetSpeed to currentSpeed
                currentSpeed = std::copysign(std::max(minSpeed, std::min(std::abs(currentSpeed), targetSpeed)), targetSpeed);


            }
       // } else {
            // Not accelerating, set to target speed
           // currentSpeed = targetSpeed;
      //  }
    // Add the return statement to return the updated currentSpeed
    return currentSpeed;
}
*/

// LaunchControl.h

// constants.h or utils.cpp
//const double accelerationFactor = 1.05; // Defined as a global constant

// Constructor implementation
LaunchControl::LaunchControl(vex::motor& motor, vex::rotation& encoder, double slipThresholdValue)
    : motor(motor), encoder(encoder), slipThreshold(slipThresholdValue) {} // Initialized slipThreshold


// adjustSpeed method implementation
double LaunchControl::adjustSpeed(double targetPower) {
   double motorRPM =  motor.velocity(vex::velocityUnits::rpm);
    //double motorRPM = 0;
    double encoderRPMScaled = encoder.velocity(vex::velocityUnits::rpm) * (wheelCircumferenceCM / encoderWheelCircumferenceCM); // Adjust for wheel difference if needed
    double motorPower = 0;
   // Brain.Screen.printAt(10, 100, "Current Speed: %.2f, Encoder Speed: %.2f", currentSpeed, encoderSpeed);

if ((motorRPM > 0 && encoderRPMScaled >= 0) || (motorRPM < 0 && encoderRPMScaled <= 0)) {// only check for slip if wheels and encoder are spining in same direction
    if (std::abs(motorRPM) > std::abs(encoderRPMScaled) * (slipThreshold)) {                      // Checking for slip plus slip threshold allowance
        motorPower = ((motorRPM / accelerationFactor) / absoluteMaxRPM) * 100;         
    } else {
        motorPower = ((motorRPM * accelerationFactor) / absoluteMaxRPM) * 100;      
    }
}
//rain.Screen.clearLine(1);   
//Brain.Screen.printAt(10, 130, "PRE Power: %.2f", motorPower);  
// Clear line to avoid overlapping prints

  motorPower = std::copysign(std::max(minLaunchPower, std::min(std::abs(motorPower), std::abs(targetPower))), targetPower);

// Print each value on a separate line for clarity
   //Brain.Screen.printAt(10, 20, "Current Speed: %.2f", currentSpeed);
   //Brain.Screen.printAt(10, 40, "Encoder Speed: %.2f", encoderSpeed);
  // Brain.Screen.printAt(10, 60, "RPM Enc: %.2f", encoder.velocity(vex::velocityUnits::rpm));
  // Brain.Screen.printAt(10, 90, "RPM Mot: %.2f", abs(motorRPM));//Brain.Screen.printAt(10, 60, "RPM: %.2f", rpm);
  // Brain.Screen.printAt(10, 110, "Pct: %.2f", motor.velocity(vex::velocityUnits::pct));
  // Brain.Screen.printAt(10, 150, "Final Power: %.2f", motorPower);  
                         

   return motorPower;
}



// instanceMotorSpeed method implementation
//double LaunchControl::instanceMotorSpeed() {
//    return motorRPM * wheelCircumferenceCM / 60.0;
//}

// instanceEncoderSpeed method implementation
//double LaunchControl::instanceEncoderSpeed() {
//    return encoderRPM * encoderWheelCircumferenceCM / 60.0;
//}

/*
// instanceIsSlipping method implementation
bool LaunchControl::instanceIsSlipping() {

if (std::abs(motorRPM) > std::abs(encoderRPMAdjusted) * (1 + slipThreshold)) {
        return true; // 
   } else { 
        return false; // 
}
}
*/

//ABS V2
antiLockBrake::antiLockBrake(vex::motor& motor, vex::rotation& encoder, double minSpeedVoltage, vex::brakeType brakeMode)
    : motor(motor), encoder(encoder), minSpeedVoltage(minSpeedVoltage), brakeMode(brakeMode) {} // Initialized motor, encoder, and brakeMode

    // Getter method for brakeMode
    vex::brakeType antiLockBrake::getBrakeMode() const {
    return brakeMode;
}

// adjustSpeed method implementation
double antiLockBrake::reduceSpeed() {
    double ABSMotorVoltage = 0;    
    double motorRPM =  motor.velocity(vex::velocityUnits::rpm);
    double encoderRPMScaled = encoder.velocity(vex::velocityUnits::rpm) * (wheelCircumferenceCM / encoderWheelCircumferenceCM); // Adjust for wheel difference if needed
;
   // Brain.Screen.printAt(10, 100, "Current Speed: %.2f, Encoder Speed: %.2f", currentSpeed, encoderSpeed);

//if (encoderRPMScaled > targetRPM) { // If encoder wheel or actual robot speed is higher than target, need ABS braking to slow down
    if (ABSMotorVoltage = minSpeedVoltage) {; //minSpeedVoltage is a global varaible
       brakeMode = coast; 
       Brain.Screen.printAt(60, 20, "coast");
       
     } else if (std::abs(motorRPM) >= std::abs(encoderRPMScaled) * lockThreshold) {
       ABSMotorVoltage = 0;
       brakeMode = brake;
       Brain.Screen.printAt(60, 40, "brake");
    }
//} else {
    // Optional: only set motor to coast and apply target voltage if no braking is needed
   // motor.setBrake(brakeType::coast);
   // motorVoltage = targetVoltage;
//}
//Brain.Screen.clearLine(1);   
//Brain.Screen.printAt(10, 130, "PRE Power: %.2f", motorVoltage);  
// Clear line to avoid overlapping prints


// Print each value on a separate line for clarity
   //Brain.Screen.printAt(10, 20, "Current Speed: %.2f", currentSpeed);
   //Brain.Screen.printAt(10, 20, "Decel Phase");
   //Brain.Screen.printAt(10, 80, "Motor RPM: %.2f", motor.velocity(vex::velocityUnits::rpm));
   //Brain.Screen.printAt(10, 100, "Lt Motor RPM: %.2f", leftMotor[2].velocity(vex::velocityUnits::rpm));
    //Brain.Screen.printAt(10, 120, "Encoder RPM: %.2f", encoderRPMScaled);
   //Brain.Screen.printAt(10, 60, "RPM Enc: %.2f", encoder.velocity(vex::velocityUnits::rpm));
   //Brain.Screen.printAt(10, 90, "RPM Mot: %.2f", abs(motorRPM));//Brain.Screen.printAt(10, 60, "RPM: %.2f", rpm);
//motor.stop(brakeType::brake);
   return ABSMotorVoltage;
}


void pidStraightDistanceABS(double targetHeading, double targetDistance, double maxSpeed, double kp_heading, double ki_heading, double kd_heading, double kp_distance, double ki_distance, double kd_distance, double minSpeed, double breakDistance) { 
    
    // Reset the encoders to start counting from zero
    passiveEncoderLeft.resetPosition();
    passiveEncoderRight.resetPosition();  

    //double deadzone = 1;
    double normTargetHeading = normalizeHeading(targetHeading);

    // Initialize PID controllers
    PID headingPID(kp_heading, ki_heading, kd_heading);
    PID distancePID(kp_distance, ki_distance, kd_distance);
    
    headingPID.pidReset();
    distancePID.pidReset();
  
    //Convert % Speed input to voltage with max voltage of 12
    // Calculate maximum speed voltage and match its sign with targetDistance.
    double maxSpeedVoltage = std::copysign(maxSpeed * 0.01 * 12, targetDistance);

    // Calculate minimum speed voltage and match its sign with targetDistance.
    double minSpeedVoltage = std::copysign(minSpeed * 0.01 * 12, targetDistance);

    double minDriveMotorRPM = (minSpeedVoltage/absoluteMaxVoltage) * absoluteMaxRPM;    
    //int stabilityCounter = 0;                      // Counts consecutive iterations within target zone
    //const int stabilityThreshold = 1;              // Number of consecutive stable iterations required to exit loop
    //bool hasStabilizedOnce = false; // Flag to track if stabilityCounter has been incremented at least once
    bool decelCompleted = false;

    double currentDistance = 0;
    //double targetDriveVoltageLeft = 0; 
    //double targetDriveVoltageRight = 0;
    double motorVoltageLeft[3] = {0, 0, 0};  // Initialize all elements to 0
    double motorVoltageRight[3] = {0, 0, 0};  // Initialize all elements to 0

// Declare arrays of Antilock Brake instances for each wheel
antiLockBrake ABSLeft[3] = {antiLockBrake(leftMotor[0], passiveEncoderRight, minSpeedVoltage, coast),
                            antiLockBrake(leftMotor[1], passiveEncoderRight, minSpeedVoltage, coast),
                            antiLockBrake(leftMotor[2], passiveEncoderRight, minSpeedVoltage, coast)};
                            
antiLockBrake ABSRight[3] = {antiLockBrake(rightMotor[0], passiveEncoderRight, minSpeedVoltage, coast),
                             antiLockBrake(rightMotor[1], passiveEncoderRight, minSpeedVoltage, coast),
                             antiLockBrake(rightMotor[2], passiveEncoderRight, minSpeedVoltage, coast)};

    

    //launchControl (60, 60, 20);
        
    // Loop to continuously adjust motor power based on PID control 
while (std::abs(currentDistance) <= std::abs(targetDistance) - 2) {

    currentDistance = ((passiveEncoderLeft.position(degrees) + passiveEncoderRight.position(degrees)) / 2.0 / 360.0) * encoderWheelCircumferenceCM;    

    // Calculate the heading correction using the PID controller 
    double headingCorrection = headingPID.calculate(normTargetHeading, normalizeHeading(InertialSensor.heading()));
    double avgEncoderSpeedRPM = (passiveEncoderLeft.velocity(vex::velocityUnits::rpm) + passiveEncoderRight.velocity(vex::velocityUnits::rpm))/2;

//Cruise Phase         
if (std::abs(currentDistance) < (std::abs(targetDistance) - breakDistance) && decelCompleted == false) {
    // Else condition - specify actions here if the if condition is not met
    for (int i = 0; i < 3; i++) {
        // Example action: Set motor voltage to target voltage directly
        motorVoltageLeft[i] = maxSpeedVoltage + headingCorrection;
        motorVoltageRight[i] = maxSpeedVoltage - headingCorrection;
    }
    
    Brain.Screen.printAt(10, 20, "Cruise Phase");

// Decel Phase

//If declerating then go to ABS routine
} else if (std::abs(currentDistance) >= (std::abs(targetDistance) - breakDistance) && decelCompleted == false) {
    Brain.Screen.printAt(10, 20, "Decel Phase");


    for (int i = 0; i < 3; i++) {   
        //Left Side
        motorVoltageLeft[i] = ABSLeft[i].reduceSpeed();
        if (motorVoltageLeft[i] == 0) {
        motorVoltageLeft[i] = 0; // Set motor voltage to 0 if no speed.
        } else {
         motorVoltageLeft[i] = motorVoltageLeft[i]; // Add heading correction to reduced speed.
        }
        leftMotor[i].setBrake(ABSLeft[i].getBrakeMode());  

        //Right Side
        motorVoltageRight[i] = ABSRight[i].reduceSpeed();
        if (motorVoltageRight[i] == 0) {
        motorVoltageRight[i] = 0; // Set motor voltage to 0 if no speed.
        } else {
         motorVoltageRight[i] = motorVoltageRight[i]; // Add heading correction to reduced speed.
        }
        rightMotor[i].setBrake(ABSRight[i].getBrakeMode());  
        }     

 /*          
     // Stop the motors
    for (int i = 0; i < 3; i++) {  
    leftMotor[i].stop(brake);
    rightMotor[i].stop(brake);
    }
*/


  //  decelCompleted = true;
    // If all drivetrain motors decel to min speed then change DecelCompleted State variable to true to start Approach Phase


if (avgEncoderSpeedRPM < minDriveMotorRPM) {
    decelCompleted = true;

    /*
         // Stop the motors
    for (int i = 0; i < 3; i++) {  
    leftMotor[i].stop(brake);
    rightMotor[i].stop(brake);
    }
    vex::task::sleep(3000);
  */
}



//Final Approach Phase 
} else if (decelCompleted == true) {
        Brain.Screen.printAt(10, 20, "Approach Phase");    
        Brain.Screen.printAt(10, 40, "Decel Compl: %d", decelCompleted); 

         // Calculate the PID output for distance control
        //double distanceCorrection = distancePID.calculate(targetDistance, currentDistance);

        // **Check if Distance Error is Within Target Zone and Update Stability Counter**
       // double distanceError = targetDistance - currentDistance;
                   
            // Set the speed to minSpeed after the first stabilization
        //    distanceCorrection = (distanceCorrection > 0) ? minSpeedVoltage : -minSpeedVoltage;
        double adjustedHeadingCorrection = headingCorrection * avgEncoderSpeedRPM / absoluteMaxRPM * 2.0; //dynamically reduce heading correction at slower speed based on percentage of max speed

    // Else condition - specify actions here if the if condition is not met
    for (int i = 0; i < 3; i++) {
        // Example action: Set motor voltage to target voltage directly
        motorVoltageLeft[i] = minSpeedVoltage + (adjustedHeadingCorrection * 0.2);
        motorVoltageRight[i] = minSpeedVoltage - (adjustedHeadingCorrection * 0.2);
    }
    Brain.Screen.printAt(10, 20, "Approach Phase");
} 


// Power Drive Motors        
        for (int i = 0; i < 3; i++) {   
 
        leftMotor[i].spin(forward, motorVoltageLeft[i], voltageUnits::volt);
        rightMotor[i].spin(forward, motorVoltageRight[i], voltageUnits::volt); 
        }
       
        //Brain.Screen.printAt(10, 20, "Rotation Sensor: %.2f degrees", rotationDegrees);
        //Brain.Screen.printAt(10, 40, "Current Distance: %.4f", currentDistance);
        //Brain.Screen.printAt(10, 80, "Break Distance: %.2f", (targetDistance - breakDistance));
        //Brain.Screen.printAt(10, 100, "Norm Heading: %.2f", normalizeHeading(InertialSensor.heading()));
        Brain.Screen.printAt(10, 120, "Lt Motor Speed: %.2f", leftMotor[2].velocity(vex::velocityUnits::pct));
        Brain.Screen.printAt(10, 140, "Rt Motor Speed: %.2f", rightMotor[2].velocity(vex::velocityUnits::rpm));
       // Brain.Screen.printAt(10, 140, "Rt Motor Speed: %.2f", rightMotorSpeed);
       // Brain.Screen.printAt(10, 170, "Max Speed Voltage: %.2f", maxSpeedVoltage);
       // Brain.Screen.printAt(10, 190, "Min Speed Voltage: %.2f", minSpeedVoltage);

        vex::task::sleep(20);
    }
    
    // Stop the motors
    for (int i = 0; i < 3; i++) {  
    leftMotor[i].stop(brake);
    rightMotor[i].stop(brake);
    }

/*
    //task::sleep(800);  // Small delay to prevent overwhelming the CPU
    currentDistance = (passiveEncoderLeft.position(degrees) + passiveEncoderRight.position(degrees) ) / 2 * (encoderWheelCircumferenceCM / 360.0);
    // Debug print: Stopping motors
    Brain.Screen.clearLine(8);
    Brain.Screen.setCursor(8, 1);
    Brain.Screen.print("Distance Complete");
    Brain.Screen.print("Current Distance: %.2f", currentDistance);
*/
}


void pidStraightDistanceSlipABS(double targetHeading, double targetDistance, double maxSpeed, double kp_heading, double ki_heading, double kd_heading, double kp_distance, double ki_distance, double kd_distance, double minSpeed, double breakDistance) { 
    
    // Reset the encoders to start counting from zero
    passiveEncoderLeft.resetPosition();
    passiveEncoderRight.resetPosition();  

    //double deadzone = 1;
    double normTargetHeading = normalizeHeading(targetHeading);

    // Initialize PID controllers
    PID headingPID(kp_heading, ki_heading, kd_heading);
    PID distancePID(kp_distance, ki_distance, kd_distance);
    
    headingPID.pidReset();
    distancePID.pidReset();
  
    //Convert % Speed input to voltage with max voltage of 12
    // Calculate maximum speed voltage and match its sign with targetDistance.
    double maxSpeedVoltage = std::copysign(maxSpeed * 0.01 * 12, targetDistance);
    double minSpeedVoltage = std::copysign(minSpeed * 0.01 * 12, targetDistance);// Calculate minimum speed voltage and match its sign with targetDistance.
    double minLaunchSpeedVoltage = 2; 
    double minDriveMotorRPM = (minSpeedVoltage/absoluteMaxVoltage) * absoluteMaxRPM;
    double maxDriveMotorRPM = (maxSpeedVoltage/absoluteMaxVoltage) * absoluteMaxRPM; 
    double currentDistance = 0;
    double targetDriveVoltageLeft = 0; 
    double targetDriveVoltageRight = 0;
    double motorVoltageLeft[3] = {minLaunchSpeedVoltage, minLaunchSpeedVoltage, minLaunchSpeedVoltage};  // Initialize all elements to 0
    double motorVoltageRight[3] = {minLaunchSpeedVoltage, minLaunchSpeedVoltage, minLaunchSpeedVoltage};  // Initialize all elements to 0
    //bool decelCompleted = false;
    double avgMotorVoltage = 0; 

    MotionPhase currentPhase = READY; 

    // Declare arrays of Slip Control instances for each wheel
    slipControl antiSlipLeft[3] = {slipControl(leftMotor[0], passiveEncoderLeft, minSpeedVoltage, maxSpeedVoltage),
                                slipControl(leftMotor[1], passiveEncoderLeft, minSpeedVoltage, maxSpeedVoltage),
                                slipControl(leftMotor[2], passiveEncoderLeft, minSpeedVoltage, maxSpeedVoltage)};
                                
    slipControl antiSlipRight[3] = {slipControl(rightMotor[0], passiveEncoderRight, minSpeedVoltage, maxSpeedVoltage),
                                slipControl(rightMotor[1], passiveEncoderRight, minSpeedVoltage, maxSpeedVoltage),
                                slipControl(rightMotor[2], passiveEncoderRight, minSpeedVoltage, maxSpeedVoltage)};


    // Declare arrays of Antilock Brake instances for each wheel
    antiLockBrake ABSLeft[3] = {antiLockBrake(leftMotor[0], passiveEncoderLeft, minSpeedVoltage, coast),
                                antiLockBrake(leftMotor[1], passiveEncoderLeft, minSpeedVoltage, coast),
                                antiLockBrake(leftMotor[2], passiveEncoderLeft, minSpeedVoltage, coast)};
                                
    antiLockBrake ABSRight[3] = {antiLockBrake(rightMotor[0], passiveEncoderRight, minSpeedVoltage, coast),
                                antiLockBrake(rightMotor[1], passiveEncoderRight, minSpeedVoltage, coast),
                                antiLockBrake(rightMotor[2], passiveEncoderRight, minSpeedVoltage, coast)};

    
        
    // Loop to continuously adjust motor power based on PID control 
while ( std::abs(currentDistance) <=  std::abs(targetDistance) - 2) {

    currentDistance = ((passiveEncoderLeft.position(degrees) + passiveEncoderRight.position(degrees)) / 2.0 / 360.0) * encoderWheelCircumferenceCM;    

    // Calculate the heading correction using the PID controller 
    double headingCorrection = headingPID.calculate(normTargetHeading, normalizeHeading(InertialSensor.heading()));
    double encoderAvgRPMScaled = ((passiveEncoderLeft.velocity(vex::velocityUnits::rpm) + passiveEncoderRight.velocity(vex::velocityUnits::rpm))/2)  * (wheelCircumferenceCM / encoderWheelCircumferenceCM);
    double adjustedHeadingCorrection = headingCorrection * encoderAvgRPMScaled / absoluteMaxRPM; //dynamically reduce heading correction at slower speed based on percentage of max speed

    // Launch Phase
if ( std::abs(avgMotorVoltage) > 0 &&  std::abs(avgMotorVoltage) <  std::abs(maxSpeedVoltage) && (currentPhase == READY || currentPhase == LAUNCH)) {
/*
    for (int i = 0; i < 3; i++) {   
        // Use leftLaunchControl if minLaunchPower threshold is met for the left side
        motorVoltageLeft[i] = antiSlipLeft[i].increaseSpeed(motorVoltageLeft[i]) + adjustedHeadingCorrection;      // get slip voltage and Adjust for heading correction
        motorVoltageRight[i] = antiSlipRight[i].increaseSpeed(motorVoltageRight[i]) - adjustedHeadingCorrection;   

    //    Brain.Screen.printAt(10, 20, "Launch Phase");
    }
*/    

currentPhase = LAUNCH; 

for (int i = 0; i < 3; i++) {   
        // Use leftLaunchControl if minLaunchPower threshold is met for the left side
       motorVoltageLeft[i] = (motorVoltageLeft[i] * 1.2) + adjustedHeadingCorrection;       // get slip voltage and Adjust for heading correction
       motorVoltageRight[i] = (motorVoltageRight[i] * 1.2) - adjustedHeadingCorrection; 

        //motorVoltageLeft[i] = (motorVoltageLeft[i] * 1.2);       // get slip voltage and Adjust for heading correction
        //motorVoltageRight[i] = (motorVoltageRight[i] * 1.2); 

    //    Brain.Screen.printAt(10, 20, "Launch Phase");
    }
    
        Brain.Screen.printAt(10, 20, "Launch Phase");
//Cruise Phase   
  

} else if ( std::abs(avgMotorVoltage) >=  std::abs(maxSpeedVoltage) &&  std::abs(currentDistance) < ( std::abs(targetDistance) - breakDistance) && (currentPhase == LAUNCH || currentPhase == CRUISE)) {
    currentPhase = CRUISE;
   Brain.Screen.printAt(10, 20, "Cruise Phase"); 
   // Else condition - specify actions here if the if condition is not met
   //     break;  // Exit the loop //testing only - temporary, remove after testing
    for (int i = 0; i < 3; i++) {
        // Example action: Set motor voltage to target voltage directly
        motorVoltageLeft[i] = maxSpeedVoltage + headingCorrection;
        motorVoltageRight[i] = maxSpeedVoltage - headingCorrection;
    }
    
 

// Decel Phase

//If declerating then go to ABS routine


} else if ( std::abs(currentDistance) >= ( std::abs(targetDistance) - breakDistance) && (currentPhase == CRUISE || currentPhase == DECELERATE)) {
    currentPhase = DECELERATE; 
    Brain.Screen.printAt(10, 20, "Decel Phase");
  break;

    for (int i = 0; i < 3; i++) {   
        //Left Side
        motorVoltageLeft[i] = ABSLeft[i].reduceSpeed();
        if (motorVoltageLeft[i] == 0) {
        motorVoltageLeft[i] = 0; // Set motor voltage to 0 if no speed.
        } else {
         motorVoltageLeft[i] = motorVoltageLeft[i] + adjustedHeadingCorrection; // Add heading correction to reduced speed.
        }
        leftMotor[i].setBrake(ABSLeft[i].getBrakeMode());  

        //Right Side
        motorVoltageRight[i] = ABSRight[i].reduceSpeed();
        if (motorVoltageRight[i] == 0) {
        motorVoltageRight[i] = 0; // Set motor voltage to 0 if no speed.
        } else {
         motorVoltageRight[i] = motorVoltageRight[i] - adjustedHeadingCorrection; // Add heading correction to reduced speed.
        }
        rightMotor[i].setBrake(ABSRight[i].getBrakeMode());  
        }     

    if (avgMotorVoltage <= minLaunchSpeedVoltage) {
    currentPhase = APPROACH; 
    }

//Final Approach Phase    


} else if (currentPhase == APPROACH) {
    
        Brain.Screen.printAt(10, 20, "Approach Phase");    
        Brain.Screen.printAt(10, 40, "Decel Compl: %d", currentPhase); 

        double adjustedHeadingCorrection = headingCorrection * encoderAvgRPMScaled / absoluteMaxRPM * 2.0; //dynamically reduce heading correction at slower speed based on percentage of max speed

    // Else condition - specify actions here if the if condition is not met
    for (int i = 0; i < 3; i++) {
        // Example action: Set motor voltage to target voltage directly
        motorVoltageLeft[i] = minSpeedVoltage + adjustedHeadingCorrection;
        motorVoltageRight[i] = minSpeedVoltage - adjustedHeadingCorrection;
    }
    Brain.Screen.printAt(10, 20, "Approach Phase");
} 


// Power Drive Motors        
for (int i = 0; i < 3; i++) {   

leftMotor[i].spin(forward, motorVoltageLeft[i], voltageUnits::volt);
rightMotor[i].spin(forward, motorVoltageRight[i], voltageUnits::volt); 
}

avgMotorVoltage = (motorVoltageLeft[0] + motorVoltageLeft[1] + motorVoltageLeft[2] + motorVoltageRight[0] + motorVoltageRight[1] + motorVoltageRight[2])/6;


//Brain.Screen.printAt(10, 20, "Rotation Sensor: %.2f degrees", rotationDegrees);
//Brain.Screen.printAt(10, 40, "Current Distance: %.4f", currentDistance);
//Brain.Screen.printAt(10, 80, "Break Distance: %.2f", (targetDistance - breakDistance));
//Brain.Screen.printAt(10, 100, "Norm Heading: %.2f", normalizeHeading(InertialSensor.heading()));
Brain.Screen.printAt(10, 100, "Avg Encoder RPM: %.2f", encoderAvgRPMScaled);
Brain.Screen.printAt(10, 120, "Max RPM: %.2f", maxDriveMotorRPM);
Brain.Screen.printAt(10, 140, "Min RPM: %.2f", minDriveMotorRPM);
// Brain.Screen.printAt(10, 140, "Rt Motor Speed: %.2f", rightMotorSpeed);
// Brain.Screen.printAt(10, 170, "Max Speed Voltage: %.2f", maxSpeedVoltage);
// Brain.Screen.printAt(10, 190, "Min Speed Voltage: %.2f", minSpeedVoltage);

vex::task::sleep(20);
}
    
// Stop the motors
currentPhase = STOP; 

for (int i = 0; i < 3; i++) {  
leftMotor[i].stop(brake);
rightMotor[i].stop(brake);
}

}


// Slip Control Constructor implementation
slipControl::slipControl(vex::motor& motor, vex::rotation& encoder, double minSpeedVoltage, double maxSpeedVoltage)
    : motor(motor), encoder(encoder), minSpeedVoltage(minSpeedVoltage), maxSpeedVoltage(maxSpeedVoltage) {} // Initialized slipThreshold

    // adjustSpeed method implementation
    double slipControl::increaseSpeed(double slipMotorVoltage) { 
    double motorRPM =  motor.velocity(vex::velocityUnits::rpm);
    double encoderRPMScaled = encoder.velocity(vex::velocityUnits::rpm) * (wheelCircumferenceCM / encoderWheelCircumferenceCM); // Adjust for wheel difference if needed
   // Brain.Screen.printAt(10, 100, "Current Speed: %.2f, Encoder Speed: %.2f", currentSpeed, encoderSpeed);

if ((motorRPM > 0 && encoderRPMScaled >= 0) || (motorRPM < 0 && encoderRPMScaled <= 0)) {// only check for slip if wheels and encoder are spining in same direction
    Brain.Screen.printAt(10, 20, "Launch Phase");
    if (std::abs(motorRPM) > std::abs(encoderRPMScaled) * (slipThreshold)) {                      // Checking for slip plus slip threshold allowance
       slipMotorVoltage = slipMotorVoltage / accelerationFactor;    
       // slipMotorVoltage = 12;       
    } else {
        slipMotorVoltage = slipMotorVoltage * accelerationFactor;      
        //slipMotorVoltage = 12;    
    }
}

 slipMotorVoltage = std::copysign(std::max(minSpeedVoltage, std::min(std::abs(slipMotorVoltage), std::abs(maxSpeedVoltage))), maxSpeedVoltage);                      

   return slipMotorVoltage;
}




void pidStraightDistanceLaunchABS(double targetHeading, double targetDistance, double maxSpeed, double kp_heading, double ki_heading, double kd_heading, double accelHeadingScaling, double decelHeadingScaling, double minSpeed, double breakDistance) { 
    
    // Reset the encoders to start counting from zero
    passiveEncoderLeft.resetPosition();
    passiveEncoderRight.resetPosition();  

    double deadzone = 1;
    double normTargetHeading = normalizeHeading(targetHeading);

    // Initialize PID controllers
    PID headingPID(kp_heading, ki_heading, kd_heading);
    
    headingPID.pidReset();
  
    //Convert % Speed input to voltage with max voltage of 12
    // Calculate maximum speed voltage and match its sign with targetDistance.
    double maxSpeedVoltage = std::copysign(maxSpeed * 0.01 * 12, targetDistance);

    // Calculate minimum speed voltage and match its sign with targetDistance.
    double minSpeedVoltage = std::copysign(minSpeed * 0.01 * 12, targetDistance);
    double avgMotorVoltage = 0;
    double minLaunchSpeedVoltage = 5;
    double percentRPMLoss = 0.15;
    double minDriveMotorRPM = (minSpeed * .01) * absoluteMaxRPM; //convert minspeed to percentage first
    double maxDriveMotorRPM = (maxSpeed * .01) * absoluteMaxRPM; //convert maxspeed to percentage first

    //int stabilityCounter = 0;                      // Counts consecutive iterations within target zone
    //const int stabilityThreshold = 1;              // Number of consecutive stable iterations required to exit loop
    //bool hasStabilizedOnce = false; // Flag to track if stabilityCounter has been incremented at least once
    bool decelCompleted = false;
    bool accelCompleted = false;
    double currentDistance = 0;
    double targetDriveVoltageLeft = 0; 
    double targetDriveVoltageRight = 0;
    double motorVoltageLeft[3] = {minLaunchSpeedVoltage, minLaunchSpeedVoltage, minLaunchSpeedVoltage};  // Initialize all elements to minimum launch speed
    double motorVoltageRight[3] = {minLaunchSpeedVoltage, minLaunchSpeedVoltage, minLaunchSpeedVoltage};  // Initialize all elements to minimum launch speed

   // MotionPhase currentPhase = READY; 

    // Declare arrays of Slip Control instances for each wheel
    slipControl antiSlipLeft[3] = {slipControl(leftMotor[0], passiveEncoderLeft, minSpeedVoltage, maxSpeedVoltage),
                                slipControl(leftMotor[1], passiveEncoderLeft, minSpeedVoltage, maxSpeedVoltage),
                                slipControl(leftMotor[2], passiveEncoderLeft, minSpeedVoltage, maxSpeedVoltage)};
                                
    slipControl antiSlipRight[3] = {slipControl(rightMotor[0], passiveEncoderRight, minSpeedVoltage, maxSpeedVoltage),
                                slipControl(rightMotor[1], passiveEncoderRight, minSpeedVoltage, maxSpeedVoltage),
                                slipControl(rightMotor[2], passiveEncoderRight, minSpeedVoltage, maxSpeedVoltage)};


// Declare arrays of Antilock Brake instances for each wheel
/*
antiLockBrake ABSLeft[3] = {antiLockBrake(leftMotor[0], passiveEncoderRight, minSpeedVoltage, coast),
                            antiLockBrake(leftMotor[1], passiveEncoderRight, minSpeedVoltage, coast),
                            antiLockBrake(leftMotor[2], passiveEncoderRight, minSpeedVoltage, coast)};
                            
antiLockBrake ABSRight[3] = {antiLockBrake(rightMotor[0], passiveEncoderRight, minSpeedVoltage, coast),
                             antiLockBrake(rightMotor[1], passiveEncoderRight, minSpeedVoltage, coast),
                             antiLockBrake(rightMotor[2], passiveEncoderRight, minSpeedVoltage, coast)};
*/
ABS ABSLeft[3] = {ABS(leftMotor[0], passiveEncoderLeft, minSpeedVoltage, vex::brakeType::coast),
                  ABS(leftMotor[1], passiveEncoderLeft, minSpeedVoltage, vex::brakeType::coast),
                  ABS(leftMotor[2], passiveEncoderLeft, minSpeedVoltage, vex::brakeType::coast)};
                            
ABS ABSRight[3] = {ABS(rightMotor[0], passiveEncoderRight, minSpeedVoltage, vex::brakeType::coast),
                   ABS(rightMotor[1], passiveEncoderRight, minSpeedVoltage, vex::brakeType::coast),
                   ABS(rightMotor[2], passiveEncoderRight, minSpeedVoltage, vex::brakeType::coast)};

    //launchControl (60, 60, 20);
        
    // Loop to continuously adjust motor power based on PID control 
while (std::abs(currentDistance) <= std::abs(targetDistance) - 2) {

    currentDistance = ((passiveEncoderLeft.position(degrees) + passiveEncoderRight.position(degrees)) / 2.0 / 360.0) * encoderWheelCircumferenceCM;    
    avgMotorVoltage = (motorVoltageLeft[0] + motorVoltageLeft[1] + motorVoltageLeft[2] + motorVoltageRight[0] + motorVoltageRight[1] + motorVoltageRight[2])/6;

    // Calculate the heading correction using the PID controller 
    double headingCorrection = headingPID.calculate(normTargetHeading, normalizeHeading(InertialSensor.heading()));
    double avgEncoderSpeedRPM = (passiveEncoderLeft.velocity(vex::velocityUnits::rpm) + passiveEncoderRight.velocity(vex::velocityUnits::rpm))/2;
    double encoderAvgRPMScaled = ((passiveEncoderLeft.velocity(vex::velocityUnits::rpm) + passiveEncoderRight.velocity(vex::velocityUnits::rpm))/2)  * (wheelCircumferenceCM / encoderWheelCircumferenceCM);
    double adjustedHeadingCorrection = headingCorrection * encoderAvgRPMScaled / absoluteMaxRPM; //dynamically reduce heading correction at slower speed based on percentage of max speed

//Launch Phase
if (std::abs(currentDistance) < (std::abs(targetDistance) - breakDistance) && accelCompleted == false) {

    for (int i = 0; i < 3; i++) {   
        // Use leftLaunchControl if minLaunchPower threshold is met for the left side
        motorVoltageLeft[i] = antiSlipLeft[i].increaseSpeed(motorVoltageLeft[i]) + (adjustedHeadingCorrection * accelHeadingScaling);      // get slip voltage and Adjust for heading correction
        motorVoltageRight[i] = antiSlipRight[i].increaseSpeed(motorVoltageRight[i]) - (adjustedHeadingCorrection * accelHeadingScaling);   

    //    Brain.Screen.printAt(10, 20, "Launch Phase");
    }

     if (std::abs(encoderAvgRPMScaled) >= maxDriveMotorRPM * (1 - percentRPMLoss)){
        accelCompleted = true;
    }

//currentPhase = LAUNCH; 
/*
for (int i = 0; i < 3; i++) {   
        // Use leftLaunchControl if minLaunchPower threshold is met for the left side
       motorVoltageLeft[i] = (motorVoltageLeft[i] * 1.2) + adjustedHeadingCorrection;       // get slip voltage and Adjust for heading correction
       motorVoltageRight[i] = (motorVoltageRight[i] * 1.2) - adjustedHeadingCorrection; 

        //motorVoltageLeft[i] = (motorVoltageLeft[i] * 1.2);       // get slip voltage and Adjust for heading correction
        //motorVoltageRight[i] = (motorVoltageRight[i] * 1.2); 

    //    Brain.Screen.printAt(10, 20, "Launch Phase");
    }
 */   
        Brain.Screen.printAt(10, 20, "Launch Phase");

//Cruise Phase      


} else if (std::abs(currentDistance) < (std::abs(targetDistance) - breakDistance) && accelCompleted == true) {
break;  
    Brain.Screen.printAt(10, 20, "Cruise Phase");    // Else condition - specify actions here if the if condition is not met
    for (int i = 0; i < 3; i++) {
        // Example action: Set motor voltage to target voltage directly
        motorVoltageLeft[i] = maxSpeedVoltage + headingCorrection;
        motorVoltageRight[i] = maxSpeedVoltage - headingCorrection;
    }
    

// Decel Phase

//If declerating then go to ABS routine
} else if (std::abs(currentDistance) >= (std::abs(targetDistance) - breakDistance) && decelCompleted == false) {
 break;   
  Brain.Screen.printAt(10, 20, "Decel Phase");
   

    for (int i = 0; i < 3; i++) {   
        
        //Left Side
        ABSResult leftResult = ABSLeft[i].reduceCurrentSpeed(motorVoltageLeft[i]); // Get ABSResult containing motor voltage and brake mode
        motorVoltageLeft[i] = leftResult.motorVoltage; // Extract motor voltage from ABSResult
        if (motorVoltageLeft[i] != 0) {
            motorVoltageLeft[i] += (adjustedHeadingCorrection * decelHeadingScaling); // Add heading correction only if voltage is not zero
        }
        leftMotor[i].setBrake(leftResult.brakeMode); // Set brake mode from ABSResult

        //Right Side
        ABSResult rightResult = ABSRight[i].reduceCurrentSpeed(motorVoltageRight[i]); // Get ABSResult containing motor voltage and brake mode
        motorVoltageRight[i] = rightResult.motorVoltage; // Extract motor voltage from ABSResult
        if (motorVoltageRight[i] != 0) {
            motorVoltageRight[i] -= (adjustedHeadingCorrection * decelHeadingScaling); // Subtract heading correction only if voltage is not zero
        }
        rightMotor[i].setBrake(rightResult.brakeMode); // Set brake mode from ABSResult

        }
    

 /*          
     // Stop the motors
    for (int i = 0; i < 3; i++) {  
    leftMotor[i].stop(brake);
    rightMotor[i].stop(brake);
    }
*/


  //  decelCompleted = true;
    // If all drivetrain motors decel to min speed then change DecelCompleted State variable to true to start Approach Phase

    
if (avgEncoderSpeedRPM < minDriveMotorRPM) {
    decelCompleted = true;

    /*
         // Stop the motors
    for (int i = 0; i < 3; i++) {  
    leftMotor[i].stop(brake);
    rightMotor[i].stop(brake);
    }
    vex::task::sleep(3000);
  */

}



//Final Approach Phase 
} else if (decelCompleted == true) {
    break;
        Brain.Screen.printAt(10, 20, "Approach Phase");    
        Brain.Screen.printAt(10, 40, "Decel Compl: %d", decelCompleted); 

         // Calculate the PID output for distance control
        //double distanceCorrection = distancePID.calculate(targetDistance, currentDistance);

        // **Check if Distance Error is Within Target Zone and Update Stability Counter**
       // double distanceError = targetDistance - currentDistance;
                   
            // Set the speed to minSpeed after the first stabilization
        //    distanceCorrection = (distanceCorrection > 0) ? minSpeedVoltage : -minSpeedVoltage;
        double adjustedHeadingCorrection = headingCorrection * (avgEncoderSpeedRPM / absoluteMaxRPM) * 2.0; //dynamically reduce heading correction at slower speed based on percentage of max speed

    // Else condition - specify actions here if the if condition is not met
    for (int i = 0; i < 3; i++) {
        // Example action: Set motor voltage to target voltage directly
        motorVoltageLeft[i] = minSpeedVoltage + (adjustedHeadingCorrection * 0.2);
        motorVoltageRight[i] = minSpeedVoltage - (adjustedHeadingCorrection * 0.2);
    }
    Brain.Screen.printAt(10, 20, "Approach Phase");
} 


// Power Drive Motors        
        for (int i = 0; i < 3; i++) {   
 
        leftMotor[i].spin(forward, motorVoltageLeft[i], voltageUnits::volt);
        rightMotor[i].spin(forward, motorVoltageRight[i], voltageUnits::volt); 
        }
       
        //Brain.Screen.printAt(10, 20, "Rotation Sensor: %.2f degrees", rotationDegrees);
        //Brain.Screen.printAt(10, 40, "Current Distance: %.4f", currentDistance);
        //Brain.Screen.printAt(10, 80, "Break Distance: %.2f", (targetDistance - breakDistance));
        //Brain.Screen.printAt(10, 100, "Norm Heading: %.2f", normalizeHeading(InertialSensor.heading()));
        Brain.Screen.printAt(10, 120, "Lt Motor Speed: %.2f", leftMotor[2].velocity(vex::velocityUnits::pct));
        Brain.Screen.printAt(10, 140, "Rt Motor Speed: %.2f", rightMotor[2].velocity(vex::velocityUnits::rpm));
       // Brain.Screen.printAt(10, 140, "Rt Motor Speed: %.2f", rightMotorSpeed);
       // Brain.Screen.printAt(10, 170, "Max Speed Voltage: %.2f", maxSpeedVoltage);
       // Brain.Screen.printAt(10, 190, "Min Speed Voltage: %.2f", minSpeedVoltage);
        vex::task::sleep(20);
    }
    
 // Set brake mode to brake for all motors
    for (int i = 0; i < 3; i++) {
        leftMotor[i].setBrake(brakeType::brake);
        rightMotor[i].setBrake(brakeType::brake);
    }
    
    // Stop all motors simultaneously
    for (int i = 0; i < 3; i++) {
        leftMotor[i].stop();
        rightMotor[i].stop();
    }

/*
    //task::sleep(800);  // Small delay to prevent overwhelming the CPU
    currentDistance = (passiveEncoderLeft.position(degrees) + passiveEncoderRight.position(degrees) ) / 2 * (encoderWheelCircumferenceCM / 360.0);
    // Debug print: Stopping motors
    Brain.Screen.clearLine(8);
    Brain.Screen.setCursor(8, 1);
    Brain.Screen.print("Distance Complete");
    Brain.Screen.print("Current Distance: %.2f", currentDistance);
*/
}

//ABS V3
/*
ABS::ABS(vex::motor& motor, vex::rotation& encoder, double minSpeedVoltage, vex::brakeType brakeMode)
    : motor(motor), encoder(encoder), minSpeedVoltage(minSpeedVoltage), brakeMode(brakeMode) {} // Initialized motor, encoder, and brakeMode

// adjustSpeed method implementation
double ABS::reduceCurrentSpeed(double ABSMotorVoltage) {
    double motorRPM =  motor.velocity(vex::velocityUnits::rpm);
    double encoderRPMScaled = encoder.velocity(vex::velocityUnits::rpm) * (wheelCircumferenceCM / encoderWheelCircumferenceCM); // Adjust for wheel difference if needed
;
   // Brain.Screen.printAt(10, 100, "Current Speed: %.2f, Encoder Speed: %.2f", currentSpeed, encoderSpeed);

//if (encoderRPMScaled > targetRPM) { // If encoder wheel or actual robot speed is higher than target, need ABS braking to slow down
if (ABSMotorVoltage <= minSpeedVoltage) {
    // Wheel has reduced speed to the target or slower
    ABSMotorVoltage = 0;
    brakeMode = coast; // Prevent locking
    Brain.Screen.printAt(60, 20, "coast");
} else {
    // Check wheel locking condition
    if (std::abs(motorRPM) >= std::abs(encoderRPMScaled) * lockThreshold) {
        // Wheels are locking
        ABSMotorVoltage = 0;
        brakeMode = coast; // Prevent further locking
        Brain.Screen.printAt(60, 40, "coast (lock)");
    } else {
        // Wheels are not locking but need braking
        ABSMotorVoltage = 0;
        brakeMode = brake; // Apply brakes to slow down
        Brain.Screen.printAt(60, 60, "brake");
    }
}
//} else {
    // Optional: only set motor to coast and apply target voltage if no braking is needed
   // motor.setBrake(brakeType::coast);
   // motorVoltage = targetVoltage;
//}
//Brain.Screen.clearLine(1);   
//Brain.Screen.printAt(10, 130, "PRE Power: %.2f", motorVoltage);  
// Clear line to avoid overlapping prints


// Print each value on a separate line for clarity
   //Brain.Screen.printAt(10, 20, "Current Speed: %.2f", currentSpeed);
   //Brain.Screen.printAt(10, 20, "Decel Phase");
   //Brain.Screen.printAt(10, 80, "Motor RPM: %.2f", motor.velocity(vex::velocityUnits::rpm));
   //Brain.Screen.printAt(10, 100, "Lt Motor RPM: %.2f", leftMotor[2].velocity(vex::velocityUnits::rpm));
    //Brain.Screen.printAt(10, 120, "Encoder RPM: %.2f", encoderRPMScaled);
   //Brain.Screen.printAt(10, 60, "RPM Enc: %.2f", encoder.velocity(vex::velocityUnits::rpm));
   //Brain.Screen.printAt(10, 90, "RPM Mot: %.2f", abs(motorRPM));//Brain.Screen.printAt(10, 60, "RPM: %.2f", rpm);
//motor.stop(brakeType::brake);
   return ABSMotorVoltage;
}
*/

ABS::ABS(vex::motor& motor, vex::rotation& encoder, double minSpeedVoltage, vex::brakeType brakeMode)
    : motor(motor), encoder(encoder), minSpeedVoltage(minSpeedVoltage), brakeMode(brakeMode) {}

// adjustSpeed method implementation
ABSResult ABS::reduceCurrentSpeed(double ABSMotorVoltage) {
    ABSResult result;  // Create ABSResult instance to store motor voltage and brake mode

    double motorRPM = motor.velocity(vex::velocityUnits::rpm);
    double encoderRPMScaled = encoder.velocity(vex::velocityUnits::rpm) * (wheelCircumferenceCM / encoderWheelCircumferenceCM);

    if (ABSMotorVoltage <= minSpeedVoltage) {
        // Wheel has reduced speed to the target or slower
        result.motorVoltage = minSpeedVoltage;  // Set motor voltage to min Speed
        result.brakeMode = vex::brakeType::coast;  // Prevent locking
        Brain.Screen.printAt(60, 20, "coast");
    } else {
        // Check wheel locking condition
        if (std::abs(motorRPM) >= std::abs(encoderRPMScaled) * lockThreshold) {
            // Wheels are locking
            result.motorVoltage = 0;  // Set motor voltage to 0
            result.brakeMode = vex::brakeType::coast;  // Prevent further locking
            Brain.Screen.printAt(60, 40, "coast (lock)");
        } else {
            // Wheels are not locking but need braking
            result.motorVoltage = 0;  // Set motor voltage to 0
            result.brakeMode = vex::brakeType::brake;  // Apply brakes to slow down
            Brain.Screen.printAt(60, 60, "brake");
        }
    }

    return result;  // Return ABSResult containing motor voltage and brake mode
}


/*
// Slip Control Constructor implementation
spotTurnSlipControl::spotTurnSlipControl(vex::motor& motor, vex::inertial& inertialSensor, double minSpeedVoltage, double maxSpeedVoltage)
    : motor(motor), intertial(inertialSensor), minSpeedVoltage(minSpeedVoltage), maxSpeedVoltage(maxSpeedVoltage) {} // Initialized slipThreshold

    // adjustSpeed method implementation
    double spotTurnSlipControl::spotTurnincreaseSpeed(double SpotTurnSlipMotorVoltage) { 
   
   // Convert motor RPM to Radians per second
    double motorRPM =  motor.velocity(vex::velocityUnits::rpm);
    // Step 1: Convert RPM to Degrees per Second
    double motorDegreePerSecond = MotorRPM * 6.0;
    // Step 2: Convert Degrees per Second to Radians per Second
    double motorRadianPerSecond = degreePerSecond * (M_PI / 180.0);
   
    // Get actual rotation speed from inertial sensor
    double botRotationDegreePerSecond = InertialSensor.gyroRate(axisType::zaxis, velocityUnits::dps);
    double botRotationRadianPerSecond = botRotationDegreePerSecond * (M_PI / 180.0);

   // Brain.Screen.printAt(10, 100, "Current Speed: %.2f, Encoder Speed: %.2f", currentSpeed, encoderSpeed);

    Brain.Screen.printAt(10, 20, "Launch Phase");
    if (std::abs(motorRadianPerSecond) > std::abs(botRotationRadianPerSecond) * (slipThreshold)) {                      // Checking for slip plus slip threshold allowance
       slipMotorVoltage = slipMotorVoltage / accelerationFactor;    
       // slipMotorVoltage = 12;       
    } else {
        slipMotorVoltage = slipMotorVoltage * accelerationFactor;      
        //slipMotorVoltage = 12;    
    }

 slipMotorVoltage = std::copysign(std::max(minSpeedVoltage, std::min(std::abs(slipMotorVoltage), std::abs(maxSpeedVoltage))), maxSpeedVoltage);                      

   return slipMotorVoltage;
}        
*/

void spotTurnMP(double targetHeading, double maxSpeed, double minSpeed, double breakDistanceInDegrees, double accelHeadingScaling, double decelHeadingScaling, double approachHeadingScaling) { 
    double normTargetHeading = normalizeHeading(targetHeading);
    double startingDegrees = normalizeHeading(InertialSensor.heading(degrees));
    double normCurrentHeading = startingDegrees;
   
    // Calculate the initial error
    double targetDistanceInDegrees = normTargetHeading - normCurrentHeading;

    
    //Convert % Speed input to voltage with max voltage of 12
    // Calculate maximum speed voltage and match its sign with targetDistance.
    double maxSpeedVoltage = std::copysign(maxSpeed * 0.01 * 12, targetDistanceInDegrees);

    // Calculate minimum speed voltage and match its sign with targetDistance.
    double minSpeedVoltage = std::copysign(minSpeed * 0.01 * 12, targetDistanceInDegrees);
    //double avgMotorVoltage = 0;
    double minLaunchSpeedVoltage = 6;
    double percentSpeedLoss = .11;
    double totalMotorRadiansPerSecond = 0.0;
    double minRadiansPerSecond = (2 * fabs(absoluteMaxRPM * (minSpeed * .01) * (wheelCircumferenceCM / 60.0))) / trackWidth; //convert minspeed to percentage first
    double maxRadiansPerSecond = (2 * fabs(absoluteMaxRPM * (maxSpeed * .01) * (wheelCircumferenceCM / 60.0))) / trackWidth; //convert maxspeed to percentage first
    double turnDirection = 1;

    //int stabilityCounter = 0;                      // Counts consecutive iterations within target zone
    //const int stabilityThreshold = 1;              // Number of consecutive stable iterations required to exit loop
    //bool hasStabilizedOnce = false; // Flag to track if stabilityCounter has been incremented at least once
    bool decelCompleted = false;
    bool accelCompleted = false;
    double currentDistanceInDegrees = 0;
    double targetDriveVoltageLeft = 0; 
    double targetDriveVoltageRight = 0;
    double motorRadiansPerSecondLeft[3];
    double motorRadiansPerSecondRight[3];
    double leftMotorCMPerSecond[3];
    double rightMotorCMPerSecond[3];
    double leftMotorAngularRadians[3];
    double rightMotorAngularRadians[3];
    double robotRadiansPerSecond = 0;
    double motorVoltageLeft[3] = {minLaunchSpeedVoltage, minLaunchSpeedVoltage, minLaunchSpeedVoltage};  // Initialize all elements to minimum launch speed
    double motorVoltageRight[3] = {minLaunchSpeedVoltage, minLaunchSpeedVoltage, minLaunchSpeedVoltage};  // Initialize all elements to minimum launch speed
    double motorRadiansPerSecond[3] = {0, 0, 0};

   // MotionPhase currentPhase = READY; 

    // Declare arrays of Slip Control instances for each wheel
    tractionControl tractionControlLeft[3] = {tractionControl(minLaunchSpeedVoltage, maxSpeedVoltage),
                                              tractionControl(minLaunchSpeedVoltage, maxSpeedVoltage),
                                              tractionControl(minLaunchSpeedVoltage, maxSpeedVoltage)};
                                
    tractionControl tractionControlRight[3] = {tractionControl(minLaunchSpeedVoltage, maxSpeedVoltage),
                                               tractionControl(minLaunchSpeedVoltage, maxSpeedVoltage),
                                               tractionControl(minLaunchSpeedVoltage, maxSpeedVoltage)};

// Declare arrays of Antilock Brake instances for each wheel
/*
antiLockBrake ABSLeft[3] = {antiLockBrake(leftMotor[0], passiveEncoderRight, minSpeedVoltage, coast),
                            antiLockBrake(leftMotor[1], passiveEncoderRight, minSpeedVoltage, coast),
                            antiLockBrake(leftMotor[2], passiveEncoderRight, minSpeedVoltage, coast)};
                            
antiLockBrake ABSRight[3] = {antiLockBrake(rightMotor[0], passiveEncoderRight, minSpeedVoltage, coast),
                             antiLockBrake(rightMotor[1], passiveEncoderRight, minSpeedVoltage, coast),
                             antiLockBrake(rightMotor[2], passiveEncoderRight, minSpeedVoltage, coast)};
*/
ABSController ABSControllerLeft[3] = {ABSController(motorRadiansPerSecondLeft[0], robotRadiansPerSecond, minSpeedVoltage),
                  ABSController(motorRadiansPerSecondLeft[1], robotRadiansPerSecond, minSpeedVoltage),
                  ABSController(motorRadiansPerSecondLeft[2], robotRadiansPerSecond, minSpeedVoltage)};
                            
ABSController ABSControllerRight[3] = {ABSController(motorRadiansPerSecondRight[0], robotRadiansPerSecond, minSpeedVoltage),
                   ABSController(motorRadiansPerSecondRight[1], robotRadiansPerSecond, minSpeedVoltage),
                   ABSController(motorRadiansPerSecondRight[2], robotRadiansPerSecond, minSpeedVoltage)};

    //launchControl (60, 60, 20);
        
    // Loop to continuously adjust motor power based on PID control 
while (std::abs(currentDistanceInDegrees) <= std::abs(targetDistanceInDegrees) - 2) {

   
    //Calculate angular drive motor speed in radians
    for (int i = 0; i < 3; i++) {
        //Get motor RPM and convert to radians per second
        leftMotorCMPerSecond[i] = leftMotor[i].velocity(velocityUnits::rpm) * (wheelCircumferenceCM / 60.0);
        rightMotorCMPerSecond[i] = rightMotor[i].velocity(velocityUnits::rpm) * (wheelCircumferenceCM / 60.0);

        leftMotorAngularRadians[i] = (2 * fabs(leftMotorCMPerSecond[i])) / trackWidth;
        rightMotorAngularRadians[i] = (2 * fabs(rightMotorCMPerSecond[i])) / trackWidth;
        
        //Add up all radians per second to prepare for calculating average motor radians per second
        totalMotorRadiansPerSecond += motorRadiansPerSecondLeft[i] + motorRadiansPerSecondRight[i];
    }
   
    double avgMotorRadianPerSecond =  totalMotorRadiansPerSecond / numberDriveMotor; // Calculate average motor radians per second

    // Get actual robot angular speed in radians using inertial sensor
    robotRadiansPerSecond = InertialSensor.gyroRate(axisType::zaxis, velocityUnits::dps) * (M_PI / 180.0);

    //Calculate current distance in degrees
    currentDistanceInDegrees = normalizeHeading(InertialSensor.heading()) - startingDegrees;    

    //avgMotorVoltage = (motorVoltageLeft[0] + motorVoltageLeft[1] + motorVoltageLeft[2] + motorVoltageRight[0] + motorVoltageRight[1] + motorVoltageRight[2])/6;

//Launch Phase
if (std::abs(currentDistanceInDegrees) < (std::abs(targetDistanceInDegrees) - breakDistanceInDegrees) && accelCompleted == false) {

    for (int i = 0; i < 3; i++) {   
        // Use leftLaunchControl if minLaunchPower threshold is met for the left side
        motorVoltageLeft[i] = tractionControlLeft[i].tractionControlSpeed(motorVoltageLeft[i], leftMotorAngularRadians[i], robotRadiansPerSecond);      // get slip voltage 
        motorVoltageRight[i] = tractionControlRight[i].tractionControlSpeed(motorVoltageRight[i], rightMotorAngularRadians[i], robotRadiansPerSecond);   

    //    Brain.Screen.printAt(10, 20, "Launch Phase");
    }

     if (std::abs(robotRadiansPerSecond) >= maxRadiansPerSecond * (1 - percentSpeedLoss)){
        accelCompleted = true;
    }

//currentPhase = LAUNCH; 
/*
for (int i = 0; i < 3; i++) {   
        // Use leftLaunchControl if minLaunchPower threshold is met for the left side
       motorVoltageLeft[i] = (motorVoltageLeft[i] * 1.2) + adjustedHeadingCorrection;       // get slip voltage and Adjust for heading correction
       motorVoltageRight[i] = (motorVoltageRight[i] * 1.2) - adjustedHeadingCorrection; 

        //motorVoltageLeft[i] = (motorVoltageLeft[i] * 1.2);       // get slip voltage and Adjust for heading correction
        //motorVoltageRight[i] = (motorVoltageRight[i] * 1.2); 

    //    Brain.Screen.printAt(10, 20, "Launch Phase");
    }
 */   
        Brain.Screen.printAt(10, 20, "Launch Phase");
        Brain.Screen.printAt(10, 180, "Max Robot Angular Speed: %.2f", maxRadiansPerSecond);

//Cruise Phase      


} else if (std::abs(currentDistanceInDegrees) < (std::abs(targetDistanceInDegrees) - breakDistanceInDegrees) && accelCompleted == true) {
break;  
    Brain.Screen.printAt(10, 20, "Cruise Phase");    // Else condition - specify actions here if the if condition is not met
    for (int i = 0; i < 3; i++) {
        // Example action: Set motor voltage to target voltage directly
        motorVoltageLeft[i] = maxSpeedVoltage;
        motorVoltageRight[i] = maxSpeedVoltage;
    }
    

// Decel Phase

//If declerating then go to ABS routine
} else if (std::abs(currentDistanceInDegrees) >= (std::abs(targetDistanceInDegrees) - breakDistanceInDegrees) && decelCompleted == false) {
 break;   
  Brain.Screen.printAt(10, 20, "Decel Phase");
   

    for (int i = 0; i < 3; i++) {   
        
        //Left Side
        ABSReturn leftResult = ABSControllerLeft[i].ABSSpeedReduction(motorVoltageLeft[i]); // Get ABSResult containing motor voltage and brake mode
        motorVoltageLeft[i] = leftResult.motorVoltage; // Extract motor voltage from ABSResult
        if (motorVoltageLeft[i] != 0) {
            motorVoltageLeft[i]; // Add heading correction only if voltage is not zero
        }
        leftMotor[i].setBrake(leftResult.brakeMode); // Set brake mode from ABSResult

        //Right Side
        ABSReturn rightResult = ABSControllerRight[i].ABSSpeedReduction(motorVoltageRight[i]); // Get ABSResult containing motor voltage and brake mode
        motorVoltageRight[i] = rightResult.motorVoltage; // Extract motor voltage from ABSResult
        if (motorVoltageRight[i] != 0) {
            motorVoltageRight[i]; // Subtract heading correction only if voltage is not zero
        }
        rightMotor[i].setBrake(rightResult.brakeMode); // Set brake mode from ABSResult

        }
    

 /*          
     // Stop the motors
    for (int i = 0; i < 3; i++) {  
    leftMotor[i].stop(brake);
    rightMotor[i].stop(brake);
    }
*/


  //  decelCompleted = true;
    // If all drivetrain motors decel to min speed then change DecelCompleted State variable to true to start Approach Phase

    
if (robotRadiansPerSecond < minRadiansPerSecond) {
    decelCompleted = true;

    /*
         // Stop the motors
    for (int i = 0; i < 3; i++) {  
    leftMotor[i].stop(brake);
    rightMotor[i].stop(brake);
    }
    vex::task::sleep(3000);
  */

}



//Final Approach Phase 
} else if (decelCompleted == true) {
    break;
        Brain.Screen.printAt(10, 20, "Approach Phase");    
        Brain.Screen.printAt(10, 40, "Decel Compl: %d", decelCompleted); 

         // Calculate the PID output for distance control
        //double distanceCorrection = distancePID.calculate(targetDistance, currentDistance);

        // **Check if Distance Error is Within Target Zone and Update Stability Counter**
       // double distanceError = targetDistance - currentDistance;
                   
            // Set the speed to minSpeed after the first stabilization
        //    distanceCorrection = (distanceCorrection > 0) ? minSpeedVoltage : -minSpeedVoltage;
        //double adjustedHeadingCorrection = headingCorrection * (avgEncoderSpeedRPM / maxRPM) * 2.0; //dynamically reduce heading correction at slower speed based on percentage of max speed

    // Else condition - specify actions here if the if condition is not met
    for (int i = 0; i < 3; i++) {
        // Example action: Set motor voltage to target voltage directly
        motorVoltageLeft[i] = minSpeedVoltage;
        motorVoltageRight[i] = minSpeedVoltage;
    }
    Brain.Screen.printAt(10, 20, "Approach Phase");
} 


// Power Drive Motors        

std::copysign(turnDirection, targetDistanceInDegrees);

        for (int i = 0; i < 3; i++) {   
 
        leftMotor[i].spin(forward, motorVoltageLeft[i] * turnDirection, voltageUnits::volt);
        rightMotor[i].spin(forward, -motorVoltageRight[i] * turnDirection, voltageUnits::volt); 
        }
       
        //Brain.Screen.printAt(10, 20, "Rotation Sensor: %.2f degrees", rotationDegrees);
        //Brain.Screen.printAt(10, 40, "Current Distance: %.4f", currentDistance);
        //Brain.Screen.printAt(10, 80, "Break Distance: %.2f", (targetDistance - breakDistance));
        //Brain.Screen.printAt(10, 100, "Norm Heading: %.2f", normalizeHeading(InertialSensor.heading()));
        Brain.Screen.printAt(10, 120, "Lt Motor Speed: %.2f", leftMotor[2].velocity(vex::velocityUnits::pct));
        Brain.Screen.printAt(10, 140, "Rt Motor Speed: %.2f", rightMotor[2].velocity(vex::velocityUnits::rpm));
       // Brain.Screen.printAt(10, 140, "Rt Motor Speed: %.2f", rightMotorSpeed);
       // Brain.Screen.printAt(10, 170, "Max Speed Voltage: %.2f", maxSpeedVoltage);
       // Brain.Screen.printAt(10, 190, "Min Speed Voltage: %.2f", minSpeedVoltage);
        vex::task::sleep(20);
    }
    
 // Set brake mode to brake for all motors
    for (int i = 0; i < 3; i++) {
        leftMotor[i].setBrake(brakeType::brake);
        rightMotor[i].setBrake(brakeType::brake);
    }
    
    // Stop all motors simultaneously
    for (int i = 0; i < 3; i++) {
        leftMotor[i].stop();
        rightMotor[i].stop();
    }

/*
    //task::sleep(800);  // Small delay to prevent overwhelming the CPU
    currentDistance = (passiveEncoderLeft.position(degrees) + passiveEncoderRight.position(degrees) ) / 2 * (encoderWheelCircumferenceCM / 360.0);
    // Debug print: Stopping motors
    Brain.Screen.clearLine(8);
    Brain.Screen.setCursor(8, 1);
    Brain.Screen.print("Distance Complete");
    Brain.Screen.print("Current Distance: %.2f", currentDistance);
*/
}

// Traction Control Constructor implementation
tractionControl::tractionControl(double minSpeedVoltage, double maxSpeedVoltage)
    : minSpeedVoltage(minSpeedVoltage), maxSpeedVoltage(maxSpeedVoltage) {} 

    // adjustSpeed method implementation
    double tractionControl::tractionControlSpeed(double tractionMotorVoltage, double motorSpeed, double robotSpeed) { 

   Brain.Screen.printAt(10, 100, "Motor Speed: %.2f, Robot Speed: %.2f", motorSpeed,robotSpeed); 

    Brain.Screen.printAt(10, 20, "Launch Phase");
    if (std::abs(motorSpeed) > std::abs(robotSpeed) * (slipThreshold)) {                      // Checking for slip plus slip threshold allowance
       tractionMotorVoltage = tractionMotorVoltage / accelFactor;    
    
    } else {
        tractionMotorVoltage = tractionMotorVoltage * accelFactor;      
  
    }
    
    // Clamp voltage between the min and max while keeping the sign of maxSpeedVoltage.
    tractionMotorVoltage = std::copysign(std::max(minSpeedVoltage, std::min(std::abs(tractionMotorVoltage), std::abs(maxSpeedVoltage))), maxSpeedVoltage);                      

    return tractionMotorVoltage;
}  


ABSController::ABSController(double motorSpeed, double robotSpeed, double minSpeedVoltage)
    : motorSpeed(motorSpeed), robotSpeed(robotSpeed), minSpeedVoltage(minSpeedVoltage) {}

// adjustSpeed method implementation
ABSReturn ABSController::ABSSpeedReduction(double ABSMotorVoltage) {
    ABSReturn result;  // Create ABSResult instance to store motor voltage and brake mode

    if (ABSMotorVoltage <= minSpeedVoltage) {
        // Wheel has reduced speed to the target or slower
        result.motorVoltage = minSpeedVoltage;  // Set motor voltage to min Speed
        result.brakeMode = vex::brakeType::coast;  // Prevent locking
        Brain.Screen.printAt(60, 20, "coast");
    } else {
        // Check wheel locking condition
        if (std::abs(motorSpeed) >= std::abs(robotSpeed) * ABSLockThreshold) {
            // Wheels are locking
            result.motorVoltage = 0;  // Set motor voltage to 0
            result.brakeMode = vex::brakeType::coast;  // Prevent further locking
            Brain.Screen.printAt(60, 40, "coast (lock)");
        } else {
            // Wheels are not locking but need braking
            result.motorVoltage = 0;  // Set motor voltage to 0
            result.brakeMode = vex::brakeType::brake;  // Apply brakes to slow down
            Brain.Screen.printAt(60, 60, "brake");
        }
    }

    return result;  // Return ABSResult containing motor voltage and brake mode
} // end of code