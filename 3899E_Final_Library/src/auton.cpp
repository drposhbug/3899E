#include "pid.h"          // Include the PID header file
#include "vex.h"          // Include the VEX library
#include "utils.h"        // Include the utility header for normalizeHeading
#include "robot_config.h" // Include the robot configuration
#include "navigation.h"
#include "odometry.h"
#include <cmath> // Include math library for M_PI

using namespace vex; // Use the VEX namespace

/*straightOdometry(targetDistance, breakDistance, targetHeading, minSpeed,
                    kp_heading, ki_heading, kd_heading,
                    accelHeadingScaling, decelHeadingScaling,
                    approachHeadingScaling, maxSpeed);
*/                    

//turnOdometry(turnAmount, breakDistance, minSpeed, maxSpeed)
   void autonLeft()
{
    initializeOpticalSensor();

    // Reset gyro to ensure clean starting state
    InertialSensor.setRotation(0, degrees);
    //InertialSensor.setHeading(0, degrees);

    //bool isMatchLoadPneumaticsActive = false;
 
    
    headingOffset = 180; 
    
    //(targetDistance, breakDistance, targetHeading, minSpeed, Kp, Ki, Kd, 
    // accelHeadingScaling, decelHeadingScaling,approachHeadingScaling, maxSpeed
    
    /*/frontHoodPneumatics.set(true);
    backHoodPneumatics.set(true);
    matchLoadPneumatics.set(true);
    intakeMotor1.spin(reverse, 12, vex::voltageUnits::volt); //forward is outtake reverse is intake
    intakeMotor2.spin(reverse, 12, vex::voltageUnits::volt);
    wait(500, msec);
    frontHoodPneumatics.set(false
    backHoodPneumatics.set(false);
    matchLoadPneumatics.set(false);*/
    


    //george this is MY stuffnegnagaffgafgniafa
    forwardMP(79, 38, 180, 20, 0.815, 0.0, 0.0, 0.0, 0.55, 0.3, 80);
    wait(200, msec);
    leftMP(90,60,12.5);
    wait(200, msec);
    forwardMP(15, 10, 90, 15, 0.815, 0.0, 0.0, 0.0, 0.55, 0.3, 70);
    wait(200, msec);
    backwardMP(20, 15, 90, 10, 0.815, 0.00, 0.00, 0., 0.55, 0.3, 50);
    wait(200, msec);
    leftMP(175,125,20);
    wait(200, msec);
    forwardMP(35, 18, 0, 15, 0.815, 0.0, 0.0, 0.0, 0, 0, 80);




    //leftMP(180,140,15);
    //backwardMP(200, 60, 0, 30, 0.5, 0.00, 0.00, 0.5, 0.5, 0.5, 100);
    //leftMP(180,136,8);
    //turnOdometry(180, 20, 10, 100);
    //wait(300, msec);
    //turnOdometry(0, 20, 10, 100);
    //wait(200, msec);
    //turnOdometry(180, 20, 10, 100);
    //wait(200, msec);
    


    /*
    // Explicitly ensure all motors are braked at the end
    for (int i = 0; i < 3; i++) {
        leftMotor[i].setBrake(brake);
        rightMotor[i].setBrake(brake);
        leftMotor[i].stop();
        rightMotor[i].stop();
    }
    */

    //wait(10000, msec);  // Wait to test resistance
}

void autonRight(){
    initializeOpticalSensor();

    InertialSensor.setRotation(0, degrees);
 
    headingOffset = 0;
    
    forwardMP(80, 38, 0, 20, 0.815, 0.0, 0.0, 0.0, 0.55, 0.3, 80);
    wait(200, msec);
    rightMP(90,65,12.5);
    wait(200, msec);
    forwardMP(15, 10, 90, 15, 0.815, 0.0, 0.0, 0.0, 0.55, 0.3, 70);
    wait(200, msec);
    backwardMP(20, 15, 90, 10, 0.815, 0.00, 0.00, 0., 0.55, 0.3, 50);
    wait(200, msec);
    rightMP(78,50,20);
    wait(200, msec);
    backwardMP(3, 3, 90, 10, 0, 0.00, 0.00, 0., 0, 0, 20);
    wait(200, msec);
    rightMP(78,50,20);
    wait(200, msec);
    forwardMP(37, 18, 270, 15, 0.815, 0.0, 0.0, 0.0, 0, 0, 80);
}


