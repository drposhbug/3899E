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
    
    forwardMP(10, 5, 0, 20);
    /*forwardMP(80, 38, 0, 20, 0.815, 0.0, 0.0, 0.0, 0.55, 0.3, 80);
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
    forwardMP(37, 18, 270, 15, 0.815, 0.0, 0.0, 0.0, 0, 0, 80);*/
}


void SpeedwayAutonLeft(){
    initializeOpticalSensor();
    InertialSensor.setRotation(0, degrees);
    headingOffset = 0;
    ptoPneumatics.set(true);
    backHoodPneumatics.set(false);
    frontHoodPneumatics.set(true);

    leftMP(16,10,20,50);
    wait(400, msec);
    intake(true,100);
    forwardMP(80,60,-18,20,0.615,0,0,0.1,0.05,0.05,50);
    wait(400, msec);
    backwardMP(7,4,-20,15);
    intake(false, 0);
    rightMP(52,47,20);
    forwardMP(18,10,29,30,0.5,0,0,0.1,0.05,0.05,100);
    ptoPneumatics.set(true);
    backHoodPneumatics.set(true);
    frontHoodPneumatics.set(false);
    score(400, 95);
    score(1000, 65);
    score(1000, 65);
    backwardMP(116,67,45,15);
    rightMP(97,60,20);
    wait(200, msec);
    matchLoadPneumatics.set(true);
    //forwardMP(27,19,180,20);
    rightMotor[0].spin(forward,12,vex::voltageUnits::volt);
    rightMotor[1].spin(forward,12,vex::voltageUnits::volt);
    rightMotor[2].spin(forward,12,vex::voltageUnits::volt);
    leftMotor[0].spin(forward,12,vex::voltageUnits::volt);
    leftMotor[1].spin(forward,12,vex::voltageUnits::volt);
    leftMotor[2].spin(forward,12,vex::voltageUnits::volt);
    wait(1000, msec);
    rightMotor[0].stop();
    rightMotor[1].stop();
    rightMotor[2].stop();
    leftMotor[0].stop();
    leftMotor[1].stop();
    leftMotor[2].stop();
    intake(true,100);
    wait(700, msec);
    intake(false,0);
    



    /*
    leftMP(105,82,20);
    wait(400, msec);
    forwardMP(76, 49, -124.5, 20);
    wait(200, msec);
    leftMP(54.5,45,15);
    wait(200, msec);
    matchLoadPneumatics.set(true);
    intake(true);
    forwardMP(25,15,-180,20);   
    wait(200, msec);
    intake(false);
    backwardMP(20,15,-180,15);
    wait(200, msec);
    //leftMP(180,150,20);
    //forwardMP(35,18,90,15,0.815,0.0,0.0,0.0,0.55,0.3,80);
    //score(1000);
    double currentHeading = InertialSensor.rotation(vex::degrees);
Controller.Screen.print("Heading: %.2f", currentHeading);
    
    //intake(miliseconds, true/false for on/off pistons)
        //outtake and scoring are just how many ms
*/
}

void SpeedwayAutonRight(){
   initializeOpticalSensor();
    InertialSensor.setRotation(0, degrees);
    headingOffset = 0;
    ptoPneumatics.set(false);
    backHoodPneumatics.set(false);
    frontHoodPneumatics.set(false);
    rightMP(14,10,20,50);
    wait(400, msec);
    intakeStart(2000, 75, true, false);
    forwardMP(80,58,20,20,0.615,0,0,0.1,0.05,0.05,50);
    wait(400, msec);
    backwardMP(7,4,20,15);
    leftMP(60,50,20);
    forwardMP(24,10,-40,30,0.5,0,0,0.1,0.05,0.05,100);
    ptoPneumatics.set(true);
    outtake(2000);
    wait(200, msec);
    rightMP(190,6,20,50);
    forwardMP(80,10,180,20);
    leftMP(50,10,20,50);
    intake(true,50);
    wait(500, msec);
    intake(false,0);
}