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
    
    forwardMP(79, 38, 180, 20, 0.815, 0.0, 0.0, 0.0, 0.55, 0.3, 80);
    wait(200, msec);
    leftMP(90,60,12.5);
    wait(200, msec);
    forwardMP(15, 10, 90, 15, 0.815, 0.0, 0.0, 0.0, 0.55, 0.3, 70);
    intake(1000, true);
    wait(200, msec);
    backwardMP(20, 15, 90, 10, 0.815, 0.00, 0.00, 0., 0.55, 0.3, 50);
    wait(200, msec);
    leftMP(175,125,20);
    wait(200, msec);
    forwardMP(35, 18, 0, 15, 0.815, 0.0, 0.0, 0.0, 0, 0, 80);
    score(1000);




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
    intake(1000, true);
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
    score(1000);
}

void autonTest(){
    initializeOpticalSensor();
    InertialSensor.setRotation(0, degrees);
    headingOffset = 0;

    intake(500, true);  //runs intake for 5 miliseconds with pistons
    wait(500, msec);
    score(1000);
}

void autonFwdRight(){
    initializeOpticalSensor();
    InertialSensor.setRotation(0, degrees);
    headingOffset = 0;
    ptoPneumatics.set(false);

    intakeStart(3000, true);  //runs intake for 3 seconds with pistons
    forwardMP(45, 25, 0, 20, 0.70, 0.0, 0.0, 0.0, 0.55, 0, 80);
    matchLoadPneumatics.set(false);
    wait(200, msec);
    leftMP(80,40,12.5);
    wait(200, msec);
    forwardMP(25, 10, 90, 15, 0.7, 0.0, 0.0, 0.0, 0.55, 0.3, 50);
    wait(200, msec);
    outtake(3000); //outtake for 3 seconds
}

void autonFwdLeft(){
    initializeOpticalSensor();
    InertialSensor.setRotation(0, degrees);
    headingOffset = 0;
    ptoPneumatics.set(false);

    forwardMP(200,80,0,30,0,0,0,0,0,0,100);
    /*intake(3000, true);  //runs intake for 3 seconds with pistons
    forwardMP(45, 25, 0, 20, 0.70, 0.0, 0.0, 0.0, 0.55, 0.3, 80);
    matchLoadPneumatics.set(false);
    wait(200, msec);
    rightMP(90,40,12.5);
    wait(200, msec);
    forwardMP(20, 10, 90, 15, 0.70, 0.0, 0.0, 0.0, 0.55, 0.3, 50);
    wait(200, msec);
    score(4000); //score for 3 seconds*/
}

void SpeedwayAutonLeft(){
    initializeOpticalSensor();
    InertialSensor.setRotation(0, degrees);
    headingOffset = 0;
    ptoPneumatics.set(false);

    leftMP(13,10,10,50);
    wait(400, msec);
    forwardMP(50,30,-13,20);
    wait(400, msec);
    leftMP(105,82,20);
    wait(400, msec);
    forwardMP(76, 49, -124.5, 20);
    wait(200, msec);
    leftMP(54.5,45,15);
    wait(200, msec);
    //intake(2000,true);
    forwardMP(25
        ,15,-180,20);
    wait(200, msec);
    backwardMP(20,15,-180,15);
    //wait(200, msec);
    //leftMP(180,150,20);
    //forwardMP(35,18,90,15,0.815,0.0,0.0,0.0,0.55,0.3,80);
    //score(1000);
    double currentHeading = InertialSensor.rotation(vex::degrees);
Controller.Screen.print("Heading: %.2f", currentHeading);
    
    //intake(miliseconds, true/false for on/off pistons)
        //outtake and scoring are just how many ms

}

void SpeedwayAutonRight(){
    initializeOpticalSensor();
    InertialSensor.setRotation(0, degrees);
    headingOffset = 0;
    ptoPneumatics.set(false);

    forwardMP(150,60,0,30,0.5,0,0,0,0.55,0.3,100);
}