#include "pid.h"          // Include the PID header file
#include "vex.h"          // Include the VEX library
#include "utils.h"        // Include the utility header for normalizeHeading
#include "robot_config.h" // Include the robot configuration
#include "navigation.h"
#include "odometry.h"
#include "autontasks.h"
#include <cmath> // Include math library for M_PI

using namespace vex; // Use the VEX namespace

/*straightOdometry(targetDistance, breakDistance, targetHeading, minSpeed,
                    kp_heading, ki_heading, kd_heading,
                    accelHeadingScaling, decelHeadingScaling,
                    approachHeadingScaling, maxSpeed);
*/                    

void autonTest(){
   initializeOpticalSensor();
    InertialSensor.setRotation(0, degrees);
    headingOffset = 0;
    matchloadStart(800,100,0,true);
    wait(200, msec);
    smartMove(100, 30, forward, 150); //for matchload smart wall stop, no pid
    wait(950, msec);
    //driveBackward(20, 15, 0);
   /*driveBackward(40, 20, 0);
   turnLeft(90, 80); 
   driveBackward(40, 20, 90);
   turnLeft(180,80); 
   driveBackward(40, 20, 180);
   turnLeft(270, 80); 
   driveBackward(40, 20, 270);
   turnLeft(360, 80); 
   driveBackward(40, 20, 360);*/
}

/*/void Calibration
{
    initializeOpticalSensor();
    InertialSensor.setRotation(0, degrees);
    headingOffset = 0;
    setStartPosition(0.0, 0.0, 0.0);
    startOdometryTask();
    forwardToPoint(200, 0, 20);

}*/


void odomTest(){
    initializeOpticalSensor();
    InertialSensor.setRotation(0, degrees);
    headingOffset = 0;
    
    // Set starting position and start odometry tracking
    setStartPosition(0.0, 0.0, 0.0);
    startOdometryTask();
    
    // Robot faces 0°, move backward 70cm by going forward to (-70, 0)
    forwardToPoint(0, 70, 40);  // This will make robot face 180° and move forward to reach (-70, 0)
    turnRightToPoint (70, 0, 70);
    //forwardToPoint(-100, 140, 40); 
    /*
    // Or if you want true backward movement, turn around first then move forward
    turnToPoint(-70, 0);  // Turn to face the target
    forwardToPoint(-70, 0);  // Move forward to the target
    */
    
    stopOdometryTask();
}

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
    
    //driveForward(200,110,0,25,0.8,0.09,0,0.1,0,0.1,100);

    //turnLeft(180,145,25,90,16); //bestoverall speed and lateral shift - Use this for speed
    
    
    //turnLeft(180,155,25,70, 16);// nest so far for 180, no lateral shift but some forward shift
    //turnLeft(180,160,25,70, 12);//good 180
    //turnOdometry(180,180,25,70, 90);
    //  intakeHopperStart(3000, 100, 500);  
    //smartMove(100, 40, forward, 150); //for matchload smart wall stop, no pid

    //driveForward(100,60,0,24,0.6,0.005,0,0.1,1,0.3,100); //Best for most distances
    //driveForward(200,60,0,18,1.1,0.005,0,0.1,1,0.3,81); //best for long distance

    //  wait(1000, msec); // brief pause to allow motors to settle
    /*  
       for (int i = 0; i < 3; i++)
    {
        leftMotor[i].setBrake(hold);
        rightMotor[i].setBrake(hold);
        leftMotor[i].stop();
        rightMotor[i].stop();
    }
*/
    

    //driveForward(100,60,0,20,1.1,0.005,0,0.1,1,0.3,100); // testing hold
    //driveForward(200,90,0,25,1.1,0.005,0,0.1,1,0.3,100); //testing hold

    //driveForward(200,80,0,25,1.1,0.0,0,0.0,1,0.2,100); //quite good on heading and distance
   //const double DECEL_STEP_PERCENT = 20;    // Voltage step as % of 12V (range: 1-10)
   //const double LOCK_THRESHOLD_DECEL = 0.25;



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

void leftSideLong(){
    initializeOpticalSensor();
    InertialSensor.setRotation(0, degrees);
    headingOffset = 16;
    ptoPneumatics.set(false);
    backHoodPneumatics.set(false);
    frontHoodPneumatics.set(false);
    wingPneumatics.set(false);

    wait(50, msec);
    intakeStart(700, 100, false);
    matchloadStart(6700,100,725,true);
    driveForward(83, 60, 16);
    wait(250, msec);
    turnLeft(150,118,26,80,16);
    wait(200, msec);
       
    driveForward(90,70,150,24,0.3,0.002,0,0.1,1,0.3,90);
    wait(200, msec);
    turnLeft(175,25,26,80,13);
    wait(200, msec);
    smartStraight(43, 21, 180, 15, 150);
    wait(270, msec);

    driveBackward(26, 14, 176);
    wait(200, msec);
    turnRight(-13,145,25,90,16);
    wait(200, msec);
    smartStraight(40, 19, -13, 24, 200);

    score(7000, 100);
    driveBackward(20, 14, -10);
    wait(100, msec);
    turnRight(-40,18,25,90,16);
    wait(100, msec);

    driveForward(35,25,-40,24,0.3,0.002,0,0.1,1,0.3,90);
    wait(100, msec);

    turnLeft(-4,30,26,80);
    wait(100, msec);
    driveForward(12,0,-4,24,0.3,0.002,0,0.1,1,0.3,90);
    wingPneumatics.set(true);

    driveForward(40,0,-4,24,0.3,0.002,0,0.1,1,0.3,40);
}

void leftSidemiddle(){
    initializeOpticalSensor();
    InertialSensor.setRotation(0, degrees);
    headingOffset = 16;
    ptoPneumatics.set(false);
    backHoodPneumatics.set(false);
    frontHoodPneumatics.set(false);
    wingPneumatics.set(false);

    wait(50, msec);
    intakeStart(300, 100, false);
    matchloadStart(1200,100,750,true);
    matchLoadPneumatics.set(true);
    driveForward(70, 60, 16,26);
        wait(200, msec);

        //driveBackward(8, 4, 16);
       // move(3, 50, vex::reverse); //simple move without PID
     //driveBackwardV2(8, 3, 16, 24, 1); // try this one, new motion profile with distance tolerance added as the last parameter.
    //driveBackwardV2(8,3,16,24,1,0.005,0,0.1,1,0.3,60); //This one gives you full control, chnage power to 60, too fast at that distance.

     turnRight(-50,50,25,90,16);

      //   wait(100, msec);
    //scoreStart(1200, 70);
     driveForward(25,13,-50,24,0.3,0.002,0,0.1,1,0.3,90);
     //        wait(200, msec);
    //driveBackward(105, 82, -45);

   


     //turnRight(-180,190,25,90,16);


    /*
    turnLeft(145.5,118,26,80,16);
    wait(200, msec);
       
    driveForward(98,78,145.5,24,0.3,0.002,0,0.1,1,0.3,90);
    wait(200, msec);
    turnLeft(175,25,26,80,13);
    wait(200, msec);
    smartStraight(30, 21, 180, 15, 150);
    wait(27000, msec);

    driveBackward(26, 14, 176);
    wait(200, msec);
    turnRight(-10,145,25,90,16);
    wait(200, msec);
    smartStraight(40, 19, -10, 24, 150);

    score(3200, 100);
    driveBackward(20, 14, -10);
    wait(200, msec);
    turnRight(-40,18,25,90,16);
    wait(100, msec);

    driveForward(35,25,-40,24,0.3,0.002,0,0.1,1,0.3,90);
    wait(100, msec);

    turnLeft(-4,30,26,80);
    wait(100, msec);
    driveForward(12,0,-4,24,0.3,0.002,0,0.1,1,0.3,90);
    wingPneumatics.set(true);

    driveForward(40,0,-4,24,0.3,0.002,0,0.1,1,0.3,40);
    */
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
    wait(800, msec);
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

void SevenBallRight(){
   matchLoadPneumatics.set(true);
    initializeOpticalSensor();
    InertialSensor.setRotation(0, degrees);
    headingOffset = -12;
    ptoPneumatics.set(false);
    backHoodPneumatics.set(false);
    frontHoodPneumatics.set(false);
    wait(50, msec);//necessary in order for matchload pneumatics to engage properly epstein fn
    matchLoadPneumatics.set(false);
   /* intakeStart(100, 75, true, false);
    wait(200, msec);
    intakeStart(100, 75, true, false);
    wait(200, msec); */
    intakeStart(1000, 40, true);
    intakeStart(3500, 75, true);
    driveForward(49, 38, -12);
    wait(300, msec);
    matchLoadPneumatics.set(true);
    driveForward(30, 24, -12);
    wait(200, msec);
    driveBackward(25, 18, -12);
    wait(100, msec);
    //turnRight(-88,65);
    //driveForward(72, 35, -88);
    //matchLoadPneumatics.set(false);
    //wait(100, msec);
    //turnLeft(-2,65);
    //pidlessForward(600, 20);
    //driveForward(19, 18, 0);

    //ptoPneumatics.set(true);
    //intakeStart(7500, 75, true, true);
    //score(7500, 75);
    //turnRight(180,80);
    //driveForward(40,30,180);
    //intakeStart(3000,75,true,true);
    //turnRight(180,80);

    //driveForward(71.5, 58, -126);
    //wait(250, msec);
    //turnRight(-179,25);
    //matchLoadPneumatics.set(true);
    //wait(250, msec);
    //driveForward(26, 20, -179);
    //intakeStart(700, 75, false, true);
    //wait(400, msec);
    //matchLoadPneumatics.set(false);
    //driveBackward(26, 20, -179);
    //wait(250, msec); 
    //turnRight(1.5,155);
    //wait(250, msec);
    //driveForward(40, 35, -358.5);
    //outtake(3000);
}

void SevenBallLeft(){
    matchLoadPneumatics.set(true);
    initializeOpticalSensor();
    InertialSensor.setRotation(0, degrees);
    headingOffset = 12;
    ptoPneumatics.set(false);
    backHoodPneumatics.set(false);
    frontHoodPneumatics.set(false);
    wait(50, msec);//necessary in order for matchload pneumatics to engage properly epstein fn
    matchLoadPneumatics.set(false);
    intakeStart(1000, 35, true);
    intakeStart(5500, 75, true);
    driveForward(40, 30, 16);
    wait(300, msec);
    //matchLoadPneumatics.set(true);
    driveForward(21, 21, 16);
    wait(100, msec);
    driveBackward(25, 15, 16);
    wait(100, msec);
    turnLeft(88,65);
    driveForward(72, 35, 88);
    //matchLoadPneumatics.set(false);
    wait(100, msec);
    turnRight(2,65);
    pidlessForward(600, 20);
    score(7500, 75);
    //driveForward(19, 18, 0);

    //ptoPneumatics.set(true);
    //intakeStart(7500, 75, true, true);
}
void rightMiddleAuto(){
    initializeOpticalSensor();
    InertialSensor.setRotation(0, degrees);
    headingOffset = -16;
    ptoPneumatics.set(false);
    backHoodPneumatics.set(false);
    frontHoodPneumatics.set(false);
       wingPneumatics.set(false);

    wait(50, msec);
    intakeStart(470, 100, false);
    matchloadStart(2000,40,630,true);
    driveForward(91, 58, -16);
     wait(200, msec);
    driveBackward(19, 12, -16);
        wait(100, msec);
     turnLeft(45,50,25,90,16);

    
  //  turnRight(-148,114,26,80,14);
    wait(200, msec);
   smartStraight(70, 50, 45, 15, 150);
outtake(500, 100);
        wait(100, msec);

    driveBackward(130, 110, 45);
        wait(100, msec);
     turnLeft(180,110,25,90,16);
   smartStraight(30, 21, 180, 15, 150);
       wait(270, msec);

    //smartMove(34, 60, forward, 250); //for matchload smart wall stop, no pid
    driveBackward(26, 14, 176);
        wait(200, msec);
    turnLeft(10,145,25,90,16);
        wait(200, msec);
   smartStraight(40, 19, 10, 24, 150);

    score(3200, 100);
    driveBackward(20, 14, 10);
            wait(200, msec);
    turnLeft(40,18,25,90,16);
                wait(100, msec);

    driveForward(35,25,40,24,0.3,0.002,0,0.1,1,0.3,90);
                    wait(100, msec);

    turnRight(4,30,26,80); 
                    wait(100, msec);
     driveForward(12,0,4,24,0.3,0.002,0,0.1,1,0.3,90);
   wingPneumatics.set(true);

    driveForward(40,0,4,24,0.3,0.002,0,0.1,1,0.3,40);

     /*
    driveForward(99,78,-148,24,0.3,0.002,0,0.1,1,0.3,90);
    wait(200, msec);
    turnRight(-176,20,26,80); 
    wait(200, msec);
   smartStraight(30, 21, -180, 15, 150);
       wait(270, msec);

    //smartMove(34, 60, forward, 250); //for matchload smart wall stop, no pid
    driveBackward(26, 14, -176);
        wait(200, msec);
    turnLeft(10,145,25,90,16);
        wait(200, msec);
   smartStraight(40, 19, 10, 24, 150);

    score(3200, 100);
    driveBackward(20, 14, 10);
            wait(200, msec);
    turnLeft(40,18,25,90,16);
                wait(100, msec);

    driveForward(35,25,40,24,0.3,0.002,0,0.1,1,0.3,90);
                    wait(100, msec);

    turnRight(4,30,26,80); 
                    wait(100, msec);
     driveForward(12,0,4,24,0.3,0.002,0,0.1,1,0.3,90);
   wingPneumatics.set(true);

    driveForward(40,0,4,24,0.3,0.002,0,0.1,1,0.3,40);


    //wait(250, msec);
    //driveForward(15, 10, -174.5);
    //intakeStart(700, 75, false);

    //wait(400, msec);
    //matchLoadPneumatics.set(false);
    //driveBackward(26, 20, -179);
    //wait(250, msec); 
    //turnRight(1.5,155,15,70);
    //wait(250, msec);
    //driveForward(40, 35, -358.5);
    //outtake(1000);
    //driveBackward(42.5, 28, -360);
    //wait(200, msec);
    //turnLeft(-315, 33, 15, 70);
    //wait(300, msec);
    //driveForward(126, 75, -315);
    //outtake(1000);
    //driveBackward(20, 15, -315);
    */
}
void soloAWP(){
    initializeOpticalSensor();
    InertialSensor.setRotation(0, degrees);
    headingOffset = -16;
    ptoPneumatics.set(false);
    backHoodPneumatics.set(false);
    frontHoodPneumatics.set(false);
       wingPneumatics.set(true);

    intakeStart(470, 100, false);
    matchloadStart(6400,100,1150,true);
    driveForward(85, 60, -16, 30);
    wait(250, msec);
    turnRight(-148,118,26,80,14);
    wait(200, msec);
       
    driveForward(105,70,-148,24,0.3,0.002,0,0.1,1,0.3,90);
    wait(200, msec);
    turnRight(-176,20,26,80); 
    wait(200, msec);
       smartStraight(45, 40, -180, 15, 200, 0.4, 0.01, 0.05, 0.2, 0.2, 0.2, 40);
       wait(80, msec);

    //smartMove(34, 60, forward, 250); //for matchload smart wall stop, no pid
    driveBackward(26, 14, -176);
        wait(200, msec);
    turnLeft(5,160,25,90,16);
        wait(200, msec);
   smartStraight(50, 20, 5, 24, 150, 0.4, 0.01, 0.05, 0.2, 0.2, 0.2, 150);

    score(3200, 100);
    driveBackward(20, 14, 10);
            wait(200, msec);
    turnLeft(45,18,25,90,16);
                wait(100, msec);

    driveForward(27,23,40,24,0.3,0.002,0,0.1,1,0.3,90);
                    wait(100, msec);

    turnRight(6,30,26,80); 
                    wait(100, msec);
     driveForward(12,0,6,24,0.3,0.002,0,0.1,1,0.3,90);
   wingPneumatics.set(false);

    driveForward(60,0,6,24,0.3,0.002,0,0.1,1,0.3,30);


    //wait(250, msec);
    //driveForward(15, 10, -174.5);
    //intakeStart(700, 75, false);

    //wait(400, msec);
    //matchLoadPneumatics.set(false);
    //driveBackward(26, 20, -179);
    //wait(250, msec); 
    //turnRight(1.5,155,15,70);
    //wait(250, msec);
    //driveForward(40, 35, -358.5);
    //outtake(1000);
    //driveBackward(42.5, 28, -360);
    //wait(200, msec);
    //turnLeft(-315, 33, 15, 70);
    //wait(300, msec);
    //driveForward(126, 75, -315);
    //outtake(1000);
    //driveBackward(20, 15, -315);
}

void colourTest(){
    initializeOpticalSensor();
    InertialSensor.setRotation(0, degrees);
    intakeStart(10000, 50, true);
}

/*
void odomTest(){
    initializeOpticalSensor();
    InertialSensor.setRotation(0, degrees);
    headingOffset = 0;

    setStartingPosition(0.0, 0.0, 0.0);
    startOdometryTask();

    forwardToPoint(100, 100, 20, 15, 0.615, 0, 0, 0.1, 0.05, 0.05, 50);
    
}

*/