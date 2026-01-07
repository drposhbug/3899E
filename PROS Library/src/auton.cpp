#include "main.h"
#include "pid.hpp"
#include "utils.hpp"
#include "robot_config.hpp"
#include "navigation.hpp"
#include "autontasks.hpp"
#include <cmath>

/*straightOdometry(targetDistance, breakDistance, targetHeading, minSpeed,
                    kp_heading, ki_heading, kd_heading,
                    accelHeadingScaling, decelHeadingScaling,
                    approachHeadingScaling, maxSpeed);
*/                    

void autonTest() {
    initializeOpticalSensor();
    inertialSensor.set_rotation(0);
    headingOffset = 0;
    matchloadStart(800, 100, 0, true);
    pros::delay(200);
    smartMove(100, 30, true, 150); // true = forward, for matchload smart wall stop, no pid
    pros::delay(950);
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
    inertialSensor.set_rotation(0);
    headingOffset = 0;
    setStartPosition(0.0, 0.0, 0.0);
    startOdometryTask();
    forwardToPoint(200, 0, 20);
}*/


void odomTest() {
    initializeOpticalSensor();
    inertialSensor.set_rotation(0);
    headingOffset = 0;
    
    // Set starting position and start odometry tracking
    //setStartPosition(0.0, 0.0, 0.0);
   // startOdometryTask();
    
    // Robot faces 0°, move backward 70cm by going forward to (-70, 0)
    //forwardToPoint(0, 70, 40);  // This will make robot face 180° and move forward to reach (-70, 0)
   // turnRightToPoint(70, 0, 70);
    //forwardToPoint(-100, 140, 40); 
    /*
    // Or if you want true backward movement, turn around first then move forward
    turnToPoint(-70, 0);  // Turn to face the target
    forwardToPoint(-70, 0);  // Move forward to the target
    */
    
    //stopOdometryTask();
}

//turnOdometry(turnAmount, breakDistance, minSpeed, maxSpeed)
void autonLeft() {
    initializeOpticalSensor();

    // Reset gyro to ensure clean starting state
    inertialSensor.set_rotation(0);
    //inertialSensor.set_heading(0);

    //bool isMatchLoadPneumaticsActive = false;
 
    headingOffset = 180; 
    
    //(targetDistance, breakDistance, targetHeading, minSpeed, Kp, Ki, Kd, 
    // accelHeadingScaling, decelHeadingScaling,approachHeadingScaling, maxSpeed
    
    /*/frontHoodPneumatics.set_value(true);
    backHoodPneumatics.set_value(true);
    matchLoadPneumatics.set_value(true);
    intakeMotor1.move_voltage(-12000); //forward is outtake reverse is intake
    intakeMotor2.move_voltage(-12000);
    pros::delay(500);
    frontHoodPneumatics.set_value(false);
    backHoodPneumatics.set_value(false);
    matchLoadPneumatics.set_value(false);*/
    

    //george this is MY stuffnegnagaffgafgniafa
    forwardMP(79, 38, 180, 20, 0.815, 0.0, 0.0, 0.0, 0.55, 0.3, 80);
    pros::delay(200);
    leftMP(90, 60, 12.5);
    pros::delay(200);
    forwardMP(15, 10, 90, 15, 0.815, 0.0, 0.0, 0.0, 0.55, 0.3, 70);
    pros::delay(200);
    backwardMP(20, 15, 90, 10, 0.815, 0.00, 0.00, 0., 0.55, 0.3, 50);
    pros::delay(200);
    leftMP(175, 125, 20);
    pros::delay(200);
    forwardMP(35, 18, 0, 15, 0.815, 0.0, 0.0, 0.0, 0, 0, 80);

    //leftMP(180,140,15);
    //backwardMP(200, 60, 0, 30, 0.5, 0.00, 0.00, 0.5, 0.5, 0.5, 100);
    //leftMP(180,136,8);
    //turnOdometry(180, 20, 10, 100);
    //pros::delay(300);
    //turnOdometry(0, 20, 10, 100);
    //pros::delay(200);
    //turnOdometry(180, 20, 10, 100);
    //pros::delay(200);

    /*
    // Explicitly ensure all motors are braked at the end
    for (int i = 0; i < 3; i++) {
        leftMotor[i]->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        rightMotor[i]->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        leftMotor[i]->brake();
        rightMotor[i]->brake();
    }
    */

    //pros::delay(10000);  // Wait to test resistance
}

void autonRight() {
    initializeOpticalSensor();

    inertialSensor.set_rotation(0);
 
    headingOffset = 0;
    
    //driveForward(200,110,0,25,0.8,0.09,0,0.1,0,0.1,100);

    //turnLeft(180,145,25,90,16); //bestoverall speed and lateral shift - Use this for speed
    
    //turnLeft(180,155,25,70, 16);// nest so far for 180, no lateral shift but some forward shift
    //turnLeft(180,160,25,70, 12);//good 180
    //turnOdometry(180,180,25,70, 90);
    //  intakeHopperStart(3000, 100, 500);  
    //smartMove(100, 40, true, 150); //for matchload smart wall stop, no pid

    //driveForward(100,60,0,24,0.6,0.005,0,0.1,1,0.3,100); //Best for most distances
    //driveForward(200,60,0,18,1.1,0.005,0,0.1,1,0.3,81); //best for long distance

    //  pros::delay(1000); // brief pause to allow motors to settle
    /*  
    for (int i = 0; i < 3; i++) {
        leftMotor[i]->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        rightMotor[i]->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        leftMotor[i]->brake();
        rightMotor[i]->brake();
    }
    */

    //driveForward(100,60,0,20,1.1,0.005,0,0.1,1,0.3,100); // testing hold
    //driveForward(200,90,0,25,1.1,0.005,0,0.1,1,0.3,100); //testing hold

    //driveForward(200,80,0,25,1.1,0.0,0,0.0,1,0.2,100); //quite good on heading and distance
    //const double DECEL_STEP_PERCENT = 20;    // Voltage step as % of 12V (range: 1-10)
    //const double LOCK_THRESHOLD_DECEL = 0.25;

    /*forwardMP(80, 38, 0, 20, 0.815, 0.0, 0.0, 0.0, 0.55, 0.3, 80);
    pros::delay(200);
    rightMP(90,65,12.5);
    pros::delay(200);
    forwardMP(15, 10, 90, 15, 0.815, 0.0, 0.0, 0.0, 0.55, 0.3, 70);
    pros::delay(200);
    backwardMP(20, 15, 90, 10, 0.815, 0.00, 0.00, 0., 0.55, 0.3, 50);
    pros::delay(200);
    rightMP(78,50,20);
    pros::delay(200);
    backwardMP(3, 3, 90, 10, 0, 0.00, 0.00, 0., 0, 0, 20);
    pros::delay(200);
    rightMP(78,50,20);
    pros::delay(200);
    forwardMP(37, 18, 270, 15, 0.815, 0.0, 0.0, 0.0, 0, 0, 80);*/
}

void leftSideLong() {
    initializeOpticalSensor();
    inertialSensor.set_rotation(0);
    headingOffset = 16;
    ptoPneumatics.set_value(false);
    backHoodPneumatics.set_value(false);
    frontHoodPneumatics.set_value(false);
    wingPneumatics.set_value(false);

    pros::delay(50);
    intakeStart(1000, 100, false);
    matchloadStart(6000, 100, 1050, true);
    driveForward(83, 60, 16);
    pros::delay(250);
    turnLeft(147, 118, 26, 80, 16);
    pros::delay(200);
       
    driveForward(102, 70, 147, 24, 0.3, 0.002, 0, 0.1, 1, 0.3, 90);
    pros::delay(200);
    turnLeft(175, 25, 26, 80, 13);
    pros::delay(200);
    smartStraight(50, 26, 180, 15, 150);
    pros::delay(270);

    driveBackward(26, 14, 176);
    pros::delay(200);
    turnRight(0, 145, 25, 90, 16);
    pros::delay(200);
    smartStraight(40, 19, 0, 24, 200);

    score(3200, 100);
    driveBackward(20, 14, -10);
    pros::delay(100);
    turnLeft(40, 18, 25, 90, 16);
    
    pros::delay(100);

    driveForward(35, 25, 40, 24, 0.3, 0.002, 0, 0.1, 1, 0.3, 90);
    pros::delay(100);

    turnRight(4, 30, 26, 80);
    pros::delay(100);
    driveForward(12, 0, 4, 24, 0.3, 0.002, 0, 0.1, 1, 0.3, 90);
    wingPneumatics.set_value(true);

    driveForward(40, 0, 4, 24, 0.3, 0.002, 0, 0.1, 1, 0.3, 40);
}

void leftSidemiddle() {
    initializeOpticalSensor();
    inertialSensor.set_rotation(0);
    headingOffset = 16;
    ptoPneumatics.set_value(false);
    backHoodPneumatics.set_value(false);
    frontHoodPneumatics.set_value(false);
    wingPneumatics.set_value(false);

    pros::delay(50);
    intakeStart(300, 100, false);
    matchloadStart(1200, 100, 750, true);
    matchLoadPneumatics.set_value(true);
    driveForward(70, 60, 16, 26);
    pros::delay(200);

    //driveBackward(8, 4, 16);
    // move(3, 50, false); //simple move without PID (false = reverse)
    //driveBackwardV2(8, 3, 16, 24, 1); // try this one, new motion profile with distance tolerance added as the last parameter.
    //driveBackwardV2(8,3,16,24,1,0.005,0,0.1,1,0.3,60); //This one gives you full control, change power to 60, too fast at that distance.

    turnRight(-50, 50, 25, 90, 16);

    //   pros::delay(100);
    //scoreStart(1200, 70);
    driveForward(25, 13, -50, 24, 0.3, 0.002, 0, 0.1, 1, 0.3, 90);
    //        pros::delay(200);
    //driveBackward(105, 82, -45);

    //turnRight(-180,190,25,90,16);

    /*
    turnLeft(145.5,118,26,80,16);
    pros::delay(200);
       
    driveForward(98,78,145.5,24,0.3,0.002,0,0.1,1,0.3,90);
    pros::delay(200);
    turnLeft(175,25,26,80,13);
    pros::delay(200);
    smartStraight(30, 21, 180, 15, 150);
    pros::delay(27000);

    driveBackward(26, 14, 176);
    pros::delay(200);
    turnRight(-10,145,25,90,16);
    pros::delay(200);
    smartStraight(40, 19, -10, 24, 150);

    score(3200, 100);
    driveBackward(20, 14, -10);
    pros::delay(200);
    turnRight(-40,18,25,90,16);
    pros::delay(100);

    driveForward(35,25,-40,24,0.3,0.002,0,0.1,1,0.3,90);
    pros::delay(100);

    turnLeft(-4,30,26,80);
    pros::delay(100);
    driveForward(12,0,-4,24,0.3,0.002,0,0.1,1,0.3,90);
    wingPneumatics.set_value(true);

    driveForward(40,0,-4,24,0.3,0.002,0,0.1,1,0.3,40);
    */
}

void SpeedwayAutonLeft() {
    initializeOpticalSensor();
    inertialSensor.set_rotation(0);
    headingOffset = 0;
    ptoPneumatics.set_value(true);
    backHoodPneumatics.set_value(false);
    frontHoodPneumatics.set_value(true);

    leftMP(16, 10, 20, 50);
    pros::delay(400);
    intake(true, 100);
    forwardMP(80, 60, -18, 20, 0.615, 0, 0, 0.1, 0.05, 0.05, 50);
    pros::delay(400);
    backwardMP(7, 4, -20, 15);
    intake(false, 0);
    rightMP(52, 47, 20);
    forwardMP(18, 10, 29, 30, 0.5, 0, 0, 0.1, 0.05, 0.05, 100);
    ptoPneumatics.set_value(true);
    backHoodPneumatics.set_value(true);
    frontHoodPneumatics.set_value(false);
    score(400, 95);
    score(1000, 65);
    score(1000, 65);
    backwardMP(116, 67, 45, 15);
    rightMP(97, 60, 20);
    pros::delay(200);
    matchLoadPneumatics.set_value(true);
    //forwardMP(27,19,180,20);
    rightMotor[0]->move_voltage(12000);
    rightMotor[1]->move_voltage(12000);
    rightMotor[2]->move_voltage(12000);
    leftMotor[0]->move_voltage(12000);
    leftMotor[1]->move_voltage(12000);
    leftMotor[2]->move_voltage(12000);
    pros::delay(800);
    rightMotor[0]->brake();
    rightMotor[1]->brake();
    rightMotor[2]->brake();
    leftMotor[0]->brake();
    leftMotor[1]->brake();
    leftMotor[2]->brake();
    intake(true, 100);
    pros::delay(700);
    intake(false, 0);

    /*
    leftMP(105,82,20);
    pros::delay(400);
    forwardMP(76, 49, -124.5, 20);
    pros::delay(200);
    leftMP(54.5,45,15);
    pros::delay(200);
    matchLoadPneumatics.set_value(true);
    intake(true);
    forwardMP(25,15,-180,20);   
    pros::delay(200);
    intake(false);
    backwardMP(20,15,-180,15);
    pros::delay(200);
    //leftMP(180,150,20);
    //forwardMP(35,18,90,15,0.815,0.0,0.0,0.0,0.55,0.3,80);
    //score(1000);
    double currentHeading = inertialSensor.get_rotation();
    pros::lcd::print(0, "Heading: %.2f", currentHeading);
    
    //intake(miliseconds, true/false for on/off pistons)
    //outtake and scoring are just how many ms
    */
}

void SevenBallRight() {
    matchLoadPneumatics.set_value(true);
    initializeOpticalSensor();
    inertialSensor.set_rotation(0);
    headingOffset = -12;
    ptoPneumatics.set_value(false);
    backHoodPneumatics.set_value(false);
    frontHoodPneumatics.set_value(false);
    pros::delay(50); //necessary in order for matchload pneumatics to engage properly
    matchLoadPneumatics.set_value(false);
    /* intakeStart(100, 75, true, false);
    pros::delay(200);
    intakeStart(100, 75, true, false);
    pros::delay(200); */
    intakeStart(1000, 40, true);
    intakeStart(3500, 75, true);
    driveForward(49, 38, -12);
    pros::delay(300);
    matchLoadPneumatics.set_value(true);
    driveForward(30, 24, -12);
    pros::delay(200);
    driveBackward(25, 18, -12);
    pros::delay(100);
    //turnRight(-88,65);
    //driveForward(72, 35, -88);
    //matchLoadPneumatics.set_value(false);
    //pros::delay(100);
    //turnLeft(-2,65);
    //pidlessForward(600, 20);
    //driveForward(19, 18, 0);

    //ptoPneumatics.set_value(true);
    //intakeStart(7500, 75, true, true);
    //score(7500, 75);
    //turnRight(180,80);
    //driveForward(40,30,180);
    //intakeStart(3000,75,true,true);
    //turnRight(180,80);

    //driveForward(71.5, 58, -126);
    //pros::delay(250);
    //turnRight(-179,25);
    //matchLoadPneumatics.set_value(true);
    //pros::delay(250);
    //driveForward(26, 20, -179);
    //intakeStart(700, 75, false, true);
    //pros::delay(400);
    //matchLoadPneumatics.set_value(false);
    //driveBackward(26, 20, -179);
    //pros::delay(250); 
    //turnRight(1.5,155);
    //pros::delay(250);
    //driveForward(40, 35, -358.5);
    //outtake(3000);
}

void SevenBallLeft() {
    matchLoadPneumatics.set_value(true);
    initializeOpticalSensor();
    inertialSensor.set_rotation(0);
    headingOffset = 12;
    ptoPneumatics.set_value(false);
    backHoodPneumatics.set_value(false);
    frontHoodPneumatics.set_value(false);
    pros::delay(50); //necessary in order for matchload pneumatics to engage properly
    matchLoadPneumatics.set_value(false);
    intakeStart(1000, 35, true);
    intakeStart(5500, 75, true);
    driveForward(40, 30, 16);
    pros::delay(300);
    //matchLoadPneumatics.set_value(true);
    driveForward(21, 21, 16);
    pros::delay(100);
    driveBackward(25, 15, 16);
    pros::delay(100);
    turnLeft(88, 65);
    driveForward(72, 35, 88);
    //matchLoadPneumatics.set_value(false);
    pros::delay(100);
    turnRight(2, 65);
    pidlessForward(600, 20);
    score(7500, 75);
    //driveForward(19, 18, 0);

    //ptoPneumatics.set_value(true);
    //intakeStart(7500, 75, true, true);
}

void rightMiddleAuto() {
    initializeOpticalSensor();
    inertialSensor.set_rotation(0);
    headingOffset = -16;
    ptoPneumatics.set_value(false);
    backHoodPneumatics.set_value(false);
    frontHoodPneumatics.set_value(false);
    wingPneumatics.set_value(false);

    pros::delay(50);
    intakeStart(470, 100, false);
    matchloadStart(2000, 40, 630, true);
    driveForward(91, 58, -16);
    pros::delay(200);
    driveBackward(19, 12, -16);
    pros::delay(100);
    turnLeft(45, 50, 25, 90, 16);

    //  turnRight(-148,114,26,80,14);
    pros::delay(200);
    smartStraight(70, 50, 45, 15, 150);
    outtake(500, 100);
    pros::delay(100);

    driveBackward(130, 110, 45);
    pros::delay(100);
    turnLeft(180, 110, 25, 90, 16);
    smartStraight(30, 21, 180, 15, 150);
    pros::delay(270);

    //smartMove(34, 60, true, 250); //for matchload smart wall stop, no pid
    driveBackward(26, 14, 176);
    pros::delay(200);
    turnLeft(10, 145, 25, 90, 16);
    pros::delay(200);
    smartStraight(40, 19, 10, 24, 150);

    score(3200, 100);
    driveBackward(20, 14, 10);
    pros::delay(200);
    turnLeft(40, 18, 25, 90, 16);
    pros::delay(100);

    driveForward(35, 25, 40, 24, 0.3, 0.002, 0, 0.1, 1, 0.3, 90);
    pros::delay(100);

    turnRight(4, 30, 26, 80); 
    pros::delay(100);
    driveForward(12, 0, 4, 24, 0.3, 0.002, 0, 0.1, 1, 0.3, 90);
    wingPneumatics.set_value(true);

    driveForward(40, 0, 4, 24, 0.3, 0.002, 0, 0.1, 1, 0.3, 40);

    /*
    driveForward(99,78,-148,24,0.3,0.002,0,0.1,1,0.3,90);
    pros::delay(200);
    turnRight(-176,20,26,80); 
    pros::delay(200);
    smartStraight(30, 21, -180, 15, 150);
    pros::delay(270);

    //smartMove(34, 60, true, 250); //for matchload smart wall stop, no pid
    driveBackward(26, 14, -176);
    pros::delay(200);
    turnLeft(10,145,25,90,16);
    pros::delay(200);
    smartStraight(40, 19, 10, 24, 150);

    score(3200, 100);
    driveBackward(20, 14, 10);
    pros::delay(200);
    turnLeft(40,18,25,90,16);
    pros::delay(100);

    driveForward(35,25,40,24,0.3,0.002,0,0.1,1,0.3,90);
    pros::delay(100);

    turnRight(4,30,26,80); 
    pros::delay(100);
    driveForward(12,0,4,24,0.3,0.002,0,0.1,1,0.3,90);
    wingPneumatics.set_value(true);

    driveForward(40,0,4,24,0.3,0.002,0,0.1,1,0.3,40);


    //pros::delay(250);
    //driveForward(15, 10, -174.5);
    //intakeStart(700, 75, false);

    //pros::delay(400);
    //matchLoadPneumatics.set_value(false);
    //driveBackward(26, 20, -179);
    //pros::delay(250); 
    //turnRight(1.5,155,15,70);
    //pros::delay(250);
    //driveForward(40, 35, -358.5);
    //outtake(1000);
    //driveBackward(42.5, 28, -360);
    //pros::delay(200);
    //turnLeft(-315, 33, 15, 70);
    //pros::delay(300);
    //driveForward(126, 75, -315);
    //outtake(1000);
    //driveBackward(20, 15, -315);
    */
}

void soloAWP() {
    initializeOpticalSensor();
    inertialSensor.set_rotation(0);
    headingOffset = -16;
    ptoPneumatics.set_value(false);
    backHoodPneumatics.set_value(false);
    frontHoodPneumatics.set_value(false);
    wingPneumatics.set_value(true);

    intakeStart(1000, 100, false);
    matchloadStart(5500, 100, 1050, true);
    driveForward(85, 60, -16, 30);
    pros::delay(250);
    turnRight(-148, 118, 26, 80, 14);
    pros::delay(200);
       
    driveForward(105, 76, -148, 24, 0.3, 0.002, 0, 0.1, 1, 0.3, 90);
    pros::delay(200);
    turnRight(-176, 20, 26, 80); 
    pros::delay(200);
    smartStraight(47, 40, -180, 15, 200, 0.4, 0.01, 0.05, 0.2, 0.2, 0.2, 40);
    pros::delay(80);

    //smartMove(34, 60, true, 250); //for matchload smart wall stop, no pid
    driveBackward(26, 14, -176);
    pros::delay(200);
    turnLeft(8, 160, 25, 90, 16);
    pros::delay(200);
    smartStraight(50, 20, 8, 24, 150, 0.4, 0.01, 0.05, 0.2, 0.2, 0.2, 150);

    score(3200, 100);
    driveBackward(20, 14, 10);
    pros::delay(200);
    turnLeft(45, 18, 25, 90, 16);
    pros::delay(100);

    driveForward(27, 23, 40, 24, 0.3, 0.002, 0, 0.1, 1, 0.3, 90);
    pros::delay(100);

    turnRight(6, 30, 26, 80); 
    pros::delay(100);
    driveForward(12, 0, 6, 24, 0.3, 0.002, 0, 0.1, 1, 0.3, 90);
    wingPneumatics.set_value(false);

    driveForward(60, 0, 6, 24, 0.3, 0.002, 0, 0.1, 1, 0.3, 30);

    //pros::delay(250);
    //driveForward(15, 10, -174.5);
    //intakeStart(700, 75, false);

    //pros::delay(400);
    //matchLoadPneumatics.set_value(false);
    //driveBackward(26, 20, -179);
    //pros::delay(250); 
    //turnRight(1.5,155,15,70);
    //pros::delay(250);
    //driveForward(40, 35, -358.5);
    //outtake(1000);
    //driveBackward(42.5, 28, -360);
    //pros::delay(200);
    //turnLeft(-315, 33, 15, 70);
    //pros::delay(300);
    //driveForward(126, 75, -315);
    //outtake(1000);
    //driveBackward(20, 15, -315);
}

void colourTest() {
    initializeOpticalSensor();
    inertialSensor.set_rotation(0);
    intakeStart(10000, 50, true);
}

/*
void odomTest(){
    initializeOpticalSensor();
    inertialSensor.set_rotation(0);
    headingOffset = 0;

    setStartingPosition(0.0, 0.0, 0.0);
    startOdometryTask();

    forwardToPoint(100, 100, 20, 15, 0.615, 0, 0, 0.1, 0.05, 0.05, 50);
}
*/
