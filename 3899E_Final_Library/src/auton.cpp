#include "pid.h"          
#include "vex.h"          
#include "utils.h"        
#include "robot_config.h" 
//#include "navigation.h"
#include "odometry.h"
#include "autontasks.h"
#include <cmath> 

using namespace vex; 

/*straightOdometry(targetDistance, breakDistance, targetHeading, minSpeed,
                    kp_heading, ki_heading, kd_heading,
                    accelHeadingScaling, decelHeadingScaling,
                    approachHeadingScaling, maxSpeed);
*/                    

void visionSensorTest() {
    // REMOVED: visionSensor.setLight(...) -> This function does not exist for AI Vision
    
    Brain.Screen.clearScreen();
    Brain.Screen.setPenColor(vex::color::white);
    Brain.Screen.printAt(10, 20, "VISION TEST MODE");
    Brain.Screen.printAt(10, 40, "Use Full Frame (0-320, 0-240)");

    while (true) {
        // --- TEST RED BLOCK ---
        // Uses the ID for Red Block (2)
        AIVision20.takeSnapshot(AIVision20__redCube);
        int redCount = AIVision20.objectCount;
        int redX = (redCount > 0) ? AIVision20.objects[0].centerX : 0;
        int redW = (redCount > 0) ? AIVision20.objects[0].width   : 0;

        // --- TEST BLUE BLOCK ---
        // Uses the ID for Blue Block (1)
        AIVision20.takeSnapshot(AIVision20__blueCube);
        int blueCount = AIVision20.objectCount;
        int blueX = (blueCount > 0) ? AIVision20.objects[0].centerX : 0;
        int blueW = (blueCount > 0) ? AIVision20.objects[0].width   : 0;

        // --- DISPLAY DATA ---
        // Line 3: Red Stats
        Brain.Screen.setCursor(3, 1);
        if (redCount > 0) {
            Brain.Screen.setPenColor(vex::color::red);
            Brain.Screen.print("RED : FOUND! X=%3d W=%3d  ", redX, redW);
        } else {
            Brain.Screen.setPenColor(vex::color::white);
            Brain.Screen.print("RED : SEARCHING...        ");
        }

        // Line 4: Blue Stats
        Brain.Screen.setCursor(4, 1);
        if (blueCount > 0) {
            Brain.Screen.setPenColor(vex::color::cyan);
            Brain.Screen.print("BLUE: FOUND! X=%3d W=%3d  ", blueX, blueW);
        } else {
            Brain.Screen.setPenColor(vex::color::white);
            Brain.Screen.print("BLUE: SEARCHING...        ");
        }

        // Line 6: Debug Tip
        Brain.Screen.setCursor(6, 1);
        Brain.Screen.setPenColor(vex::color::yellow);
        Brain.Screen.print("If 0 found: Check IDs in Utility");

        vex::task::sleep(100); // Update every 100ms
    }
}

/*
void autonTest(){
   initializeOpticalSensor();
    InertialSensor.setRotation(0, degrees);
    robotStartingHeading = 0;
    //visionDrive(AIVision20__blueCube, 50.0, 0.0);
    visionDriveMinimal(AIVision20__blueCube, 150, 0, 22, 40);

    



    //visionSensorTest();
    //matchloadStart(800,100,0,true);
    //wait(200, msec);
    //smartMove(100, 30, forward, 150); //for matchload smart wall stop, no pid
    //wait(950, msec);
    //driveBackward(20, 15, 0);
      //driveBackward(40, 20, 0);
   turnLeft(90, 80);
   driveBackward(40, 20, 90);
   turnLeft(180,80); 
   driveBackward(40, 20, 180);
   turnLeft(270, 80); 
   driveBackward(40, 20, 270);
   turnLeft(360, 80); 
   driveBackward(40, 20, 360);
}
*/

void autonTest() {
    // Initialize sensor
    initializeOpticalSensor();
    setStartPosition(40, 0.0, -90.0);
    startOdometryTask();
    startCoordinateFinder();
    moveOdometry(40, 10, 20); 
    // Set starting position in Standard Cartesian
    // Starting at East (0° Standard)
    //robotStartingHeadingStandard = 0.0;
    /*
    gyroReadingAtStart = InertialSensor.rotation(degrees);
    
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(10, 20, "Turn Odometry Test");
    Brain.Screen.printAt(10, 40, "Starting: 0 deg Standard (East)");
    Brain.Screen.printAt(10, 60, "Target: 90 deg Standard (North)");
    Brain.Screen.printAt(10, 80, "Expected: 90 deg CCW turn");
    */
       
    // Test 1: Turn 90° CCW (from East to North)
    // Current: 0°, Target: 90°, Error: +90° (should turn CCW in Standard)
    //turnOdometry(0, 20, 24, 24, 1.0);
  //turnRight(180, 40, 25, 25); 
   // wait(2000, msec);
   
    
 /*   
    double finalHeading = getContinuousStandardHeading();
    Brain.Screen.printAt(10, 120, "Final heading: %.2f deg", finalHeading);
    Brain.Screen.printAt(10, 140, "Expected: ~90 deg");
    // driveForward(100, 40, 0);
    //turnLeft(90,20, 25, 25);
    double rawVex = InertialSensor.rotation(degrees);
double relativeVex = rawVex - gyroReadingAtStart;
Brain.Screen.printAt(10, 160, "Raw: %.2f Relative: %.2f", rawVex, relativeVex);
Brain.Screen.printAt(10, 180, "StartStd: %.2f GyroStart: %.2f", robotStartingHeadingStandard, gyroReadingAtStart);
Brain.Screen.printAt(10, 200, "Final: %.2f", getContinuousStandardHeading());


Brain.Screen.clearScreen();
Brain.Screen.printAt(10, 20, "START: H=%.1f Pos=(%.1f,%.1f)", 
                     getContinuousStandardHeading(), globalX, globalY);
wait(2000, msec);
*/
// First straight
//forwardToPoint(0, 30, 10, 15, 1, 0.25, 0, 0, 0.1, 0.1, 0.3, 15);

/*
Brain.Screen.printAt(10, 40, "AFTER FWD: H=%.1f Pos=(%.1f,%.1f)", 
                     getContinuousStandardHeading(), globalX, globalY);
wait(3000, msec);

// Calculate what the turn SHOULD be
double dx = -30 - globalX;
double dy = 30 - globalY;
double calcHeading = atan2(dy, dx) * 180.0 / M_PI;
Brain.Screen.printAt(10, 60, "Target calc: %.1f (atan2)", calcHeading);
Brain.Screen.printAt(10, 80, "Current: %.1f", getContinuousStandardHeading());
wait(3000, msec);

// NOW do the turn
turnLeftToPoint(-30, 30, 10, 22, 22, 0.5);
Brain.Screen.printAt(10, 100, "AFTER TURN: H=%.1f", getContinuousStandardHeading());
Brain.Screen.printAt(10, 120, "Pos=(%.1f,%.1f)", globalX, globalY);

*/



 //wait(2000, msec);    
//straightOdometryV3(50, 20, 90);
   // wait(2000, msec);
    //straightOdometryV3(-50, 20, 90);

   /*     
    double bD=10, minS=15, tol=1, kp=0.75, ki=0.0, kd=0, accS=0.1, decS=.1, appS=0.3, maxS=15;
    double tBrk=10, tMin=22, tMax=22;

    forwardToPoint(0, 30, bD, minS, tol, kp, ki, kd, accS, decS, appS, maxS);
    wait(500, msec);
    turnLeftToPoint(-30, 30, tBrk, tMin, tMax, 0.5);
    wait(500, msec);
    forwardToPoint(-30, 30, bD, minS, tol, kp, ki, kd, accS, decS, appS, maxS);
    wait(500, msec);
    turnLeftToPoint(-30, 0, tBrk, tMin, tMax, 0.5);
    wait(500, msec);
    forwardToPoint(-30, 0, bD, minS, tol, kp, ki, kd, accS, decS, appS, maxS);
    wait(500, msec);
    turnLeftToPoint(0, 0, tBrk, tMin, tMax, 0.5);
    wait(500, msec);
    forwardToPoint(0, 0, bD, minS, tol, kp, ki, kd, accS, decS, appS, maxS);
    wait(500, msec);
    turnLeftToPoint(0, 30, tBrk, tMin, tMax, 0.5);
    
    
*/

    
    //driveForwardV3(30, 10, 0, 15, 1, 0.25, 0.0, 0, 0.1, 0.1, 0.3, 15);
    /*
    wait(500, msec);  
    turnLeft(90, 40, 25, 25);
    wait(500, msec);  
    driveForwardV3(30, 20, 90, 25, 5, 1.1, 0.005, 0, 0.1, 1, 0.3, 25);
    wait(500, msec);  
    turnLeft(180, 40, 25, 25);
    wait(500, msec);  
    driveForwardV3(30, 20, 180, 25, 5, 1.1, 0.005, 0, 0.1, 1, 0.3, 25);
    wait(500, msec);  
    turnLeft(270, 40, 25, 25);
    wait(500, msec);  
    driveForwardV3(30, 20, 270, 25, 5, 1.1, 0.005, 0, 0.1, 1, 0.3, 25);
    wait(500, msec);  
    turnLeft(0, 40, 25, 25);
    wait(500, msec);  
    driveForwardV3(30, 20, 0, 25, 5, 1.1, 0.005, 0, 0.1, 1, 0.3, 25);
    wait(500, msec);  
    */
    /*
    visionDriveMinimal(// not bad for first 3 center balls
        AIVision20__orangeGoal, 
        150,                    
        0.0,                    
        24.0, 75.0,             
        brakeType::hold,       
        .75, 0.0, 0.0, 
        1.5,       
        1.50, 0.0, 0.0        
    );

    */

      /*  visionDriveMinimal(
        AIVision20__orangeGoal, 
        100,                    
        0.0,                    
        24.0, 60.0,             
        brakeType::hold,       
        .6, 0.0, 0.0, 
        10,       
        1.50, 0.0, 0.0        
    );*/
}


/*/void Calibration
{
    initializeOpticalSensor();
    InertialSensor.setRotation(0, degrees);
    robotStartingHeading = 0;
    setStartPosition(0.0, 0.0, 0.0);
    startOdometryTask();
    forwardToPoint(200, 0, 20);

}*/


void odomTest(){
    initializeOpticalSensor();
    startOdometryTask();
    setStartPosition(0.0, 0.0, -90.0);
    forwardToPoint(0, 70, 40);  
    turnRightToPoint(70, 0, 70);
    
    stopOdometryTask();
}

//turnOdometry(turnAmount, breakDistance, minSpeed, maxSpeed)
void autonLeft()
{
    initializeOpticalSensor();
    setStartPosition(0.0, 0.0, 180.0);

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
}

void autonRight(){
    initializeOpticalSensor();
    setStartPosition(0.0, 0.0, 0.0);

    forwardMP(79, 38, 0, 20, 0.815, 0.0, 0.0, 0.0, 0.55, 0.3, 80);
    wait(200, msec);
    rightMP(90,60,12.5);
    wait(200, msec);
    forwardMP(15, 10, 270, 15, 0.815, 0.0, 0.0, 0.0, 0.55, 0.3, 70);
    wait(200, msec);
    backwardMP(20, 15, 270, 10, 0.815, 0.00, 0.00, 0., 0.55, 0.3, 50);
    wait(200, msec);
    rightMP(175,125,20);
    wait(200, msec);
    forwardMP(35, 18, 180, 15, 0.815, 0.0, 0.0, 0.0, 0, 0, 80);
}

void leftSideLong(){
    initializeOpticalSensor();
    InertialSensor.setRotation(0, degrees);
    robotStartingHeading = 16;
    ptoPneumatics.set(false);
    backHoodPneumatics.set(false);
    frontHoodPneumatics.set(false);
    wingPneumatics.set(false);

    wait(50, msec);
    intakeStart(1000, 100, false);
    matchloadStart(6000,100,1050,true);
    driveForward(83, 60, 16);
    wait(250, msec);
    turnLeft(147,118,26,80,16);
    wait(200, msec);
       
    driveForward(102,70,147,24,0.3,0.002,0,0.1,1,0.3,90);
    wait(200, msec);
    turnLeft(175,25,26,80,13);
    wait(200, msec);
    smartStraight(50, 26, 180, 15, 150);
    wait(270, msec);

    driveBackward(26, 14, 176);
    wait(200, msec);
    turnRight(0,145,25,90,16);
    wait(200, msec);
    smartStraight(40, 19, 0, 24, 200);

    score(3200, 100);
    driveBackward(20, 14, -10);
    wait(100, msec);
    turnLeft(40,18,25,90,16);
    
    wait(100, msec);

    driveForward(35,25,40,24,0.3,0.002,0,0.1,1,0.3,90);
    wait(100, msec);

    turnRight(4,30,26,80);
    wait(100, msec);
    driveForward(12,0,4,24,0.3,0.002,0,0.1,1,0.3,90);
    wingPneumatics.set(true);

    driveForward(40,0,4,24,0.3,0.002,0,0.1,1,0.3,40);
}

void leftSidemiddle(){
    initializeOpticalSensor();
    InertialSensor.setRotation(0, degrees);
    robotStartingHeading = 16;
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
    robotStartingHeading = 0;
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
    robotStartingHeading = -12;
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
    robotStartingHeading = 12;
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
    robotStartingHeading = -16;
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
    //setStartPosition(0.0, 25, -90.0);
    //startOdometryTask();
    //startCoordinateFinder();
    //initializeOpticalSensor();    
    ptoPneumatics.set(false);
    backHoodPneumatics.set(false);
    frontHoodPneumatics.set(false);
    wingPneumatics.set(true);

    //forwardToPoint(-47, 0, 70, 15, 1, 0.05, 0, 0, 0.2, 0.2, 0.2, 40);
    driveForwardV3(36,0,-90);
    turnRight(-135,0,24,100,20);
    smartStop(5, 10, 300, false);
    smartStraight(47, 0, -180, 15, 200, 0, 0, 0.05, 0.2, 0.2, 0.2, 40);
    driveBackwardV3(12,0,-180);
    turnRight(0,0,25,100,86);
    smartStop(5, 10, 300, false);
    smartStraight(60, 0, 0, 15, 200, 0, 0, 0.05, 0.2, 0.2, 0.2, 40);
    driveBackwardV3(12,0,0);
    turnLeft(45,0,25,100,25);
    //visionDrive(AIVision20__orangeGoal, 150, 0, 22, 40);
    visionDriveMinimal(
            AIVision20__orangeGoal, 
        130,                    
        0.0,                    
        24.0, 60.0,             
        brakeType::coast,       
        0.6, 0.0, 0.0, 
        10,       
        1.50, 0.0, 0.0);   
    //driveForward(30,0,-360);
    

    /*intakeStart(1000, 100, false);
    matchloadStart(5500,100,1050,true);
    driveForward(85, 60, -16, 30);
    wait(250, msec);
    turnRight(-148,118,26,80,14);
    wait(200, msec);
       
    driveForward(105,76,-148,24, 0.3,0.002,0,0.1,1,0.3,90);
    wait(200, msec);
    turnRight(-176,20,26,80); 
    wait(200, msec);
    smartStraight(47, 40, -180, 15, 200, 0.4, 0.01, 0.05, 0.2, 0.2, 0.2, 40);
    wait(80, msec);

    //smartMove(34, 60, forward, 250); //for matchload smart wall stop, no pid
    driveBackward(26, 14, -176);
    wait(200, msec);
    turnLeft(8,160,25,90,16);
    wait(200, msec);
    smartStraight(50, 20, 8, 24, 150, 0.4, 0.01, 0.05, 0.2, 0.2, 0.2, 150);

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

    driveForwardV2(60,0,6,24,0.3,0.002,0,0.1,1,0.3,30);


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
    //driveBackward(20, 15, -315);*/
}

void colourTest(){
    initializeOpticalSensor();
    InertialSensor.setRotation(0, degrees);
    intakeStart(10000, 50, true);
}

void soloAWPMiddle(){
    initializeOpticalSensor();
    InertialSensor.setRotation(0, degrees);
    robotStartingHeading = -90;
    ptoPneumatics.set(false);
    backHoodPneumatics.set(false);
    frontHoodPneumatics.set(false);
    wingPneumatics.set(true);
    driveForwardV2(60,45,-90,24,3,0.3,0.005,0,0.1,1,0.3,100);

    for (int i = 0; i < 3; i++)
    {
        leftMotor[i].setBrake(hold);
        rightMotor[i].setBrake(hold);
        leftMotor[i].stop();
        rightMotor[i].stop();
    }
    turnRight(-179,65,25,80,28);
}

void skills() {
    initializeOpticalSensor();
    setStartPosition(0.0, 0.0, 0.0);
}

void Inertial_Calib(){
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Calibrating...");
    InertialSensor.calibrate();
    while (InertialSensor.isCalibrating()) { 
        wait(100, msec); 
    }
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("Calibration Complete!");
    wait(500, msec);
    Brain.Screen.clearScreen();
}

void autonSelector(){
    int autonMode = 0;
    const char* autonNames[] = {
        "Auton Left",
        "Auton Right", 
        "Solo AWP",
        "Skills",
        "Odom Test",
        "Vision Test",
        "Auton Test"
    };
    const int numAutons = 7;

    Brain.Screen.clearScreen();
    Brain.Screen.setPenColor(vex::color::white);

    while(true) {
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Selected: %s          ", autonNames[autonMode]);
        
        Brain.Screen.setCursor(3, 1);
        Brain.Screen.print("Press Left/Right to change");
        Brain.Screen.setCursor(4, 1);
        Brain.Screen.print("Press Center to confirm");

        if(Controller.ButtonLeft.pressing()) {
            autonMode = (autonMode - 1 + numAutons) % numAutons;
            wait(200, msec);
        }
        else if(Controller.ButtonRight.pressing()) {
            autonMode = (autonMode + 1) % numAutons;
            wait(200, msec);
        }
        else if(Controller.ButtonA.pressing()) {
            Brain.Screen.clearScreen();
            Brain.Screen.setCursor(1, 1);
            Brain.Screen.print("Running: %s", autonNames[autonMode]);
            wait(500, msec);
            
            switch(autonMode) {
                case 0: autonLeft(); break;
                case 1: autonRight(); break;
                case 2: soloAWP(); break;
                case 3: skills(); break;
                case 4: odomTest(); break;
                case 5: visionSensorTest(); break;
                case 6: autonTest(); break;
            }
            break;
        }
        
        wait(20, msec);
    }
}

/*
void odomTest(){
    initializeOpticalSensor();
    InertialSensor.setRotation(0, degrees);
    robotStartingHeading = 0;

    setStartingPosition(0.0, 0.0, 0.0);
    startOdometryTask();

    forwardToPoint(100, 100, 20, 15, 0.615, 0, 0, 0.1, 0.05, 0.05, 50);
    
}

*/