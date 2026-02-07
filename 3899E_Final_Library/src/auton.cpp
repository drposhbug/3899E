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

extern std::atomic<double> visionHorizontalNormalizedOffset;
extern std::atomic<int>    visionCurrentObjectWidth;
extern std::atomic<bool>   visionTargetTracked;
extern const vex::aivision::colordesc* currentVisionSignature;
extern int currentMinObjectWidth;
extern std::atomic<bool> visionTaskShouldRun;

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



void CoordinateFinderTask(){
    setStartPosition(0, 36, 0.0);
    while(true){
        startCoordinateFinder();
        vex::task::sleep(500);
    }
}

void testVisionOnly() {
    currentVisionSignature = &AIVision20__orangeGoal;
    currentMinObjectWidth = 10;
    visionTargetTracked = false;
    visionTaskShouldRun = true;
    
    vex::task(visionTrackingTask);
    vex::wait(50, vex::msec);
    
    Brain.Screen.clearScreen();
    
    while (!Brain.Screen.pressing()) {
        Brain.Screen.clearScreen();  // Full clear every loop
        
        Brain.Screen.printAt(10, 20, true, "=== VISION TEST ===");
        Brain.Screen.printAt(10, 40, true, "Press to exit");
        Brain.Screen.printAt(10, 60, true, "");
        Brain.Screen.printAt(10, 80, true, "Tracked: %s", visionTargetTracked.load() ? "**YES**" : "NO");
        Brain.Screen.printAt(10, 100, true, "Width: %d px", visionCurrentObjectWidth.load());
        Brain.Screen.printAt(10, 120, true, "Offset: %.3f", visionHorizontalNormalizedOffset.load());
        Brain.Screen.printAt(10, 140, true, "");
        Brain.Screen.printAt(10, 160, true, "Raw Count: %d", AIVision20.objectCount);
        Brain.Screen.printAt(10, 180, true, "Min Filter: %d", currentMinObjectWidth);
        
        vex::wait(200, vex::msec);  // Slower update so you can read
    }
    
    visionTaskShouldRun = false;
    vex::wait(50, vex::msec);
    Brain.Screen.clearScreen();
}
void testVisionDirect() {
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(10, 20, true, "Direct Vision Test");
    Brain.Screen.printAt(10, 40, true, "Press screen to exit");
       
    while (!Brain.Screen.pressing()) {
        // Take snapshot directly (no task)
        AIVision20.takeSnapshot(AIVision20__orangeGoal);
        
        Brain.Screen.clearLine(3);
        Brain.Screen.clearLine(4);
        Brain.Screen.clearLine(5);
        Brain.Screen.clearLine(6);
        
        Brain.Screen.printAt(10, 60, true, "Object Count: %d", AIVision20.objectCount);
        
        if (AIVision20.objectCount > 0) {
            Brain.Screen.printAt(10, 80, true, "X: %d  Y: %d", 
                AIVision20.objects[0].centerX, 
                AIVision20.objects[0].centerY);
            Brain.Screen.printAt(10, 100, true, "Width: %d  Height: %d",
                AIVision20.objects[0].width,
                AIVision20.objects[0].height);
        } else {
            Brain.Screen.printAt(10, 80, true, "NO OBJECTS DETECTED");
        }
        
        vex::wait(100, vex::msec);
    }
    
    Brain.Screen.clearScreen();
}

void testVisionBasic() {
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(10, 20, true, "BASIC VISION TEST");
    Brain.Screen.printAt(10, 40, true, "Press to exit");
    
    while (!Brain.Screen.pressing()) {
        AIVision20.takeSnapshot(AIVision20__orangeGoal);
        
        Brain.Screen.clearLine(4);
        Brain.Screen.clearLine(5);
        Brain.Screen.clearLine(6);
        
        Brain.Screen.printAt(10, 80, true, "Count: %d", AIVision20.objectCount);
        
        if (AIVision20.objectCount > 0) {
            Brain.Screen.printAt(10, 100, true, "Width: %d", AIVision20.objects[0].width);
            Brain.Screen.printAt(10, 120, true, "X: %d Y: %d", 
                AIVision20.objects[0].centerX,
                AIVision20.objects[0].centerY);
        }
        
        vex::wait(100, vex::msec);
    }
}

void autonTest() {
    // Initialize sensor
    //initializeOpticalSensor();
    setStartPosition(0, 0, 0);
    
    /*
    visionDriveMinimal(
        AIVision20__orangeGoal, 
        110,                    
        0.0,                    
        20.0, 80.0,             
        brakeType::hold,       
        .5, 0.0, 0.0, 
        0.5,       
        1.50, 0.0, 0.0    
     ); 
    */
/*
    visionDriveV2(
    AIVision20__orangeGoal, 
    nullptr,              // aiObjectSignature (not used for color signatures)
    120,                  // targetPixelWidth
    0.0,                  // targetHeading (fallback)
    20.0,                 // minSpeedPct
    75.0,                 // maxSpeedPct (Reduced slightly for voltage headroom)
    0.0,                  // timeoutDistanceCM (optional logic)
    0.5, 0.0, 0.25,       // Heading PID (Kp, Ki, Kd) - Scaled for normalized error
    0.2,                 // kp_distToHeadScaling - Lowered to prevent twitchiness
    1.2, 0.0, 0.15,       // Distance PID (Kp, Ki, Kd) - Added Kd for smooth arrival
    0, 320,               // minX, maxX (Full camera width)
    0, 240,               // minY, maxY (Full camera height)
    brakeType::hold,      // brakeMode
    25                    // minObjectWidth (filters noise)
);
*/

    //startOdometryTask();
 
    //moveOdometry(41, 82, 10, 20, 2, 2.0, 0.0, 0.0, brakeType::brake, 1.0, 1.0, 1.0, 100);
    //moveOdometry(-18, 87.9, 10, 20, 2, 2, 0.0, 0.0, brakeType::brake, 0.05, 0.05, 0.05, 100); 



    //testVisionOnly();
    //testVisionDirect(); 
   // testVisionBasic();

      
    //forwardToPoint(0, -75, 20, 16, 2.0, 2, 0.0, 0.0, 0.1, 0.1, 0.1, 60); 
    //startCoordinateFinder();

    // moveOdometry(targetX, targetY, breakDist, minSpeed, tolerance, kP, kI, kD, brakeType, accelScale, decelScale, approachScale, maxSpeed)

// Example: Driving 60cm forward while centering the Orange Goal
moveVisionOdometry(
    40.0,                   // targetX (cm)
    0.0,                    // targetY (cm)
    25.0,                   // breakDistance (cm)
    15.0,                   // minSpeed (%)
    2.0,                    // distanceTolerance (cm)
    0.45, 0.0, 0.15,        // Heading PID (kp, ki, kd)
    vex::brakeType::hold,   // brakeMode
    1.2, 0.8, 0.5,          // Accel/Decel/Approach Heading Scaling
    40.0,                   // maxSpeed (%)
    AIVision20__orangeGoal, // targetSignature
    0.4,                   // kp_headingFusionWeight (45% Vision / 55% Odo)
    25                      // minObjectWidth (pixels)
);

/*
moveOdometry(
    80.0,              // targetX (North/South distance in cm)
    35.0,              // targetY (East/West distance in cm)
    25.0,              // breakDistance (Start slowing down 25cm from target)
    15.0,              // minSpeed (15% power for final approach)
    2.5,               // distanceTolerance (Stop when within 2.5cm of target)
    0.15,              // kp_heading (Proportional gain for staying on path)
    0.0,               // ki_heading (Integral gain)
    0.01,              // kd_heading (Derivative gain to prevent oscillation)
    vex::brakeType::coast, // brakeMode (Lock motors after stopping)
    0.5,               // accelHeadingScaling (Half correction strength during launch)
    1.0,               // decelHeadingScaling (Full correction strength during braking)
    0.3,               // approachHeadingScaling (Gentle correction during creep)
    45.0               // maxSpeed (85% power cruise speed)
);
*/

/*
(80, 35)
(85, 17)
(91,75)
*/
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
    forwardToPoint(200, 0, 
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
    //rightMotor[0].spin(forward,12,vex::voltageUnits::volt);
    //rightMotor[1].spin(forward,12,vex::voltageUnits::volt);
    //rightMotor[2].spin(forward,12,vex::voltageUnits::volt);
    //leftMotor[0].spin(forward,12,vex::voltageUnits::volt);
    //leftMotor[1].spin(forward,12,vex::voltageUnits::volt);
    //leftMotor[2].spin(forward,12,vex::voltageUnits::volt);
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

void skillsAuton(){
    setStartPosition(0.0, 0, -90.0);
    //startOdometryTask();
    startCoordinateFinder();
    initializeOpticalSensor();    
    ptoPneumatics.set(false);
    backHoodPneumatics.set(false);
    frontHoodPneumatics.set(false);
    wingPneumatics.set(true);

    /*
    //Intake from match load
    driveForwardV3(35,0,-90);
    turnRight(-135,0,24,100,22);
    smartStop(5, 10, 300, false);
    smartStraight(47, 0, -180, 15, 200, 0, 0, 0.05, 0.2, 0.2, 0.2, 40);
    

    //Bottom Middle Right 4 Cubes
    //setStartPosition(0.0, 0, -180.0); //******TEMP COMMENT OUT********
    driveBackwardV3(13.5,0,-180);
    turnRight(45,0,25,100,70);
    wait(350, msec);

    visionDriveV2(
        AIVision20__redCube,           // Custom color signature
        &AIVision20__redBlock,         // AI object fallback
        60,                            // Target pixel width
        45,                           // Target heading
        24.0, 75.0,                    // Min and max speed
        100.0,                         // Timeout distance (cm)
        0.1, 0.0, 0.0,                 // Heading PID (kp, ki, kd)
        0.3,                           // Distance-to-heading scaling
        1.5, 0.0, 0.0,                // Distance PID (kp, ki, kd)
        0, 320,                     // Bounding box X (minX, maxX)
        0, 240,                     // Bounding box Y (minY, maxY)
        vex::brakeType::hold,          // Brake mode
        10                             // Min object width
    );   
       
    matchLoadPneumatics.set(true);
    driveForwardV3(50,20,45,24,5,0.5,0,0.1,10.1,0.3,50);
    */
    

    //Top Middle Right 4 Cubes
    setStartPosition(0.0, 0, 45); //******TEMP COMMENT OUT********/
    turnRight(-45,0,25,100,70);

        visionDriveV2(
        AIVision20__blueCube,           // Custom color signature
        &AIVision20__blueBlock,         // AI object fallback
        60,                            // Target pixel width
        45,                           // Target heading
        24.0, 75.0,                    // Min and max speed
        100.0,                         // Timeout distance (cm)
        0.1, 0.0, 0.0,                 // Heading PID (kp, ki, kd)
        0.3,                           // Distance-to-heading scaling
        1.5, 0.0, 0.0,                // Distance PID (kp, ki, kd)
        0, 320,                     // Bounding box X (minX, maxX)
        0, 240,                     // Bounding box Y (minY, maxY)
        vex::brakeType::hold,          // Brake mode
        10                             // Min object width
    );   
       

     /*   
    visionDriveMinimal(
        AIVision20__redCube, 
        60,                    
        0.0,                    
        24.0, 75.0,             
        brakeType::hold,       
        .1, 0.0, 0.0, 
        0.3,       
        1.50, 0.0, 0.0    
     ); 
*/
//wait(15000, msec);


}

void soloAWP(){
    //setStartPosition(0.0, 0, -90.0);
    //startOdometryTask();
    //startCoordinateFinder();
    InertialSensor.setRotation(0, degrees);
    robotStartingHeading = -90;
    initializeOpticalSensor();
        
    ptoPneumatics.set(false);
    backHoodPneumatics.set(false);
    frontHoodPneumatics.set(false);
    wingPneumatics.set(true);
    

    intakeStart(1000, 100, false);
    matchloadStart(5500,100,1050,true);
    turnRight(-16,60,25,90,16);
    wait(200, msec);
    driveForward(85, 60, -16, 30);
    wait(250, msec);
    turnRight(-148,118,26,80,14);
    wait(200, msec);
       
    /*driveForward(105,76,-148,24, 0.3,0.002,0,0.1,1,0.3,90);
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

    driveForwardV2(60,0,6,24,0.3,0.002,0,0.1,1,0.3,30);*/


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

void nothing(){
    initializeOpticalSensor();
    InertialSensor.setRotation(0, degrees);
    robotStartingHeading = 0;

    intakeStart(1000, 50, false);
    //driveForward(15, 9, 0, 15, 0.615, 0, 0, 0.1, 0.05, 0.05, 50);
}

void soloAwp2(){
    setStartPosition(0.0, 0, -90.0);
    //startOdometryTask();
    //startCoordinateFinder();
    //InertialSensor.setRotation(0, degrees);
    //robotStartingHeading = -90;
    initializeOpticalSensor();

    //forwardToPoint(-47, 0, 70, 15, 1, 0.05, 0, 0, 0.2, 0.2, 0.2, 40);
    matchLoadPneumatics.set(true);
    wait(200, msec);
    driveForwardV3(44,25,-90);
    turnRight(-142,5,2,85,22);
    smartStop(5, 10, 300, false);
    matchloadStart(2700,100,0,true);
    //smartStraight(47, 0, -180, 15, 220, 0, 0, 0.05, 0.2, 0.2, 0.2, 40);
    visionDriveMinimal(
        AIVision20__redCube, 
        70,                    
        0.0,                    
        24.0, 40.0,             
        brakeType::hold,       
        .06, 0.0, 0.0, 
        0.3,       
        1.50, 0.0, 0.0    
    );
   smartStraight(47, 40, -180, 15, 200, 0.4, 0.01, 0.05, 0.2, 0.2, 0.2, 40);
    wait(200, msec);
    driveBackward(26, 14, -180,24,1.1,0,0,0.1,0.2,0.3,70);
    //driveBackwardV3(12,0,-180);
        wait(400, msec);

    turnLeft(-4,0,25,85,80);
  //  smartStop(5, 10, 300, false);
  wait(400, msec);
  smartStraight(50, 48, -4, 24, 150, 0.05, 0, 0., 0.2, 0.2, 0.2, 40);
    score(1000, 100);
  //  driveBackward(10, -4, -180,24,1.1,0,0,0.1,0.2,0.3,70);
    

  ///  turnLeft(-330,0,25,100,20);
    //wait(200, msec);
  //  visionDriveMinimal(// not bad for first 3 center balls
   //     AIVision20__redCube, 
     //   60,                    
   //     0.0,                    
     //   24.0, 75.0,             
   //     brakeType::hold,       
    //    .1, 0.0, 0.0, 
    //    1.75,       
    //    1.50, 0.0, 0.0        
//    );
    
  //  driveForwardV3(45,15,-330);
//
  /*visionDriveMinimal(
        AIVision20__orangeGoal, 
        140,                    
        0.0,                    
        24.0, 60.0,             
        brakeType::coast,       
        0.6, 0.0, 0.0, 
        10,       
        1.50, 0.0, 0.0);*/    
    //score(3200, 100);
    //intakeStart(3000, 100, true);
    /*wait (3000, msec);
    driveBackwardV3(22,0,0);
    turnLeft(-330,0,25,100,20);
    wait(200, msec);*/
    /*visionDriveMinimal(// not bad for first 3 center balls
        AIVision20__blueCube, 
        100,                    
        0.0,                    
        24.0, 75.0,             
        brakeType::hold,       
        .1, 0.0, 0.0, 
        1.75,       
        1.50, 0.0, 0.0        
    );*/
    //forwardToPoint(45, 0, 20, 15, 1, 0.05, 0, 0, 0.2, 0.2, 0.2, 40);
    //driveForwardV3(100,50,42);
    //visionDrive(AIVision20__orangeGoal, 150, 0, 22, 40);
    /*visionDriveMinimal(
            AIVision20__orangeGoal, 
        130,                    
        0.0,                    
        24.0, 60.0,             
        brakeType::coast,       
        0.6, 0.0, 0.0, 
        10,       
        1.50, 0.0, 0.0);   
    //driveForward(30,0,-360);*/
}

void runEverything(){
    initializeOpticalSensor();
    InertialSensor.setRotation(0, degrees);

    driveForwardV3(20,10,0);

    driveBackwardV3(20,10,0);

    intakeStart(1000, 50, true);

    matchloadStart(1000, 50, 0, true);

    score(1000, 50);

    outtake(1000, 50);

    wingPneumatics.set(true);
}

void skillsAutonGateway(){
    setStartPosition(0.0, 0, -90.0);
    //startOdometryTask();
    //startCoordinateFinder();
    //InertialSensor.setRotation(0, degrees);
    //robotStartingHeading = -90;
    initializeOpticalSensor();

    //forwardToPoint(-47, 0, 70, 15, 1, 0.05, 0, 0, 0.2, 0.2, 0.2, 40);
    matchLoadPneumatics.set(true);
    wait(200, msec);
    driveForwardV3(44,25,-90);
    turnRight(-142,5,2,85,22);
    smartStop(5, 10, 300, false);
    matchloadStart(2700,100,0,true);
    //smartStraight(47, 0, -180, 15, 220, 0, 0, 0.05, 0.2, 0.2, 0.2, 40);
    visionDriveMinimal(
        AIVision20__redCube, 
        70,                    
        0.0,                    
        24.0, 40.0,             
        brakeType::hold,       
        .06, 0.0, 0.0, 
        0.3,       
        1.50, 0.0, 0.0    
    );
   smartStraight(47, 40, -180, 15, 200, 0.4, 0.01, 0.05, 0.2, 0.2, 0.2, 40);
    wait(2000, msec);
    driveBackward(26, 14, -180,24,1.1,0,0,0.1,0.2,0.3,70);
    //driveBackwardV3(12,0,-180);
        wait(400, msec);

    turnLeft(-4,0,25,85,80);
  //  smartStop(5, 10, 300, false);
  wait(400, msec);
  smartStraight(50, 48, -4, 24, 150, 0.6, 0.01, 0.05, 0.2, 0.2, 0.2, 50);
    score(320000, 100);
    driveBackward(5, -4, -180,24,1.1,0,0,0.1,0.2,0.3,70);
    

    turnLeft(-330,0,25,100,20);
    wait(200, msec);
    visionDriveMinimal(// not bad for first 3 center balls
        AIVision20__blueCube, 
        100,                    
        0.0,                    
        24.0, 75.0,             
        brakeType::hold,       
        .1, 0.0, 0.0, 
        1.75,       
        1.50, 0.0, 0.0        
    );
        driveForwardV3(44,25,-330);
}

void soloAwpOdom(){
    setStartPosition(0, 0, 0);
    startOdometryTask();
    startCoordinateFinder();
    initializeOpticalSensor();

    
}