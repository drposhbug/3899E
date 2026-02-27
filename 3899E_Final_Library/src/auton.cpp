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
    setStartPosition(0, 36.2, 0.0);
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

    //Good for fast 100 power vision
/*
moveVisionOdometry(
    AIVision20__redCube,  // targetSignature
    60,                   // targetPixelWidth — stop when object is this wide in pixels
    40.0,                 // targetX (cm)
    116,                   // targetY (cm)
    70.0,                 // breakDistance — begin decel this many cm from target
    brakeType::hold,      // brakeMode
    100.0,                // maxSpeed (%)
    0.43,                  // kp_head
    0.0,                  // ki_head
    0.04,                  // kd_head
    1.05,                    // kp_distToHeadScaling — 1.0 = full vision correction immediately, flat approach
    10,                   // minObjectWidth — ignore detections narrower than this
    0,                    // minX — left bound of valid detection region (pixels)
    320,                  // maxX — right bound of valid detection region (pixels)
    0,                    // minY — top bound of valid detection region (pixels)
    240,                  // maxY — bottom bound of valid detection region (pixels)
    16.0,                 // minSpeed (%)
    1.0,                  // distanceTolerance — odometry fallback stopping bubble (cm)
    0.22,                  // accelHeadingScaling
    0.2,                  // decelHeadingScaling
    0.25,                  // approachHeadingScaling
    15,                     // headingLockDistance 
    5.0                   // timeout (seconds)
);*/
    
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

void systemTest(){ //last chance to look at me hector ding ding ding
    initializeOpticalSensor();
    InertialSensor.setRotation(0, degrees);

    vex::motor* allMotors[] = {
        &LeftMotor1, &LeftMotor2, &LeftMotor3,
        &RightMotor1, &RightMotor2, &RightMotor3,
        &intakeMotor1, &intakeMotor2
    };

    const int numMotors = sizeof(allMotors) / sizeof(allMotors[0]);

    for (int i = 0; i < numMotors; ++i) {
        vex::motor* m = allMotors[i];
        m->spin(forward, 100, percent);

        vex::task::sleep(500);

        if (m->isSpinning()) {
            //brain will ding
            Controller.rumble(".");
        }
        vex::task::sleep(500);

        m->stop();
    }

    wingPneumatics.set(true);
    wait(500, msec);
    wingPneumatics.set(false);
    wait(500, msec);
    matchLoadPneumatics.set(true);
    wait(500, msec);
    matchLoadPneumatics.set(false);
    wait(500, msec);
    frontHoodPneumatics.set(true);
    wait(500, msec);
    frontHoodPneumatics.set(false);
    wait(500, msec);
    leftGatePneumatics.set(true);
    wait(500, msec);
    leftGatePneumatics.set(false);
    wait(500, msec);
    rightGatePneumatics.set(true);
    wait(500, msec);
    rightGatePneumatics.set(false);

    //optical sensor test
    bool lNear = leftLaneOptical.isNearObject();
    bool rNear = rightLaneOptical.isNearObject();
    if (lNear) {
        Controller.rumble(".");
    }
    vex::task::sleep(500);
    if (rNear) {
        Controller.rumble(".");
    }
    vex::task::sleep(500);
}


void skillsAuton(){
    //Initialize and set starting position in Standard Cartesian
    // Starting at East (0° Standard)
    setStartPosition(0, 36, 0);
    startOdometryTask();
    leftGatePneumatics.set(true);   
    rightGatePneumatics.set(true);   
   // startCoordinateFinder();
    initializeOpticalSensor(); 

    //Testing Area
    /*
    wait(15000, msec);
    */   
    //End of Testing Area
   
    //*********************************************************
    //Stop 1: South East MatchLoader (6 Blocks + 1 Preload)
    //********************************************************* */
    driveForwardV3(36,0,0);
    turnRight(-39,20);
    //wait(10000, msec);
    smartStop(5, 5, 200, false);
    //intakeHopperStart(3000, 100, 0, true);
    //intakeStart(3000, 100, true);
    //matchloadStart(10500,100,0,true);
    intakeHopperStart(10500, 100, 0, true);
    matchloadPneumaticStart(10500, 0, true);
    //matchLoadPneumatics.set(true);
/* //Maybe close to roginal, kp was probably lower and may the 0.3 too
   visionDriveMinimal(
        AIVision20__redCube, 
        100,                    
        0.0,                    
        24.0, 85.0,             
        brakeType::coast,       
        .6, 0.0, 0.0, 
        0.3,       
        1.0, 0.0, 0.0
    );
*/
    visionDriveMinimal(
        AIVision20__redCube, 
        100,                    
        0.0,                    
        24.0, 85.0,             
        brakeType::coast,       
        1.5, 0.0, 0.0, 
        1.2,       
        1.0, 0.0, 0.0
    );
    smartStraight(47, 0, -90, 15, 100, 0.4, 0.0, 0.0, 0.2, 0.2, 0.2, 50);
    wait(900, msec);

    //********************************************************* 
    //Stop 2: South West Middle (4 Blocks)
    //********************************************************* 
    //Driving out
    driveBackwardV3(4, 0, -90,24,1,1.1,0,0,0.1,0.2,0.3,100); 
    //matchLoadPneumatics.set(false);   
    matchloadPneumaticStop();
    turnRight(-185,40);
   //wait(30000, msec);
    //driveForwardV3(36,0,-225);
    smartStop(5, 5, 400, false);
    //matchloadPneumaticStart(2000, 350, true);


   // wait(500, msec);



    // Going to get the 4 blocks

/*    
visionDriveMinimal(
    AIVision20__redCube,  // targetSignature — red cube color descriptor
    70,                   // targetPixelWidth — stop when cube is 70px wide
    0.0,                  // targetHeading — no fixed heading, vision steers freely
    24.0, 40.0,           // minSpeedPct, maxSpeedPct — speed range (%)
    brakeType::hold,     // brakeMode — coast to stop on exit
    0.06, 0.0, 0.0,       // kp_head, ki_head, kd_head — gentle heading correction
    0.3,                  // kp_distToHeadScaling — moderate distance-based steering weight
    1.50, 0.0, 0.0        // kp_dist, ki_dist, kd_dist — aggressive approach, no integral/derivative
);
*/


/*
moveOdometry(
    48.1,             // targetX (cm)
    67.4,             // targetY (cm)
    30.0,             // breakDistance
    25.0,             // minSpeed (%)
    1.0,              // distanceTolerance (cm)
    0.43,             // kp_heading
    0.0,              // ki_heading
    0.04,             // kd_heading
    brakeType::hold,  // brakeMode
    0.22,             // accelHeadingScaling
    0.2,              // decelHeadingScaling
    0.25,             // approachHeadingScaling
    60.0,             // maxSpeed (%)
    15.0,              // headingLockDistance (cm) — this is what we're testing
    5.0               // timeout (seconds)
);
*/
   //wait(30000, msec);


    moveVisionOdometry(
        AIVision20__redCube,  // targetSignature
        60,                   // targetPixelWidth — stop when object is this wide in pixels
        48.1,                 // targetX (cm)
        67.4,                 // targetY (cm)
        30.0,                 // breakDistance — 48 worked well before 
        brakeType::coast,      // brakeMode
        60.0,                 // maxSpeed (%)
        0.43,                 // kp_head
        0.0,                  // ki_head
        0.04,                 // kd_head
        1.05,                 // kp_distToHeadScaling — 1.0 = full vision correction immediately, flat approach
        10,                   // minObjectWidth — ignore detections narrower than this
        50,                   // minX — left bound of valid detection region (pixels)
        260,                  // maxX — right bound of valid detection region (pixels)
        0,                    // minY — top bound of valid detection region (pixels)
        240,                  // maxY — bottom bound of valid detection region (pixels)
        25.0,                 // minSpeed (%)
        1.0,                  // distanceTolerance — odometry fallback stopping bubble (cm)
        0.22,                 // accelHeadingScaling
        0.2,                  // decelHeadingScaling
        0.25,                 // approachHeadingScaling
        15,                   // headingLockDistance (cm)
        5.0                   // timeout (seconds)
    );
    matchloadPneumaticStart(2500, 425, true);
    //wait(30000, msec);
    //moveOdometry(26.88, 125.99, 0, 20, 2, 0.75, 0, 0, vex::brakeType::hold, 0.2, 0.2, 0.2, 60, 15, 5000);
    moveOdometry(
    30.5,             // targetX (cm)
    125.99,             // targetY (cm)
    0.0,             // breakDistance
    25.0,             // minSpeed (%)
    2.0,              // distanceTolerance (cm)
    0.75,             // kp_heading // .75 before
    0.0,              // ki_heading
    0.04,             // kd_heading
    brakeType::hold,  // brakeMode
    0.22,             // accelHeadingScaling
    0.2,              // decelHeadingScaling
    0.25,             // approachHeadingScaling
    60.0,             // maxSpeed (%)
    20.0,              // headingLockDistance (cm) — this is what we're testing
    5.0               // timeout (seconds)
);
   // wait(30000, msec);
    //matchloadPistonStart(2500, 400);
    //intakeHopperStart(2000, 100, 0, true);  
    //driveForwardV3(40, 0, -220, 60, 2, 2, 0, 0, 0.2, 0.2, 0.2, 60); //kp good at 2
   //// driveForwardV3(30, 0, -225, 60, 2, 3, 0, 0, 0.2, 0.2, 0.2, 60); //kp good at 2
    //wait(1000, msec);
    smartStop(5, 5, 300, false);

    //*****************************************************
    //Stop 3: North West (2 Blocks)
    //***************************************************** 
    turnRight(45,20);
    wingPneumatics.set(true);  
    
/*
for (int i = 0; i < 3; i++)
    {
        leftMotor[i].setBrake(brakeType::coast);
        rightMotor[i].setBrake(brakeType::coast);
    }

*/
    
    //wait(30000, msec);
    //turnRight(80,20); //Tyler & Justin's
     smartStop(5, 5, 200, false);
    //matchloadPistonStop();
     matchloadPneumaticStop();
    moveOdometry(
    59.32,             // targetX (cm)
    153.35,             // targetY (cm)
    30,             // breakDistance
    25.0,             // minSpeed (%)
    2.0,              // distanceTolerance (cm)
    0.85,             // kp_heading
    0.0,              // ki_heading
    0.04,             // kd_heading
    brakeType::brake,  // brakeMode
    0.22,             // accelHeadingScaling
    0.2,              // decelHeadingScaling
    0.25,             // approachHeadingScaling
    60.0,             // maxSpeed (%)
    20.0,              // headingLockDistance (cm) — this is what we're testing
    5.0               // timeout (seconds)
);
   matchloadPneumaticStart(2500, 0, true);

/*
    driveForwardV3(10, 0, 43, 25, 2, 0.4, 0, 0, 0.2, 0.2, 0.2, 70);
    //moveOdometry(30, 140, 0, 20, 2, 0.4, 0, 0, vex::brakeType::hold, 0.2, 0.2, 0.2, 70, 0, 5000);
    //driveToWall(20, 0, 15, 100, 10, vex::brakeType::hold, 3000, 40);
    //wait(100000,msec);
    //driveForwardV3(47, 30, 80, 50, 2, 0.4, 0, 0, 0.2, 0.2, 0.2, 50);
    //wait(300, msec);
    //turnRight(0,40, 10, 40, 0);
    //driveForwardV3(6, 0, 0, 5, 0, 0.4, 0, 0, 0.2, 0.2, 0.2, 20);
    
    //matchloadPistonStart(3000, 100);
    matchloadPneumaticStart(3000, 200, true);
    wait(100000,msec);
    moveVisionOdometry(
    AIVision20__blueCube,  // targetSignature
    30,                   // targetPixelWidth — stop when object is this wide in pixels
    48.1,                 // targetX (cm)
    161,                   // targetY (cm)
    40.0,                 // breakDistance — begin decel this many cm from target
    brakeType::hold,      // brakeMode
    60.0,                // maxSpeed (%)
    0.43,                  // kp_head
    0.0,                  // ki_head
    0.04,                  // kd_head
    1.05,                    // kp_distToHeadScaling — 1.0 = full vision correction immediately, flat approach
    10,                   // minObjectWidth — ignore detections narrower than this
    0,                    // minX — left bound of valid detection region (pixels)
    320,                  // maxX — right bound of valid detection region (pixels)
    0,                    // minY — top bound of valid detection region (pixels)
    240,                  // maxY — bottom bound of valid detection region (pixels)
    25.0,                 // minSpeed (%)
    1.0,                  // distanceTolerance — odometry fallback stopping bubble (cm)
    0.22,                  // accelHeadingScaling
    0.2,                  // decelHeadingScaling
    0.25,                  // approachHeadingScaling
      15,                     // headingLockDistance 
    5.0                   // timeout (seconds)
);

//driveBackwardV3(10, 0, 32, 50, 2, 2, 0, 0, 0.2, 0.2, 0.2, 50);


    /*moveVisionOdometry(
    AIVision20__blueCube,
    70,
    41,
    80.0,
    25.0,
    brakeType::coast,
    40.0,
    0.15,                      // kp_head — matching forwardToPoint default
    0.0,                      // ki_head
    0.5,                      // kd_head — forwardToPoint default
    0.9,
    10,
    0,
    320,
    150,
    240,
    16.0,
    2.0,
    0.1,                      // accelHeadingScaling — matching forwardToPoint default
    0.1,                      // decelHeadingScaling — matching forwardToPoint default
    0.05,                      // approachHeadingScaling — matching forwardToPoint default
      15,                     // headingLockDistance 
    5.0
);*/

    //forwardToPoint(15, 135, 70, 15, 1, 0.05, 0, 0, 0.2, 0.2, 0.2, 40);
/*
    moveOdometry(
    20.0,              // targetX (North/South distance in cm)
    135.0,              // targetY (East/West distance in cm)
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

    forwardToPoint(-47, 0, 70, 15, 1, 0.05, 0, 0, 0.2, 0.2, 0.2, 40);
    //driveForwardV3(36,0,-90);
    turnRight(-135,0,24,100,20);
    smartStop(5, 10, 300, false);
    smartStraight(47, 0, -180, 15, 200, 0, 0, 0.05, 0.2, 0.2, 0.2, 40);
    

    //Bottom Middle Right 4 Cubes
    //setStartPosition(0.0, 0, -180.0); //******TEMP COMMENT OUT********
    driveBackwardV3(13.5,0,-180);
    turnRight(45,0,25,100,70);
    wait(350, msec);

    visionDriveV2(
        AIVision20__redCube,           // 1. targetSignature
        60,                            // 2. targetPixelWidth
        45,                            // 3. targetHeading
        vex::brakeType::hold,          // 4. brakeMode
        75.0,                          // 5. maxSpeedPct
        0.1,                           // 6. kp_head
        0.0,                           // 7. ki_head
        0.0,                           // 8. kd_head
        0.3,                           // 9. kp_distToHeadScaling
        10,                            // 10. minObjectWidth
        0,                             // 11. minX
        320,                           // 12. maxX
        0,                             // 13. minY
        240,                           // 14. maxY
        24.0,                          // 15. minSpeedPct
        100.0,                         // 16. timeoutDistanceCM
        1.5,                           // 17. kp_dist
        0.0,                           // 18. ki_dist
        0.0                            // 19. kd_dist
    );
       
    matchLoadPneumatics.set(true);
    driveForwardV3(50,20,45,24,5,0.5,0,0.1,10.1,0.3,50);
    
    

    //Top Middle Right 4 Cubes
    setStartPosition(0.0, 0, 45); //******TEMP COMMENT OUT********/
    turnRight(-45,0,25,100,70);

       visionDriveV2(
        AIVision20__blueCube,          // 1. targetSignature
        60,                            // 2. targetPixelWidth
        45,                            // 3. targetHeading
        vex::brakeType::hold,          // 4. brakeMode
        75.0,                          // 5. maxSpeedPct
        0.1,                           // 6. kp_head
        0.0,                           // 7. ki_head
        0.0,                           // 8. kd_head
        0.3,                           // 9. kp_distToHeadScaling
        10,                            // 10. minObjectWidth
        0,                             // 11. minX
        320,                           // 12. maxX
        0,                             // 13. minY
        240,                           // 14. maxY
        24.0,                          // 15. minSpeedPct
        100.0,                         // 16. timeoutDistanceCM
        1.5,                           // 17. kp_dist
        0.0,                           // 18. ki_dist
        0.0                            // 19. kd_dist
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

/*
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
    //driveBackward(20, 15, -315);
    
}
*/

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

    intakeStart(20000, 50, true);
    //matchloadStart(20000, 100, 30000, true);
    pidlessForward(1000, 60);
    //driveForward(15, 9, 0, 15, 0.615, 0, 0, 0.1, 0.05, 0.05, 50);
}

void soloAwp2(){
    setStartPosition(0, 36, 0);
    startOdometryTask();
    //startCoordinateFinder();
    //InertialSensor.setRotation(0, degrees);
    //robotStartingHeading = -90;
    initializeOpticalSensor();

   // intakeHopperStart(2000, 100, 500, true); // 500ms delay, then runs intake, non-blocking
   
//intakeStart2(2000, 100, true, false);
/*
moveOdometry(
    70.0,                     // targetX
    36,                       // targetY
    12.0,                     // breakDistance
    24.0,                     // minSpeed
    2.0,                      // distanceTolerance
    0.15,                     // kp_heading
    0.0,                      // ki_heading
    0.2,                      // kd_heading
    brakeType::hold,          // brakeMode
    0.5,                      // accelHeadingScaling
    0.5,                      // decelHeadingScaling
    0.5,                      // approachHeadingScaling
    40.0,                     // maxSpeed
    8.0,                      // headingLockDistance
    5.0                       // timeout
);
*/
/*    visionDriveMinimal(
        AIVision20__redCube, 
        70,                    
        0.0,                    
        24.0, 40.0,             
        brakeType::hold,       
        .06, 0.0, 0.0, 
        0.3,       
        1.50, 0.0, 0.0    
    );
  
  wait(10000, msec);
*/
//rally good vision following
/*
moveVisionOdometry(
    AIVision20__redCube,
    60,
    70.0,
    36,
    12.0,
    brakeType::hold,
    40.0,
    0.15,                      // kp_head — matching forwardToPoint default
    0.0,                      // ki_head
    0.2,                      // kd_head — forwardToPoint default
    1,
    10,
    0,
    320,
    0,
    240,
    24.0,
    2.0,
    1.2,                      // accelHeadingScaling — matching forwardToPoint default
    0.5,                      // decelHeadingScaling — matching forwardToPoint default
    1.2,                      // approachHeadingScaling — matching forwardToPoint default
      15,                     // headingLockDistance 
    5.0
);
*/


moveVisionOdometry(
    AIVision20__redCube,  // targetSignature
    60,                   // targetPixelWidth — stop when object is this wide in pixels
    110.0,                 // targetX (cm)
    36,                   // targetY (cm)
    70.0,                 // breakDistance — begin decel this many cm from target
    brakeType::hold,      // brakeMode
    100.0,                // maxSpeed (%)
    0.43,                  // kp_head
    0.0,                  // ki_head
    0.04,                  // kd_head
    1.05,                    // kp_distToHeadScaling — 1.0 = full vision correction immediately, flat approach
    10,                   // minObjectWidth — ignore detections narrower than this
    0,                    // minX — left bound of valid detection region (pixels)
    320,                  // maxX — right bound of valid detection region (pixels)
    0,                    // minY — top bound of valid detection region (pixels)
    240,                  // maxY — bottom bound of valid detection region (pixels)
    16.0,                 // minSpeed (%)
    1.0,                  // distanceTolerance — odometry fallback stopping bubble (cm)
    0.22,                  // accelHeadingScaling
    0.2,                  // decelHeadingScaling
    0.25,                  // approachHeadingScaling
      15,                     // headingLockDistance 
    5.0                   // timeout (seconds)
);


    //forwardToPoint(-47, 0, 70, 15, 1, 0.05, 0, 0, 0.2, 0.2, 0.2, 40);
    //matchLoadPneumatics.set(true);
  wait(20000, msec);
    driveForwardV3(27,0,0);
    
    //turnRight(-52,5,2,85,22);
    //turnRight(-29.2,0,2,100,22);
    turnRight(-29,0);
    // wait(20000, msec);
    smartStop(5, 0, 200, false);
    //matchloadStart(3100,100,0,true);
    //frontHoodPneumatics.set(false);
    //smartStraight(47, 0, -180, 15, 220, 0, 0, 0.05, 0.2, 0.2, 0.2, 40);
    /*
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

       wait(20000, msec);

visionDriveV2(
    AIVision20__redCube,   // 1. targetSignature
    70,                      // 2. targetPixelWidth
    0.0,                      // 3. targetHeading
    brakeType::hold,          // 4. brakeMode
    40.0,                     // 5. maxSpeedPct
    0.06,                      // 6. kp_head
    0.0,                      // 7. ki_head
    0.0,                      // 8. kd_head
    0.3,                      // 9. kp_distToHeadScaling
    40,                       // 10. minObjectWidth
    0,                        // 11. minX
    320,                      // 12. maxX
    120,                      // 13. minY
    240,                      // 14. maxY
    24.0,                     // 15. minSpeedPct
    100.0,                    // 16. timeoutDistanceCM
    1.50,                     // 17. kp_dist
    0.0,                      // 18. ki_dist
    0.0                       // 19. kd_dist
); 

*/
smartStraight(47, 0, -90, 15, 100, 0.4, 0.0, 0.0, 0.2, 0.2, 0.2, 80);
wait(500,msec);
driveBackwardV3(8, 0, -90,24,1,1.1,0,0,0.1,0.2,0.3,100);
        //driveBackwardV3(12,0,-180);
       // wait(400, msec);
matchLoadPneumatics.set(false);   
wait(200, msec);   
 turnLeft(22,0,20,100);
wait(500, msec);
 //turnLeft(86,0,25,85,80);
  // smartStop(5, 10, 400, false);
//wait(200, msec);

/*
visionDriveMinimal(
        AIVision20__orangeGoal, 
        130,                    
        0.0,                    
        24.0, 40.0,             
        brakeType::hold,       
        .06, 0.0, 0.0, 
        0.3,       
        1.50, 0.0, 0.0    
    );

*/    

/*visionDriveV2(
    AIVision20__orangeGoal,   // 1. targetSignature
    150,                      // 2. targetPixelWidth
    -5.0,                      // 3. targetHeading
    brakeType::hold,          // 4. brakeMode
    40.0,                     // 5. maxSpeedPct
    0.1,                      // 6. kp_head
    0.0,                      // 7. ki_head
    0.0,                      // 8. kd_head
    0.3,                      // 9. kp_distToHeadScaling
    40,                       // 10. minObjectWidth
    0,                        // 11. minX
    320,                      // 12. maxX
    120,                      // 13. minY
    240,                      // 14. maxY
    24.0,                     // 15. minSpeedPct
    100.0,                    // 16. timeoutDistanceCM
    1.250,                     // 17. kp_dist
    0.0,                      // 18. ki_dist
    0.0                       // 19. kd_dist
); */

/*
// Replacing Vision tracking with Odometry-based movement to (100, 200)
// Replacing visionDriveV2 with moveVisionOdometry
moveVisionOdometry(
    100.0,                    // 1. targetX
    200.0,                    // 2. targetY
    15.0,                     // 3. breakDistance (Distance to start slowing down)
    24.0,                     // 4. minSpeed (from visionDriveV2 minSpeedPct)
    2.0,                      // 5. distanceTolerance (cm error allowed at target)
    0.1,                      // 6. kp_heading (from visionDriveV2 kp_head)
    0.0,                      // 7. ki_heading (from visionDriveV2 ki_head)
    0.0,                      // 8. kd_heading (from visionDriveV2 kd_head)
    brakeType::hold,          // 9. brakeMode (from visionDriveV2 brakeMode)
    1.0,                      // 10. accelHeadingScaling (Standard multiplier)
    1.0,                      // 11. decelHeadingScaling (Standard multiplier)
    0.3,                      // 12. approachHeadingScaling (Standard multiplier)
    40.0,                     // 13. maxSpeed (from visionDriveV2 maxSpeedPct)
    AIVision20__orangeGoal,   // 14. targetSignature (Vision color description)
    0.3,                      // 15. kp_distanceToHeadingScaling (from visionDriveV2 kp_distToHeadScaling)
    50                        // 16. minObjectWidthfrom visionDriveV2 minObjectWidth)
);
*/

smartStraight(67, 57, 92, 24, 150, 0.05, 0, 0., 0.2, 0.2, 0.2, 40);
leftGatePneumatics.set(false);
rightGatePneumatics.set(false);
score(30000, 100);
driveBackwardV3(8, 0, 92,24,1,1.1,0,0,0.1,0.2,0.3,100);

/*  //remove to see route


driveBackwardV3(10, -4, 90,24,1.1,0,0,0.1,0.2,0.3,70);
turnLeft(135,0,25,100,30);
visionDriveV2(
    AIVision20__redCube,   // 1. targetSignature
    60,                      // 2. targetPixelWidth
    135,                      // 3. targetHeading
    brakeType::hold,          // 4. brakeMode
    75.0,                     // 5. maxSpeedPct
    0.1,                      // 6. kp_head
    0.0,                      // 7. ki_head
    0.0,                      // 8. kd_head
    1.75,                      // 9. kp_distToHeadScaling
    20,                       // 10. minObjectWidth
    0,                        // 11. minX
    320,                      // 12. maxX
    120,                      // 13. minY
    240,                      // 14. maxY
    24.0,                     // 15. minSpeedPct
    100.0,                    // 16. timeoutDistanceCM
    1.50,                     // 17. kp_dist
    0.0,                      // 18. ki_dist
    0.0                       // 19. kd_dist
); 

matchloadStart(600,100,300,true);
driveForwardV3(30,15,135);
wait(200, msec);
matchLoadPneumatics.set(false);
smartStraight(47, 40, 135, 15, 150, 0.4, 0.01, 0.05, 0.2, 0.2, 0.2, 40);
outtake(1000, 100);

driveBackwardV3(35, 20, 135,24,1.1,0,0,0.1,0.2,0.3,70);
turnLeft(150,60,25,100,2);
visionDriveV2(
    AIVision20__redCube,   // 1. targetSignature
    50,                      // 2. targetPixelWidth
    180,                      // 3. targetHeading
    brakeType::hold,          // 4. brakeMode
    75.0,                     // 5. maxSpeedPct
    0.1,                      // 6. kp_head
    0.0,                      // 7. ki_head
    0.0,                      // 8. kd_head
    1.75,                      // 9. kp_distToHeadScaling
    20,                       // 10. minObjectWidth
    0,                        // 11. minX
    320,                      // 12. maxX
    120,                      // 13. minY
    240,                      // 14. maxY
    24.0,                     // 15. minSpeedPct
    100.0,                    // 16. timeoutDistanceCM
    1.50,                     // 17. kp_dist
    0.0,                      // 18. ki_dist
    0.0                       // 19. kd_dist
); 



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
    startCoordinateFinder();
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
    

    matchloadPneumaticStart(500, 0, true);   // drop piston, hold 500ms, retract
intakeHopperStart(500, 100, 0, true);    // runs intake at same time

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