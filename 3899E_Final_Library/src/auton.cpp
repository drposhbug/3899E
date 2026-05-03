#include "main.h"
#include "pid.h"
#include "utils.h"
#include "robot_config.h"
#include "vision_follow.h"
//#include "navigation.h"
#include "odometry.h"
#include "autontasks.h"
#include <cmath>

extern std::atomic<double> visionHorizontalNormalizedOffset;
extern std::atomic<int>    visionCurrentObjectWidth;
extern std::atomic<bool>   visionTargetTracked;
extern const pros::vision_signature_s_t* currentVisionSignature;
extern int currentMinObjectWidth;
extern std::atomic<bool> visionTaskShouldRun;

void test() {
// ======================================================================
// TEST: straightOdometryV3 — mostly defaults
// Drives forward 60cm, holding heading 0° (north), all PID/scaling defaults
// ======================================================================
    updateOdometry(); // Sync position before move

    // Required params only; all others use header defaults:
    //   minSpeed=16, distanceTolerance=2.0
    //   kp=0.4, ki=0.01, kd=0.05
    //   accelScaling=0.2, decelScaling=0.2, approachScaling=0.2
    //   maxSpeed=100, brakeMode=brake, timeout=3.0s
    straightOdometryV3(
        61,   // targetDistance (cm) — forward
        15,   // breakDistance (cm) — start decel 15cm out
        0     // targetHeading — standard Cartesian (east=0°, north=90°)
    );

    pros::delay(500);

    // Drive back to origin
    straightOdometryV3(
        -60,  // negative = reverse
        15,
        0
    );

    updateOdometry();
}    

void visionSensorTest() {
    pros::screen::erase();
    pros::screen::print(pros::E_TEXT_MEDIUM, 1, "VISION TEST MODE");
    pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Use Full Frame (0-320, 0-240)");

    while (true) {
        // --- TEST RED BLOCK ---
        pros::vision_object_s_t redObj  = aiVision.get_by_sig(0,aiVision_redCube.id);
        bool redFound  = (redObj.signature == (unsigned)aiVision_redCube.id  && redObj.width > 0);
        int  redX = redFound  ? redObj.x_middle_coord : 0;
        int  redW = redFound  ? redObj.width          : 0;

        // --- TEST BLUE BLOCK ---
        pros::vision_object_s_t blueObj = aiVision.get_by_sig(0,aiVision_blueCube.id);
        bool blueFound = (blueObj.signature == (unsigned)aiVision_blueCube.id && blueObj.width > 0);
        int  blueX = blueFound ? blueObj.x_middle_coord : 0;
        int  blueW = blueFound ? blueObj.width          : 0;

        if (redFound) {
            pros::screen::print(pros::E_TEXT_MEDIUM, 3, "RED : FOUND! X=%3d W=%3d  ", redX, redW);
        } else {
            pros::screen::print(pros::E_TEXT_MEDIUM, 3, "RED : SEARCHING...        ");
        }

        if (blueFound) {
            pros::screen::print(pros::E_TEXT_MEDIUM, 4, "BLUE: FOUND! X=%3d W=%3d  ", blueX, blueW);
        } else {
            pros::screen::print(pros::E_TEXT_MEDIUM, 4, "BLUE: SEARCHING...        ");
        }

        pros::screen::print(pros::E_TEXT_MEDIUM, 6, "If 0 found: Check IDs in Utility");

        pros::delay(100);
    }
}



void CoordinateFinderTask(){
    setStartPosition(0, 36.2, 0.0);
    while(true){
        startCoordinateFinder();
        pros::delay(500);
    }
}

void testVisionOnly() {
    currentVisionSignature = &aiVision_orangeGoal;
    currentMinObjectWidth = 10;
    visionTargetTracked = false;
    visionTaskShouldRun = true;

    pros::Task(visionTrackingTask, nullptr, "visionTracking");
    pros::delay(50);

    pros::screen::erase();

    while (pros::screen::touch_status().touch_status == pros::E_TOUCH_RELEASED) {
        pros::screen::erase();

        pros::screen::print(pros::E_TEXT_MEDIUM, 1, "=== VISION TEST ===");
        pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Press to exit");
        pros::screen::print(pros::E_TEXT_MEDIUM, 4, "Tracked: %s", visionTargetTracked.load() ? "**YES**" : "NO");
        pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Width: %d px", visionCurrentObjectWidth.load());
        pros::screen::print(pros::E_TEXT_MEDIUM, 6, "Offset: %.3f", visionHorizontalNormalizedOffset.load());
        pros::screen::print(pros::E_TEXT_MEDIUM, 8, "Raw Count: %d", aiVision.get_object_count());
        pros::screen::print(pros::E_TEXT_MEDIUM, 9, "Min Filter: %d", currentMinObjectWidth);

        pros::delay(200);
    }

    visionTaskShouldRun = false;
    pros::delay(50);
    pros::screen::erase();
}

void testVisionDirect() {
    pros::screen::erase();
    pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Direct Vision Test");
    pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Press screen to exit");

    while (pros::screen::touch_status().touch_status == pros::E_TOUCH_RELEASED) {
        pros::vision_object_s_t obj = aiVision.get_by_sig(0,aiVision_orangeGoal.id);
        bool found = (obj.signature == (unsigned)aiVision_orangeGoal.id && obj.width > 0);

        pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Object Count: %d   ", aiVision.get_object_count());

        if (found) {
            pros::screen::print(pros::E_TEXT_MEDIUM, 4, "X: %d  Y: %d        ", obj.x_middle_coord, obj.y_middle_coord);
            pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Width: %d  Height: %d   ", obj.width, obj.height);
        } else {
            pros::screen::print(pros::E_TEXT_MEDIUM, 4, "NO OBJECTS DETECTED     ");
        }

        pros::delay(100);
    }

    pros::screen::erase();
}

void testVisionBasic() {
    pros::screen::erase();
    pros::screen::print(pros::E_TEXT_MEDIUM, 1, "BASIC VISION TEST");
    pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Press to exit");

    while (pros::screen::touch_status().touch_status == pros::E_TOUCH_RELEASED) {
        pros::vision_object_s_t obj = aiVision.get_by_sig(0,aiVision_orangeGoal.id);
        bool found = (obj.signature == (unsigned)aiVision_orangeGoal.id && obj.width > 0);

        pros::screen::print(pros::E_TEXT_MEDIUM, 4, "Count: %d   ", aiVision.get_object_count());

        if (found) {
            pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Width: %d   ", obj.width);
            pros::screen::print(pros::E_TEXT_MEDIUM, 6, "X: %d Y: %d   ", obj.x_middle_coord, obj.y_middle_coord);
        }

        pros::delay(100);
    }
}

void autonTest() {
    // Initialize sensor
    //initializeOpticalSensor();
    setStartPosition(0, 0, 0);

    //Good for fast 100 power vision

    //startOdometryTask();

    //moveOdometry(41, 82, 10, 20, 2, 2.0, 0.0, 0.0, brakeType::brake, 1.0, 1.0, 1.0, 100);
    //moveOdometry(-18, 87.9, 10, 20, 2, 2, 0.0, 0.0, brakeType::brake, 0.05, 0.05, 0.05, 100);



    //testVisionOnly();
    //testVisionDirect();
    // testVisionBasic();


    //forwardToPoint(0, -75, 20, 16, 2.0, 2, 0.0, 0.0, 0.1, 0.1, 0.1, 60);
    //startCoordinateFinder();

    // moveOdometry(targetX, targetY, breakDist, minSpeed, tolerance, kP, kI, kD, brakeType, accelScale, decelScale, approachScale, maxSpeed)
}




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
    pros::delay(200);
    pros::delay(200);
    forwardMP(15, 10, 90, 15, 0.815, 0.0, 0.0, 0.0, 0.55, 0.3, 70);
    pros::delay(200);
    backwardMP(20, 15, 90, 10, 0.815, 0.00, 0.00, 0., 0.55, 0.3, 50);
    pros::delay(200);
    pros::delay(200);
    forwardMP(35, 18, 0, 15, 0.815, 0.0, 0.0, 0.0, 0, 0, 80);
}

void autonRight(){
    initializeOpticalSensor();
    setStartPosition(0.0, 0.0, 0.0);

    forwardMP(79, 38, 0, 20, 0.815, 0.0, 0.0, 0.0, 0.55, 0.3, 80);
    pros::delay(200);
    pros::delay(200);
    forwardMP(15, 10, 270, 15, 0.815, 0.0, 0.0, 0.0, 0.55, 0.3, 70);
    pros::delay(200);
    backwardMP(20, 15, 270, 10, 0.815, 0.00, 0.00, 0., 0.55, 0.3, 50);
    pros::delay(200);
    pros::delay(200);
    forwardMP(35, 18, 180, 15, 0.815, 0.0, 0.0, 0.0, 0, 0, 80);
}

void leftSideLong(){
    initializeOpticalSensor();
    InertialSensor.set_rotation(0);
    robotStartingHeading = 16;
    ptoPneumatics.set_value(false);
    backHoodPneumatics.set_value(false);
    frontHoodPneumatics.set_value(false);
    wingPneumatics.set_value(false);

    pros::delay(50);
    //intakeStart(1000, 100, false);
    matchloadStart(6000,100,1050,true);
    driveForward(83, 60, 16);
    pros::delay(250);
    turnLeft(147,118,26,80,16);
    pros::delay(200);

    driveForward(102,70,147,24,0.3,0.002,0,0.1,1,0.3,90);
    pros::delay(200);
    turnLeft(175,25,26,80,13);
    pros::delay(200);
    smartStraight(50, 26, 180, 15, 150);
    pros::delay(270);

    driveBackward(26, 14, 176);
    pros::delay(200);
    turnRight(0,145,25,90,16);
    pros::delay(200);
    smartStraight(40, 19, 0, 24, 200);

    score(3200, 100);
    driveBackward(20, 14, -10);
    pros::delay(100);
    turnLeft(40,18,25,90,16);

    pros::delay(100);

    driveForward(35,25,40,24,0.3,0.002,0,0.1,1,0.3,90);
    pros::delay(100);

    turnRight(4,30,26,80);
    pros::delay(100);
    driveForward(12,0,4,24,0.3,0.002,0,0.1,1,0.3,90);
    wingPneumatics.set_value(true);

    driveForward(40,0,4,24,0.3,0.002,0,0.1,1,0.3,40);
}

void leftSidemiddle(){
    initializeOpticalSensor();
    InertialSensor.set_rotation(0);
    robotStartingHeading = 16;
    ptoPneumatics.set_value(false);
    backHoodPneumatics.set_value(false);
    frontHoodPneumatics.set_value(false);
    wingPneumatics.set_value(false);

    pros::delay(50);
    //intakeStart(300, 100, false);
    matchloadStart(1200,100,750,true);
    matchLoadPneumatics.set_value(true);
    driveForward(70, 60, 16,26);
    pros::delay(200);

    //driveBackward(8, 4, 16);
    // move(3, 50, vex::reverse); //simple move without PID
    //driveBackwardV2(8, 3, 16, 24, 1);
    //driveBackwardV2(8,3,16,24,1,0.005,0,0.1,1,0.3,60);

    turnRight(-50,50,25,90,16);

    //   pros::delay(100);
    //scoreStart(1200, 70);
    driveForward(25,13,-50,24,0.3,0.002,0,0.1,1,0.3,90);
    //        pros::delay(200);
    //driveBackward(105, 82, -45);


    //turnRight(-180,190,25,90,16);
}

void SpeedwayAutonLeft(){
    initializeOpticalSensor();
    InertialSensor.set_rotation(0);
    robotStartingHeading = 0;
    ptoPneumatics.set_value(true);
    backHoodPneumatics.set_value(false);
    frontHoodPneumatics.set_value(true);

    pros::delay(400);
    intake(true,100);
    forwardMP(80,60,-18,20,0.615,0,0,0.1,0.05,0.05,50);
    pros::delay(400);
    backwardMP(7,4,-20,15);
    intake(false, 0);
    forwardMP(18,10,29,30,0.5,0,0,0.1,0.05,0.05,100);
    ptoPneumatics.set_value(true);
    backHoodPneumatics.set_value(true);
    frontHoodPneumatics.set_value(false);
    score(400, 95);
    score(1000, 65);
    score(1000, 65);
    backwardMP(116,67,45,15);
    pros::delay(200);
    matchLoadPneumatics.set_value(true);
    //forwardMP(27,19,180,20);
    //rightMotor[0].spin(forward,12,vex::voltageUnits::volt);
    //rightMotor[1].spin(forward,12,vex::voltageUnits::volt);
    //rightMotor[2].spin(forward,12,vex::voltageUnits::volt);
    //leftMotor[0].spin(forward,12,vex::voltageUnits::volt);
    //leftMotor[1].spin(forward,12,vex::voltageUnits::volt);
    //leftMotor[2].spin(forward,12,vex::voltageUnits::volt);
    pros::delay(800);
    rightDrive.brake();
    leftDrive.brake();
    intake(true,100);
    pros::delay(700);
    intake(false,0);
}

void SevenBallRight(){
    matchLoadPneumatics.set_value(true);
    initializeOpticalSensor();
    InertialSensor.set_rotation(0);
    robotStartingHeading = -12;
    ptoPneumatics.set_value(false);
    backHoodPneumatics.set_value(false);
    frontHoodPneumatics.set_value(false);
    pros::delay(50);//necessary in order for matchload pneumatics to engage properly epstein fn
    matchLoadPneumatics.set_value(false);
    driveForward(49, 38, -12);
    pros::delay(300);
    matchLoadPneumatics.set_value(true);
    driveForward(30, 24, -12);
    pros::delay(200);
    driveBackward(25, 18, -12);
    pros::delay(100);
}

void SevenBallLeft(){
    matchLoadPneumatics.set_value(true);
    initializeOpticalSensor();
    InertialSensor.set_rotation(0);
    robotStartingHeading = 12;
    ptoPneumatics.set_value(false);
    backHoodPneumatics.set_value(false);
    frontHoodPneumatics.set_value(false);
    pros::delay(50);//necessary in order for matchload pneumatics to engage properly epstein fn
    matchLoadPneumatics.set_value(false);
    //intakeStart(1000, 35, true);
    //intakeStart(5500, 75, true);
    driveForward(40, 30, 16);
    pros::delay(300);
    //matchLoadPneumatics.set_value(true);
    driveForward(21, 21, 16);
    pros::delay(100);
    driveBackward(25, 15, 16);
    pros::delay(100);
    turnLeft(88,65);
    driveForward(72, 35, 88);
    //matchLoadPneumatics.set_value(false);
    pros::delay(100);
    turnRight(2,65);
    pidlessForward(600, 20);
    score(7500, 75);
    //driveForward(19, 18, 0);

    //ptoPneumatics.set_value(true);
    //intakeStart(7500, 75, true, true);
}

void rightMiddleAuto(){
    initializeOpticalSensor();
    InertialSensor.set_rotation(0);
    robotStartingHeading = -16;
    ptoPneumatics.set_value(false);
    backHoodPneumatics.set_value(false);
    frontHoodPneumatics.set_value(false);
    wingPneumatics.set_value(false);

    pros::delay(50);
   // intakeStart(470, 100, false);
    matchloadStart(2000,40,630,true);
    driveForward(91, 58, -16);
    pros::delay(200);
    driveBackward(19, 12, -16);
    pros::delay(100);
    turnLeft(45,50,25,90,16);

    //  turnRight(-148,114,26,80,14);
    pros::delay(200);
    smartStraight(70, 50, 45, 15, 150);
    outtake(500, 100);
    pros::delay(100);

    driveBackward(130, 110, 45);
    pros::delay(100);
    turnLeft(180,110,25,90,16);
    smartStraight(30, 21, 180, 15, 150);
    pros::delay(270);

    //smartMove(34, 60, forward, 250); //for matchload smart wall stop, no pid
    driveBackward(26, 14, 176);
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
}

void systemTest(){ //last chance to look at me hector ding ding ding
    initializeOpticalSensor();
    InertialSensor.set_rotation(0);

    pros::Motor* allMotors[] = {
        &LeftMotor1, &LeftMotor2, &LeftMotor3,
        &RightMotor1, &RightMotor2, &RightMotor3,
        &intakeMotor1, &intakeMotor2
    };

    const int numMotors = sizeof(allMotors) / sizeof(allMotors[0]);

    for (int i = 0; i < numMotors; ++i) {
        pros::Motor* m = allMotors[i];
        m->move(127);  // full forward

        pros::delay(500);

        if (m->get_actual_velocity() != 0) {
            Controller.rumble(".");
        }
        pros::delay(500);

        m->brake();
    }

    wingPneumatics.set_value(true);
    pros::delay(500);
    wingPneumatics.set_value(false);
    pros::delay(500);
    matchLoadPneumatics.set_value(true);
    pros::delay(500);
    matchLoadPneumatics.set_value(false);
    pros::delay(500);
    frontHoodPneumatics.set_value(true);
    pros::delay(500);
    frontHoodPneumatics.set_value(false);
    pros::delay(500);
    leftGatePneumatics.set_value(true);
    pros::delay(500);
    leftGatePneumatics.set_value(false);
    pros::delay(500);
    rightGatePneumatics.set_value(true);
    pros::delay(500);
    rightGatePneumatics.set_value(false);

    //optical sensor test
    bool lNear = leftLaneOptical.get_proximity()  > 50;
    bool rNear = rightLaneOptical.get_proximity() > 50;
    if (lNear) {
        Controller.rumble(".");
    }
    pros::delay(500);
    if (rNear) {
        Controller.rumble(".");
    }
    pros::delay(500);
}


void skillsAuton(){
    //Initialize and set starting position in Standard Cartesian
    // Starting at East (0° Standard)
    setStartPosition(0, 36, 0);
    startOdometryTask();
    leftGatePneumatics.set_value(true);
    rightGatePneumatics.set_value(true);
   // startCoordinateFinder();
    initializeOpticalSensor();
    leftLaneOptical.set_led_pwm(100);
    rightLaneOptical.set_led_pwm(100);
    rudderPneumatics.set_value(true);
    //*********************************************************
    //Stop 1: South East MatchLoader (6 Blocks + 1 Preload)
    //********************************************************* */
    turnRight(-42,20);
    smartStop(5, 5, 200, false);
    intakeColourStart(9000, 100, false, false);
    matchloadPneumaticStart(9000, 0, true);

    visionDriveMinimal(
        aiVision_redCube,
        100,
        0.0,
        24.0, 65.0,
        pros::E_MOTOR_BRAKE_COAST,
        .5, 0.0, 0.0,
        1.2,
        1.0, 0.0, 0.0
    );

    smartStraight(47, 0, -90, 15, 100, 0.4, 0.0, 0.0, 0.2, 0.2, 0.2, 50);
    pros::delay(2000);
    //*********************************************************
    //Stop 2: South West Middle (4 Blocks)
    //*********************************************************
    //Driving out
    //matchLoadPneumatics.set_value(false);
    matchloadPneumaticStop();
    turnRight(-175,40);
    smartStop(5, 5, 400, false);

    // Going to get the 4 blocks
    moveVisionOdometry(
        aiVision_redCube,     // targetSignature
        60,                   // targetPixelWidth — stop when object is this wide in pixels
        48.1,                 // targetX (cm)
        67.4,                 // targetY (cm)
        30.0,                 // breakDistance — 48 worked well before
        pros::E_MOTOR_BRAKE_COAST,  // brakeMode
        60.0,                 // maxSpeed (%)
        0.43,                 // kp_head
        0.0,                  // ki_head
        0.04,                 // kd_head
        1.05,                 // kp_distToHeadScaling
        10,                   // minObjectWidth
        50,                   // minX
        260,                  // maxX
        0,                    // minY
        240,                  // maxY
        25.0,                 // minSpeed (%)
        1.0,                  // distanceTolerance (cm)
        0.22,                 // accelHeadingScaling
        0.2,                  // decelHeadingScaling
        0.25,                 // approachHeadingScaling
        15,                   // headingLockDistance (cm)
        5.0                   // timeout (seconds)
    );

    matchloadPneumaticStart(400, 800, true);
    //pros::delay(30000);
    //moveOdometry(26.88, 125.99, 0, 20, 2, 0.75, 0, 0, pros::E_MOTOR_BRAKE_HOLD, 0.2, 0.2, 0.2, 60, 15, 5000);
    moveOdometry(
    37.71,             // targetX (cm)
    100,               // targetY (cm)
    0.0,               // breakDistance
    25.0,              // minSpeed (%)
    2.0,               // distanceTolerance (cm)
    0.75,              // kp_heading
    0.0,               // ki_heading
    0.04,              // kd_heading
    pros::E_MOTOR_BRAKE_HOLD,  // brakeMode
    0.22,              // accelHeadingScaling
    0.2,               // decelHeadingScaling
    0.25,              // approachHeadingScaling
    60.0,              // maxSpeed (%)
    20.0,              // headingLockDistance (cm)
    5.0                // timeout (seconds)
);
//pros::delay(50000);
moveVisionOdometry(
        aiVision_redCube,     // targetSignature
        60,                   // targetPixelWidth
        31.61,                // targetX (cm)
        180,                  // targetY (cm)
        30.0,                 // breakDistance
        pros::E_MOTOR_BRAKE_HOLD,   // brakeMode
        60.0,                 // maxSpeed (%)
        0.43,                 // kp_head
        0.0,                  // ki_head
        0.04,                 // kd_head
        1.05,                 // kp_distToHeadScaling
        10,                   // minObjectWidth
        50,                   // minX
        260,                  // maxX
        0,                    // minY
        240,                  // maxY
        25.0,                 // minSpeed (%)
        1.0,                  // distanceTolerance (cm)
        0.22,                 // accelHeadingScaling
        0.2,                  // decelHeadingScaling
        0.25,                 // approachHeadingScaling
        15,                   // headingLockDistance (cm)
        5.0                   // timeout (seconds)
    );

   pros::delay(30000);

    moveOdometry(
    30.31,             // targetX (cm)
    210,               // targetY (cm)
    0.0,               // breakDistance
    25.0,              // minSpeed (%)
    2.0,               // distanceTolerance (cm)
    0.75,              // kp_heading
    0.0,               // ki_heading
    0.04,              // kd_heading
    pros::E_MOTOR_BRAKE_HOLD,  // brakeMode
    0.22,              // accelHeadingScaling
    0.2,               // decelHeadingScaling
    0.25,              // approachHeadingScaling
    60.0,              // maxSpeed (%)
    20.0,              // headingLockDistance (cm)
    5.0                // timeout (seconds)
);
   pros::delay(30000);
    smartStop(5, 5, 300, false);
    pros::delay(30000);
    //*****************************************************
    //Stop 3: North West (2 Blocks)
    //*****************************************************
    turnRight(45,20);
    wingPneumatics.set_value(true);

    //pros::delay(30000);
    //turnRight(80,20); //Tyler & Justin's
    smartStop(5, 5, 200, false);
    //matchloadPistonStop();
    matchloadPneumaticStop();
    moveOdometry(
    59.32,             // targetX (cm)
    153.35,            // targetY (cm)
    30,                // breakDistance
    25.0,              // minSpeed (%)
    2.0,               // distanceTolerance (cm)
    0.85,              // kp_heading
    0.0,               // ki_heading
    0.04,              // kd_heading
    pros::E_MOTOR_BRAKE_HOLD,  // brakeMode
    0.22,              // accelHeadingScaling
    0.2,               // decelHeadingScaling
    0.25,              // approachHeadingScaling
    60.0,              // maxSpeed (%)
    20.0,              // headingLockDistance (cm)
    5.0                // timeout (seconds)
);
   matchloadPneumaticStart(1000, 0, true);

    //*****************************************************
    //Stop 4: North East (2 Blocks)
    //*****************************************************

    //matchloadPneumaticStop();
    turnRight(-185,40);

    smartStop(5, 5, 400, false);
    moveVisionOdometry(
        aiVision_redCube,     // targetSignature
        60,                   // targetPixelWidth
        48.1,                 // targetX (cm)
        67.4,                 // targetY (cm)
        30.0,                 // breakDistance
        pros::E_MOTOR_BRAKE_COAST,  // brakeMode
        60.0,                 // maxSpeed (%)
        0.43,                 // kp_head
        0.0,                  // ki_head
        0.04,                 // kd_head
        1.05,                 // kp_distToHeadScaling
        10,                   // minObjectWidth
        50,                   // minX
        260,                  // maxX
        0,                    // minY
        240,                  // maxY
        25.0,                 // minSpeed (%)
        1.0,                  // distanceTolerance (cm)
        0.22,                 // accelHeadingScaling
        0.2,                  // decelHeadingScaling
        0.25,                 // approachHeadingScaling
        15,                   // headingLockDistance (cm)
        5.0                   // timeout (seconds)
    );
    matchloadPneumaticStart(2500, 425, true);
    //pros::delay(30000);
    //moveOdometry(26.88, 125.99, 0, 20, 2, 0.75, 0, 0, pros::E_MOTOR_BRAKE_HOLD, 0.2, 0.2, 0.2, 60, 15, 5000);
    moveOdometry(
    30.5,              // targetX (cm)
    125.99,            // targetY (cm)
    0.0,               // breakDistance
    25.0,              // minSpeed (%)
    2.0,               // distanceTolerance (cm)
    0.75,              // kp_heading
    0.0,               // ki_heading
    0.04,              // kd_heading
    pros::E_MOTOR_BRAKE_HOLD,  // brakeMode
    0.22,              // accelHeadingScaling
    0.2,               // decelHeadingScaling
    0.25,              // approachHeadingScaling
    60.0,              // maxSpeed (%)
    20.0,              // headingLockDistance (cm)
    5.0                // timeout (seconds)
);
   // pros::delay(30000);
    smartStop(5, 5, 300, false);
}


void skillsAuton2(){
    //Initialize and set starting position in Standard Cartesian
    // Starting at East (0° Standard)
    setStartPosition(0, 36, 0);
    startOdometryTask();
    leftGatePneumatics.set_value(true);
    rightGatePneumatics.set_value(true);
   // startCoordinateFinder();
    initializeOpticalSensor();
    leftLaneOptical.set_led_pwm(100);
    rightLaneOptical.set_led_pwm(100);
    rudderPneumatics.set_value(false);
    //*********************************************************
    //Stop 1: South East MatchLoader (6 Blocks + 1 Preload)
    //********************************************************* */
    turnRight(-42,20);
    smartStop(5, 5, 200, false);
    intakeColourStart(9000, 100, false, false);
    matchloadPneumaticStart(9000, 0, true);

    visionDriveMinimal(
        aiVision_redCube,
        100,
        0.0,
        24.0, 65.0,
        pros::E_MOTOR_BRAKE_COAST,
        .5, 0.0, 0.0,
        1.2,
        1.0, 0.0, 0.0
    );

 // pros::delay(20000);
    smartStraight(47, 0, -90, 15, 100, 0.4, 0.0, 0.0, 0.2, 0.2, 0.2, 50);
    pros::delay(1000);
    //*********************************************************
    //Stop 2: South West Middle (4 Blocks)
    //*********************************************************
    //Driving out
    //matchLoadPneumatics.set_value(false);
    matchloadPneumaticStop();
    turnLeft(90,140,25,100,4);
    smartStop(5, 5, 400, false);
    pros::delay(5000);
    smartStraight(47, 0, -90, 15, 100, 0.4, 0.0, 0.0, 0.2, 0.2, 0.2, 50);
   pros::delay(50000);

    moveVisionOdometry(
        aiVision_orangeGoal,  // targetSignature
        100,                  // targetPixelWidth
        70,                   // targetX (cm)
        53.5,                 // targetY (cm)
        15.0,                 // breakDistance
        pros::E_MOTOR_BRAKE_COAST,  // brakeMode
        60.0,                 // maxSpeed (%)
        0.75,                 // kp_head
        0.0,                  // ki_head
        0.04,                 // kd_head
        1.05,                 // kp_distToHeadScaling
        10,                   // minObjectWidth
        0,                    // minX
        260,                  // maxX
        0,                    // minY
        240,                  // maxY
        25.0,                 // minSpeed (%)
        1.0,                  // distanceTolerance (cm)
        0.22,                 // accelHeadingScaling
        0.2,                  // decelHeadingScaling
        0.25,                 // approachHeadingScaling
        20,                   // headingLockDistance (cm)
        5.0                   // timeout (seconds)
    );
    smartStop(5, 5, 400, false);
    //matchloadPneumaticStart(2000, 350, true);

    // Going to get the 4 blocks
    moveVisionOdometry(
        aiVision_redCube,     // targetSignature
        60,                   // targetPixelWidth
        48.1,                 // targetX (cm)
        67.4,                 // targetY (cm)
        30.0,                 // breakDistance
        pros::E_MOTOR_BRAKE_COAST,  // brakeMode
        60.0,                 // maxSpeed (%)
        0.43,                 // kp_head
        0.0,                  // ki_head
        0.04,                 // kd_head
        1.05,                 // kp_distToHeadScaling
        10,                   // minObjectWidth
        50,                   // minX
        260,                  // maxX
        0,                    // minY
        240,                  // maxY
        25.0,                 // minSpeed (%)
        1.0,                  // distanceTolerance (cm)
        0.22,                 // accelHeadingScaling
        0.2,                  // decelHeadingScaling
        0.25,                 // approachHeadingScaling
        15,                   // headingLockDistance (cm)
        5.0                   // timeout (seconds)
    );

    matchloadPneumaticStart(400, 800, true);
    //pros::delay(30000);
    //moveOdometry(26.88, 125.99, 0, 20, 2, 0.75, 0, 0, pros::E_MOTOR_BRAKE_HOLD, 0.2, 0.2, 0.2, 60, 15, 5000);
    moveOdometry(
    37.71,             // targetX (cm)
    100,               // targetY (cm)
    0.0,               // breakDistance
    25.0,              // minSpeed (%)
    2.0,               // distanceTolerance (cm)
    0.75,              // kp_heading
    0.0,               // ki_heading
    0.04,              // kd_heading
    pros::E_MOTOR_BRAKE_HOLD,  // brakeMode
    0.22,              // accelHeadingScaling
    0.2,               // decelHeadingScaling
    0.25,              // approachHeadingScaling
    60.0,              // maxSpeed (%)
    20.0,              // headingLockDistance (cm)
    5.0                // timeout (seconds)
);
//pros::delay(50000);
moveVisionOdometry(
        aiVision_redCube,     // targetSignature
        60,                   // targetPixelWidth
        31.61,                // targetX (cm)
        180,                  // targetY (cm)
        30.0,                 // breakDistance
        pros::E_MOTOR_BRAKE_HOLD,   // brakeMode
        60.0,                 // maxSpeed (%)
        0.43,                 // kp_head
        0.0,                  // ki_head
        0.04,                 // kd_head
        1.05,                 // kp_distToHeadScaling
        10,                   // minObjectWidth
        50,                   // minX
        260,                  // maxX
        0,                    // minY
        240,                  // maxY
        25.0,                 // minSpeed (%)
        1.0,                  // distanceTolerance (cm)
        0.22,                 // accelHeadingScaling
        0.2,                  // decelHeadingScaling
        0.25,                 // approachHeadingScaling
        15,                   // headingLockDistance (cm)
        5.0                   // timeout (seconds)
    );

   pros::delay(30000);

    moveOdometry(
    30.31,             // targetX (cm)
    210,               // targetY (cm)
    0.0,               // breakDistance
    25.0,              // minSpeed (%)
    2.0,               // distanceTolerance (cm)
    0.75,              // kp_heading
    0.0,               // ki_heading
    0.04,              // kd_heading
    pros::E_MOTOR_BRAKE_HOLD,  // brakeMode
    0.22,              // accelHeadingScaling
    0.2,               // decelHeadingScaling
    0.25,              // approachHeadingScaling
    60.0,              // maxSpeed (%)
    20.0,              // headingLockDistance (cm)
    5.0                // timeout (seconds)
);
   pros::delay(30000);
    smartStop(5, 5, 300, false);
    pros::delay(30000);
    //*****************************************************
    //Stop 3: North West (2 Blocks)
    //*****************************************************
    turnRight(45,20);
    wingPneumatics.set_value(true);

    //pros::delay(30000);
    //turnRight(80,20); //Tyler & Justin's
    smartStop(5, 5, 200, false);
    //matchloadPistonStop();
    matchloadPneumaticStop();
    moveOdometry(
    59.32,             // targetX (cm)
    153.35,            // targetY (cm)
    30,                // breakDistance
    25.0,              // minSpeed (%)
    2.0,               // distanceTolerance (cm)
    0.85,              // kp_heading
    0.0,               // ki_heading
    0.04,              // kd_heading
    pros::E_MOTOR_BRAKE_HOLD,  // brakeMode
    0.22,              // accelHeadingScaling
    0.2,               // decelHeadingScaling
    0.25,              // approachHeadingScaling
    60.0,              // maxSpeed (%)
    20.0,              // headingLockDistance (cm)
    5.0                // timeout (seconds)
);
   matchloadPneumaticStart(1000, 0, true);

    //*****************************************************
    //Stop 4: North East (2 Blocks)
    //*****************************************************

    //matchloadPneumaticStop();
    turnRight(-185,40);

    smartStop(5, 5, 400, false);
    moveVisionOdometry(
        aiVision_redCube,     // targetSignature
        60,                   // targetPixelWidth
        48.1,                 // targetX (cm)
        67.4,                 // targetY (cm)
        30.0,                 // breakDistance
        pros::E_MOTOR_BRAKE_COAST,  // brakeMode
        60.0,                 // maxSpeed (%)
        0.43,                 // kp_head
        0.0,                  // ki_head
        0.04,                 // kd_head
        1.05,                 // kp_distToHeadScaling
        10,                   // minObjectWidth
        50,                   // minX
        260,                  // maxX
        0,                    // minY
        240,                  // maxY
        25.0,                 // minSpeed (%)
        1.0,                  // distanceTolerance (cm)
        0.22,                 // accelHeadingScaling
        0.2,                  // decelHeadingScaling
        0.25,                 // approachHeadingScaling
        15,                   // headingLockDistance (cm)
        5.0                   // timeout (seconds)
    );
    matchloadPneumaticStart(2500, 425, true);
    //pros::delay(30000);
    //moveOdometry(26.88, 125.99, 0, 20, 2, 0.75, 0, 0, pros::E_MOTOR_BRAKE_HOLD, 0.2, 0.2, 0.2, 60, 15, 5000);
    moveOdometry(
    30.5,              // targetX (cm)
    125.99,            // targetY (cm)
    0.0,               // breakDistance
    25.0,              // minSpeed (%)
    2.0,               // distanceTolerance (cm)
    0.75,              // kp_heading
    0.0,               // ki_heading
    0.04,              // kd_heading
    pros::E_MOTOR_BRAKE_HOLD,  // brakeMode
    0.22,              // accelHeadingScaling
    0.2,               // decelHeadingScaling
    0.25,              // approachHeadingScaling
    60.0,              // maxSpeed (%)
    20.0,              // headingLockDistance (cm)
    5.0                // timeout (seconds)
);
   // pros::delay(30000);
    smartStop(5, 5, 300, false);
}


void soloAWP(){
    //setStartPosition(0.0, 25, -90.0);
    //startOdometryTask();
    //startCoordinateFinder();
    //initializeOpticalSensor();
    ptoPneumatics.set_value(false);
    backHoodPneumatics.set_value(false);
    frontHoodPneumatics.set_value(false);
    wingPneumatics.set_value(true);

    forwardToPoint(-47, 0, 70, 15, 1, 0.05, 0, 0, 0.2, 0.2, 0.2, 40);
    turnRight(-135,0,24,100,20);
    smartStop(5, 10, 300, false);
    smartStraight(47, 0, -180, 15, 200, 0, 0, 0.05, 0.2, 0.2, 0.2, 40);


    //Bottom Middle Right 4 Cubes
    //setStartPosition(0.0, 0, -180.0); //******TEMP COMMENT OUT********
    turnRight(45,0,25,100,70);
    pros::delay(350);

    visionDriveV2(
        aiVision_redCube,              // 1. targetSignature
        60,                            // 2. targetPixelWidth
        45,                            // 3. targetHeading
        pros::E_MOTOR_BRAKE_HOLD,      // 4. brakeMode
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

    matchLoadPneumatics.set_value(true);



    //Top Middle Right 4 Cubes
    setStartPosition(0.0, 0, 45); //******TEMP COMMENT OUT********/
    turnRight(-45,0,25,100,70);

    visionDriveV2(
        aiVision_blueCube,             // 1. targetSignature
        60,                            // 2. targetPixelWidth
        45,                            // 3. targetHeading
        pros::E_MOTOR_BRAKE_HOLD,      // 4. brakeMode
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
}


void colourTest(){
    initializeOpticalSensor();
    InertialSensor.set_rotation(0);
    //intakeStart(10000, 50, true);
}

void soloAWPMiddle(){
    initializeOpticalSensor();
    InertialSensor.set_rotation(0);
    robotStartingHeading = -90;
    ptoPneumatics.set_value(false);
    backHoodPneumatics.set_value(false);
    frontHoodPneumatics.set_value(false);
    wingPneumatics.set_value(true);
    driveForwardV2(60,45,-90,24,3,0.3,0.005,0,0.1,1,0.3,100);

    leftDrive.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    rightDrive.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    leftDrive.brake();
    rightDrive.brake();
    turnRight(-179,65,25,80,28);
}

void skills() {
    initializeOpticalSensor();
    setStartPosition(0.0, 0.0, 0.0);
}

void Inertial_Calib(){
    pros::screen::erase();
    pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Calibrating...");
    InertialSensor.reset();
    while (InertialSensor.is_calibrating()) {
        pros::delay(100);
    }
    pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Calibration Complete!");
    pros::delay(500);
    pros::screen::erase();
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

    pros::screen::erase();

    while(true) {
        pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Selected: %s          ", autonNames[autonMode]);
        pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Press Left/Right to change");
        pros::screen::print(pros::E_TEXT_MEDIUM, 4, "Press Center to confirm");

        if(Controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            autonMode = (autonMode - 1 + numAutons) % numAutons;
            pros::delay(200);
        }
        else if(Controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            autonMode = (autonMode + 1) % numAutons;
            pros::delay(200);
        }
        else if(Controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            pros::screen::erase();
            pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Running: %s", autonNames[autonMode]);
            pros::delay(500);

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

        pros::delay(20);
    }
}


void nothing(){
    initializeOpticalSensor();
    InertialSensor.set_rotation(0);
    robotStartingHeading = 0;
    //matchloadPistonStart(2000, 0);
    intakeStart(5000, 60, false, false);
    //matchloadStart(20000, 100, 30000, true);
    //pros::delay(5000);
    pidlessForward(1000, 100);
    //driveForward(15, 9, 0, 15, 0.615, 0, 0, 0.1, 0.05, 0.05, 50);
}

void soloAwp2(){
    setStartPosition(0, 36, 0);
    startOdometryTask();
    //startCoordinateFinder();
    //InertialSensor.set_rotation(0);
    //robotStartingHeading = -90;
    initializeOpticalSensor();

   intakeHopperStart(2000, 100, 500, true); // 500ms delay, then runs intake, non-blocking

moveVisionOdometry(
    aiVision_redCube,     // targetSignature
    60,                   // targetPixelWidth
    110.0,                // targetX (cm)
    36,                   // targetY (cm)
    70.0,                 // breakDistance
    pros::E_MOTOR_BRAKE_HOLD,   // brakeMode
    100.0,                // maxSpeed (%)
    0.43,                 // kp_head
    0.0,                  // ki_head
    0.04,                 // kd_head
    1.05,                 // kp_distToHeadScaling
    10,                   // minObjectWidth
    0,                    // minX
    320,                  // maxX
    0,                    // minY
    240,                  // maxY
    16.0,                 // minSpeed (%)
    1.0,                  // distanceTolerance (cm)
    0.22,                 // accelHeadingScaling
    0.2,                  // decelHeadingScaling
    0.25,                 // approachHeadingScaling
    15,                   // headingLockDistance
    5.0                   // timeout (seconds)
);

    //forwardToPoint(-47, 0, 70, 15, 1, 0.05, 0, 0, 0.2, 0.2, 0.2, 40);
    //matchLoadPneumatics.set_value(true);
  pros::delay(20000);
    //turnRight(-52,5,2,85,22);
    //turnRight(-29.2,0,2,100,22);
    turnRight(-29,0);
    // pros::delay(20000);
    smartStop(5, 0, 200, false);
    //matchloadStart(3100,100,0,true);
    //frontHoodPneumatics.set_value(false);
    //smartStraight(47, 0, -180, 15, 220, 0, 0, 0.05, 0.2, 0.2, 0.2, 40);
smartStraight(47, 0, -90, 15, 100, 0.4, 0.0, 0.0, 0.2, 0.2, 0.2, 80);
pros::delay(500);
matchLoadPneumatics.set_value(false);
pros::delay(200);
 turnLeft(22,0,20,100);
pros::delay(500);
 //turnLeft(86,0,25,85,80);
  // smartStop(5, 10, 400, false);
//pros::delay(200);

smartStraight(67, 57, 92, 24, 150, 0.05, 0, 0., 0.2, 0.2, 0.2, 40);
leftGatePneumatics.set_value(false);
rightGatePneumatics.set_value(false);
score(30000, 100);
    startCoordinateFinder();
}

void runEverything(){
    initializeOpticalSensor();
    InertialSensor.set_rotation(0);

    //intakeStart(1000, 50, true);

    matchloadStart(1000, 50, 0, true);

    score(1000, 50);

    outtake(1000, 50);

    wingPneumatics.set_value(true);
}

void skillsAutonGateway(){
    setStartPosition(0.0, 0, -90.0);
    //startOdometryTask();
    //startCoordinateFinder();
    //InertialSensor.set_rotation(0);
    //robotStartingHeading = -90;
    initializeOpticalSensor();

    //forwardToPoint(-47, 0, 70, 15, 1, 0.05, 0, 0, 0.2, 0.2, 0.2, 40);
    matchLoadPneumatics.set_value(true);
    pros::delay(200);
    turnRight(-142,5,2,85,22);
    smartStop(5, 10, 300, false);
    matchloadStart(2700,100,0,true);
    //smartStraight(47, 0, -180, 15, 220, 0, 0, 0.05, 0.2, 0.2, 0.2, 40);
    visionDriveMinimal(
        aiVision_redCube,
        70,
        0.0,
        24.0, 40.0,
        pros::E_MOTOR_BRAKE_HOLD,
        .06, 0.0, 0.0,
        0.3,
        1.50, 0.0, 0.0
    );
   smartStraight(47, 40, -180, 15, 200, 0.4, 0.01, 0.05, 0.2, 0.2, 0.2, 40);
    pros::delay(2000);
    driveBackward(26, 14, -180,24,1.1,0,0,0.1,0.2,0.3,70);
    pros::delay(400);

    turnLeft(-4,0,25,85,80);
  //  smartStop(5, 10, 300, false);
  pros::delay(400);
  smartStraight(50, 48, -4, 24, 150, 0.6, 0.01, 0.05, 0.2, 0.2, 0.2, 50);
    score(320000, 100);
    driveBackward(5, -4, -180,24,1.1,0,0,0.1,0.2,0.3,70);


    matchloadPneumaticStart(500, 0, true);   // drop piston, hold 500ms, retract
intakeHopperStart(500, 100, 0, true);    // runs intake at same time

    turnLeft(-330,0,25,100,20);
    pros::delay(200);
    visionDriveMinimal(// not bad for first 3 center balls
        aiVision_blueCube,
        100,
        0.0,
        24.0, 75.0,
        pros::E_MOTOR_BRAKE_HOLD,
        .1, 0.0, 0.0,
        1.75,
        1.50, 0.0, 0.0
    );
}

void soloAwpOdom(){
    setStartPosition(0, 0, 0);
    startOdometryTask();
    startCoordinateFinder();
    initializeOpticalSensor();
}

void provsAuto(){
    setStartPosition(0, 36, 0);
    startOdometryTask();
    leftGatePneumatics.set_value(true);
    rightGatePneumatics.set_value(true);
   // startCoordinateFinder();
    initializeOpticalSensor();
    leftLaneOptical.set_led_pwm(100);
    rightLaneOptical.set_led_pwm(100);
    rudderPneumatics.set_value(false);

    turnRight(-42,36);
    smartStop(5, 5, 200, false);
    //intakeColourStart(1000, 100, false, false);
    matchloadPneumaticStart(9000, 0, true);
    visionDriveMinimal(
        aiVision_redCube,
        100,
        0.0,
        24.0, 65.0,
        pros::E_MOTOR_BRAKE_COAST,
        .5, 0.0, 0.0,
        1.2,
        1.0, 0.0, 0.0
    );
    intakeStart(1500, 100, false, true);
    smartStraight(30, 15, -90, 15, 200, 0.4, 0.0, 0.0, 0.2, 0.2, 0.2, 50);

    matchloadPneumaticStop();
    turnLeft(87,50,25,50,4);
    smartStop(5, 5, 400, false);
    smartStraight(15, 12, 87, 10, 10, 0.4, 0.01, 0.05, 0.2, 0.2, 0.2, 30);
    rightScore(3000, 100);
    turnLeft(140,30,20,80);

    visionDriveMinimal(
        aiVision_redCube,
        100,
        0.0,
        24.0, 65.0,
        pros::E_MOTOR_BRAKE_COAST,
        .5, 0.0, 0.0,
        1.2,
        1.0, 0.0, 0.0
    );
}

void provsAutoLeft(){
     setStartPosition(0, 36, 180);
    startOdometryTask();
    leftGatePneumatics.set_value(true);
    rightGatePneumatics.set_value(true);
   // startCoordinateFinder();
    initializeOpticalSensor();
    leftLaneOptical.set_led_pwm(100);
    rightLaneOptical.set_led_pwm(100);
    rudderPneumatics.set_value(false);
    //*********************************************************
    //Stop 1: South East MatchLoader (6 Blocks + 1 Preload)
    //********************************************************* */
    turnLeft(267,36);
    smartStop(5, 5, 200, false);
    //intakeColourStart(1000, 100, false, false);
    matchloadPneumaticStart(9000, 0, true);
    visionDriveMinimal(
        aiVision_blueCube,
        100,
        0.0,
        24.0, 65.0,
        pros::E_MOTOR_BRAKE_COAST,
        .5, 0.0, 0.0,
        1.2,
        1.0, 0.0, 0.0
    );
    intakeStart(1500, 100, false, true);
    smartStraight(30, 15, -90, 15, 200, 0.4, 0.0, 0.0, 0.2, 0.2, 0.2, 50);

    matchloadPneumaticStop();
    turnRight(93,50,25,50,4);
    smartStop(5, 5, 400, false);
    smartStraight(15, 12, 93, 10, 10, 0.4, 0.01, 0.05, 0.2, 0.2, 0.2, 30);
    rightScore(3000, 100);
}

void visionDemo(){
    followRedBall();
    pros::delay(10000);
    stopFollowing();
}
