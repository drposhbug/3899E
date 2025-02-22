#include "pid.h"          // Include the PID header file
#include "vex.h"          // Include the VEX library
#include "utils.h"        // Include the utility header for normalizeHeading
#include "robot_config.h" // Include the robot configuration
#include "navigation.h"
#include "odometry.h"
#include "vision_tracking.h"
#include <cmath> // Include math library for M_PI

using namespace vex; // Use the VEX namespace

// Example autonomous routine 1
void autonRoutine1()
{ // Red Safe

    // arm score
    // armMotor.spinFor(ScoringAlliance, rotationUnits::deg, 100, velocityUnits::pct);

    // back up and grab mobile goal
    pidStraightDistanceABS(1, -75, 50, .15, 0, 0, 1, 0, 0, 15, 20);
    goalPneumatics.set(true);
    // armMotor.spinFor(Starting, rotationUnits::deg, 100, velocityUnits::pct);

    // go and turn towards blue ring on bottom and red ring on top knock it off with the doinker
    pidStraightDistanceABS(0, 50, 50, .10, 0, 0, 1, 0, 0, 15, 15);
    doinkerPneumatics.set(true);
    spotTurn(-80, 10, 2, 0.4, 0, 0.05);
    doinkerPneumatics.set(false);
    intakeMotor.spin(directionType::rev, 12, voltageUnits::volt);
    pidStraightDistanceABS(-79, 40, 50, .15, 0, 0, 1, 0, 0, 15, 15);

    // back up towards to stack with red on the bottom and blue on top
    pidStraightDistanceABS(-79, -70, 50, .15, 0, 0, 1, 0, 0, 15, 15);
    // armMotor.spinToPosition(22, rotationUnits::deg, 100, velocityUnits::pct);
    spotTurn(170, 50, 2, 0.4, 0, 0.05);
    pidStraightDistanceABS(170, 70, 70, .15, 0, 0, 1, 0, 0, 15, 15);
    spotTurn(-80, 50, 2, 0.4, 0, 0.05);
    pidStraightDistanceABS(-80, 75, 30, .15, 0, 0, 1, 0, 0, 15, 15);
}

void autonRoutine2()
{ // Blue Safe Route

    // score alliance stake
    // armMotor.spinFor(ScoringAlliance, rotationUnits::deg, 100, velocityUnits::pct);

    // backup and grab mobile goal
    pidStraightDistanceABS(-1, -75, 50, .15, 0, 0, 1, 0, 0, 15, 20);
    goalPneumatics.set(true);
    // armMotor.spinFor(Starting, rotationUnits::deg, 100, velocityUnits::pct);
    /**/
    // drive towards and grab stack with blue ring on top and red ring on the bottom using the doinker
    pidStraightDistanceABS(0, 50, 50, .10, 0, 0, 1, 0, 0, 15, 15);
    doinkerPneumatics.set(true);
    spotTurn(68, 7, 2, 0.4, 0, 0.05);
    doinkerPneumatics.set(false);
    intakeMotor.spin(directionType::rev, 12, voltageUnits::volt);
    pidStraightDistanceABS(68, 40, 50, .15, 0, 0, 1, 0, 0, 15, 15);

    // grab stack with blue ring on the bottom and red ring on top
    pidStraightDistanceABS(79, -70, 50, .15, 0, 0, 1, 0, 0, 15, 15);
    spotTurn(-170, 50, 2, 0.4, 0, 0.05);
    pidStraightDistanceABS(-170, 70, 70, .15, 0, 0, 1, 0, 0, 15, 15);
    vex::task::sleep(1000);
    spotTurn(80, 50, 2, 0.4, 0, 0.05);
    pidStraightDistanceABS(80, 75, 30, .15, 0, 0, 1, 0, 0, 15, 15);
}

void autonRoutine3()
{
    MotorControlParams intakeParams;
    intakeParams.targetMotor = &intakeMotor;
    intakeParams.DelayStart = 200; // 0.5 seconds
    intakeParams.OnTime = 1300;    // 2 seconds
    intakeParams.dir = directionType::rev;

    // intakeMotor.spin(reverse, 12, voltageUnits::volt);
    thread intakeTask(MotorControlThread, &intakeParams);
    pidStraightDistanceLaunch(0, 94, 60, 0.4, 0, 0, 0.09, 0, 0, 10, brakeType::brake);
    spotTurn(90, 100, 2, 0.3, 0, 0.01);
    pidStraightDistanceLaunch(90, -55, 32, 0.5, 0, 0, 0.15, 0, 0, 20, brakeType::coast);
    // pidStraightDistanceLaunch(270, -50, 50, 0.4, .02, .5, 0.09, 0, 0, 20, brakeType::coast);
    isGoalPneumaticsActive = !isGoalPneumaticsActive;
    goalPneumatics.set(isGoalPneumaticsActive);
    spotTurn(210, 50, 2, 0.3, 0, 0.01);
    intakeMotor.spin(reverse, 12, voltageUnits::volt);
    vexDelay(1000);
    intakeMotor.stop();
    armMotor.spinToPosition(Alliance, rotationUnits::deg, 100, velocityUnits::pct, false);
    task::sleep(150);
    isElbowPneumaticsActive = !isElbowPneumaticsActive;
    elbow1Pneumatics.set(isElbowPneumaticsActive);
    elbow2Pneumatics.set(isElbowPneumaticsActive);
    armstat = ArmPosition::Alliance;

    pidStraightDistanceLaunch(210, 95, 60, 0.5, 0, 0, 0.09, 0, 0, 15, brakeType::brake);
    armMotor.spinToPosition(Load1, rotationUnits::deg, 100, velocityUnits::pct, false);
    vexDelay(500);
    pidStraightDistanceLaunch(210, -30, 90, 0.4, 0, 0, 0.15, 0, 0, 10, brakeType::brake);
}

// Example autonomous routine 4
void autonRoutine4()
{
    pidStraightDistanceABS(0, 75, 30, .15, 0, 0, 1, 0, 0, 15, 15);
}

void autonRoutine5()
{

    pidStraightDistanceLaunch(0, 96, 70, 0.4, 0, 0, 0.09, 0, 0, 10, brakeType::brake);
    spotTurn(275, 100, 2, 0.3, 0, 0.01);
    pidStraightDistanceLaunch(275, -35, 70, 0.4, .02, .5, 0.09, 0, 0, 12, brakeType::brake);
}

void autonRoutine6()
{
    // double startingHeading = 0;
    // InertialSensor.setHeading(startingHeading, rotationUnits::deg);
    armMotor.spinToPosition(500, rotationUnits::deg, 100, velocityUnits::pct, true);
    straight(-50, 100, 0, 20); // use this one
    armMotor.spinToPosition(-50, rotationUnits::deg, 100, velocityUnits::pct, false);
    spotTurn(55, 70, 2, 0.4, 0, 0.05);
    straight(-55, 100, 55, 40); // use this one
    goalPneumatics.set(true);
    spotTurn(150, 70, 2, 0.4, 0, 0.05);
    intakeMotor.spinFor(-8000, rotationUnits::deg, 100, velocityUnits::pct, false);
    vex::task::sleep(500);
    straight(70, 100, 150, 30); // use this one
    goalPneumatics.set(false);
    spotTurn(76, 70, 2, 0.4, 0, 0.05);
    straight(-39, 30, 76, 10);
    goalPneumatics.set(true);
}

void autonRoutine7()
{
    const double RED_HUE_MIN_1 = 340.0; // First red range (340°-360°)
    const double RED_HUE_MAX_1 = 360.0;
    const double RED_HUE_MIN_2 = 0.0; // Second red range (0°-15°)
    const double RED_HUE_MAX_2 = 15.0;
    const double BLUE_HUE_MIN = 215.0; // Blue range
    const double BLUE_HUE_MAX = 225.0;

    initializeOpticalSensor();

    // Define task parameters
    ColorTaskParams colorTaskParams;
    colorTaskParams.isRunning = true;
    colorTaskParams.targetColor = Color::BLUE; // Set ejection colour RED or BLUE
    colorTaskParams.delayMs = 80;              // Set delay before stopping intake

    // Start the color detection task
    vex::task colorTask(colorDetectionTask, &colorTaskParams);

    straight(-70, 40); // use this one

    goalPneumatics.set(true);
    task::sleep(1000); // Small delay to prevent overwhelming the CPU
    intakeMotor.spin(reverse, 100, velocityUnits::pct);
    task::sleep(1000); // Small delay to prevent overwhelming the CPU
    task::sleep(3000); // Small delay to prevent overwhelming the CPU
}

void autonRoutine8()
{

    const double RED_HUE_MIN_1 = 340.0; // First red range (340°-360°)
    const double RED_HUE_MAX_1 = 360.0;
    const double RED_HUE_MIN_2 = 0.0; // Second red range (0°-15°)
    const double RED_HUE_MAX_2 = 15.0;
    const double BLUE_HUE_MIN = 215.0; // Blue range
    const double BLUE_HUE_MAX = 225.0;

    initializeOpticalSensor();

    // Define task parameters
    ColorTaskParams colorTaskParams;
    colorTaskParams.isRunning = true;
    colorTaskParams.targetColor = Color::RED; // Set ejection colour RED or BLUE
    colorTaskParams.delayMs = 80;             // Set delay before stopping intake

    // Start the color detection task
    vex::task colorTask(colorDetectionTask, &colorTaskParams);

    // intakeMotor.spinFor(-200000, rotationUnits::deg, 100, velocityUnits::pct, false);
    armMotor.spinToPosition(400, rotationUnits::deg, 80, velocityUnits::pct, false);
    // Score Alliance
    straight(113, 70); // use this one
    armMotor.spinToPosition(590, rotationUnits::deg, 100, velocityUnits::pct, false);
    straight(-30, 10); // use this one
    armMotor.spinToPosition(-590, rotationUnits::deg, 100, velocityUnits::pct, false);

    turn(-90, 50, 20);
    straight(-60, 30); // use this one
    goalPneumatics.set(true);
    intakeMotor.spin(reverse, 100, velocityUnits::pct);
    turn(25, 10, 20);
    straight(85, 30); // use this one
}

void autonRoutine9()
{

    const double RED_HUE_MIN_1 = 340.0; // First red range (340°-360°)
    const double RED_HUE_MAX_1 = 360.0;
    const double RED_HUE_MIN_2 = 0.0; // Second red range (0°-15°)
    const double RED_HUE_MAX_2 = 15.0;
    const double BLUE_HUE_MIN = 215.0; // Blue range
    const double BLUE_HUE_MAX = 225.0;

    initializeOpticalSensor();

    // Define task parameters
    ColorTaskParams colorTaskParams;
    colorTaskParams.isRunning = true;
    colorTaskParams.targetColor = Color::RED; // Set ejection colour RED or BLUE
    colorTaskParams.delayMs = 80;             // Set delay before stopping intake

    // Start the color detection task
    vex::task colorTask(colorDetectionTask, &colorTaskParams);

    // intakeMotor.spinFor(-200000, rotationUnits::deg, 100, velocityUnits::pct, false);

    // Score Alliance
    straight(10, 5); // use this one
    armMotor.spinToPosition(570, rotationUnits::deg, 100, velocityUnits::pct, true);
    intakeMotor.spinFor(2000, rotationUnits::deg, 100, velocityUnits::pct, false);

    // Get Mobile Goal
    straight(-85, 40, 35, -5, 0.6);
    goalPneumatics.set(true);

    // Backing up to Border
    armMotor.spinToPosition(-70, rotationUnits::deg, 100, velocityUnits::pct, false);
    intakeMotor.spin(reverse, 100, velocityUnits::pct);

    turn(-105, 60, 17);
    straight(30, 10, 40);
    turn(-90, 60, 17);
    straight(30, 20, 30);
    straight(-10, 5, 30);
    turn(20, 10, 17);
    straight(10, 10, 30);
}

void autonRoutine10()
{

    const double RED_HUE_MIN_1 = 340.0; // First red range (340°-360°)
    const double RED_HUE_MAX_1 = 360.0;
    const double RED_HUE_MIN_2 = 0.0; // Second red range (0°-15°)
    const double RED_HUE_MAX_2 = 15.0;
    const double BLUE_HUE_MIN = 215.0; // Blue range
    const double BLUE_HUE_MAX = 225.0;

    initializeOpticalSensor();

    // Define task parameters
    ColorTaskParams colorTaskParams;
    colorTaskParams.isRunning = true;
    colorTaskParams.targetColor = Color::BLUE; // Set ejection colour RED or BLUE
    colorTaskParams.delayMs = 80;              // Set delay before stopping intake

    // Start the color detection task
    vex::task colorTask(colorDetectionTask, &colorTaskParams);

    // intakeMotor.spinFor(-200000, rotationUnits::deg, 100, velocityUnits::pct, false);

    // Score Alliance
    straight(13, 5); // use this one
    armMotor.spinToPosition(570, rotationUnits::deg, 100, velocityUnits::pct, true);
    intakeMotor.spinFor(2000, rotationUnits::deg, 100, velocityUnits::pct, false);

    // Get Mobile Goal
    straight(-70, 43, 30, 6, 0.6);
    goalPneumatics.set(true);

    // Backing up to Border
    armMotor.spinToPosition(-70, rotationUnits::deg, 100, velocityUnits::pct, false);
    turn(120, 75, 17);
    straight(50, 5, 18);
    intakeMotor.spin(reverse, 100, velocityUnits::pct);
}

// Red Side Alliance
void autonRoutine11()
{

    // Idea 100 cm travel
    // backward(100,50);
    // backward(50, 30, 17, 0, 1.6);
    // straightOdometry(100, 50);
    // straightOdometry(50, 30);

    wait(2000, msec);
    straightOdometry(17, 9);
    armMotor.spinToPosition(570, rotationUnits::deg, 100, velocityUnits::pct, true);
    backward(-90, 48, 25, -3, 1.8);
    // backward(273,48,25,-3, 1.8);
    goalPneumatics.set(true);
    armMotor.spinToPosition(-40, rotationUnits::deg, 100, velocityUnits::pct, true);
    intakeMotor.spin(reverse, 100, velocityUnits::pct);

    turnOdometry(-100, 75, 25);

    straightOdometry(30, 20, 17, -100, 0.3);
    turnOdometry(-140, 130, 25);
    goalPneumatics.set(false);

    /*
    doinkerPneumatics.set(true);
    backward(-47,20,17,95,2.2);
    turnOdometry(140, 180, 25);
    intakeMotor.spin(reverse, 100, velocityUnits::pct);
    doinkerPneumatics.set(false);
    turnOdometry(168, 195, 20);
    //turnOdometry(235, 195, 25);
    straightOdometry(35, 25,17,168,0.4);
    //backward(-10,5,17,55,0);
    //InertialSensor.resetRotation();  // Resets continuous rotation counting
    //InertialSensor.resetHeading();
    //wait(1000,msec);
    //turn360(-104, 70, 25);
    //straight360(100,50, 18);
    //goalPneumatics.set(false);

    */

    // wait(1000,msec);

    // turnOdometry(30, 290,20);
    // wait(2000,msec);
    // straightOdometry(90, 50,17, 275);

    // forwardToPoint(100, 0, 50);
    // backwardToPoint(20, 0, 50, 17, 0.4, 0, 0, 0.4, 0.25, 0.25, -100);
    /*
    //Non-Odom turn test
    turnOdometry(90, 75, 15, 100);
    turnOdometry(180, 75, 15, 100);
    turnOdometry(270, 75, 15, 100);
    turnOdometry(360, 75, 15, 100);
    turnOdometry(270, 75, 15, 100);
    turnOdometry(180, 75, 15, 100);
    turnOdometry(90, 75, 15, 100);
    turnOdometry(0, 75, 15, 100);
    wait (3000,msec);
    */

    /*
    //auton Route Red
    forwardToPoint(23,0,7);
    backwardToPoint(-60,0,45);
    wait(1000,msec);
    */
    // backward(-50, 40, 17, 345);
    // wait(1000,msec);
    // backward(-50, 40, 17, 0);
    // wait(1000,msec);
    // backward(-50, 40, 17, 20);

    // turnOdometry(180, 50, 15, 100);
    // turnOdometry(270, 75, 15, 100);
    // turnOdometry(359, 100, 15, 100);

    // forwardToPoint(40, 0, 50);
    // rightToPoint (40,100, 20);
    // forwardToPoint(40, 0, 50);

    /*
    straightOdometry(120, 50, 17, 0);
    turnOdometry(180, 80, 17, 17);
    wait(1000,msec);
    straightOdometry(80, 50, 17, 180);
    turnOdometry(270, 80, 17, 17);

    //straightOdometry(100, 80, 17);
    */
}

// Blue Side Aliiance
void autonRoutine12()
{
    straightOdometry(17.2, 9, 17);
    armMotor.spinToPosition(570, rotationUnits::deg, 100, velocityUnits::pct, true);
    backward(-72, 40, 25, 3, 2.1); // Changed -3 to 3 (relative)
    goalPneumatics.set(true);
    armMotor.spinToPosition(-40, rotationUnits::deg, 100, velocityUnits::pct, true);
    turnOdometry(-125, 95, 20);              // Changed 93 to -93 (relative)
    straightOdometry(58, 35, 17, -125, 0.3); // Changed 93 to -93 (relative)
    doinkerPneumatics.set(true);
    wait(500, msec);
    backward(-47, 20, 17, -125); // Changed 95 to -95 (relative)
    turnOdometry(-200, 180, 25); // Changed 140 to 220 (absolute, 360-140=220)
    intakeMotor.spin(reverse, 100, velocityUnits::pct);
    doinkerPneumatics.set(false);
    turnOdometry(-189, 180, 35); // Changed 168 to 192 (absolute, 360-168=192)
    // turnOdometry(125, 195, 25);  // Changed 235 to 125 (absolute, 360-235=125)
    straightOdometry(40, 25, 17, -189, 0.2); // Changed 168 to 192 (matching above absolute heading)
                                             // backward(-35,20,17,-189);  // Changed 95 to -95 (relative)
    turnOdometry(-230, 230, 20);
    straightOdometry(35, 25, 17, -230, 0.2); // Changed 168 to 192 (matching above absolute heading)
}

// Red Side No Alliance
void autonRoutine13()
{

    // Idea 100 cm travel
    // backward(100,50);
    // backward(50, 30, 17, 0, 1.6);
    // straightOdometry(100, 50);
    // straightOdometry(50, 30);

    wait(2000, msec);

    backward(-73, 48, 25, -3, 1);
    goalPneumatics.set(true);
    // armMotor.spinToPosition(-40, rotationUnits::deg, 100, velocityUnits::pct, true);
    turnOdometry(93, 75, 25);
    straightOdometry(46, 20, 17, 93, 0.3);
    doinkerPneumatics.set(true);
    backward(-47, 20, 17, 95, 2.2);
    turnOdometry(140, 180, 25);
    intakeMotor.spin(reverse, 100, velocityUnits::pct);
    doinkerPneumatics.set(false);
    turnOdometry(168, 195, 20);
    // turnOdometry(235, 195, 25);
    straightOdometry(35, 25, 17, 168, 0.4);
    // backward(-10,5,17,55,0);
    // InertialSensor.resetRotation();  // Resets continuous rotation counting
    // InertialSensor.resetHeading();
    // wait(1000,msec);
    // turn360(-104, 70, 25);
    // straight360(100,50, 18);
    // goalPneumatics.set(false);

    // wait(1000,msec);

    // turnOdometry(30, 290,20);
    // wait(2000,msec);
    // straightOdometry(90, 50,17, 275);

    // forwardToPoint(100, 0, 50);
    // backwardToPoint(20, 0, 50, 17, 0.4, 0, 0, 0.4, 0.25, 0.25, -100);
    /*
    //Non-Odom turn test
    turnOdometry(90, 75, 15, 100);
    turnOdometry(180, 75, 15, 100);
    turnOdometry(270, 75, 15, 100);
    turnOdometry(360, 75, 15, 100);
    turnOdometry(270, 75, 15, 100);
    turnOdometry(180, 75, 15, 100);
    turnOdometry(90, 75, 15, 100);
    turnOdometry(0, 75, 15, 100);
    wait (3000,msec);
    */

    /*
    //auton Route Red
    forwardToPoint(23,0,7);
    backwardToPoint(-60,0,45);
    wait(1000,msec);
    */
    // backward(-50, 40, 17, 345);
    // wait(1000,msec);
    // backward(-50, 40, 17, 0);
    // wait(1000,msec);
    // backward(-50, 40, 17, 20);

    // turnOdometry(180, 50, 15, 100);
    // turnOdometry(270, 75, 15, 100);
    // turnOdometry(359, 100, 15, 100);

    // forwardToPoint(40, 0, 50);
    // rightToPoint (40,100, 20);
    // forwardToPoint(40, 0, 50);

    /*
    straightOdometry(120, 50, 17, 0);
    turnOdometry(180, 80, 17, 17);
    wait(1000,msec);
    straightOdometry(80, 50, 17, 180);
    turnOdometry(270, 80, 17, 17);

    //straightOdometry(100, 80, 17);
    */
}

// Blue Side Aliiance
void autonRoutine14()
{
    // straightOdometry(17.2, 9,17);
    // armMotor.spinToPosition(570, rotationUnits::deg, 100, velocityUnits::pct, true);
    // backward(-56,40,25,3, 2.1);  // Original route Kp
    wait(2000, msec);
    backward(-52, 40, 25, 3, 1.7); // Changed -3 to 3 (relative)
    goalPneumatics.set(true);
    // armMotor.spinToPosition(-40, rotationUnits::deg, 100, velocityUnits::pct, true);
    turnOdometry(-126, 95, 20);              // Changed 93 to -93 (relative)
    straightOdometry(58, 35, 17, -125, 0.3); // Changed 93 to -93 (relative)
    doinkerPneumatics.set(true);
    wait(500, msec);
    backward(-47, 20, 17, -125); // Changed 95 to -95 (relative)
    turnOdometry(-200, 180, 25); // Changed 140 to 220 (absolute, 360-140=220)
    intakeMotor.spin(reverse, 100, velocityUnits::pct);
    doinkerPneumatics.set(false);
    turnOdometry(-189, 180, 35); // Changed 168 to 192 (absolute, 360-168=192)
    // turnOdometry(125, 195, 25);  // Changed 235 to 125 (absolute, 360-235=125)
    straightOdometry(40, 25, 17, -189, 0.2); // Changed 168 to 192 (matching above absolute heading)
                                             // backward(-35,20,17,-189);  // Changed 95 to -95 (relative)
    turnOdometry(-230, 230, 20);
    straightOdometry(35, 25, 17, -230, 0.2); // Changed 168 to 192 (matching above absolute heading)
}
