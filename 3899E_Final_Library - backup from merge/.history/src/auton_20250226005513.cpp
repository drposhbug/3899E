#include "pid.h"          // Include the PID header file
#include "vex.h"          // Include the VEX library
#include "utils.h"        // Include the utility header for normalizeHeading
#include "robot_config.h" // Include the robot configuration
#include "navigation.h"
#include "odometry.h"
#include <cmath> // Include math library for M_PI

using namespace vex; // Use the VEX namespace

// 180 Degree Turns
//rightMP(180,110,18,100); 
//leftMP(180,110,18,100);

//135 Degree Turns
//rightMP(135,88,17,100);
//leftMP(135,88,17,100);

//90 Degree Turns
//rightMP(90,67,18,100); 
//leftMP(90,67,18,100);

//45 Degree Turns
//leftMP(45,35,15,100);
//rightMP(45,35,15,100);

//straight 50
//forwardMP(50, 35, 0);

//Straignt 100    

//forwardMP(100, 50, 0, 16, 1.5, 0.03, 0.1);

void autonDoubleDoinker()
{
    const double RED_HUE_MIN_1 = 350.0; // First red range
const double RED_HUE_MAX_1 = 360.0;
const double RED_HUE_MIN_2 = 0.0;   // Second red range
const double RED_HUE_MAX_2 = 18.0;  // Reduced from 30
const double BLUE_HUE_MIN = 207.0;  // Blue range - narrower
const double BLUE_HUE_MAX = 230.0;  // Reduced from 240

    initializeOpticalSensor();

    // Define task parameters
    ColorTaskParams colorTaskParams;
    colorTaskParams.isRunning = true;
    colorTaskParams.targetColor = Color::BLUE; // Set ejection colour RED or BLUE
    colorTaskParams.delayMs = 80;              // Set delay before stopping intake

    // Start the color detection task
    vex::task colorTask(colorDetectionTask, &colorTaskParams);
//forwardMP(100, 50, 0, 16, 1.5, 0.03, 0.1);


}





void calibration()
{
//leftMP(135,88,18,100);
backwardMP(100, 49, 0, 17, 1.6, 0.03, 0.1);

}

void autonRoutine1()
{
    backwardMP(100, 48, 0, 16, 1.9, 0.01, 0.1);


/*
    straightOdometry(50, 20, 17, 0, 0.4);
    wait(1000,msec);
    straightOdometry(50, 20, 17, 20, 0.4);
    wait(1000,msec);
    straightOdometry(50, 20, 17, 0, 0.4);
    wait(1000,msec);
    straightOdometry(50, 20, 17, 340, 0.4);
    //straightOdometry(50, 20, 17, -30, 0.7);
*/
/*
forwardMP(25, 20, 17, 0, 0.4);

rightMP(90,50);



leftMP(90,50);
wait(100,msec);
forwardMP(25, 20, 17, 90, 0);
leftMP(180,50);
wait(100,msec);
forwardMP(25, 20, 17, 180, 0);
leftMP(270,50);
wait(100,msec);
forwardMP(25, 20, 17, 270, 0);
leftMP(360,50);
wait(100,msec);
forwardMP(25, 20, 17, 360, 0);

rightMP(270,50);
wait(100,msec);
forwardMP(25, 20, 17, 270, 0);
rightMP(180,50);
wait(100,msec);
forwardMP(25, 20, 17, 180, 0);
rightMP(90,50);
wait(100,msec);
forwardMP(25, 20, 17, 90, 0);
rightMP(0,50);
wait(100,msec);
forwardMP(25, 20, 17, 0, 0);

leftMP(90,50);
wait(100,msec);
backwardMP(25, 20, 17, 90, 0);
leftMP(180,50);
wait(100,msec);
backwardMP(25, 20, 17, 180, 0);
leftMP(270,50);
wait(100,msec);
backwardMP(25, 20, 17, 270, 0);
leftMP(360,50);
wait(100,msec);
backwardMP(25, 20, 17, 360, 0);

rightMP(270,50);
wait(100,msec);
backwardMP(25, 20, 17, 270, 0);
rightMP(180,50);
wait(100,msec);
backwardMP(25, 20, 17, 180, 0);
rightMP(90,50);
wait(100,msec);
backwardMP(25, 20, 17, 90, 0);
rightMP(0,50);
wait(100,msec);
backwardMP(25, 20, 17, 0, 0);
*/

/*
turnOdometry(90,70);
wait(100,msec);
straightOdometry(-25, 20, 15, 90, 0);
turnOdometry(180,70);
wait(100,msec);
straightOdometry(-25, 20, 15, 180, 0);
turnOdometry(270,70);
wait(100,msec);
straightOdometry(-25, 20, 15, 270, 0);
turnOdometry(360,70);
wait(100,msec);
straightOdometry(-25, 20, 15, 360, 0);
//turnOdometry(180,60);
//wait(1000,msec);
//straightOdometry(180, 20, 17, 0, 0.4);
//turnOdometry(270,60);
//wait(1000,msec);
//straightOdometry(270, 20, 17, 0, 0.4);
//turnOdometry(360,60);
//wait(1000,msec);
//straightOdometry(360, 20, 17, 0, 0.4);
*/


/*
wait(1000,msec);
wait(1000,msec);
turnOdometry(270,60);
wait(1000,msec);
turnOdometry(180,60);
wait(1000,msec);
turnOdometry(0,20);
wait(1000,msec);
turnOdometry(0,60);
*/
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

    //wait(2000, msec);

    headingOffset = 250;
    //go to Alliance Stake
    straightOdometry(17, 9);
    armMotor.spinToPosition(570, rotationUnits::deg, 100, velocityUnits::pct, true);
    //wait(200, msec);

    //Backup to Pick up Mobile Goal
    backward(-95, 58, 25, -3, 1);
    // backward(273,48,25,-3, 1.8);
    goalPneumatics.set(true);
    armMotor.spinToPosition(-40, rotationUnits::deg, 100, velocityUnits::pct, true);
    intakeMotor.spin(reverse, 100, velocityUnits::pct);
    //wait(2000, msec);

    //Turn to go into Tower
    turnOdometry(100, 55);
      //  wait(2000, msec);
    straightOdometry(40, 20, 17, 100);
    //wait(2000, msec);
    doinkerPneumatics.set(true);

    //Coming out of Tower
    straightOdometry(-40, 20, 17, 90, 0.3);
   

    //Turn to ring stack
     turnOdometry(200, 80);
    goalPneumatics.set(false);
    straightOdometry(50, 10, 17, 200, 0.3);
    //wait(2000, msec);
    doinkerPneumatics.set(false);

    //Turn to corner

     turnOdometry(315, 80);
    intakeMotor.spin(reverse, 100, velocityUnits::pct);
    doinkerPneumatics.set(false);
    //turnOdometry(168, 195, 20);
    //turnOdometry(235, 195, 25);
    straightOdometry(100, 30,17,315);
    //backward(-10,5,17,55,0);
    //InertialSensor.resetRotation();  // Resets continuous rotation counting
    //InertialSensor.resetHeading();
    //wait(1000,msec);
    //turn360(-104, 70, 25);
    //straight360(100,50, 18);
    //goalPneumatics.set(false);

    turnOdometry(250, 0, 60);

    

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
