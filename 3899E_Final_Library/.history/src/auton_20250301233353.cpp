#include "pid.h"          // Include the PID header file
#include "vex.h"          // Include the VEX library
#include "utils.h"        // Include the utility header for normalizeHeading
#include "robot_config.h" // Include the robot configuration
#include "navigation.h"
#include "odometry.h"
#include <cmath> // Include math library for M_PI

using namespace vex; // Use the VEX namespace

ArmResetTaskParams armResetParams;


// 180 Degree Turns
//rightMP(180,110,18); 
//leftMP(180,110,18);

//135 Degree Turns
//rightMP(135,88,17);
//leftMP(135,88,17);

//90 Degree Turns
//rightMP(90,67,18); 
//leftMP(90,67,18);

//45 Degree Turns
//leftMP(45,35,15);
//rightMP(45,35,15);

//straight 50
//forwardMP(50, 35, 0);
//backwardMP(50, 33, 0, 16, 1.5, 0.0008, 0.5, 0.35);

//Straignt 100    
//forwardMP(100, 50, 0, 16, 1.5, 0.03, 0.1);
//backwardMP(100, 47, 0, 20, 1.5, 0.0008, 0.5, 0.35);

void doubleDoinkerRed()
{
const double RED_HUE_MIN_1 = 350.0; // First red range
const double RED_HUE_MAX_1 = 360.0;
const double RED_HUE_MIN_2 = 0.0;   // Second red range
const double RED_HUE_MAX_2 = 18.0;  // Reduced from 30
const double BLUE_HUE_MIN = 207.0;  // Blue range - narrower
const double BLUE_HUE_MAX = 230.0;  // Reduced from 240

    initializeOpticalSensor();
/*
    // Define task parameters
    ColorTaskParams colorTaskParams;
    colorTaskParams.isRunning = false;
    colorTaskParams.targetColor = Color::BLUE; // Set ejection colour RED or BLUE
    colorTaskParams.delayMs = 80;              // Set delay before stopping intake
    // Start the color detection task
    vex::task colorTask(colorDetectionTask, &colorTaskParams);
*/




    headingOffset = 240;
    
    
    //go to Alliance Stake
   // moveArm(ArmPosition::Alliance, -95,00);

    //forwardMP(18, 9, 240);

    //wait(400, msec);

    //Backup to Pick up Mobile Goal
    backwardMP(70, 50, 244, 20);
    armMotor1.spinToPosition(550, rotationUnits::deg, 100, velocityUnits::pct, false);
    armMotor2.spinToPosition(550, rotationUnits::deg, 100, velocityUnits::pct, false);
    // backward(273,48,25,-3, 1.8);
    goalPneumatics.set(true);
    armResetParams.isRunning = true;
    armResetParams.isResetComplete = false; 
    //armMotor1.spinToPosition(100, rotationUnits::deg, 100, velocityUnits::pct, false);
    //armMotor2.spinToPosition(100, rotationUnits::deg, 100, velocityUnits::pct, true);
    //intakeMotor.spin(reverse, 100, velocityUnits::pct);
    //wait(2000, msec);

    //Turn to go into Tower
    rightMP(138,65, 20);
      
     forwardMP(43, 27, 138);
    
     //waitForButtonPress();
        doinkerPneumaticsRight.set(true);
           wait(200, msec);
    pivotRightMP(113, 13, 20, 100);
    

    doinkerPneumaticsLeft.set(true);
    wait(300, msec);
backwardMP(110, 56, 113, 21, 1.5, 0.0008, 0.5, 0.35);
intakeMotor.spinFor(forward, 0.5, rotationUnits::rev, 100, velocityUnits::pct, true);
armMotor1.spinToPosition(180, rotationUnits::deg, 100, velocityUnits::pct, false);
armMotor2.spinToPosition(180, rotationUnits::deg, 100, velocityUnits::pct, false);
    doinkerPneumaticsLeft.set(false);
    doinkerPneumaticsRight.set(false);
    intakeMotor.spin(reverse, 100, velocityUnits::pct);

    leftMP(143,15, 20);

    forwardMP(45,20, 143,20,0.8);


    rightMP(26,70, 20);
  // startIntakeStallDetection();
 //   moveArm(ArmPosition::Load1, -82, 1000);
   // forwardMP(98, 50, 26, 16, 1.5, 0.03, 0.1);


    forwardMP(100,60, 26, 23,1);
    intakeMotor.spin(reverse,  0, velocityUnits::pct);

    rightMP(298,35, 20);
    intakeMotor.spin(reverse, 100, velocityUnits::pct);
    
    doinkerPneumaticsLeft.set(true);
    forwardMP(85,35, 298, 25,1);
    rightMP(230, 0, 40);
    doinkerPneumaticsLeft.set(false);
    forwardMP(50,12, 235, 25,1);

    //forwardMP(85,60, 298, 25,1);

    //backwardMP(5,2, 298, 20);



    //leftMP(170, 80, 18);
    //forwardMP(40,60, 170, 22,0.4);
    /// goalPneumatics.set(false);

    //leftMP(95, 45, 18);
  // forwardMP(10,6, 100, 20,0.4);
  //  pivotRightMP(58, 5, 20, 100);

 //   armPneumatics.set(true);
//wait(100, msec);
    //armMotor1.spinToPosition(Side-250, rotationUnits::deg, 100, velocityUnits::pct, false);
  //armMotor2.spinToPosition(Side-250, rotationUnits::deg, 100, velocityUnits::pct, true);
//wait(300, msec);
  //  armPneumatics.set(false);

  //  armMotor1.spinToPosition(Alliance-50, rotationUnits::deg, 100, velocityUnits::pct, false);
    //armMotor2.spinToPosition(Alliance-50, rotationUnits::deg, 100, velocityUnits::pct, false);
   // backwardMP(10, 0, 46, 20, 1);
    LeftMotor1.setBrake(brakeType::coast);
    LeftMotor2.setBrake(brakeType::coast);
    LeftMotor3.setBrake(brakeType::coast);
    RightMotor1.setBrake(brakeType::coast);
    RightMotor2.setBrake(brakeType::coast);
    RightMotor3.setBrake(brakeType::coast);
   // leftMP(181,65, 18);
   // backwardMP(95, 181, 5, 20, 0.5, 0, 0);
   



    //Getting arm to load postion before side goal


    //pivotRightMP(100, 10, 20, 100);
/*
    //Coming out of Tower
    straightOdometry(-40, 20, 17, 90, 0.3);
   

    //Turn to ring stack
     turnOdometry(200, 80);
    goalPneumatics.set(false);
    straightOdometry(50, 10, 17, 200, 0.3);
    //wait(2000, msec);
    doinkerPneumaticsRight.set(false);

    //Turn to corner

     turnOdometry(315, 80);
    intakeMotor.spin(reverse, 100, velocityUnits::pct);
    doinkerPneumaticsRight.set(false);
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

*/
}





void calibration()
{
//headingOffset = 0;
//leftMP(135,88,18,100);
//move(100.0, 25.0, vex::directionType::fwd);
//backwardMP(50, 33, 0, 16, 1.5, 0.0008, 0.5, 0.35);
pivotTurnOdometry(90, 10, 20, 100);

}


//Skills
void skills()
{
    
    // Initialize the optical sensor
    initializeOpticalSensor();
    
    // Create parameters for the color detection task
    ColorTaskParams colorParams;
    colorParams.isRunning = false;
    colorParams.targetColor = Color::RED;  // Set to RED or BLUE depending on your needs
    colorParams.delayMs = 200;  // Adjust delay as needed (200ms shown as example)
    
    headingOffset = 220;





    //go to Alliance Stake
   armMotor1.spinToPosition(500, rotationUnits::deg, 100, velocityUnits::pct, false);
   armMotor2.spinToPosition(500, rotationUnits::deg, 100, velocityUnits::pct, false);
   wait(500, msec);

    //Backup to Pick up Mobile Goal #1
    backwardMP(47, 35, 224, 21, 1.5, 0.0008, 0.5, 0.35);
       //Reset Arm back to Ready
    armResetParams.isRunning = true;
    armResetParams.isResetComplete = false; 
    
    goalPneumatics.set(true);


    
    //Drive & intake Ring #2
    rightMP(90,85,21);     
    intakeMotor.spin(reverse, 100, velocityUnits::pct);





    
    //intakeMotor.stop(brakeType::coast); 
    forwardMP(48, 20, 90, 19);
    startIntakeStallDetection();

    //Turn & Drive to Side Goal Right
  
   
    rightMP(30,50,18); 
    armMotor1.spinToPosition(Load1-75, rotationUnits::deg, 100, velocityUnits::pct, false);
    armMotor2.spinToPosition(Load1-75, rotationUnits::deg, 100, velocityUnits::pct, false);     
    //wait(5000, msec);
   

//Scoring on side Stake

    forwardMP(95, 65, 30, 20, 0.6, 0.003, 0.01);
    pivotRightMP(0, 20, 20, 100);
    forwardMP(4, 2, 0, 20, 0.6, 0.003, 0.01);

    task::sleep(500); // Small delay to prevent overwhelming the CPU

    //waitForButton();
    armPneumatics.set(true);

    armMotor1.spinToPosition(Side-75, rotationUnits::deg, 100, velocityUnits::pct, false);
    armMotor2.spinToPosition(Side-75, rotationUnits::deg, 100, velocityUnits::pct, true);

    armPneumatics.set(false);

    armMotor1.spinToPosition(Starting-75, rotationUnits::deg, 100, velocityUnits::pct, false);
    armMotor2.spinToPosition(Starting-75, rotationUnits::deg, 100, velocityUnits::pct, true);
    backwardMP(15, 6, 4, 20, 1.5, 0.0008, 0.5, 0.35);
    rightMP(270,67,20); 
    intakeMotor.spin(reverse, 100, velocityUnits::pct);

    forwardMP(135, 60, 270, 20, 0.6, 0.003, 0.01);
    leftMP(40,67,20); 
    forwardMP(25, 10, 40, 20, 0.6, 0.003, 0.01);
    leftMP(110,40,20); 
    backwardMP(20, 10, 110, 20, 0.6, 0.0008, 0.5, 0.35);
    goalPneumatics.set(false);
    forwardMP(12, 7, 110, 20, 0.6, 0.003, 0.01);
    rightMP(180,80,20); 
    forwardMP(160, 60, 180, 20, 0.6, 0.003, 0.01);
    rightMP(0,100,20); 
    backwardMP(20, 10, 0, 20, 0.6, 0.0008, 0.5, 0.35);
    leftMP(150,80,20); 
    forwardMP(95, 65, 15, 0, 20, 0.6, 0.003, 0.01);





/*
    wait(500, msec);
    //Backup after scoring first right side stake
    backwardMP(15, 8, 28, 20, 1.5, 0.0008, 0.5, 0.35);

    armPneumatics.set(false);

    armResetParams.isRunning = true;
    armResetParams.isResetComplete = false; 

    rightMP(240,67,20); 
*/
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
    armMotor1.spinToPosition(400, rotationUnits::deg, 80, velocityUnits::pct, false);
    armMotor2.spinToPosition(400, rotationUnits::deg, 80, velocityUnits::pct, false);
    // Score Alliance
    straight(113, 70); // use this one
    armMotor1.spinToPosition(590, rotationUnits::deg, 100, velocityUnits::pct, false);
    armMotor2.spinToPosition(590, rotationUnits::deg, 100, velocityUnits::pct, false);
    straight(-30, 10); // use this one
    armMotor1.spinToPosition(-590, rotationUnits::deg, 100, velocityUnits::pct, false);
    armMotor2.spinToPosition(-590, rotationUnits::deg, 100, velocityUnits::pct, false);

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
    armMotor1.spinToPosition(570, rotationUnits::deg, 100, velocityUnits::pct, true);
    armMotor2.spinToPosition(570, rotationUnits::deg, 100, velocityUnits::pct, true);
    intakeMotor.spinFor(2000, rotationUnits::deg, 100, velocityUnits::pct, false);

    // Get Mobile Goal
    straight(-85, 40, 35, -5, 0.6);
    goalPneumatics.set(true);

    // Backing up to Border
    armMotor1.spinToPosition(-70, rotationUnits::deg, 100, velocityUnits::pct, false);
    armMotor2.spinToPosition(-70, rotationUnits::deg, 100, velocityUnits::pct, false);
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
    armMotor1.spinToPosition(570, rotationUnits::deg, 100, velocityUnits::pct, true);
    armMotor2.spinToPosition(570, rotationUnits::deg, 100, velocityUnits::pct, true);
    intakeMotor.spinFor(2000, rotationUnits::deg, 100, velocityUnits::pct, false);

    // Get Mobile Goal
    straight(-70, 43, 30, 6, 0.6);
    goalPneumatics.set(true);

    // Backing up to Border
    armMotor1.spinToPosition(-70, rotationUnits::deg, 100, velocityUnits::pct, false);
    armMotor2.spinToPosition(-70, rotationUnits::deg, 100, velocityUnits::pct, false);
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
    armMotor1.spinToPosition(570, rotationUnits::deg, 100, velocityUnits::pct, true);
    armMotor2.spinToPosition(570, rotationUnits::deg, 100, velocityUnits::pct, true);
    //wait(200, msec);

    //Backup to Pick up Mobile Goal
    backward(-95, 58, 25, -3, 1);
    // backward(273,48,25,-3, 1.8);
    goalPneumatics.set(true);
    armMotor1.spinToPosition(-40, rotationUnits::deg, 100, velocityUnits::pct, true);
    armMotor2.spinToPosition(-40, rotationUnits::deg, 100, velocityUnits::pct, true);
    intakeMotor.spin(reverse, 100, velocityUnits::pct);
    //wait(2000, msec);

    //Turn to go into Tower
    turnOdometry(100, 55);
      //  wait(2000, msec);
    straightOdometry(40, 20, 17, 100);
    //wait(2000, msec);
    doinkerPneumaticsRight.set(true);

    //Coming out of Tower
    straightOdometry(-40, 20, 17, 90, 0.3);
   

    //Turn to ring stack
     turnOdometry(200, 80);
    goalPneumatics.set(false);
    straightOdometry(50, 10, 17, 200, 0.3);
    //wait(2000, msec);
    doinkerPneumaticsRight.set(false);

    //Turn to corner

     turnOdometry(315, 80);
    intakeMotor.spin(reverse, 100, velocityUnits::pct);
    doinkerPneumaticsRight.set(false);
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
    armMotor1.spinToPosition(570, rotationUnits::deg, 100, velocityUnits::pct, true);
    armMotor2.spinToPosition(570, rotationUnits::deg, 100, velocityUnits::pct, true);
    backward(-72, 40, 25, 3, 2.1); // Changed -3 to 3 (relative)
    goalPneumatics.set(true);
    armMotor1.spinToPosition(-40, rotationUnits::deg, 100, velocityUnits::pct, true);
    armMotor2.spinToPosition(-40, rotationUnits::deg, 100, velocityUnits::pct, true);
    turnOdometry(-125, 95, 20);              // Changed 93 to -93 (relative)
    straightOdometry(58, 35, 17, -125, 0.3); // Changed 93 to -93 (relative)
    doinkerPneumaticsRight.set(true);
    wait(500, msec);
    backward(-47, 20, 17, -125); // Changed 95 to -95 (relative)
    turnOdometry(-200, 180, 25); // Changed 140 to 220 (absolute, 360-140=220)
    intakeMotor.spin(reverse, 100, velocityUnits::pct);
    doinkerPneumaticsRight.set(false);
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
    doinkerPneumaticsRight.set(true);
    backward(-47, 20, 17, 95, 2.2);
    turnOdometry(140, 180, 25);
    intakeMotor.spin(reverse, 100, velocityUnits::pct);
    doinkerPneumaticsRight.set(false);
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
    doinkerPneumaticsRight.set(true);
    wait(500, msec);
    backward(-47, 20, 17, -125); // Changed 95 to -95 (relative)
    turnOdometry(-200, 180, 25); // Changed 140 to 220 (absolute, 360-140=220)
    intakeMotor.spin(reverse, 100, velocityUnits::pct);
    doinkerPneumaticsRight.set(false);
    turnOdometry(-189, 180, 35); // Changed 168 to 192 (absolute, 360-168=192)
    // turnOdometry(125, 195, 25);  // Changed 235 to 125 (absolute, 360-235=125)
    straightOdometry(40, 25, 17, -189, 0.2); // Changed 168 to 192 (matching above absolute heading)
                                             // backward(-35,20,17,-189);  // Changed 95 to -95 (relative)
    turnOdometry(-230, 230, 20);
    straightOdometry(35, 25, 17, -230, 0.2); // Changed 168 to 192 (matching above absolute heading)
}


void doubleDoinkerBlue()
{
const double RED_HUE_MIN_1 = 350.0; // First red range
const double RED_HUE_MAX_1 = 360.0;
const double RED_HUE_MIN_2 = 0.0;   // Second red range
const double RED_HUE_MAX_2 = 18.0;  // Reduced from 30
const double BLUE_HUE_MIN = 207.0;  // Blue range - narrower
const double BLUE_HUE_MAX = 230.0;  // Reduced from 240

    initializeOpticalSensor();
/*
    // Define task parameters
    ColorTaskParams colorTaskParams;
    colorTaskParams.isRunning = false;
    colorTaskParams.targetColor = Color::RED; // Set ejection colour RED or BLUE
    colorTaskParams.delayMs = 80;              // Set delay before stopping intake
    // Start the color detection task
    vex::task colorTask(colorDetectionTask, &colorTaskParams);
*/

    headingOffset = 120; // Mirrored from 240
    
    //go to Alliance Stake
    moveArm(ArmPosition::Alliance, -75, 00);

    forwardMP(18, 9, 120); // Mirrored angle from 240

    //wait(400, msec);

    //Backup to Pick up Mobile Goal
    backwardMP(88, 49, 115, 20); // Mirrored angle from 245
    armMotor1.spinToPosition(520, rotationUnits::deg, 100, velocityUnits::pct, false);
    armMotor2.spinToPosition(520, rotationUnits::deg, 100, velocityUnits::pct, false);
    // backward(273,48,25,-3, 1.8);
    goalPneumatics.set(true);
    armResetParams.isRunning = true;
    armResetParams.isResetComplete = false; 
    //armMotor1.spinToPosition(100, rotationUnits::deg, 100, velocityUnits::pct, false);
    //armMotor2.spinToPosition(100, rotationUnits::deg, 100, velocityUnits::pct, true);
    //intakeMotor.spin(reverse, 100, velocityUnits::pct);
    //wait(2000, msec);

    //Turn to go into Tower
    leftMP(42, 65, 20); // Mirrored from rightMP(138) -> leftMP(42)
      
    forwardMP(43, 27, 42); // Mirrored angle from 138 to 42
    
    //waitForButtonPress();
    doinkerPneumaticsLeft.set(true); // Mirrored from doinkerPneumaticsRight
    wait(200, msec);
    pivotLeftMP(67, 13, 20, 100); // Mirrored from pivotRightMP(113) -> pivotLeftMP(67)
    
    doinkerPneumaticsRight.set(true); // Mirrored from doinkerPneumaticsLeft
    wait(300, msec);
    backwardMP(110, 56, 67, 21, 1.5, 0.0008, 0.5, 0.35); // Mirrored angle from 113 to 67
    intakeMotor.spinFor(forward, 0.5, rotationUnits::rev, 100, velocityUnits::pct, true);
    armMotor1.spinToPosition(180, rotationUnits::deg, 100, velocityUnits::pct, false);
    armMotor2.spinToPosition(180, rotationUnits::deg, 100, velocityUnits::pct, false);
    doinkerPneumaticsRight.set(false); // Mirrored from doinkerPneumaticsLeft
    doinkerPneumaticsLeft.set(false); // Mirrored from doinkerPneumaticsRight
    intakeMotor.spin(reverse, 100, velocityUnits::pct);

    rightMP(37, 15, 20); // Mirrored from leftMP(143) -> rightMP(37)

    forwardMP(45, 20, 37, 20, 0.8); // Mirrored angle from 143 to 37

    leftMP(154, 70, 20); // Mirrored from rightMP(26) -> leftMP(154)
    startIntakeStallDetection();
    moveArm(ArmPosition::Load2, -82, 1000);
    // forwardMP(98, 50, 26, 16, 1.5, 0.03, 0.1);

    forwardMP(90, 60, 157, 22, 1); // Mirrored angle from 23 to 157

    goalPneumatics.set(false);

    rightMP(135, 26, 18); // Mirrored from leftMP(45) -> rightMP(135)
    forwardMP(54, 25, 135, 20, 0.4); // Mirrored angle from 45 to 135
    pivotLeftMP(122, 5, 20, 100); // Mirrored from pivotRightMP(58) -> pivotLeftMP(122)

    armPneumatics.set(true);

    armMotor1.spinToPosition(Side-100, rotationUnits::deg, 100, velocityUnits::pct, false);
    armMotor2.spinToPosition(Side-100, rotationUnits::deg, 100, velocityUnits::pct, true);
    armPneumatics.set(false);
    wait(300, msec);
    armMotor1.spinToPosition(Alliance-300, rotationUnits::deg, 100, velocityUnits::pct, false);
    armMotor2.spinToPosition(Alliance-300, rotationUnits::deg, 100, velocityUnits::pct, false);
    backwardMP(10, 0, 134, 20, 1); // Mirrored angle from 46 to 134
    LeftMotor1.setBrake(brakeType::coast);
    LeftMotor2.setBrake(brakeType::coast);
    LeftMotor3.setBrake(brakeType::coast);
    RightMotor1.setBrake(brakeType::coast);
    RightMotor2.setBrake(brakeType::coast);
    RightMotor3.setBrake(brakeType::coast);

    backwardMP(95, 0, 175, 20, 0.5, 0, 0); // Mirrored angle from 5 to 175
   
    //Getting arm to load postion before side goal

    //pivotLeftMP(80, 10, 20, 100); // Mirrored from pivotRightMP(100) -> pivotLeftMP(80)
/*
    //Coming out of Tower
    straightOdometry(-40, 20, 17, 90, 0.3);
   
    //Turn to ring stack
    turnOdometry(160, 80); // Mirrored from 200 to 160 (360-200)
    goalPneumatics.set(false);
    straightOdometry(50, 10, 17, 160, 0.3); // Mirrored angle from 200 to 160
    //wait(2000, msec);
    doinkerPneumaticsLeft.set(false); // Mirrored from doinkerPneumaticsRight

    //Turn to corner
    turnOdometry(45, 80); // Mirrored from 315 to 45 (360-315)
    intakeMotor.spin(reverse, 100, velocityUnits::pct);
    doinkerPneumaticsLeft.set(false); // Mirrored from doinkerPneumaticsRight
    //turnOdometry(168, 195, 20);
    //turnOdometry(235, 195, 25);
    straightOdometry(100, 30, 17, 45); // Mirrored angle from 315 to 45
    //backward(-10,5,17,55,0);
    //InertialSensor.resetRotation();  // Resets continuous rotation counting
    //InertialSensor.resetHeading();
    //wait(1000,msec);
    //turn360(-104, 70, 25);
    //straight360(100,50, 18);
    //goalPneumatics.set(false);

    turnOdometry(110, 0, 60); // Mirrored from 250 to 110 (360-250)
*/
}