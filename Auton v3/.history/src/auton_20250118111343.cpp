#include "pid.h"           // Include the PID header file
#include "vex.h"           // Include the VEX library
#include "utils.h"         // Include the utility header for normalizeHeading
#include "robot-config.h"  // Include the robot configuration
#include "navigation.h" 
#include "vision_tracking.h"
#include <cmath>           // Include math library for M_PI

using namespace vex;       // Use the VEX namespace

/**
 * @brief Executes an in-place spot turn to achieve a specified target heading.
 *
 * This function utilizes a PID controller to rotate the robot smoothly and accurately 
 * to a desired heading. It continuously adjusts motor speeds based on the current 
 * heading error until the robot stabilizes within a defined tolerance range.
 *
 * @param targetHeading  The desired heading in degrees (0° to 360°). The robot will turn to face this direction.
 * @param maxSpeed       The maximum motor speed (in percentage) allowed during the turn.
 * @param minSpeed       The minimum motor speed (in percentage) to overcome static friction and ensure movement.
 * @param kp_heading     Proportional gain constant for the PID controller, affecting the reaction to current error.
 * @param ki_heading     Integral gain constant for the PID controller, affecting the reaction based on accumulated error.
 * @param kd_heading     Derivative gain constant for the PID controller, affecting the reaction based on the rate of error change.
 */

/* Best Nav Control Settings
pidStraightDistanceLaunch(0, 40, 90, 0, 0, 0, 0.15, 0, 0, 7.5, brakeType::brake); //short distance
pidStraightDistanceLaunch(0, 80, 70, 0.4, 0, 0, 0.09, 0, 0, 7.5, brakeType::brake); // long distance, min speed 7.5 
pidStraightDistanceLaunch(0, 80, 60, 0, 0, 0, 0.15, 0, 0, brakeType::brake); // long distance without heading correction for reference

Backwards
pidStraightDistanceLaunch(0, -80, 70, 0.4, .02, .5, 0.09, 0, 0, 10, brakeType::brake); Pretty good, zig zags due to heavy veer, but precise, 10 min speed
*/
 bool isClawPneumaticsActive = false; // Variable to track clawPneumatics state
    bool isGoalPneumaticsActive = false; // Variable to track goalPneumatics state
    bool isHookPneumaticsActive = true;
    bool isElbowPneumaticsActive = false;
      // Initial state reversed

bool timeoutCondition() {
//return false;
    static int counter = 0;
  counter++;
 return counter >= 10000; // Exit after 50 iterations (approx. 1 second if 20 ms per iteration)
}


/*

void driveForDistance(double distanceCm, double heading) {
    // New code to run pidStraight for 5 seconds
    pidStraight(heading, timeoutCondition);
}


void driveForDistancePID(double distanceCm, double speed) {
    pidDistance(distanceCm, speed, timeoutCondition);
}

*/


/*
// Example autonomous routine 1
void autonRoutine1() {
    // Drive for 100 cm at 50% speed using PID distance control
  //  pidDistance(100.0, 50.0, timeoutCondition);
    
    // Drive straight for 5 seconds at heading 90 degrees
    //pidStraightDistance(10, 15, 0, timeoutCondition);
    //spotTurn(-90, 100, 2, 0.3, 0, 0.01); really good setting looks visually bang on almost all the time
    //spotTurn(-90, 100, 2, 0.3, 0, 0.05); pretty good but not as good, would say 0.05 for D is the max range for 90 degree turns  
    
//pidStraight(0, 40.00, 25, 1.25, 0, 0, 10.00 );//works

pidStraightDistanceLaunch(0, -80, 70, 0.4, .02, .5, 0.09, 0, 0, brakeType::brake);
 // move(20, 10);   
    /*
    spotTurn(180, 100, 1, 0.28, 0, 0.03); 
           task::sleep(500);  // Small delay to prevent overwhelming the CPU
    spotTurn(0, 100, 1, 0.28, 0, 0.03); 
            task::sleep(500);  // Small delay to prevent overwhelming the CPU
    spotTurn(-180, 100, 1, 0.28, 0, 0.03); 
  */

//}

// Example autonomous routine 1
void autonRoutine1() {  //Red Safe

  //arm score
//armMotor.spinFor(ScoringAlliance, rotationUnits::deg, 100, velocityUnits::pct);

//back up and grab mobile goal 
pidStraightDistanceABS(1, -75, 50, .15, 0, 0, 1, 0, 0, 15, 20);  
goalPneumatics.set(true);
//armMotor.spinFor(Starting, rotationUnits::deg, 100, velocityUnits::pct);

//go and turn towards blue ring on bottom and red ring on top knock it off with the doinker 
pidStraightDistanceABS(0, 50, 50, .10, 0, 0, 1, 0, 0, 15, 15); 
doinkerPneumatics.set(true);
spotTurn(-80, 10, 2, 0.4, 0, 0.05);
doinkerPneumatics.set(false);
intakeMotor.spin(directionType::rev, 12, voltageUnits::volt);
pidStraightDistanceABS(-79, 40, 50, .15, 0, 0, 1, 0, 0, 15, 15); 

//back up towards to stack with red on the bottom and blue on top 
pidStraightDistanceABS(-79, -70, 50, .15, 0, 0, 1, 0, 0, 15, 15); 
//armMotor.spinToPosition(22, rotationUnits::deg, 100, velocityUnits::pct);
spotTurn(170, 50, 2, 0.4, 0, 0.05);
pidStraightDistanceABS(170, 70, 70, .15, 0, 0, 1, 0, 0, 15, 15);
spotTurn(-80, 50, 2, 0.4, 0, 0.05);
pidStraightDistanceABS(-80, 75, 30, .15, 0, 0, 1, 0, 0, 15, 15); 

/*
//load and raise arm to score wall stake
vex::task::sleep(600); 
intakeMotor.stop(coast);
intakeMotor.spinFor(100, rotationUnits::deg, 58, velocityUnits::pct);
armMotor.spinFor(270, rotationUnits::deg, 30, velocityUnits::pct);

//drive towards wall stake
pidStraightDistanceABS(172, 30, 50, .15, 0, 0, 1, 0, 0, 15, 15); 

//score and rest 
armMotor.spinFor(50, rotationUnits::deg, 80, velocityUnits::pct);
armMotor.stop(coast);
armMotor.spinFor(-600, rotationUnits::deg, 80, velocityUnits::pct);
armMotor.stop(coast);
*/









//vex::task::sleep(1000); // Wait 1 second to ensure activation


/*  
//double startingHeading = 0;  
//InertialSensor.setHeading(startingHeading, rotationUnits::deg);
//intakeMotor.spin(reverse, 12, voltageUnits::volt);

// Define parameters for intake motor control
    // Define parameters for intake motor control
    MotorControlParams intakeParams;
    intakeParams.targetMotor = &intakeMotor;
    intakeParams.DelayStart = 200; // 0.5 seconds
    intakeParams.OnTime = 1300;     // 2 seconds
    intakeParams.dir = directionType::rev;

//intakeMotor.spin(reverse, 12, voltageUnits::volt);
thread intakeTask(MotorControlThread, &intakeParams);
pidStraightDistanceLaunch(0, 95.8, 60, 0.525, 0, 0, 0.08, 0, 0, 10, brakeType::brake);
spotTurn(270, 100, 2, 0.4, 0, 0.05);
pidStraightDistanceLaunch(270, -55, 50, 0.42, 0, 0, 0.145, 0, 0,14, brakeType::brake);
//pidStraightDistanceLaunch(270, -50, 50, 0.4, .02, .5, 0.09, 0, 0, 20, brakeType::coast);
   isGoalPneumaticsActive = !isGoalPneumaticsActive;
    goalPneumatics.set(isGoalPneumaticsActive);
     spotTurn(154, 50, 2, 0.3, 0, 0.01);
         intakeMotor.spin(reverse, 12, voltageUnits::volt);
 vexDelay(1000);
    intakeMotor.stop();
      armMotor.spinToPosition(Alliance, rotationUnits::deg, 100, velocityUnits::pct, false);
    
      task::sleep(150);
         isElbowPneumaticsActive = !isElbowPneumaticsActive;
            elbow1Pneumatics.set(isElbowPneumaticsActive);
            elbow2Pneumatics.set(isElbowPneumaticsActive);   
             armstat = ArmPosition::Alliance;
    pidStraightDistanceLaunch(154, 100, 60, 0.5, 0, 0, 0.09, 0, 0, 10, brakeType::brake);
   armMotor.spinToPosition(Ready, rotationUnits::deg, 100, velocityUnits::pct, false);
    vexDelay(500);
   pidStraightDistanceLaunch(170, -78, 90, 0.4, 0, 0, 0.15, 0, 0,10, brakeType::brake);

*/
}


void autonRoutine2() {  //Blue Safe Route

//score alliance stake 
//armMotor.spinFor(ScoringAlliance, rotationUnits::deg, 100, velocityUnits::pct);

//backup and grab mobile goal
pidStraightDistanceABS(-1, -75, 50, .15, 0, 0, 1, 0, 0, 15, 20);  
goalPneumatics.set(true);
//armMotor.spinFor(Starting, rotationUnits::deg, 100, velocityUnits::pct);
/**/
//drive towards and grab stack with blue ring on top and red ring on the bottom using the doinker
pidStraightDistanceABS(0, 50, 50, .10, 0, 0, 1, 0, 0, 15, 15); 
doinkerPneumatics.set(true);
spotTurn(68, 7, 2, 0.4, 0, 0.05);
doinkerPneumatics.set(false);
intakeMotor.spin(directionType::rev, 12, voltageUnits::volt);
pidStraightDistanceABS(68, 40, 50, .15, 0, 0, 1, 0, 0, 15, 15); 

//grab stack with blue ring on the bottom and red ring on top 
pidStraightDistanceABS(79, -70, 50, .15, 0, 0, 1, 0, 0, 15, 15); 
spotTurn(-170, 50, 2, 0.4, 0, 0.05);
pidStraightDistanceABS(-170, 70, 70, .15, 0, 0, 1, 0, 0, 15, 15);
vex::task::sleep(1000); 
spotTurn(80, 50, 2, 0.4, 0, 0.05);
pidStraightDistanceABS(80, 75, 30, .15, 0, 0, 1, 0, 0, 15, 15); 


    // Reset motor encoder
  //  armMotor.resetPosition();
//
    // Update arm state
  //  armstat = ArmPosition::Starting;

    // Reset the arm to starting position
   // armMotor.spinToPosition(-ArmPosition::Starting, rotationUnits::deg, 50, velocityUnits::pct, true);
   // armstat = ArmPosition::Starting;












/*
intakeMotor.stop(coast);
intakeMotor.spinFor(100, rotationUnits::deg, 58, velocityUnits::pct);
armMotor.spinFor(270, rotationUnits::deg, 30, velocityUnits::pct);
pidStraightDistanceABS(172, 30, 50, .15, 0, 0, 1, 0, 0, 15, 15); 

armMotor.spinFor(80, rotationUnits::deg, 80, velocityUnits::pct);
armMotor.spinFor(-600, rotationUnits::deg, 80, velocityUnits::pct);
*/
}
/**
// Example autonomous routine 2
void autonSkills() {
   MotorControlParams intakeParams;
    intakeParams.targetMotor = &intakeMotor;
    intakeParams.DelayStart = 200; // 0.5 seconds
    intakeParams.OnTime = 1300;     // 2 seconds
    intakeParams.dir = directionType::rev;
//goal 1
     armMotor.spinToPosition(50, rotationUnits::deg, -100, velocityUnits::pct, false);// score red alliance stake 
     pidStraightDistanceLaunch(180, -10, 90, 0.4, 0, 0, 0.15, 0, 0,18, brakeType::coast);//back up
      spotTurn(135, 50, 2, 0.3, 0, 0.01);// turn towards mobile goal 
      pidStraightDistanceLaunch(135, -55, 90, 0.4, 0, 0, 0.15, 0, 0,18, brakeType::coast);//drive into mobile goal 
      isGoalPneumaticsActive = !isGoalPneumaticsActive;//grab mobile goal 
    goalPneumatics.set(isGoalPneumaticsActive);
spotTurn(70, 100, 2, 0.3, 0, 0.01);//  turns toward ring 
  intakeMotor.spin(reverse, 12, voltageUnits::volt); //intake on
pidStraightDistanceLaunch(70, 35, 90, 0.4, 0, 0, 0.15, 0, 0,18, brakeType::coast);//drive into intake 
intakeMotor.stop();
spotTurn(250, 100, 2, 0.3, 0, 0.01);// turn to score goal 
pidStraightDistanceLaunch(250, -50, 50, 0.4, .02, .5, 0.09, 0, 0, 20, brakeType::coast);// back into corner
!isGoalPneumaticsActive = isGoalPneumaticsActive;//release goal 
    goalPneumatics.set(!isGoalPneumaticsActive);

    //Goal 2
pidStraightDistanceLaunch(250, 80, 50, 0.4, .02, .5, 0.09, 0, 0, 20, brakeType::coast);//drive towards the ladder
spotTurn(140, 100, 2, 0.3, 0, 0.01);//turn towards the mobile goal 
pidStraightDistanceLaunch(140, -70, 50, 0.4, .02, .5, 0.09, 0, 0, 20, brakeType::coast);// back into the mobile goal 
 isGoalPneumaticsActive = !isGoalPneumaticsActive;//grab mobile goal 
    goalPneumatics.set(isGoalPneumaticsActive);
    spotTurn(100, 100, 2, 0.3, 0, 0.01);// turn in between the gaps of the rings
    !isGoalPneumaticsActive = isGoalPneumaticsActive;//release the mobile goal 
    goalPneumatics.set(!isGoalPneumaticsActive);
    pidStraightDistanceLaunch(100, -30, 50, 0.4, .02, .5, 0.09, 0, 0, 20, brakeType::coast);// backup into corner

    //goal 3
spotTurn(170, 100, 2, 0.3, 0, 0.01);//turn towards ring 
  thread intakeTask(MotorControlThread, &intakeParams);//intake task because you have to hold the ring
pidStraightDistanceLaunch(170, 200, 50, 0.4, .02, .5, 0.09, 0, 0, 20, brakeType::coast);// drive a long distance to the middle mobile goal 
spotTurn(280, 100, 2, 0.3, 0, 0.01);// turn to grab the mobile goal 
pidStraightDistanceLaunch(280, -50, 50, 0.4, .02, .5, 0.09, 0, 0, 20, brakeType::coast);// back into the mobile goal
 isGoalPneumaticsActive = !isGoalPneumaticsActive;//grab mobile goal 
    goalPneumatics.set(isGoalPneumaticsActive);
  intakeMotor.spin(reverse, 12, voltageUnits::volt);//score held ring 
vexdelay(500);
intakeMotor.stop();
  intakeMotor.spin(forward, 12, voltageUnits::volt);//reverse the intake 
spotTurn(270, 100, 2, 0.3, 0, 0.01);//turn towards the line of blue rings around the corner
pidStraightDistanceLaunch(270, 50, 50, 0.4, .02, .5, 0.09, 0, 0, 20, brakeType::coast);//outake the rings against the wall 
spotTurn(110, 100, 2, 0.3, 0, 0.01);// turn into the corner 
pidStraightDistanceLaunch(110, -20, 50, 0.4, .02, .5, 0.09, 0, 0, 20, brakeType::coast);
 !isGoalPneumaticsActive = isGoalPneumaticsActive;//release the mobile goal 
    goalPneumatics.set(!isGoalPneumaticsActive);

// goal 4
pidStraightDistanceLaunch(110, 20, 50, 0.4, .02, .5, 0.09, 0, 0, 20, brakeType::coast);//drive out of the corner
spotTurn(280, 100, 2, 0.3, 0, 0.01);//turn towards the mobile goal 
pidStraightDistanceLaunch(280, -30, 50, 0.4, .02, .5, 0.09, 0, 0, 20, brakeType::coast);//back into the mobile goal 
 isGoalPneumaticsActive = !isGoalPneumaticsActive;//grab mobile goal 
    goalPneumatics.set(isGoalPneumaticsActive);
spotTurn(70, 100, 2, 0.3, 0, 0.01);//turn towards to the ring near the ladder
  intakeMotor.spin(reverse, 12, voltageUnits::volt);//turn intake on 
pidStraightDistanceLaunch(70, 150, 50, 0.4, .02, .5, 0.09, 0, 0, 20, brakeType::coast);//
spotTurn(135, 100, 2, 0.3, 0, 0.01);//intake and score blue ring to clear the corner
pidStraightDistanceLaunch(135, 30, 50, 0.4, .02, .5, 0.09, 0, 0, 20, brakeType::coast);//drive into blue corner ring 
intakeMotor.stop();
spotTurn(315, 100, 2, 0.3, 0, 0.01);// turn to get ready to score mobile goal 
pidStraightDistanceLaunch(315, -15, 50, 0.4, .02, .5, 0.09, 0, 0, 20, brakeType::coast);//back the mobile goal in 
 !isGoalPneumaticsActive = isGoalPneumaticsActive;//release the mobile goal 
    goalPneumatics.set(!isGoalPneumaticsActive);
    pidStraightDistanceLaunch(315, 15, 50, 0.4, .02, .5, 0.09, 0, 0, 20, brakeType::coast);//drive forward to not touch the mobile goal 

    //FINIS//////////////////////////////////



    // Drive for 50 cm at 30% speed using PID distance control
    //pidStraightDistance(80,40,0, timeoutCondition);
    //pidDistance(100, 40, timeoutCondition); 
    
    // Drive straight for 3 seconds at heading 45 degrees
  //  pidStraight(0, timeoutCondition);
}
*/

// Example autonomous routine 3
void autonRoutine3() {
    MotorControlParams intakeParams;
    intakeParams.targetMotor = &intakeMotor;
    intakeParams.DelayStart = 200; // 0.5 seconds
    intakeParams.OnTime = 1300;     // 2 seconds
    intakeParams.dir = directionType::rev;

//intakeMotor.spin(reverse, 12, voltageUnits::volt);
thread intakeTask(MotorControlThread, &intakeParams);
pidStraightDistanceLaunch(0, 94, 60, 0.4, 0, 0, 0.09, 0, 0, 10, brakeType::brake);
spotTurn(90, 100, 2, 0.3, 0, 0.01);
pidStraightDistanceLaunch(90, -55, 32, 0.5, 0, 0, 0.15, 0, 0,20, brakeType::coast);
//pidStraightDistanceLaunch(270, -50, 50, 0.4, .02, .5, 0.09, 0, 0, 20, brakeType::coast);
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
   pidStraightDistanceLaunch(210, -30, 90, 0.4, 0, 0, 0.15, 0, 0,10, brakeType::brake);

 
}

// Example autonomous routine 4
void autonRoutine4() {
  pidStraightDistanceABS(0, 75, 30, .15, 0, 0, 1, 0, 0, 15, 15); 

    // Drive for 50 cm at 30% speed using PID distance control
    //followObject(visionSensor, 60, 11700);
    //pidStraightDistance(50, 0, 0, timeoutCondition);
     // armMotor.spinToPosition(-560, rotationUnits::deg, 100, velocityUnits::pct, true);
     //isGoalPneumaticsActive = !isGoalPneumaticsActive;
        //   goalPneumatics.set(isGoalPneumaticsActive);
      //  isHookPneumaticsActive = !isHookPneumaticsActive;
      //hookPneumatics.set(isHookPneumaticsActive);

                // Toggle goalPneumatics
               
    //pidStraightDistance(190, 30, 0, timeoutCondition);
   //hookPneumatics.open();
   //goalPneumatics.close();

    //spotTurn(270, 50, 0, 5);

        // armMotor.spinToPosition(-60, rotationUnits::deg, 100, velocityUnits::pct, true);
    //     vexDelay(150);
          //clawPneumatics.set(true);
      //elbowMotor.setPosition(220, rotationUnits::deg);
     //   pidStraightDistance(100, 30, 0, timeoutCondition);
          //elbowMotor.setPosition(0, rotationUnits::deg);
            //armMotor.spinToPosition(620, rotationUnits::deg, 100, velocityUnits::pct, true);
  // isGoalPneumaticsActive = !isGoalPneumaticsActive;
    //       goalPneumatics.set(isGoalPneumaticsActive);
      //  isHookPneumaticsActive = !isHookPneumaticsActive;
     // hookPneumatics.set(isHookPneumaticsActive);
    // Drive straight for 3 seconds at heading 45 degrees

  //  pidStraight(0, timeoutCondition);
  }

void autonRoutine5() {
//double startingHeading = 0;  
//InertialSensor.setHeading(startingHeading, rotationUnits::deg);
//intakeMotor.spin(reverse, 12, voltageUnits::volt);
/*
// Define parameters for intake motor control
    MotorControlParams intakeParams;
    intakeParams.targetMotor = &intakeMotor;
    intakeParams.DelayStart = 500; // 0.5 seconds
    intakeParams.OnTime = 1000;     // 2 seconds
    intakeParams.dir = directionType::rev;

    // Launch motor control threads using the wrapper function
    thread intakeTask(MotorControlThread, &intakeParams);
*/

pidStraightDistanceLaunch(0, 96, 70, 0.4, 0, 0, 0.09, 0, 0, 10, brakeType::brake);
spotTurn(275, 100, 2, 0.3, 0, 0.01);
pidStraightDistanceLaunch(275, -35, 70, 0.4, .02, .5, 0.09, 0, 0, 12, brakeType::brake);
}

void autonRoutine6() {
//double startingHeading = 0;  
//InertialSensor.setHeading(startingHeading, rotationUnits::deg);
armMotor.spinToPosition(500, rotationUnits::deg, 100, velocityUnits::pct, true);
straight(-50, 100, 0, 20); // use this one
armMotor.spinToPosition(-50, rotationUnits::deg, 100, velocityUnits::pct, false);
spotTurn(70, 70, 2, 0.4, 0, 0.05);
straight(-65, 100, 70, 40); // use this one
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

void autonRoutine7() {
armMotor.spinToPosition(500, rotationUnits::deg, 100, velocityUnits::pct, true);
straight(-50, 100, 0, 20); // use this one
armMotor.spinToPosition(-50, rotationUnits::deg, 100, velocityUnits::pct, false);
spotTurn(75, 70, 2, 0.4, 0, 0.05);
straight(-67, 100, 75, 40); // use this one
goalPneumatics.set(true);
spotTurn(150, 70, 2, 0.4, 0, 0.05);
intakeMotor.spinFor(-200000, rotationUnits::deg, 100, velocityUnits::pct, false);
vex::task::sleep(500);
straight(50, 100, 150, 30); // use this one
spotTurn(240, 70, 2, 0.4, 0, 0.05); 
straight(30, 30, 240, 10);
straight(-20, 30, 240, 10);
straight(20, 30, 220, 10);



}


void autonRoutineRedLeft() {  //Blue Safe Route

   armMotor.spinToPosition(600, rotationUnits::deg, 100, velocityUnits::pct, true);
pidStraightDistanceABS(0, -5, 100, .15, 0, 0, 1, 0, 0, 15, 0);  
spotTurn(-63, 70, 2, 0.4, 0, 0.05);
pidStraightDistanceABS(-70, 15, 80, .15, 0, 0, 1, 0, 0, 15, 10); 
//intakeMotor.spin(directionType::rev, 12, voltageUnits::volt);
armMotor.spinFor(-1200, rotationUnits::deg, 100, velocityUnits::pct, false);
intakeMotor.spinFor(-2000, rotationUnits::deg, 100, velocityUnits::pct, false);
armMotor.spinFor(-600, rotationUnits::deg, 100, velocityUnits::pct, false);
vex::task::sleep(500); 
spotTurn(29, 70, 2, 0.4, 0, 0.05);

pidStraightDistanceABS(29, -40, 70, .15, 0, 0, 1, 0, 0, 20, 20);
goalPneumatics.set(true);
intakeMotor.spinFor(-8000, rotationUnits::deg, 100, velocityUnits::pct, false);
spotTurn(150, 70, 2, 0.4, 0, 0.05);
pidStraightDistanceABS(150, 30, 70, .15, 0, 0, 1, 0, 0, 15, 15);
/*

//score alliance stake 
//armMotor.spinFor(ScoringAlliance, rotationUnits::deg, 100, velocityUnits::pct);

//backup and grab mobile goal
pidStraightDistanceABS(-1, -75, 50, .15, 0, 0, 1, 0, 0, 15, 20);  
goalPneumatics.set(true);
//armMotor.spinFor(Starting, rotationUnits::deg, 100, velocityUnits::pct);

//drive towards and grab stack with blue ring on top and red ring on the bottom using the doinker
pidStraightDistanceABS(0, 50, 50, .10, 0, 0, 1, 0, 0, 15, 15); 
doinkerPneumatics.set(true);
spotTurn(68, 7, 2, 0.4, 0, 0.05);
doinkerPneumatics.set(false);
intakeMotor.spin(directionType::rev, 12, voltageUnits::volt);
pidStraightDistanceABS(68, 40, 50, .15, 0, 0, 1, 0, 0, 15, 15); 

//grab stack with blue ring on the bottom and red ring on top 
pidStraightDistanceABS(79, -70, 50, .15, 0, 0, 1, 0, 0, 15, 15); 
spotTurn(-170, 50, 2, 0.4, 0, 0.05);
pidStraightDistanceABS(-170, 70, 70, .15, 0, 0, 1, 0, 0, 15, 15);
vex::task::sleep(1000); 
spotTurn(80, 50, 2, 0.4, 0, 0.05);
pidStraightDistanceABS(80, 75, 30, .15, 0, 0, 1, 0, 0, 15, 15); 
*/
}

    // Reset


