#include "vex.h" // Include the VEX library
#include "robot_config.h" // Include the robot configuration
#include "driver.h" // Include the driver control functions
#include "auton.h" // Include the autonomous functions
#include "utils.h"
#include "navigation.h"
#include "autontasks.h"

#define COMPETITION_PROGRAM_NAME "Test"

using namespace vex;

vex::competition Competition;

void runAuton(void) {
  Brain.Screen.clearScreen();
  Brain.Screen.print("Running Autonomous Mode...");

  // Start heading display task (autonomous-only)
  headingDisplayParams.isRunning = true;
  vex::task heading_task(headingDisplayTask, &headingDisplayParams);
  wingPneumatics.set(false);
  
  //Start heading display task
  //visionSensorTest(); 
  //autonTest();  
  //CoordinateFinderTask();  
  //autonLeft(); 
  //autonRight();
  //autonFwdRight();
  //autonFwdLeft();
  //SpeedwayAutonLeft();
  //SevenBallRight();
  //SevenBallLeft();
  //soloAWP();
  //soloAWPMiddle();
  //leftSideLong();
  //leftSidemiddle();
  //nothing();
  //soloAwp2();
  //rightMiddleAuto();
  //calibration();
  //doubleDoinkerBlue();
  //skills();
  //odomTest();
  //skillsAutonGateway();
  //soloAwpOdom();
  skillsAuton();
//systemTest();
  headingDisplayParams.isRunning = false;
  heading_task.stop();
  //Brain.Screen.print("Autonomous Program Complete");
}

void runDriver(void) {
  Brain.Screen.clearScreen();
  Brain.Screen.print("Running Driver Control Mode...");
  
  headingDisplayParams.isRunning = false;
  vex::task driver_display_task(driverDisplayTask, &headingDisplayParams); 
  Controller.Screen.clearScreen(); // stop controller prints during driver control
  driverControl(); // Start driver control function
}

int main()
{
    // Initializing Robot Configuration. DO NOT REMOVE!
    wingPneumatics.set(false);
    vexcodeInit();
    //initializeOpticalSensor();

    Competition.autonomous(runAuton);
    Competition.drivercontrol(runDriver);
}