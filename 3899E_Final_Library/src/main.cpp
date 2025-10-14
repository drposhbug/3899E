#include "vex.h" // Include the VEX library
#include "robot_config.h" // Include the robot configuration
#include "driver.h" // Include the driver control functions
#include "auton.h" // Include the autonomous functions
#include "utils.h"
#include "navigation.h"

#define COMPETITION_PROGRAM_NAME "Test"

using namespace vex;

competition Competition;

void runAuton(void) {
  Brain.Screen.clearScreen();
  Brain.Screen.print("Running Autonomous Mode...");
  //Reset Arm
  
   // Call the autonomous routine     
   autonTest(); 
   //doubleDoinkerBlue();
   //skills();
   // calibration();
  //Brain.Screen.print("Autonomous Program Complete");
}

void runDriver(void) {
  Brain.Screen.clearScreen();
  Brain.Screen.print("Running Driver Control Mode...");
  driverControl(); // Start driver control function
}

int main()
{
    // Initializing Robot Configuration. DO NOT REMOVE!
    vexcodeInit();
    //initializeOpticalSensor();

    Competition.autonomous(runAuton);
    Competition.drivercontrol(runDriver);
}