#include "vex.h"
#include "robot_config.h"  
#include "driver.h" 
#include "auton.h" 
#include "utils.h"
#include "navigation.h"

using namespace vex;

competition Competition;
void runAuton(void) {
  Brain.Screen.clearScreen();;
  Brain.Screen.print("Running Autonomous Mode...");
  //Reset Arm
  
   // Call the autonomous routine     
   //autonRoutineRedLeft(); 
  calibration();
    
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
    initializeOpticalSensor();

    Competition.autonomous(runAuton);
    Competition.drivercontrol(runDriver);
}
