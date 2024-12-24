#include "vex.h" // Include the VEX library
#include "robot-config.h" // Include the robot configuration
#include "driver.h" // Include the driver control functions
#include "auton.h" // Include the autonomous functions
#include "utils.h"
#include "navigation.h"




using namespace vex;

competition Competition;
void runAuton(void) {
  Brain.Screen.clearScreen();
  Brain.Screen.print("Running Autonomous Mode...");
    // Call the autonomous routine     
    autonRoutine7(); 
    
  Brain.Screen.print("Autonomous Program Complete");
}

void runDriver(void) {
  Brain.Screen.clearScreen();
  Brain.Screen.print("Running Driver Control Mode...");
  driverControl(); // Start driver control function
}
/*
void armMotorThread(void *) {
    // Your code for the arm motor thread
}

void elbowMotorThread(void *) 
    // Your code for the elbow motor thread
}
*/

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE! note
  //task armm(armTask);
  vexcodeInit();
  initializeOpticalSensor();
  //armMotor.setBrake(brakeType::brake);
  //elbowMotor.setBrake(brakeType::brake);
 Competition.autonomous(runAuton);
 Competition.drivercontrol(runDriver);

  // Prompt to select between autonomous and driver control
  // Controller.Screen.clearScreen();
  // Controller.Screen.print("Press X for Autonomous");
  // Controller.Screen.newLine();
  // Controller.Screen.print("Press A for Driver Control");

  while (true) {
    // if (Controller.ButtonX.pressing()) {
    //   runAuton();
    //   break;
    // } else if (Controller.ButtonA.pressing()) {
    //   runDriver();
    //   while (true) {
    //     this_thread::sleep_for(10); // Small delay to yield control to other tasks
    //   }
    // }
    vexDelay(100); // Small delay to prevent busy waiting
  }
}
