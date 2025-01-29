#include "vex.h"            // Include the VEX library
#include "robot-config.h"   // Include the robot configuration
#include <cmath>            // Include the cmath library for pow()
#include "utils.h"
#include "navigation.h"


using namespace vex;
static int deadzoneThreshold = 10; // Initial deadzone threshold
// Deadzone function to ignore small joystick movements
static int applyDeadzone(int value) {
    if (abs(value) < deadzoneThreshold) {
        return 0; // Ignore small values within the deadzone
    }
    return value; // Return the original value if outside the deadzone
}

// Add these constants at the top of your file
const double RED_HUE_MIN_1 = 340.0;  // First red range (340°-360°)
const double RED_HUE_MAX_1 = 360.0;
const double RED_HUE_MIN_2 = 0.0;    // Second red range (0°-15°)
const double RED_HUE_MAX_2 = 15.0;
const double BLUE_HUE_MIN = 215.0;   // Blue range
const double BLUE_HUE_MAX = 225.0;

// Driver Control Function: Handles joystick input and motor speed adjustments
void driverControl() {
    initializeOpticalSensor();
    // Arrays to hold the current speeds of the motors
    double motorPowerLeft[3] = {0};
    double motorPowerRight[3] = {0};
    //double currentSpeedLeft = 0;
    //double currentSpeedRight = 0;
    //double minLaunchRPM = minLaunchPower * .01 * maxRPM;
    double pistonStartTime = 500;
    bool pistonTimerActive = false;

    // Variables to set deadzone threshold and maximum speed
    int maxSpeed = 100;         // Maximum speed percentage

    // Variables to track button press states
    bool wasR1Pressed = false;
    bool wasR2Pressed = false;
    bool wasL1Pressed = false;
    bool wasL2Pressed = false;
    bool wasXPressed = false;
    bool wasRightPressed = false;
    bool wasYPressed = false;  
    //bool wasAPressed = false; 
    bool wasUpPressed = false;  
    bool wasDownPressed = false;  
    bool wasLeftPressed = false;  
    bool wasBumperPressed = false; 
    bool bumperEngaged = false;
    bool isMovingDown = false;
    bool spinForInProgress = false;
    // bool isHookPneumaticsActive = false;          // Variable to track clawPneumatics state
    //bool isIntakePneumaticsActive = false;        // Variable to track goalPneumatics state
    bool isGoalPneumaticsActive = true;           // Initial state reversed
    bool isDoinkerPneumaticsActive = false;       // Variable to track doinker pneumatics state
    //bool isElbowPneumaticsActive = false;         // Tracks the state of the elbow pneumatics    
    bool intakeRunning = false;
    //bool manualIntakeControl = false;             // Flag to indicate manual control of intake motor
    //bool completedCycle = true;  // Flag to indicate whether a motor control cycle is complete
    int intakeDirection = 0;                       // 1 for forward, -1 for reverse, 0 for off

    // Initialize arm position before starting driver control
    armMotor.spin(reverse, 100, velocityUnits::pct);  // Slower descent
    isMovingDown = true;

     // Create LaunchControl instances for each wheel
    LaunchControl leftLaunchControl1(LeftMotor1, passiveEncoderLeft,1.05);
    LaunchControl leftLaunchControl2(LeftMotor2, passiveEncoderLeft,1.05);
    LaunchControl leftLaunchControl3(LeftMotor3, passiveEncoderLeft,1.05);    
    
    LaunchControl rightLaunchControl1(RightMotor1, passiveEncoderRight,1.05);
    LaunchControl rightLaunchControl2(RightMotor2, passiveEncoderRight,1.05);
    LaunchControl rightLaunchControl3(RightMotor3, passiveEncoderRight,1.05);    
    
    while (true) {

        int targetPowerLeft = applyDeadzone(Controller.Axis3.position());
        int targetPowerRight = applyDeadzone(Controller.Axis2.position());

        // Calculate exponential scaling factor (adjust the 0.8 exponent for different curves)
        double scaleFactor = pow(0.55, abs(targetPowerLeft - targetPowerRight) / (double)maxSpeed);

        // Get controller axis values for left and right sticks
        double targetSpeedLeft = ((targetPowerLeft * scaleFactor) / 100.0) * absoluteMaxRPM * wheelCircumferenceCM / 60.0;
        double targetSpeedRight = ((targetPowerRight * scaleFactor) / 100.0) * absoluteMaxRPM * wheelCircumferenceCM / 60.0;
    
       // Brain.Screen.clearScreen();
       // Brain.Screen.printAt(1, 40, "Left Target: %.2f, Right Target: %.2f", targetSpeedLeft, targetSpeedRight);
   
   //if (completedCycle) {
        // Apply scaling to motor speeds and spin the motors
       // for (int i = 0; i < 3; i++) {
      
        motorPowerLeft[0] = targetSpeedLeft;
        motorPowerLeft[1] = targetSpeedLeft;
        motorPowerLeft[2] = targetSpeedLeft;
        motorPowerRight[0] = targetSpeedRight;
        motorPowerRight[1] = targetSpeedRight;
        motorPowerRight[2] = targetSpeedRight;

// Color detection
    double hue = opticalSensor.hue();
    Brain.Screen.clearLine(1);  // Clear line 1 before printing
    Brain.Screen.setCursor(1,1);  // Set cursor to beginning of line 1
    if ((hue >= RED_HUE_MIN_1 && hue <= RED_HUE_MAX_1) || 
        (hue >= RED_HUE_MIN_2 && hue <= RED_HUE_MAX_2)) {
      Brain.Screen.print("RED");
    }
    else if (hue >= BLUE_HUE_MIN && hue <= BLUE_HUE_MAX) {
      Brain.Screen.print("BLUE");
    }

/*
//Launch Control
   // Check if the left side's power exceeds the minimum threshold
if (abs(LeftMotor1.velocity(vex::velocityUnits::rpm)) > minLaunchRPM) {
    // Use leftLaunchControl if minLaunchPower threshold is met for the left side
    motorPowerLeft[0] = leftLaunchControl1.adjustSpeed(targetPowerLeft);
} else {
    // Set left motor power directly to target power if below minLaunchPower
    motorPowerLeft[0] = targetPowerLeft;
}

if (abs(LeftMotor2.velocity(vex::velocityUnits::rpm)) > minLaunchRPM) {
    // Use leftLaunchControl if minLaunchPower threshold is met for the left side
    motorPowerLeft[1] = leftLaunchControl1.adjustSpeed(targetPowerLeft);
} else {
    // Set left motor power directly to target power if below minLaunchPower
    motorPowerLeft[1] = targetPowerLeft;
}

if (abs(LeftMotor3.velocity(vex::velocityUnits::rpm)) > minLaunchRPM) {
    // Use leftLaunchControl if minLaunchPower threshold is met for the left side
    motorPowerLeft[2] = leftLaunchControl1.adjustSpeed(targetPowerLeft);
} else {
    // Set left motor power directly to target power if below minLaunchPower
    motorPowerLeft[2] = targetPowerLeft;
}

// Check if the right side's power exceeds the minimum threshold
if (abs(RightMotor1.velocity(vex::velocityUnits::rpm)) > minLaunchRPM) {
    // Use rightLaunchControl if minLaunchPower threshold is met for the right side
    motorPowerRight[0] = rightLaunchControl1.adjustSpeed(targetPowerRight);
} else {
    // Set right motor power directly to target power if below minLaunchPower
    motorPowerRight[0] = targetPowerRight;
}


// Check if the right side's power exceeds the minimum threshold
if (abs(RightMotor2.velocity(vex::velocityUnits::rpm)) > minLaunchRPM) {
    // Use rightLaunchControl if minLaunchPower threshold is met for the right side
    motorPowerRight[1] = rightLaunchControl1.adjustSpeed(targetPowerRight);
} else {
    // Set right motor power directly to target power if below minLaunchPower
    motorPowerRight[1] = targetPowerRight;
}


// Check if the right side's power exceeds the minimum threshold
if (abs(RightMotor3.velocity(vex::velocityUnits::rpm)) > minLaunchRPM) {
    // Use rightLaunchControl if minLaunchPower threshold is met for the right side
    motorPowerRight[2] = rightLaunchControl1.adjustSpeed(targetPowerRight);
} else {
    // Set right motor power directly to target power if below minLaunchPower
    motorPowerRight[2] = targetPowerRight;
}



*/


    
        //Brain.Screen.clearScreen();
       // Brain.Screen.printAt(1, 40, "Left Motor: %.2f, Right Motor: %.2f", motorPowerLeft, motorPowerRight);

       
/*


    // Arrays to store the motor power values for each wheel
    double motorPowerLeft[3];
    double motorPowerRight[3];

    // Adjust each left motor and convert adjusted speed to motor power
    double adjustedSpeedLeft1 = leftLaunchControl1.adjustSpeed(targetSpeed);
    motorPowerLeft[0] = SpeedToMotorPower(adjustedSpeedLeft1);
    LeftMotor1.spin(vex::directionType::fwd, motorPowerLeft[0], vex::voltageUnits::pct);

    double adjustedSpeedLeft2 = leftLaunchControl2.adjustSpeed(targetSpeed);
    motorPowerLeft[1] = SpeedToMotorPower(adjustedSpeedLeft2);
    LeftMotor2.spin(vex::directionType::fwd, motorPowerLeft[1], vex::voltageUnits::pct);

    double adjustedSpeedLeft3 = leftLaunchControl3.adjustSpeed(targetSpeed);
    motorPowerLeft[2] = SpeedToMotorPower(adjustedSpeedLeft3);
    LeftMotor3.spin(vex::directionType::fwd, motorPowerLeft[2], vex::voltageUnits::pct);

    // Adjust each right motor and convert adjusted speed to motor power
    double adjustedSpeedRight1 = rightLaunchControl1.adjustSpeed(targetSpeed);
    motorPowerRight[0] = SpeedToMotorPower(adjustedSpeedRight1);
    RightMotor1.spin(vex::directionType::fwd, motorPowerRight[0], vex::voltageUnits::pct);

    double adjustedSpeedRight2 = rightLaunchControl2.adjustSpeed(targetSpeed);
    motorPowerRight[1] = SpeedToMotorPower(adjustedSpeedRight2);
    RightMotor2.spin(vex::directionType::fwd, motorPowerRight[1], vex::voltageUnits::pct);

    double adjustedSpeedRight3 = rightLaunchControl3.adjustSpeed(targetSpeed);
    motorPowerRight[2] = SpeedToMotorPower(adjustedSpeedRight3);
    RightMotor3.spin(vex::directionType::fwd, motorPowerRight[2], vex::voltageUnits::pct);

    // Optional: Display converted motor power values on the Brain screen for debugging
    Brain.Screen.clearLine();
    Brain.Screen.printAt(10, 20, "Motor Power Left: %.2f, %.2f, %.2f", motorPowerLeft[0], motorPowerLeft[1], motorPowerLeft[2]);
    Brain.Screen.printAt(10, 40, "Motor Power Right: %.2f, %.2f, %.2f", motorPowerRight[0], motorPowerRight[1], motorPowerRight[2]);
}

*/



        //Brain.Screen.clearScreen();
        //Brain.Screen.printAt(1, 20, "Lt Encoder: %.2f, Rt Encoder: %.2f", encoderSpeedLeft, encoderSpeedRight);
       // Brain.Screen.printAt(1, 60, "Lt MotorPower: %.2f, Rt MotorPower: %.2f", motorPowerLeft[1], motorPowerRight[1]);

       // completedCycle = false;

    //    }

        // After completing the control cycle for all motors, set the flag to true
   //     completedCycle = true;
 //  }
        // Step 6: Handle Driver Buttons and Mechanisms
        // --- Existing Button Handling Code ---

        // Example: Handling R1 Button for Intake Control

        //R1 intake Button
 // Button R1 Intake
if (Controller.ButtonR1.pressing()) {
  spinForInProgress = false;
  intakeMotor.spin(reverse, 12, vex::voltageUnits::volt);
} else if (Controller.ButtonR2.pressing()) {
  spinForInProgress = false;
  intakeMotor.spin(forward, 12, vex::voltageUnits::volt);
} else if (!spinForInProgress) {
  intakeMotor.stop();
}

if (spinForInProgress && !intakeMotor.isSpinning()) {
    spinForInProgress = false;
}
        /*
        if (Controller.ButtonR1.pressing()) {
            if (!wasR1Pressed) {  // Toggle on the press event
                if (intakeDirection != 1) {
                    intakeMotor.spin(reverse, 12, voltageUnits::volt); // Spin forward at full speed
                    intakeDirection = 1;
                    intakeRunning = true;
                } else {
                    intakeMotor.stop(); // Stop the motor
                    intakeDirection = 0;
                    intakeRunning = false;
                }
                wasR1Pressed = true;
            }
        } else {
            wasR1Pressed = false;
        }
        */

        // Handle R2 Button for Outake
        /*
        if (Controller.ButtonR2.pressing()) {
            if (!wasR2Pressed) {  // Toggle on the press event
                if (intakeDirection != -1) {
                    intakeMotor.spin(forward, 12, voltageUnits::volt); // Spin in reverse at full speed
                    intakeDirection = -1;
                    intakeRunning = true;
                } else {
                    intakeMotor.stop(); // Stop the motor
                    intakeDirection = 0;
                    intakeRunning = false;
                }
                wasR2Pressed = true;
            }
        } else {
            wasR2Pressed = false;
        }
*/
        // Toggle Doinker (Button X)
        if (Controller.ButtonX.pressing()) {
            if (!wasXPressed) {
                // Toggle the state
                isDoinkerPneumaticsActive = !isDoinkerPneumaticsActive;
                doinkerPneumatics.set(isDoinkerPneumaticsActive);
                wasXPressed = true;  // Prevent multiple toggles while the button is held
            }
        } else {
            wasXPressed = false;  // Reset the flag when the button is released
        }

        // Toggle hookPneumatics and goal lift control (L1)
        if (Controller.ButtonL1.pressing()) {
            if (!wasL1Pressed) {
                // Toggle goalPneumatics
                isGoalPneumaticsActive = !isGoalPneumaticsActive;
                goalPneumatics.set(isGoalPneumaticsActive);
                wasL1Pressed = true;

                if (!isGoalPneumaticsActive) {
                intakeMotor.stop(); // Stop the motor
                intakeDirection = 0;
                intakeRunning = false;
                intakeMotor.spinFor(200, rotationUnits::deg, 100, velocityUnits::pct, false);
        }
            }
        } else {
           
            wasL1Pressed = false;
        }

        // Right button for Side Stakes
if (Controller.ButtonRight.pressing()) { 
    if (!wasRightPressed) { // Detect the press event and ensure arm is not moving
        wasRightPressed = true; // Update the state

        if (armstat == ArmPosition::Load1 || armstat == ArmPosition::Load2) {
            // Move arm to Alliance position
            intakeMotor.stop(); // Stop the motor
            //intakeMotor.spinFor(100, rotationUnits::deg, 58, velocityUnits::pct, false);
            spinForInProgress = true;
            intakeMotor.spinFor(60, rotationUnits::deg, 58, velocityUnits::pct, false);
            armMotor.setBrake(brakeType::hold);
            armMotor.spinToPosition(Side, rotationUnits::deg, 100, velocityUnits::pct, false);
            armstat = ArmPosition::Side;
                      // Fire piston immediately
            //armPneumatics.set(true); 
            // Start the timer for piston
            pistonStartTime = Brain.Timer.time(msec);
            pistonTimerActive = true; 
/*
        } else if (armstat == ArmPosition::Ready) { // Added condition
            // Move arm back to Ready position
                  armPneumatics.set(true); 
           armMotor.setBrake(brakeType::hold);        
            armMotor.spinToPosition(Side, rotationUnits::deg, 100, velocityUnits::pct, false);
            armstat = ArmPosition::Side;
*/            
        
        } else if (armstat == ArmPosition::Side) { // Added condition
            // Move arm back to Ready position
            armPneumatics.set(false); 
            armMotor.setBrake(brakeType::hold);        
            armMotor.spinToPosition(Load1, rotationUnits::deg, 70, velocityUnits::pct, false);
            armstat = ArmPosition::Load1;

          } else {
            // Move arm to Side position from any othe position
            armMotor.setBrake(brakeType::hold);
            armMotor.spinToPosition(Side, rotationUnits::deg, 70, velocityUnits::pct, false);
            armstat = ArmPosition::Side;
        }
    }
} else {
    wasRightPressed = false; // Reset the state when the button is released
}

/*
if (pistonTimerActive) {
    if (Brain.Timer.time(msec) - pistonStartTime >= 500) {
        armPneumatics.set(true);
        pistonTimerActive = false;
    }   
}
*/
/*
    // Button Y logic with toggle mechanism
    if (Controller.ButtonY.pressing()) {
        if (!wasYPressed) { // Detect the press event
                wasYPressed = true; // Update the state

                //Move from starting to Load1
                if (armstat == ArmPosition::Starting) {
                    // Move arm to Alliance position
                    armMotor.setBrake(brakeType::hold);
                    armMotor.spinToPosition(Load1, rotationUnits::deg, 70, velocityUnits::pct, false);
                    armstat = ArmPosition::Load1;

                //Move from Load1 to Load2
                }else if (armstat == ArmPosition::Load1) {
                    // Move arm to Alliance position
                    armMotor.setBrake(brakeType::hold);
                    armMotor.spinToPosition(Load2, rotationUnits::deg, 70, velocityUnits::pct, false);
                    armstat = ArmPosition::Load2;

                } else if (armstat != ArmPosition::Starting && (armstat != Load1 or armstat != Load2)) {
                    // Move arm to Starting position
                    armMotor.setBrake(brakeType::hold);
                    //armMotor.spinToPosition(Starting, rotationUnits::deg, 100, velocityUnits::pct, false);
                    armMotor.spinToPosition(Starting, rotationUnits::deg, 100, velocityUnits::pct, false);
                    armstat = ArmPosition::Starting;

                } else if (armstat == ArmPosition::Load1 or armstat == ArmPosition::Load2) { // Added condition
                    // Move arm back to Ready position
                    armMotor.setBrake(brakeType::hold);        
                    armMotor.spinToPosition(Starting, rotationUnits::deg, 60, velocityUnits::pct, false);
                    armstat = ArmPosition::Starting;
                }
            }
        } else {
            wasYPressed = false; // Reset the state when the button is released
        }
*/

// Button Y logic with toggle mechanism
if (Controller.ButtonY.pressing()) {
    if (!wasYPressed) { // Detect the press event
            wasYPressed = true; // Update the state

            //Move from starting to Load1
            if (armstat == ArmPosition::Starting) {
                // Move arm to Alliance position
                armMotor.setBrake(brakeType::hold);
                armMotor.spinToPosition(Load1, rotationUnits::deg, 40, velocityUnits::pct, false);
                armstat = ArmPosition::Load1;

            //Move from Load1 to Load2
            } else if (armstat == ArmPosition::Load1) {
                intakeMotor.stop(); // Stop the motor
                spinForInProgress = true;
                intakeMotor.spinFor(60, rotationUnits::deg, 58, velocityUnits::pct, false);
                // Move arm to Alliance position
                armMotor.setBrake(brakeType::hold);
                armMotor.spinToPosition(Load2, rotationUnits::deg, 30, velocityUnits::pct, false);
                armstat = ArmPosition::Load2;

} else if (armstat == ArmPosition::Load2) {
    armMotor.setBrake(brakeType::brake);        
    armMotor.spin(reverse, 100, velocityUnits::pct);  // Slower descent
    isMovingDown = true;

} else if (armstat == ArmPosition::Side || armstat == ArmPosition::Ready ) {
                // Move arm to Alliance position
                armPneumatics.set(false); 
                armMotor.setBrake(brakeType::hold);
                armMotor.spinToPosition(Load1, rotationUnits::deg, 70, velocityUnits::pct, false);
                armstat = ArmPosition::Load1;
            }

            else if (armstat == ArmPosition::Alliance) {
                // Move arm to Alliance position
                armMotor.setBrake(brakeType::hold);
                armMotor.spinToPosition(Load1, rotationUnits::deg, 70, velocityUnits::pct, false);
                armstat = ArmPosition::Load1;
            }
        }
    } else {
        wasYPressed = false; // Reset the state when the button is released
    }



        // Toggle Button L2 Alliance Stake Score
 if (Controller.ButtonL2.pressing()) { 
            if (!wasL2Pressed) { // Detect the press event and ensure arm is not moving
                wasL2Pressed = true; // Update the state

                if (armstat == ArmPosition::Load1 || armstat == ArmPosition::Load2) {
                    // Move arm to Alliance position
                    intakeMotor.stop(); // Stop the motor
                    intakeDirection = 0;
                    intakeRunning = false;
                    intakeMotor.spinFor(50, rotationUnits::deg, 58, velocityUnits::pct, false);
                    armMotor.setBrake(brakeType::hold);
                    armMotor.spinToPosition(Alliance, rotationUnits::deg, 90, velocityUnits::pct, false);
                    armstat = ArmPosition::Alliance;

/*
                if (armstat == ArmPosition::Starting || armstat == ArmPosition::Load) {
                    // Move arm to Alliance position
                    intakeMotor.stop(); // Stop the motor
                    intakeDirection = 0;
                    intakeRunning = false;
                    intakeMotor.spinFor(50, rotationUnits::deg, 58, velocityUnits::pct, false);
                    armMotor.setBrake(brakeType::hold);
                    armMotor.spinToPosition(Side, rotationUnits::deg, 70, velocityUnits::pct, false);
                    armstat = ArmPosition::Side;
*/
                
                } else if (armstat == ArmPosition::Alliance) { // Added condition
                    // Move arm back to Ready position
                    armMotor.setBrake(brakeType::hold);        
                    armMotor.spinToPosition(Load1, rotationUnits::deg, 80, velocityUnits::pct, false);
                    armstat = ArmPosition::Load1;
                  
                } else {
                    // Move arm to Side position from any othe position
                    armMotor.setBrake(brakeType::hold);
                    armMotor.spinToPosition(Alliance, rotationUnits::deg, 70, velocityUnits::pct, false);
                    armstat = ArmPosition::Alliance;
                }
            }
        } else {
            wasL2Pressed = false; // Reset the state when the button is released
        }


        // Check if Up or Down button is pressed for manual intake control
        if (Controller.ButtonUp.pressing()) {
            if (!wasUpPressed) { // Detect the press event and ensure arm is not moving
                wasUpPressed = true; // Update the state
        armMotor.spin(directionType::fwd, 50, velocityUnits::pct); // Spin downward at 50% speed
        }
        // If neither button is pressed, stop the motor
        else {
            armMotor.stop(brakeType::hold); // Stop and hold the arm's position
             wasUpPressed = false;
        }

        }

        // Check if Up or Down button is pressed for manual intake control
        if (Controller.ButtonDown.pressing()) {
            if (!wasDownPressed) { // Detect the press event and ensure arm is not moving
                wasDownPressed = true; // Update the state
        armMotor.spin(directionType::rev, 50, velocityUnits::pct); // Spin downward at 50% speed
        }
        // If neither button is pressed, stop the motor
        else {
            armMotor.stop(brakeType::hold); // Stop and hold the arm's position
             wasDownPressed = false;
        }


        // Check if Up or Down button is pressed for manual intake control
        if (Controller.ButtonLeft.pressing()) {
            if (!wasLeftPressed) { // Detect the press event and ensure arm is not moving
                wasDownPressed = true; // Update the state
   // Reset motor encoder
    armMotor.resetPosition();

    // Update arm state
    armstat = ArmPosition::Starting;
        }
        // If neither button is pressed, stop the motor
        else {
             wasLeftPressed = false;
        }




        }
        }
//if (armBumper.value() == 1 ) {  // Bumper is pressed
if (armBumper.value() == 1 && fabs(armMotor.velocity(velocityUnits::rpm)) < 0.5) {  // Bumper is pressed
//if (armBumper.value() == 1) {  // Bumper is pressed
    if (!wasBumperPressed && isMovingDown) {
        wasBumperPressed = true;
        armMotor.stop(brakeType::coast);
        armMotor.resetPosition();
        armstat = ArmPosition::Starting;
        isMovingDown = false;
    }
} else {
    wasBumperPressed = false;
}


        LeftMotor1.spin(forward, motorPowerLeft[0], percent);
        RightMotor1.spin(forward, motorPowerRight[0], percent);
        
        LeftMotor2.spin(forward, motorPowerLeft[1], percent);
        RightMotor2.spin(forward, motorPowerRight[1], percent);
         
        LeftMotor3.spin(forward, motorPowerLeft[2], percent);
        RightMotor3.spin(forward, motorPowerRight[2], percent);

        //Brain.Screen.printAt(10, 180, "Lt Motor Speed: %.2f", leftMotor[2].velocity(vex::velocityUnits::pct));
        //Brain.Screen.printAt(10, 200, "Rt Motor Speed: %.2f", rightMotor[2].velocity(vex::velocityUnits::rpm));

        if (pistonTimerActive && Brain.Timer.time(msec) - pistonStartTime >= 50) {
    armPneumatics.set(true);
    pistonTimerActive = false;
}

        // Small delay to prevent wasted resource
        task::sleep(20);
    }
}