#include "vex.h"          // Include the VEX library
#include "robot_config.h" // Include the robot configuration
#include <cmath>          // Include the cmath library for pow()
#include "utils.h"
#include "navigation.h"

using namespace vex;
static int deadzoneThreshold = 10; // Initial deadzone threshold
// Deadzone function to ignore small joystick movements
static int applyDeadzone(int value)
{
    if (abs(value) < deadzoneThreshold)
    {
        return 0; // Ignore small values within the deadzone
    }
    return value; // Return the original value if outside the deadzone
}

const double RED_HUE_MIN_1 = 340.0; // First red range (340°-360°)
const double RED_HUE_MAX_1 = 360.0;
const double RED_HUE_MIN_2 = 0.0; // Second red range (0°-15°)
const double RED_HUE_MAX_2 = 15.0;
const double BLUE_HUE_MIN = 215.0; // Blue range
const double BLUE_HUE_MAX = 225.0;

// Driver Control Function: Handles joystick input and motor speed adjustments
void driverControl()
{
    initializeOpticalSensor();
    // Arrays to hold the current speeds of the motors
    double motorPowerLeft[3] = {0};
    double motorPowerRight[3] = {0};
    double pistonStartTime = 500;
    bool pistonTimerActive = false;

    // Variables to set deadzone threshold and maximum speed
    int maxSpeed = 100; // Maximum speed percentage

    // Variables to track button press states
    bool wasAPressed = false;
    bool wasR1Pressed = false;
    bool wasR2Pressed = false;
    bool wasL1Pressed = false;
    bool wasL2Pressed = false;
    bool wasXPressed = false;
    bool wasRightPressed = false;
    bool wasYPressed = false;
    bool wasUpPressed = false;
    bool wasDownPressed = false;
    bool wasLeftPressed = false;
    bool wasBumperPressed = false;
    bool bumperEngaged = false;
    bool isMovingDown = false;
    bool spinForInProgress = false;
    bool isGoalPneumaticsActive = true;     // Initial state reversed
    bool isDoinkerPneumaticsLeftActive = false; // Variable to track doinker pneumatics state
    bool isDoinkerPneumaticsRightActive = false; // Variable to track doinker pneumatics state
    bool intakeRunning = false;
    int intakeDirection = 0; // 1 for forward, -1 for reverse, 0 for off

    // Initialize arm position before starting driver control
    // Ensure pneumatics are retracted when reaching Starting position
    armPneumatics.set(false);
    armMotor1.spin(reverse, 100, velocityUnits::pct); // Slower descent
    armMotor2.spin(reverse, 100, velocityUnits::pct); // Slower descent
    isMovingDown = true;

    while (true)
    {

        int targetPowerLeft = applyDeadzone(Controller.Axis3.position());
        int targetPowerRight = applyDeadzone(Controller.Axis2.position());

        // Calculate exponential scaling factor (adjust the 0.8 exponent for different curves)
        double scaleFactor = pow(0.55, abs(targetPowerLeft - targetPowerRight) / (double)maxSpeed);

        // Get controller axis values for left and right sticks
        double targetSpeedLeft = ((targetPowerLeft * scaleFactor) / 100.0) * absoluteMaxRPM * wheelCircumferenceCM / 60.0;
        double targetSpeedRight = ((targetPowerRight * scaleFactor) / 100.0) * absoluteMaxRPM * wheelCircumferenceCM / 60.0;

        motorPowerLeft[0] = targetSpeedLeft;
        motorPowerLeft[1] = targetSpeedLeft;
        motorPowerLeft[2] = targetSpeedLeft;
        motorPowerRight[0] = targetSpeedRight;
        motorPowerRight[1] = targetSpeedRight;
        motorPowerRight[2] = targetSpeedRight;

        // Button R1 Intake
        if (Controller.ButtonR1.pressing())
        {
            spinForInProgress = false;
            intakeMotor.spin(reverse, 12, vex::voltageUnits::volt);
        }
        else if (Controller.ButtonR2.pressing())
        {
            spinForInProgress = false;
            intakeMotor.spin(forward, 12, vex::voltageUnits::volt);
        }
        else if (!spinForInProgress)
        {
            intakeMotor.stop();
        }

        if (spinForInProgress && !intakeMotor.isSpinning())
        {
            spinForInProgress = false;
        }

        // Toggle hookPneumatics and goal lift control (L1)
        if (Controller.ButtonL1.pressing())
        {
            if (!wasL1Pressed)
            {
                // Toggle goalPneumatics
                isGoalPneumaticsActive = !isGoalPneumaticsActive;
                goalPneumatics.set(isGoalPneumaticsActive);
                wasL1Pressed = true;

                if (!isGoalPneumaticsActive)
                {
                    intakeMotor.stop(); // Stop the motor
                    intakeDirection = 0;
                    intakeRunning = false;
                    intakeMotor.spinFor(200, rotationUnits::deg, 100, velocityUnits::pct, false);
                }
            }
        }
        else
        {

            wasL1Pressed = false;
        }


// Hang Button (A)
if (Controller.ButtonA.pressing())
{
    if (!wasAPressed)
    {
        // Move arm to Alliance position
        armMotor1.setBrake(brakeType::hold);
        armMotor2.setBrake(brakeType::hold);
        armMotor1.spinToPosition(Load1, rotationUnits::deg, 40, velocityUnits::pct, false);
        armMotor2.spinToPosition(Load1, rotationUnits::deg, 40, velocityUnits::pct, false);
        armstat = ArmPosition::Load1;
        
        armPneumatics.set(true);  
    }
}
else
{

    wasAPressed = false;
}



        
           // Check if Up or Down button is pressed for manual intake control
           if (Controller.ButtonUp.pressing())
           {
               if (!wasUpPressed)
               {
                   // Toggle the state
                   isDoinkerPneumaticsLeftActive = !isDoinkerPneumaticsLeftActive;
                   doinkerPneumaticsLeft.set(isDoinkerPneumaticsLeftActive);
                   wasUpPressed = true; // Prevent multiple toggles while the button is held
               }
           }
           else
           {
               wasUpPressed = false; // Reset the flag when the button is released
           }

        // Toggle Doinker (Button X)
        if (Controller.ButtonX.pressing())
        {
            if (!wasXPressed)
            {
                // Toggle the state
                isDoinkerPneumaticsRightActive = !isDoinkerPneumaticsRightActive;
                doinkerPneumaticsRight.set(isDoinkerPneumaticsRightActive);
                wasXPressed = true; // Prevent multiple toggles while the button is held
            }
        }
        else
        {
            wasXPressed = false; // Reset the flag when the button is released
        }


        // Button Y logic with toggle mechanism
        if (Controller.ButtonL2.pressing())
        {
            if (!wasL2Pressed)
            {                       // Detect the press event
                wasL2Pressed = true; // Update the state

                // Move from Load to Hover
                if (armstat == ArmPosition::Load1 || armstat == ArmPosition::Load2)
                {
                    //move hook so arm can move
                    intakeMotor.stop(); // Stop the motor
                    spinForInProgress = true;
                    intakeMotor.spinFor(60, rotationUnits::deg, 58, velocityUnits::pct, false);

                    // Activate pneumatics immediately
                    armPneumatics.set(true);  // Activate first pneumatic
        
                    // Move arm to Hover position
                    armMotor1.setBrake(brakeType::hold);
                    armMotor2.setBrake(brakeType::hold);
                    armMotor1.spinToPosition(Hover, rotationUnits::deg, 100, velocityUnits::pct, false);
                    armMotor2.spinToPosition(Hover, rotationUnits::deg, 100, velocityUnits::pct, false);
                    armstat = ArmPosition::Hover;

                    // Move from Hover to Side
                }
                else if (armstat == ArmPosition::Hover)
                {
                    // Move arm to Side position
                    armMotor1.setBrake(brakeType::hold);
                    armMotor2.setBrake(brakeType::hold);
                    armMotor1.spinToPosition(Side, rotationUnits::deg, 60, velocityUnits::pct, false);
                    armMotor2.spinToPosition(Side, rotationUnits::deg, 60, velocityUnits::pct, false);
                    armstat = ArmPosition::Side;
                }
                else if (armstat == ArmPosition::Side)
                {
                    armPneumatics.set(false); 

                    armMotor1.setBrake(brakeType::coast);
                    armMotor2.setBrake(brakeType::coast);
                    armMotor1.spin(reverse, 100, velocityUnits::pct); // Slower descent
                    armMotor2.spin(reverse, 100, velocityUnits::pct); // Slower descent
                    isMovingDown = true;
                }
            }
        }
        else
        {
            wasL2Pressed = false; // Reset the state when the button is released
        }


        // Button Y logic with toggle mechanism
        if (Controller.ButtonY.pressing())
        {
            if (!wasYPressed)
            {                       // Detect the press event
                wasYPressed = true; // Update the state

                // Move from starting to Load1
                if (armstat == ArmPosition::Starting)
                {
                    // Move arm to Alliance position
                    armMotor1.setBrake(brakeType::hold);
                    armMotor2.setBrake(brakeType::hold);
                    armMotor1.spinToPosition(Load1, rotationUnits::deg, 40, velocityUnits::pct, false);
                    armMotor2.spinToPosition(Load1, rotationUnits::deg, 40, velocityUnits::pct, false);
                    armstat = ArmPosition::Load1;

                    // Move from Load1 to Load2
                }
                else if (armstat == ArmPosition::Load1)
                {
                    intakeMotor.stop(); // Stop the motor
                    spinForInProgress = true;
                    intakeMotor.spinFor(60, rotationUnits::deg, 58, velocityUnits::pct, false);
                    // Move arm to Alliance position
                    armMotor1.setBrake(brakeType::hold);
                    armMotor2.setBrake(brakeType::hold);
                    armMotor1.spinToPosition(Load2, rotationUnits::deg, 30, velocityUnits::pct, false);
                    armMotor2.spinToPosition(Load2, rotationUnits::deg, 30, velocityUnits::pct, false);
                    armstat = ArmPosition::Load2;
                }
                else if (armstat == ArmPosition::Load2)
                {
                    armMotor1.setBrake(brakeType::coast);
                    armMotor2.setBrake(brakeType::coast);
                    armMotor1.spin(reverse, 100, velocityUnits::pct); // Slower descent
                    armMotor2.spin(reverse, 100, velocityUnits::pct); // Slower descent
                    isMovingDown = true;
                }
            }
        }
        else
        {
            wasYPressed = false; // Reset the state when the button is released
        }

     

        // Check if Up or Down button is pressed for manual intake control
        if (Controller.ButtonDown.pressing())
        {
            if (!wasDownPressed)
            {                                                              // Detect the press event and ensure arm is not moving
                wasDownPressed = true;                                     // Update the state
                armMotor1.spin(directionType::rev, 50, velocityUnits::pct); // Spin downward at 50% speed
                armMotor2.spin(directionType::rev, 50, velocityUnits::pct); // Spin downward at 50% speed
            }
            // If neither button is pressed, stop the motor
            else
            {
                armMotor1.stop(brakeType::hold); // Stop and hold the arm's position
                armMotor2.stop(brakeType::hold); // Stop and hold the arm's position
                wasDownPressed = false;
            }

        }

        // Toggle Button Left Alliance Stake Score
        if (Controller.ButtonLeft.pressing())
        {
            if (!wasLeftPressed)
            {                        // Detect the press event and ensure arm is not moving
                wasLeftPressed = true; // Update the state

                if (armstat == ArmPosition::Load1 || armstat == ArmPosition::Load2)
                {
                    // Move arm to Alliance position
                    intakeMotor.stop(); // Stop the motor
                    intakeDirection = 0;
                    intakeRunning = false;
                    intakeMotor.spinFor(50, rotationUnits::deg, 58, velocityUnits::pct, false);
                    armMotor1.setBrake(brakeType::hold);
                    armMotor2.setBrake(brakeType::hold);
                    armMotor1.spinToPosition(Alliance, rotationUnits::deg, 90, velocityUnits::pct, false);
                    armMotor2.spinToPosition(Alliance, rotationUnits::deg, 90, velocityUnits::pct, false);
                    armstat = ArmPosition::Alliance;
                }
                else if (armstat == ArmPosition::Alliance)
                { // Added condition
                    // Move arm back to Ready position
                    armPneumatics.set(false);
                    armMotor1.setBrake(brakeType::coast);
                    armMotor2.setBrake(brakeType::coast);
                    armMotor1.spin(reverse, 50, velocityUnits::pct); // Slower descent
                    armMotor2.spin(reverse, 50, velocityUnits::pct); // Slower descent
                    isMovingDown = true;
                }
                else
                {
                    // Move arm to Side position from any othe position
                    armMotor1.setBrake(brakeType::hold);
                    armMotor2.setBrake(brakeType::hold);
                    armMotor1.spinToPosition(Alliance, rotationUnits::deg, 70, velocityUnits::pct, false);
                    armMotor2.spinToPosition(Alliance, rotationUnits::deg, 70, velocityUnits::pct, false);
                    armstat = ArmPosition::Alliance;
                }
            }
        }
        else
        {
            wasLeftPressed = false; // Reset the state when the button is released
        }

/* original left Button for descoring
        // left button for Side Stakes
        if (Controller.ButtonLeft.pressing())
        {
            if (!wasLeftPressed)
            {                          // Detect the press event and ensure arm is not moving
                wasLeftPressed = true; // Update the state

                if (armstat == ArmPosition::Load1 || armstat == ArmPosition::Load2 || armstat == ArmPosition::Ready || armstat == ArmPosition::Side || armstat == ArmPosition::Alliance || armstat == ArmPosition::Starting)
                {
                    pistonTimerActive = false;
                    armMotor1.setBrake(brakeType::hold);
                    armMotor2.setBrake(brakeType::hold);
                    armMotor1.spinToPosition(Descore, rotationUnits::deg, 100, velocityUnits::pct, false);
                    armMotor2.spinToPosition(Descore, rotationUnits::deg, 100, velocityUnits::pct, false);
                    armstat = ArmPosition::Descore;
                }
                else if (armstat == ArmPosition::Descore)
                {
                    // Move arm back to Ready position
                    armMotor1.setBrake(brakeType::coast);
                    armMotor2.setBrake(brakeType::coast);
                    armMotor1.spin(reverse, 100, velocityUnits::pct); // Slower descent
                    armMotor2.spin(reverse, 100, velocityUnits::pct); // Slower descent
                    isMovingDown = true;
                }
                else
                {
                    // Move arm to Side position from any othe position
                    armMotor1.setBrake(brakeType::hold);
                    armMotor2.setBrake(brakeType::hold);
                    armMotor1.spinToPosition(Descore, rotationUnits::deg, 70, velocityUnits::pct, false);
                    armMotor2.spinToPosition(Descore, rotationUnits::deg, 70, velocityUnits::pct, false);
                    armstat = ArmPosition::Descore;
                }
            }
        }
        else
        {
            wasLeftPressed = false; // Reset the state when the button is released
        }
        
*/
        // Right button for Side Stakes
        if (Controller.ButtonRight.pressing())
        {
            if (!wasRightPressed)
            {                           // Detect the press event and ensure arm is not moving
                wasRightPressed = true; // Update the state

                if (armstat == ArmPosition::Load1 || armstat == ArmPosition::Load2)
                {
                    armPneumatics.set(false);

                    // Move arm to Alliance position
                    intakeMotor.stop(); // Stop the motor
                    // intakeMotor.spinFor(100, rotationUnits::deg, 58, velocityUnits::pct, false);
                    spinForInProgress = true;
                    intakeMotor.spinFor(60, rotationUnits::deg, 58, velocityUnits::pct, false);
                    
                    // Activate pneumatics immediately
                    armPneumatics.set(true);  // Activate first pneumatic
                    armMotor1.setBrake(brakeType::hold);
                    armMotor2.setBrake(brakeType::hold);
                    armMotor1.spinToPosition(Side, rotationUnits::deg, 100, velocityUnits::pct, false);
                    armMotor2.spinToPosition(Side, rotationUnits::deg, 100, velocityUnits::pct, false);
                    armstat = ArmPosition::Side;
                }
                else if (armstat == ArmPosition::Side)
                { // Added condition
                    // Move arm back to Ready position
                    armPneumatics.set(false);
                    armMotor1.setBrake(brakeType::coast);
                    armMotor2.setBrake(brakeType::coast);
                    armMotor1.spin(reverse, 100, velocityUnits::pct); // Slower descent
                    armMotor2.spin(reverse, 100, velocityUnits::pct); // Slower descent
                    isMovingDown = true;
                    
                }
                else
                {
                    // Move arm to Side position from any othe position
                    armMotor1.setBrake(brakeType::hold);
                    armMotor2.setBrake(brakeType::hold);
                    armMotor1.spinToPosition(Side, rotationUnits::deg, 70, velocityUnits::pct, false);
                    armMotor2.spinToPosition(Side, rotationUnits::deg, 70, velocityUnits::pct, false);
                    armstat = ArmPosition::Side;
                }
            }
        }
        else
        {
            wasRightPressed = false; // Reset the state when the button is released
        }

        // if (armBumper.value() == 1 ) {  // Bumper is pressed
        if (armBumper.value() == 1 && fabs(armMotor1.velocity(velocityUnits::rpm)) == 0 && fabs(armMotor2.velocity(velocityUnits::rpm)) == 0)
        //if (armBumper.value() == 1 )
        { // Bumper is pressed
            // if (armBumper.value() == 1) {  // Bumper is pressed
            if (!wasBumperPressed && isMovingDown)
            {
                wasBumperPressed = true;
                armMotor1.stop(brakeType::coast);
                armMotor2.stop(brakeType::coast);
                armMotor1.resetPosition();
                armMotor2.resetPosition();
                armstat = ArmPosition::Starting;
                isMovingDown = false;
            }
        }
        else
        {
            wasBumperPressed = false;
        }

        LeftMotor1.spin(forward, motorPowerLeft[0], percent);
        RightMotor1.spin(forward, motorPowerRight[0], percent);

        LeftMotor2.spin(forward, motorPowerLeft[1], percent);
        RightMotor2.spin(forward, motorPowerRight[1], percent);

        LeftMotor3.spin(forward, motorPowerLeft[2], percent);
        RightMotor3.spin(forward, motorPowerRight[2], percent);

        if (pistonTimerActive && Brain.Timer.time(msec) - pistonStartTime >= 50)
        {
            armPneumatics.set(true);
            pistonTimerActive = false;
        }

        // Small delay to prevent wasted resource
        task::sleep(20);
    }
}