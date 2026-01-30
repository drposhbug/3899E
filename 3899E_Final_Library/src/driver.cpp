#include "vex.h"          // Include the VEX library
#include "robot_config.h" // Include the robot configuration
#include <cmath>          // Include the cmath library for pow()
#include "utils.h"
#include "navigation.h"
#include "autontasks.h"

using namespace vex;

// Color detection thresholds for optical sensor
const double RED_HUE_MIN_1 = 340.0;
const double RED_HUE_MAX_1 = 360.0;
const double RED_HUE_MIN_2 = 0.0;
const double RED_HUE_MAX_2 = 15.0;
const double BLUE_HUE_MIN = 215.0;
const double BLUE_HUE_MAX = 225.0;

// Joystick deadzone threshold (prevent drift)
static int deadzoneThreshold = 10;

// Filters out small joystick movements within deadzone
int applyDeadzone(int value)
{
    if (abs(value) < deadzoneThreshold)
    {
        return 0;
    }
    return value;
}

// Main driver control function
void driverControl()
{
    initializeOpticalSensor();
    headingDisplayParams.isRunning = false;

    // Motor power arrays for 3 motors per side
    double motorPowerLeft[3] = {0};
    double motorPowerRight[3] = {0};

    // Button state tracking (prevents multiple triggers per press)
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
    
    // Intake control flags
    bool spinForInProgress = false;  // Tracks if timed motor movement is running
    bool isMatchLoadPneumaticsActive = false;
    bool intakeRunning = false;
    int intakeDirection = 0;  // 1=forward, -1=reverse, 0=off

    int maxSpeed = 100;


    while (true)
    {
        // Get joystick values with deadzone applied
        int targetPowerLeft = applyDeadzone(Controller.Axis3.position());
        int targetPowerRight = applyDeadzone(Controller.Axis2.position());

        // Reduce turning sensitivity at high speeds (exponential scaling)
        double scaleFactor = pow(0.55, abs(targetPowerLeft - targetPowerRight) / (double)maxSpeed);

        // Convert joystick percent to motor speed (cm/s)
        double targetSpeedLeft = ((targetPowerLeft * scaleFactor) / 100.0) * absoluteMaxRPM * wheelCircumferenceCM / 60.0;
        double targetSpeedRight = ((targetPowerRight * scaleFactor) / 100.0) * absoluteMaxRPM * wheelCircumferenceCM / 60.0;

        
        // Apply speeds to all motors
        motorPowerLeft[0] = targetSpeedLeft;
        motorPowerLeft[1] = targetSpeedLeft;
        motorPowerLeft[2] = targetSpeedLeft;
        motorPowerRight[0] = targetSpeedRight;
        motorPowerRight[1] = targetSpeedRight;
        motorPowerRight[2] = targetSpeedRight;

/*
// Convert joystick percent to motor speed (cm/s)
        double targetSpeedLeft = ((targetPowerLeft * scaleFactor) / 100.0) * absoluteMaxRPM * wheelCircumferenceCM / 60.0;
        double targetSpeedRight = ((targetPowerRight * scaleFactor) / 100.0) * absoluteMaxRPM * wheelCircumferenceCM / 60.0;

       
        // ADDED: Deceleration ramping to prevent tipping
        static double currentSpeedLeft = 0;
        static double currentSpeedRight = 0;
        const double DECEL_RATE = 0.50;              // Normal deceleration (0.3=slow/safe, 0.5=balanced, 0.8=fast/responsive)
        const double DIRECTION_CHANGE_RATE = 0.25;    // Direction reversal rate (0.15=very safe, 0.25=balanced, 0.45=quick)
        const double NEAR_ZERO_THRESHOLD = 12;        // Speed to allow direction change (5=lenient, 12=balanced, 20=strict)

        // Detect direction changes
        bool leftDirChange = (targetSpeedLeft * currentSpeedLeft < 0) && (fabs(currentSpeedLeft) > NEAR_ZERO_THRESHOLD);
        bool rightDirChange = (targetSpeedRight * currentSpeedRight < 0) && (fabs(currentSpeedRight) > NEAR_ZERO_THRESHOLD);

        // Apply ramping (decel only)
        if (leftDirChange) {
            currentSpeedLeft += (0 - currentSpeedLeft) * DIRECTION_CHANGE_RATE;
        } else if (fabs(targetSpeedLeft) < fabs(currentSpeedLeft)) {
            currentSpeedLeft += (targetSpeedLeft - currentSpeedLeft) * DECEL_RATE;
        } else {
            currentSpeedLeft = targetSpeedLeft;
        }

        if (rightDirChange) {
            currentSpeedRight += (0 - currentSpeedRight) * DIRECTION_CHANGE_RATE;
        } else if (fabs(targetSpeedRight) < fabs(currentSpeedRight)) {
            currentSpeedRight += (targetSpeedRight - currentSpeedRight) * DECEL_RATE;
        } else {
            currentSpeedRight = targetSpeedRight;
        }

        if (fabs(targetSpeedLeft - currentSpeedLeft) < 2) currentSpeedLeft = targetSpeedLeft;
        if (fabs(targetSpeedRight - currentSpeedRight) < 2) currentSpeedRight = targetSpeedRight;

        // Apply ramped speeds to all motors
        motorPowerLeft[0] = currentSpeedLeft;
        motorPowerLeft[1] = currentSpeedLeft;
        motorPowerLeft[2] = currentSpeedLeft;
        motorPowerRight[0] = currentSpeedRight;
        motorPowerRight[1] = currentSpeedRight;
        motorPowerRight[2] = currentSpeedRight;

        */

        // ==================== BUTTON R1: NORMAL INTAKE ====================
        // Front hood closed, back hood open - standard intake position
        if (Controller.ButtonR1.pressing())
        {
            // Only set pneumatics ONCE when button is first pressed (not every frame)
            if (!wasR1Pressed)
            {
                frontHoodPneumatics.set(true);     // Close front hood for intake
                backHoodPneumatics.set(false);    // Open back hood for intake
                ptoPneumatics.set(false);
                indexPneumatics.set(true);

                wasR1Pressed = true;               // Mark that we've handled the press
            }
            
            spinForInProgress = false;
            intakeMotor1.spin(forward, 12, vex::voltageUnits::volt);
            intakeMotor2.spin(forward, 12, vex::voltageUnits::volt);
        }
        // ==================== BUTTON RIGHT: REVERSE INTAKE ====================
        // Eject cubes without changing hood position
        else if (Controller.ButtonRight.pressing())
        {
            spinForInProgress = false;
            intakeMotor1.spin(reverse, 12, vex::voltageUnits::volt);
            intakeMotor2.spin(reverse, 12, vex::voltageUnits::volt);
        }
        // Stop intake motors when no intake buttons pressed (and no other intake active)
        else
        {
            if ((wasR1Pressed || !spinForInProgress) && !Controller.ButtonR2.pressing() && 
                !Controller.ButtonL2.pressing() && !Controller.ButtonL1.pressing())
            {
                intakeMotor1.stop();
                intakeMotor2.stop();
            }
            wasR1Pressed = false;
        }

        // ==================== BUTTON R2: CHAMBER INTAKE ====================
        // Close both hoods to trap cubes in launch chamber
        if (Controller.ButtonR2.pressing())
        {
            if (!wasR2Pressed)
            {
                frontHoodPneumatics.set(false);     // Close front hood
                backHoodPneumatics.set(false);     // Close back hood
                indexPneumatics.set(false);
                wasR2Pressed = true;
            }
            
            spinForInProgress = false;
            intakeMotor1.spin(reverse, 12, vex::voltageUnits::volt);
            intakeMotor2.spin(reverse, 12, vex::voltageUnits::volt);
        }
        else
        {
            if (wasR2Pressed)
            {
                intakeMotor1.stop();
                intakeMotor2.stop();
                wasR2Pressed = false;
            }
        }

        // ========================= BUTTON RIGHT: OUTTAKE ====================
        // close front hood, close back hood, run intake in reverse
        if (Controller.ButtonRight.pressing())
        {
            if (!wasRightPressed)
            {
                frontHoodPneumatics.set(false);     // Close front hood for outtake
                backHoodPneumatics.set(false);      // Close back hood for outtake
                ptoPneumatics.set(false);
                indexPneumatics.set(true);
                wasRightPressed = true;
            }
            spinForInProgress = false;
            intakeMotor1.spin(reverse, 8, vex::voltageUnits::volt);
            intakeMotor2.spin(reverse, 8, vex::voltageUnits::volt);
        }
        else
        {
            if (wasRightPressed)
            {
                intakeMotor1.stop();
                intakeMotor2.stop();
                indexPneumatics.set(false);
                wasRightPressed = false;
            }
        }

        // ==================== BUTTON L1: SCORING ====================
        // Open both hoods, run intake to score, retract on release
        if (Controller.ButtonL1.pressing())
        {
            if (!wasL1Pressed)
            {
                frontHoodPneumatics.set(false);      // Open front hood
                backHoodPneumatics.set(true);      // Open back hood
                ptoPneumatics.set(true);
                indexPneumatics.set(false);
                wasL1Pressed = true;
            }
            
            spinForInProgress = false;
            intakeMotor1.spin(forward, 12, vex::voltageUnits::volt);
            intakeMotor2.spin(forward, 12, vex::voltageUnits::volt);
        }
        else
        {
            // When button released: close front hood and retract cubes
            if (wasL1Pressed)
            {          
                // Briefly reverse intake to pull cubes away from hood (non-blocking)
                //intakeMotor1.spinFor(reverse, 90, rotationUnits::deg, 100, velocityUnits::pct, false);
                //intakeMotor2.spinFor(forward, 90, rotationUnits::deg, 100, velocityUnits::pct, false);
                
                spinForInProgress = true;
                wasL1Pressed = false;
            }
        }

        // ==================== BUTTON L2: MATCH LOAD TOGGLE ====================
        // Toggle match load pneumatics and intake motors together
        if (Controller.ButtonL2.pressing())
        {
            if (!wasL2Pressed)
            {
                isMatchLoadPneumaticsActive = !isMatchLoadPneumaticsActive;
                matchLoadPneumatics.set(isMatchLoadPneumaticsActive);
                                
                if (isMatchLoadPneumaticsActive)
                {
                    // Pneumatic extended - start intake
                    //intakeMotor1.spin(reverse, 12, vex::voltageUnits::volt);
                    //intakeMotor2.spin(reverse, 12, vex::voltageUnits::volt);
                }
                else
                {
                    // Pneumatic retracted - stop intake
                    //intakeMotor1.stop();
                    //intakeMotor2.stop();
                    //intakeDirection = 0;
                    //intakeRunning = false;
                }
                
                wasL2Pressed = true;
            }
        }
        else
        {
            wasL2Pressed = false;
        }

        /*// ==================== BUTTON X : JUSTIN YUEH SPECIAL BUTTON ====================
        //all in
        if (Controller.ButtonX.pressing())
        {
            if (!wasXPressed)
            {
                frontHoodPneumatics.set(true);      // open everything
                backHoodPneumatics.set(true);       
                matchLoadPneumatics.set(true);      
                wasXPressed = true;
            }
            
            spinForInProgress = false;
            intakeMotor1.spin(forward, 12, vex::voltageUnits::volt);
            intakeMotor2.spin(forward, 12, vex::voltageUnits::volt);
            leftMotor[0].spin(forward, 12, vex::voltageUnits::volt);
            leftMotor[1].spin(forward, 12, vex::voltageUnits::volt);
            leftMotor[2].spin(forward, 12, vex::voltageUnits::volt);
            rightMotor[0].spin(forward, 12, vex::voltageUnits::volt);
            rightMotor[1].spin(forward, 12, vex::voltageUnits::volt);
            rightMotor[2].spin(forward, 12, vex::voltageUnits::volt);
        }
        else
        {
            if (wasXPressed)
            {
                intakeMotor1.stop();
                intakeMotor2.stop();
                leftMotor[0].stop();
                leftMotor[1].stop();
                leftMotor[2].stop();
                rightMotor[0].stop();
                rightMotor[1].stop();
                rightMotor[2].stop();
                wasXPressed = false;
            }
        }*/

        // ==================== BUTTON Y : WING TOGGLE ====================
        if (Controller.ButtonY.pressing())
        {
            if (!wasAPressed)
            {
                wingPneumatics.set(!wingPneumatics.value()); // Toggle wing pneumatics
                wasAPressed = true;
            }
        }
        else
        {
            wasAPressed = false;
        }

        // ==================== APPLY DRIVE MOTOR POWERS ====================
        LeftMotor1.spin(forward, motorPowerLeft[0], percent);
        RightMotor1.spin(forward, motorPowerRight[0], percent);

        LeftMotor2.spin(forward, motorPowerLeft[1], percent);
        RightMotor2.spin(forward, motorPowerRight[1], percent);

        LeftMotor3.spin(forward, motorPowerLeft[2], percent);
        RightMotor3.spin(forward, motorPowerRight[2], percent);

        // Loop delay (20ms = 50Hz update rate)
        task::sleep(20);
    }
}