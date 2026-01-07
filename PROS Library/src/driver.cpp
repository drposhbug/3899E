#include "main.h"
#include "robot_config.hpp"
#include "utils.hpp"
#include "navigation.hpp"
#include "autontasks.hpp"
#include <cmath>

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
int applyDeadzone(int value) {
    if (abs(value) < deadzoneThreshold) {
        return 0;
    }
    return value;
}

// Main driver control function
void driverControl() {
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
    bool spinForInProgress = false;
    bool isMatchLoadPneumaticsActive = false;
    bool intakeRunning = false;
    int intakeDirection = 0;  // 1=forward, -1=reverse, 0=off

    int maxSpeed = 100;

    while (true) {
        // Get joystick values with deadzone applied
        int targetPowerLeft = applyDeadzone(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
        int targetPowerRight = applyDeadzone(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));

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

        // ==================== BUTTON R1: NORMAL INTAKE ====================
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            if (!wasR1Pressed) {
                frontHoodPneumatics.set_value(true);
                backHoodPneumatics.set_value(false);
                ptoPneumatics.set_value(false);
                wasR1Pressed = true;
            }
            
            spinForInProgress = false;
            intakeMotor1.move_voltage(12000);
            intakeMotor2.move_voltage(12000);
        }
        // ==================== BUTTON RIGHT: REVERSE INTAKE ====================
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            spinForInProgress = false;
            intakeMotor1.move_voltage(-12000);
            intakeMotor2.move_voltage(-12000);
        }
        else {
            if ((wasR1Pressed || !spinForInProgress) && 
                !controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && 
                !controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2) && 
                !controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
                intakeMotor1.brake();
                intakeMotor2.brake();
            }
            wasR1Pressed = false;
        }

        // ==================== BUTTON R2: CHAMBER INTAKE ====================
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            if (!wasR2Pressed) {
                frontHoodPneumatics.set_value(false);
                backHoodPneumatics.set_value(false);
                wasR2Pressed = true;
            }
            
            spinForInProgress = false;
            intakeMotor1.move_voltage(-12000);
            intakeMotor2.move_voltage(-12000);
        }
        else {
            if (wasR2Pressed) {
                intakeMotor1.brake();
                intakeMotor2.brake();
                wasR2Pressed = false;
            }
        }

        // ========================= BUTTON RIGHT: OUTTAKE ====================
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            if (!wasRightPressed) {
                frontHoodPneumatics.set_value(false);
                backHoodPneumatics.set_value(false);
                ptoPneumatics.set_value(false);
                wasRightPressed = true;
            }
            spinForInProgress = false;
            intakeMotor1.move_voltage(-12000);
            intakeMotor2.move_voltage(-12000);
        }
        else {
            if (wasRightPressed) {
                intakeMotor1.brake();
                intakeMotor2.brake();
                wasRightPressed = false;
            }
        }

        // ==================== BUTTON L1: SCORING ====================
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            if (!wasL1Pressed) {
                frontHoodPneumatics.set_value(false);
                backHoodPneumatics.set_value(true);
                ptoPneumatics.set_value(true);
                wasL1Pressed = true;
            }
            
            spinForInProgress = false;
            intakeMotor1.move_voltage(12000);
            intakeMotor2.move_voltage(12000);
        }
        else {
            if (wasL1Pressed) {
                spinForInProgress = true;
                wasL1Pressed = false;
            }
        }

        // ==================== BUTTON L2: MATCH LOAD TOGGLE ====================
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            if (!wasL2Pressed) {
                isMatchLoadPneumaticsActive = !isMatchLoadPneumaticsActive;
                matchLoadPneumatics.set_value(isMatchLoadPneumaticsActive);
                wasL2Pressed = true;
            }
        }
        else {
            wasL2Pressed = false;
        }

        // ==================== BUTTON Y : WING TOGGLE ====================
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
            if (!wasAPressed) {
                // Toggle wing pneumatics
                static bool wingState = false;
                wingState = !wingState;
                wingPneumatics.set_value(wingState);
                wasAPressed = true;
            }
        }
        else {
            wasAPressed = false;
        }

        // ==================== APPLY DRIVE MOTOR POWERS ====================
        // Convert speed (cm/s) to velocity percentage for PROS
        double maxSpeedCmS = absoluteMaxRPM * wheelCircumferenceCM / 60.0;
        
        LeftMotor1.move_velocity(motorPowerLeft[0] / maxSpeedCmS * 600);
        RightMotor1.move_velocity(motorPowerRight[0] / maxSpeedCmS * 600);
        LeftMotor2.move_velocity(motorPowerLeft[1] / maxSpeedCmS * 600);
        RightMotor2.move_velocity(motorPowerRight[1] / maxSpeedCmS * 600);
        LeftMotor3.move_velocity(motorPowerLeft[2] / maxSpeedCmS * 600);
        RightMotor3.move_velocity(motorPowerRight[2] / maxSpeedCmS * 600);

        // Loop delay (20ms = 50Hz update rate)
        pros::delay(20);
    }
}
