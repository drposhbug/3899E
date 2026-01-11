#include "main.h"
#include "robot_config.hpp"
#include "utils.hpp"
#include "navigation.hpp"
#include "autontasks.hpp"
#include <cmath>

// Color detection thresholds
const double RED_HUE_MIN_1 = 340.0;
const double RED_HUE_MAX_1 = 360.0;
const double RED_HUE_MIN_2 = 0.0;
const double RED_HUE_MAX_2 = 15.0;
const double BLUE_HUE_MIN = 215.0;
const double BLUE_HUE_MAX = 225.0;

// Joystick deadzone
static int deadzoneThreshold = 10;

int applyDeadzone(int value) {
    if (abs(value) < deadzoneThreshold) {
        return 0;
    }
    return value;
}

// Main driver control function
void driverControl() {
    initializeOpticalSensor();
    
    // FIX: Use the function instead of the deleted struct
    stopHeadingDisplay(); 

    // Button state tracking
    bool wasAPressed = false;
    bool wasR1Pressed = false;
    bool wasR2Pressed = false;
    bool wasL1Pressed = false;
    bool wasL2Pressed = false;
    bool wasXPressed = false;
    bool wasRightPressed = false;
    bool wasYPressed = false;
    
    // Intake control flags
    bool spinForInProgress = false;
    bool isMatchLoadPneumaticsActive = false;

    int maxSpeed = 100;

    while (true) {
        // Get joystick values
        int targetPowerLeft = applyDeadzone(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
        int targetPowerRight = applyDeadzone(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));

        double scaleFactor = pow(0.55, abs(targetPowerLeft - targetPowerRight) / (double)maxSpeed);

        double targetSpeedLeft = ((targetPowerLeft * scaleFactor) / 100.0) * absoluteMaxRPM * wheelCircumferenceCM / 60.0;
        double targetSpeedRight = ((targetPowerRight * scaleFactor) / 100.0) * absoluteMaxRPM * wheelCircumferenceCM / 60.0;

        // R1: Normal Intake
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            if (!wasR1Pressed) {
                frontHoodPneumatics.set_value(true);
                backHoodPneumatics.set_value(false);
                ptoPneumatics.set_value(false);
                wasR1Pressed = true;
            }
            spinForInProgress = false;
            intakeMotors.move_voltage(12000);
        }
        // Right: Reverse Intake
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            spinForInProgress = false;
            intakeMotors.move_voltage(-12000);
        }
        else {
            if ((wasR1Pressed || !spinForInProgress) && 
                !controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && 
                !controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2) && 
                !controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
                intakeMotors.brake();
            }
            wasR1Pressed = false;
        }

        // R2: Chamber Intake
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            if (!wasR2Pressed) {
                frontHoodPneumatics.set_value(false);
                backHoodPneumatics.set_value(false);
                wasR2Pressed = true;
            }
            spinForInProgress = false;
            intakeMotors.move_voltage(-12000);
        }
        else {
            if (wasR2Pressed) {
                intakeMotors.brake();
                wasR2Pressed = false;
            }
        }

        // L1: Scoring
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            if (!wasL1Pressed) {
                frontHoodPneumatics.set_value(false);
                backHoodPneumatics.set_value(true);
                ptoPneumatics.set_value(true);
                wasL1Pressed = true;
            }
            spinForInProgress = false;
            intakeMotors.move_voltage(12000);
        }
        else {
            if (wasL1Pressed) {
                spinForInProgress = true;
                wasL1Pressed = false;
            }
        }

        // L2: Match Load Toggle
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

        // Y: Wing Toggle
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
            if (!wasAPressed) {
                static bool wingState = false;
                wingState = !wingState;
                wingPneumatics.set_value(wingState);
                wasAPressed = true;
            }
        }
        else {
            wasAPressed = false;
        }

        // Drive Motors
        double maxSpeedCmS = absoluteMaxRPM * wheelCircumferenceCM / 60.0;
        leftMotors.move_velocity(targetSpeedLeft / maxSpeedCmS * 600);
        rightMotors.move_velocity(targetSpeedRight / maxSpeedCmS * 600);

        pros::delay(20);
    }
}