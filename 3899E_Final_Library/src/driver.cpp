#include "robot_config.h"
#include "utils.h"
#include "navigation.h"
#include "autontasks.h"
#include "odometry.h"
#include <cmath>

// Joystick deadzone threshold — inputs below this magnitude are treated as zero
static int deadzoneThreshold = 20;

// Filters out small joystick movements within the deadzone to prevent motor drift
int applyDeadzone(int value) {
    if (abs(value) < deadzoneThreshold) {
        return 0;
    }
    return value;
}

// Applies an exponential response curve to a joystick input.
// exponent > 1 gives finer low-speed control; 1.0 = linear, 2.0 = squared.
int applyCustomCurve(int input, double exponent) {
    if (input == 0) return 0;
    int sign = (input > 0) ? 1 : -1;
    return sign * (int)(pow(abs(input) / 100.0, exponent) * 100);
}

void toggleMatchloader() { // pneumatic helper functions
    matchloaderState = !matchloaderState;
    matchloader.set_value(matchloaderState);
}

void toggleScoreFlap() {
    scoreFlapState = !scoreFlapState;
    scoreFlap.set_value(scoreFlapState);
}

void toggleScorePiston() {
    scorePistonState = !scorePistonState;
    scorePiston.set_value(scorePistonState);
}

// ══════════════════════════════════════════════════════════════════════════════
// MAIN DRIVER CONTROL
// Split-arcade steering; R1/R2 = intake with colour-sort (2× 11W intake + 2× 5.5W hood/
// indexer, main intake reversed); RIGHT = outtake; L1 = match-load piston;
// L2 = left-lane score; Y = wings; A = rudder toggle.
//
// Intake motor summary:
//   intakeMotor1  port 10  11W  600 RPM  reversed
//   intakeMotor2  port  9  11W  600 RPM  forward
//   hoodMotor     port  8  5.5W 200 RPM  forward   (hardware fixed, no cartridge)
//   upperIndexer  port  4  5.5W 200 RPM  reversed  (opposite to hood — pulls together)
//
// All four fire together on every intake/score/outtake binding.
// ══════════════════════════════════════════════════════════════════════════════
void driverControl() {
    initializeOpticalSensor();

    // Motor power arrays (one entry per motor per side)
    double motorPowerLeft[3]  = {0};
    double motorPowerRight[3] = {0};

    // Pneumatic toggle states (DigitalOut::get_value() is private in PROS 4)
    bool wingState   = false;
    bool rudderState = false;

    // Button edge-detection flags — prevent repeated triggers on a single held press
    bool wasAPressed     = false;
    bool wasR1Pressed    = false;
    bool wasR2Pressed    = false;
    bool wasL1Pressed    = false;
    bool wasL2Pressed    = false;
    bool wasXPressed     = false;
    bool wasRightPressed = false;
    bool wasYPressed     = false;
    bool wasUpPressed    = false;
    bool wasDownPressed  = false;

    // Intake control state
    bool spinForInProgress          = false;  // true while a timed motor burst is running
    bool isMatchLoadPneumaticsActive = false;
    bool isLeftGateOpen             = true;
    bool rudderOpen                 = true;
    bool intakeRunning              = false;
    int  intakeDirection            = 0;  // 1=forward, -1=reverse, 0=off

    // Consecutive-read counters for Octoball colour sort.
    // Persists across loop iterations; resets are handled inside the R1 block.
    int redConsecutive  = 0;
    int blueConsecutive = 0;

    // Consecutive reads required before the rudder fires (~20 ms at 20 ms loop rate).
    // Tune up to 3 if false triggers occur, down to 1 if detections are missed.
    const int REQUIRED_CONSECUTIVE = 1;

    int maxSpeed = 100;

    while (true) {
        // ─────────────────────────────────────────────────────────────────────
        // DRIVE  —  split-arcade with exponential curve
        // ─────────────────────────────────────────────────────────────────────

        // Curve exponents (tune to driver preference)
        const double DRIVE_EXPONENT = 2.0;  // 1.0=linear, 2.0=squared
        const double TURN_EXPONENT  = 1.5;  // separate tuning for turning feel

        // Read joystick axes and apply deadzone
        int forward = applyDeadzone(Controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));   // left stick Y
        int turn    = applyDeadzone(Controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));  // right stick X

        // Apply response curve to both axes
        forward = applyCustomCurve(forward, DRIVE_EXPONENT);
        turn    = applyCustomCurve(turn, TURN_EXPONENT);

        // Split-arcade mixing: forward ± turn for each side
        int targetPowerLeft  = forward + turn;
        int targetPowerRight = forward - turn;

        // Clamp to [-100, 100] to prevent overflow after mixing
        if (targetPowerLeft  >  100) targetPowerLeft  =  100;
        if (targetPowerLeft  < -100) targetPowerLeft  = -100;
        if (targetPowerRight >  100) targetPowerRight =  100;
        if (targetPowerRight < -100) targetPowerRight = -100;

        // Convert joystick percent to motor speed (cm/s).
        // Constants are tuned so this stays within the ±100 pct range.
        double targetSpeedLeft  = (targetPowerLeft  / 100.0) * absoluteMaxRPM * wheelCircumferenceCM / 60.0;
        double targetSpeedRight = (targetPowerRight / 100.0) * absoluteMaxRPM * wheelCircumferenceCM / 60.0;

        // Broadcast the same speed to all three motors on each side
        motorPowerLeft[0]  = targetSpeedLeft;
        motorPowerLeft[1]  = targetSpeedLeft;
        motorPowerLeft[2]  = targetSpeedLeft;
        motorPowerRight[0] = targetSpeedRight;
        motorPowerRight[1] = targetSpeedRight;
        motorPowerRight[2] = targetSpeedRight;

        // pros::lcd::set_text(1, std::to_string(globalX));
        // pros::lcd::set_text(2, std::to_string(globalY));
        // pros::lcd::set_text(3, std::to_string(globalRotation));

        if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intakeMotor.move(127);
            colorSortMotor.move(127);
    
            lever.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            lever.move(127);
        }
        else if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intakeMotor.move(127);
            colorSortMotor.move(127);
    
            lever.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
            lever.move(0);
        }
        else if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
        // reverse intake
            intakeMotor.move(-127);
            colorSortMotor.move(-127);
    
            lever.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
            lever.move(0);
        }
        else if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            lever.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
            lever.move(-90);
        }
        else {
            intakeMotor.move(0);
            colorSortMotor.move(0);
    
            lever.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
            lever.move(0);
        }
    
        if (Controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            toggleMatchloader();
        }
        if (Controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            toggleScoreFlap();
        }
        if (Controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            toggleScorePiston();
        }

        // // ─────────────────────────────────────────────────────────────────────
        // // BUTTON R1  —  normal intake with colour-sort
        // // ─────────────────────────────────────────────────────────────────────
        // if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        //     // Octoball is 8-sided — a single hue read can land on a facet edge and
        //     // return noise. Requiring consecutive reads before firing the rudder
        //     // filters edge-reads without adding meaningful delay.
        //     // Counters reset only when the lane is confirmed empty so stale counts
        //     // don't carry over between balls.
        //     double leftHue  = leftLaneOptical.get_hue();
        //     double rightHue = rightLaneOptical.get_hue();

        //     // Red wraps near 0°/360° — needs two detection bands
        //     bool redThisCycle =
        //         ((leftHue  >= RED_HUE_MIN_1 && leftHue  <= RED_HUE_MAX_1) ||
        //          (leftHue  >= RED_HUE_MIN_2 && leftHue  <= RED_HUE_MAX_2)) ||
        //         ((rightHue >= RED_HUE_MIN_1 && rightHue <= RED_HUE_MAX_1) ||
        //          (rightHue >= RED_HUE_MIN_2 && rightHue <= RED_HUE_MAX_2));

        //     // Blue sits mid-wheel (~215–225°) — one band only
        //     bool blueThisCycle =
        //         (leftHue  >= BLUE_HUE_MIN && leftHue  <= BLUE_HUE_MAX) ||
        //         (rightHue >= BLUE_HUE_MIN && rightHue <= BLUE_HUE_MAX);

        //     // Increment matching colour counter; reset the opposite
        //     if (redThisCycle) {
        //         redConsecutive++;
        //         blueConsecutive = 0;
        //     } else if (blueThisCycle) {
        //         blueConsecutive++;
        //         redConsecutive = 0;
        //     } else {
        //         // Reset counters only when the lane is confirmed empty
        //         bool nearLeft  = leftLaneOptical.get_proximity()  > 50;
        //         bool nearRight = rightLaneOptical.get_proximity() > 50;
        //         if (!nearLeft && !nearRight) {
        //             redConsecutive  = 0;
        //             blueConsecutive = 0;
        //         }
        //     }

        //     // Debug: show live hue readings on the Brain screen
        //     pros::screen::print(pros::E_TEXT_SMALL, 1, "L:%.0f R:%.0f          ", leftHue, rightHue);

        //     // Fire rudder once colour is confirmed by REQUIRED_CONSECUTIVE reads
        //     if (redConsecutive >= REQUIRED_CONSECUTIVE) {
        //         rudderPneumatics.set_value(true);   // route to right lane
        //         redConsecutive = 0;
        //         pros::screen::print(pros::E_TEXT_SMALL, 2, "RED  L:%.0f R:%.0f          ", leftHue, rightHue);
        //     } else if (blueConsecutive >= REQUIRED_CONSECUTIVE) {
        //         rudderPneumatics.set_value(false);  // route to left lane
        //         blueConsecutive = 0;
        //         pros::screen::print(pros::E_TEXT_SMALL, 2, "BLUE L:%.0f R:%.0f          ", leftHue, rightHue);
        //     }

        //     // Configure pneumatics only once on the initial press (not every frame)
        //     if (!wasR1Pressed) {
        //         frontHoodPneumatics.set_value(false);  // close front hood for intake
        //         ptoPneumatics.set_value(false);
        //         wasR1Pressed = true;
        //     }

        //     spinForInProgress = false;
        //     intakeMotor1.move_voltage(-12000);   // full forward voltage
        //     intakeMotor2.move_voltage(-12000);
        //     hoodMotor.move_voltage(-12000);      // 5.5W hood motor — same direction, capped at 200 RPM by hardware
        //     upperIndexerMotor.move_voltage(-12000); // 5.5W upper indexer — physically reversed, pulls opposite to hood
        // } else {
        //     // Button released — stop all three intake/hood motors
        //     if (wasR1Pressed) {
        //         intakeMotor1.move(0);
        //         intakeMotor2.move(0);
        //         hoodMotor.move(0);
        //         upperIndexerMotor.move(0);
        //         wasR1Pressed = false;
        //     }
        // }

        // // ─────────────────────────────────────────────────────────────────────
        // // BUTTON R2  —  match-loader scoring (mirrors R1 with colour-sort)
        // // ─────────────────────────────────────────────────────────────────────
        // if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
        //     // Octoball is 8-sided — a single hue read can land on a facet edge and
        //     // return noise. Requiring consecutive reads before firing the rudder
        //     // filters edge-reads without adding meaningful delay.
        //     // Counters reset only when the lane is confirmed empty so stale counts
        //     // don't carry over between balls.
        //     double leftHue  = leftLaneOptical.get_hue();
        //     double rightHue = rightLaneOptical.get_hue();

        //     // Red wraps near 0°/360° — needs two detection bands
        //     bool redThisCycle =
        //         ((leftHue  >= RED_HUE_MIN_1 && leftHue  <= RED_HUE_MAX_1) ||
        //          (leftHue  >= RED_HUE_MIN_2 && leftHue  <= RED_HUE_MAX_2)) ||
        //         ((rightHue >= RED_HUE_MIN_1 && rightHue <= RED_HUE_MAX_1) ||
        //          (rightHue >= RED_HUE_MIN_2 && rightHue <= RED_HUE_MAX_2));

        //     // Blue sits mid-wheel (~215–225°) — one band only
        //     bool blueThisCycle =
        //         (leftHue  >= BLUE_HUE_MIN && leftHue  <= BLUE_HUE_MAX) ||
        //         (rightHue >= BLUE_HUE_MIN && rightHue <= BLUE_HUE_MAX);

        //     // Increment matching colour counter; reset the opposite
        //     if (redThisCycle) {
        //         redConsecutive++;
        //         blueConsecutive = 0;
        //     } else if (blueThisCycle) {
        //         blueConsecutive++;
        //         redConsecutive = 0;
        //     } else {
        //         // Reset counters only when the lane is confirmed empty
        //         bool nearLeft  = leftLaneOptical.get_proximity()  > 50;
        //         bool nearRight = rightLaneOptical.get_proximity() > 50;
        //         if (!nearLeft && !nearRight) {
        //             redConsecutive  = 0;
        //             blueConsecutive = 0;
        //         }
        //     }

        //     // Debug: show live hue readings on the Brain screen
        //     pros::screen::print(pros::E_TEXT_SMALL, 1, "L:%.0f R:%.0f          ", leftHue, rightHue);

        //     // Fire rudder once colour is confirmed by REQUIRED_CONSECUTIVE reads
        //     if (redConsecutive >= REQUIRED_CONSECUTIVE) {
        //         rudderPneumatics.set_value(true);   // route to right lane
        //         redConsecutive = 0;
        //         pros::screen::print(pros::E_TEXT_SMALL, 2, "RED  L:%.0f R:%.0f          ", leftHue, rightHue);
        //     } else if (blueConsecutive >= REQUIRED_CONSECUTIVE) {
        //         rudderPneumatics.set_value(false);  // route to left lane
        //         blueConsecutive = 0;
        //         pros::screen::print(pros::E_TEXT_SMALL, 2, "BLUE L:%.0f R:%.0f          ", leftHue, rightHue);
        //     }

        //     // Configure pneumatics only once on the initial press (not every frame)
        //     if (!wasR2Pressed) {
        //         frontHoodPneumatics.set_value(false);  // close front hood for intake
        //         ptoPneumatics.set_value(false);
        //         wasR2Pressed = true;
        //     }

        //     spinForInProgress = false;
        //     intakeMotor1.move_voltage(-12000);  // reversed — same as R1
        //     intakeMotor2.move_voltage(-12000);
        //     hoodMotor.move_voltage(-12000);      // 5.5W hood motor — same direction, capped at 200 RPM by hardware
        //     upperIndexerMotor.move_voltage(12000); // 5.5W upper indexer — physically reversed, pulls opposite to hood
        // } else {
        //     // Button released — stop all motors
        //     if (wasR2Pressed) {
        //         intakeMotor1.move(0);
        //         intakeMotor2.move(0);
        //         hoodMotor.move(0);
        //         upperIndexerMotor.move(0);
        //         wasR2Pressed = false;
        //     }
        // }

        // ─────────────────────────────────────────────────────────────────────
        // BUTTON R2  —  right-lane score  [DISABLED — replaced by match-loader scoring above]
        // ─────────────────────────────────────────────────────────────────────
        // if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
        //     if (!wasR2Pressed) {
        //         frontHoodPneumatics.set_value(true);  // open front hood for scoring
        //         ptoPneumatics.set_value(true);        // engage PTO
        //         isLeftGateOpen = true;
        //         leftGatePneumatics.set_value(isLeftGateOpen);
        //         rightGatePneumatics.set_value(!isLeftGateOpen);
        //         rudderPneumatics.set_value(false);
        //         wasR2Pressed = true;
        //     }
        //
        //     spinForInProgress = false;
        //     intakeMotor1.move_voltage(12000);
        //     intakeMotor2.move_voltage(12000);
        //     hoodMotor.move_voltage(12000);
        //     upperIndexerMotor.move_voltage(12000);
        // } else {
        //     if (wasR2Pressed) {
        //         spinForInProgress = true;
        //         intakeMotor1.move(0);
        //         intakeMotor2.move(0);
        //         hoodMotor.move(0);
        //         upperIndexerMotor.move(0);
        //         wasR2Pressed = false;
        //     }
        // }

        // ─────────────────────────────────────────────────────────────────────
        // BUTTON RIGHT  —  outtake with alternating lane selection
        // ─────────────────────────────────────────────────────────────────────
        // if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
        //     if (!wasRightPressed) {
        //         frontHoodPneumatics.set_value(false);  // close front hood for outtake
        //         ptoPneumatics.set_value(true);
        //         wasRightPressed = true;

        //         // Toggle the active lane each time the button is pressed
        //         static enum { LANE_LEFT, LANE_RIGHT } currentLane = LANE_LEFT;
        //         if (currentLane == LANE_LEFT) {
        //             currentLane = LANE_RIGHT;
        //         } else {
        //             currentLane = LANE_LEFT;
        //         }

        //         if (currentLane == LANE_LEFT) {
        //             leftGatePneumatics.set_value(false);
        //             rightGatePneumatics.set_value(true);
        //         } else {
        //             leftGatePneumatics.set_value(true);
        //             rightGatePneumatics.set_value(false);
        //         }
        //     }

        //     spinForInProgress = false;
        //     intakeMotor1.move_voltage(-12000);  // full reverse voltage
        //     intakeMotor2.move_voltage(-12000);
        //     hoodMotor.move_voltage(-12000);     // 5.5W hood motor reverse — ejects, capped at 200 RPM by hardware
        //     upperIndexerMotor.move_voltage(-12000); // 5.5W upper indexer reverse — ejects opposite to hood
        // } else {
        //     if (wasRightPressed) {
        //         intakeMotor1.move(0);
        //         intakeMotor2.move(0);
        //         hoodMotor.move(0);
        //         upperIndexerMotor.move(0);
        //         wasRightPressed = false;
        //     }
        // }

        // // ─────────────────────────────────────────────────────────────────────
        // // BUTTON L2  —  left-lane score
        // // ─────────────────────────────────────────────────────────────────────
        // if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
        //     if (!wasL2Pressed) {
        //         frontHoodPneumatics.set_value(true);  // open front hood for scoring
        //         ptoPneumatics.set_value(true);        // engage PTO
        //         isLeftGateOpen = false;
        //         leftGatePneumatics.set_value(isLeftGateOpen);
        //         rightGatePneumatics.set_value(!isLeftGateOpen);
        //         rudderPneumatics.set_value(true);
        //         wasL2Pressed = true;
        //     }

        //     // While held, keep intake running
        //     spinForInProgress = false;
        //     intakeMotor1.move_voltage(-12000);
        //     intakeMotor2.move_voltage(-12000);
        //     hoodMotor.move_voltage(12000);      // 5.5W hood motor — same direction, capped at 200 RPM by hardware
        //     upperIndexerMotor.move_voltage(12000); // 5.5W upper indexer — physically reversed, pulls opposite to hood
        // } else {
        //     if (wasL2Pressed) {
        //         spinForInProgress = true;
        //         intakeMotor1.move(0);
        //         intakeMotor2.move(0);
        //         hoodMotor.move(0);
        //         upperIndexerMotor.move(0);
        //         wasL2Pressed = false;
        //     }
        // }

        // // ─────────────────────────────────────────────────────────────────────
        // // BUTTON L1  —  match-load piston toggle
        // // ─────────────────────────────────────────────────────────────────────
        // if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
        //     if (!wasL1Pressed) {
        //         isMatchLoadPneumaticsActive = !isMatchLoadPneumaticsActive;
        //         matchLoadPneumatics.set_value(isMatchLoadPneumaticsActive);
        //         wasL1Pressed = true;
        //     }
        // } else {
        //     wasL1Pressed = false;
        // }

        // // ─────────────────────────────────────────────────────────────────────
        // // BUTTON Y  —  wing toggle
        // // ─────────────────────────────────────────────────────────────────────
        // if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
        //     if (!wasYPressed) {
        //         wingState = !wingState;
        //         wingPneumatics.set_value(wingState);  // toggle
        //         wasYPressed = true;
        //     }
        // } else {
        //     wasYPressed = false;
        // }

        // // ─────────────────────────────────────────────────────────────────────
        // // BUTTON A  —  rudder toggle
        // // ─────────────────────────────────────────────────────────────────────
        // if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
        //     if (!wasAPressed) {
        //         rudderState = !rudderState;
        //         rudderPneumatics.set_value(rudderState);  // toggle
        //         wasAPressed = true;
        //     }
        // } else {
        //     wasAPressed = false;
        // }

        // ─────────────────────────────────────────────────────────────────────
        // APPLY DRIVE MOTOR POWERS
        // Convert pct → mV: multiply by 120 to map [-100, 100] to [-12000, 12000]
        // ─────────────────────────────────────────────────────────────────────
        LeftMotor1.move_voltage(static_cast<int32_t>(motorPowerLeft[0]  * 120.0));
        RightMotor1.move_voltage(static_cast<int32_t>(motorPowerRight[0] * 120.0));

        LeftMotor2.move_voltage(static_cast<int32_t>(motorPowerLeft[1]  * 120.0));
        RightMotor2.move_voltage(static_cast<int32_t>(motorPowerRight[1] * 120.0));

        LeftMotor3.move_voltage(static_cast<int32_t>(motorPowerLeft[2]  * 120.0));
        RightMotor3.move_voltage(static_cast<int32_t>(motorPowerRight[2] * 120.0));

        // 20 ms loop delay → 50 Hz update rate
        pros::delay(20);
    }
}

// ══════════════════════════════════════════════════════════════════════════════
// TANK DRIVE TEST
// Basic tank steering: left stick = left side, right stick = right side.
// No curves, ramping, or mechanism bindings — used for testing and tuning.
// ══════════════════════════════════════════════════════════════════════════════
void driverControlTankTest() {
    while (true) {
        // Axis3 = left stick Y (forward/back); Axis2 = right stick Y
        int leftPower  = Controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightPower = Controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        // Apply deadzone — values within ±threshold are treated as zero
        if (abs(leftPower)  < deadzoneThreshold) leftPower  = 0;
        if (abs(rightPower) < deadzoneThreshold) rightPower = 0;

        // Send power directly to all left-side motors (pct → mV)
        LeftMotor1.move_voltage(leftPower * 120);
        LeftMotor2.move_voltage(leftPower * 120);
        LeftMotor3.move_voltage(leftPower * 120);

        // Send power directly to all right-side motors
        RightMotor1.move_voltage(rightPower * 120);
        RightMotor2.move_voltage(rightPower * 120);
        RightMotor3.move_voltage(rightPower * 120);

        // 10 ms delay → 100 Hz update rate
        pros::delay(10);
    }
}