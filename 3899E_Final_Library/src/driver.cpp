#include "robot_config.h"
#include "utils.h"
#include "navigation.h"
#include "autontasks.h"
#include "odometry.h"
#include "ai.h"
#include "route_planner.h"
#include "robot_geometry.h"
#include "field_targets.h"
#include <cmath>
#include <atomic>

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
// indexer, main intake reversed); L2 = left-lane score; RIGHT = right-lane score;
// L1 = match-load piston; Y = wings; A = rudder toggle.
//
// Intake motor summary:
//   intakeMotor1  port 10  11W  600 RPM  reversed
//   intakeMotor2  port  9  11W  600 RPM  forward
//   hoodMotor     port 11  5.5W 200 RPM  forward   (hardware fixed, no cartridge)
//   upperIndexer  port 15  5.5W 200 RPM  reversed  (opposite to hood — pulls together)
//
// All four fire together on every intake/score/outtake binding.
// ══════════════════════════════════════════════════════════════════════════════
void driverControl() {
    initializeOpticalSensor();

    // Motor power arrays (one entry per motor per side)
    double motorPowerLeft[3]  = {0};
    double motorPowerRight[3] = {0};

    // Pneumatic toggle states — none currently active

    // Button edge-detection flags — prevent repeated triggers on a single held press
    bool wasR1Pressed    = false;
    bool wasR2Pressed    = false;
    bool wasL2Pressed    = false;
    bool wasRightPressed = false;

    // Intake control state
    bool spinForInProgress = false;  // true while a timed motor burst is running

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
        // BUTTON RIGHT  —  right-lane score
        // Opens right gate while held, closes it on release. Mirrors L2.
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

void AITracking(std::pmr::string teamColor) {
    std::vector<pros::AIVision::Object> aiVision_front_objects;
    std::vector<pros::AIVision::Object> aiVision_back_objects;
        // Grab all currently tracked objects from the front sensor
    aiVision_front_objects = aiVision_Front.get_all_objects();
    aiVision_back_objects = aiVision_Back.get_all_objects();

    for (auto &obj : aiVision_front_objects) {
        // 1. Verify that this object is indeed a color detection type
        if (pros::AIVision::is_type(obj, pros::AivisionDetectType::color)) {
            if (teamColor == "RED") {
                // 2. Filter by your target signature ID (e.g., ID 1 for Red)
                if (obj.id == 1) { 
                    
                    // 3. Access the nested color struct data:
                    // Top-left X coordinate of the bounding box
                    int topLeftX = obj.object.color.xoffset; 
                    int topLeftY = obj.object.color.yoffset;
                    // Width of the bounding box
                    int blockWidth = obj.object.color.width; 
                    int blockHeight = obj.object.color.height;

                    // 4. Calculate the center X coordinate manually
                    int blockX = topLeftX + (blockWidth / 2);
                    // 5. Calculate the center Y coordinate manually
                    int blockY = topLeftY + (blockHeight / 2);

                    pros::lcd::print(1, "Front Red Center X: %d", blockX);
                    pros::lcd::print(2, "Front Red Center Y: %d", blockY);
                }
            }
            else if (teamColor == "BLUE") {
                // 2. Filter by your target signature ID (e.g., ID 2 for Blue)
                if (obj.id == 2) { 
                    
                    // 3. Access the nested color struct data:
                    // Top-left X coordinate of the bounding box
                    int topLeftX = obj.object.color.xoffset; 
                    int topLeftY = obj.object.color.yoffset;
                    // Width of the bounding box
                    int blockWidth = obj.object.color.width; 
                    int blockHeight = obj.object.color.height;

                    // 4. Calculate the center X coordinate manually
                    int blockX = topLeftX + (blockWidth / 2);
                    // 5. Calculate the center Y coordinate manually
                    int blockY = topLeftY + (blockHeight / 2);
                    
                    pros::lcd::print(1, "Front Blue Center X: %d", blockX);
                    pros::lcd::print(2, "Front Blue Center Y: %d", blockY);
                }
            }
            else {
                pros::lcd::print(1, "Invalid team color: %s", teamColor.c_str());
            }
            if (obj.id==3) {
                // This is the ML model signature — access object data as needed
                int topLeftXml = obj.object.color.xoffset;
                int topLeftYml = obj.object.color.yoffset;
                int mlWidth = obj.object.color.width;
                int mlHeight = obj.object.color.height;
                int mlX = topLeftXml + (mlWidth / 2);
                int mlY = topLeftYml + (mlHeight / 2);
                pros::lcd::print(3, "ML Model Center X: %d", mlX);
                pros::lcd::print(4, "ML Model Center Y: %d", mlY);
            }
        }
    }
    for (auto &obj : aiVision_back_objects) {
        if (pros::AIVision::is_type(obj, pros::AivisionDetectType::color)) {
            if (teamColor == "RED") {
                if (obj.id == 1) {
                    int topLeftX = obj.object.color.xoffset;
                    int topLeftY = obj.object.color.yoffset;
                    int blockWidth = obj.object.color.width;
                    int blockHeight = obj.object.color.height;
                    int blockX = topLeftX + (blockWidth / 2);
                    int blockY = topLeftY + (blockHeight / 2);
                    pros::lcd::print(5, "Back Red Center X: %d", blockX);
                    pros::lcd::print(6, "Back Red Center Y: %d", blockY);
                }
            }
            else if (teamColor == "BLUE") {
                if (obj.id == 2) {
                    int topLeftX = obj.object.color.xoffset;
                    int topLeftY = obj.object.color.yoffset;
                    int blockWidth = obj.object.color.width;
                    int blockHeight = obj.object.color.height;
                    int blockX = topLeftX + (blockWidth / 2);
                    int blockY = topLeftY + (blockHeight / 2);
                    pros::lcd::print(5, "Back Blue Center X: %d", blockX);
                    pros::lcd::print(6, "Back Blue Center Y: %d", blockY);
                }
            }
        }
    }
}

int blockCount;
std::pmr::string teamColorGlobal = "";
bool wasColor;
static std::atomic<bool> blockCountTaskRunning{false};

void blockCountTask() {
    blockCountTaskRunning.store(true);
    wasColor = false;
    
    while (blockCountTaskRunning.load()) {    
        int hue = opticalSensor.get_hue();  // Read hue directly each iteration
        
        if (teamColorGlobal == "RED") {
            if (hue >= RED_HUE_MIN_2 && hue <= RED_HUE_MAX_2) {
                wasColor = true;
            } else {
                if (wasColor) {
                    blockCount++;
                    wasColor = false; // Reset for next detection
                }
            }
        } else if (teamColorGlobal == "BLUE") {
            if (hue >= BLUE_HUE_MIN && hue <= BLUE_HUE_MAX) {
                wasColor = true;
            } else {
                if (wasColor) {
                    blockCount++;
                    wasColor = false; // Reset for next detection
                }
            }
        }
        
        pros::delay(20); // Check every 20 ms
    }
}

// Returns the absolute heading (0 to 360 degrees) from Point 1 to Point 2
double getHeadingToTarget(double x1, double y1, double x2, double y2) {
    // Calculate differences
    double deltaX = x2 - x1;
    double deltaY = y2 - y1;
    
    // atan2 returns radians between -PI and +PI
    double radians = std::atan2(deltaY, deltaX);
    
    // Convert to degrees (-180 to 180)
    double degrees = radians * (180.0 / M_PI);
    
    // Map standard math angle (0=Right) to VEX GPS heading (0=Up/Forward, clockwise)
    // If your coordinate system treats 0 degrees as standard math right, skip this step
    double robotHeading = 90.0 - degrees;
    
    // Normalize angle to keep it strictly between 0.0 and 360.0 degrees
    while (robotHeading >= 360.0) robotHeading -= 360.0;
    while (robotHeading < 0.0)    robotHeading += 360.0;
    
    return robotHeading;
}

void opScoring(std::pmr::string teamColor) {
    teamColorGlobal = teamColor;
    startOdometryTask();
    intakeMotor.move(127);
    colorSortMotor.move(127);
    opticalSensor.set_led_pwm(100); // Ensure the LED is on for accurate readings
    blockCount = 0;
    wasColor = false;
    bool intakeFull = true;
    int retryCount = 0;
    const int MAX_RETRIES_PER_BLOCK = 3;  // Backtrack if vision fails 3 times in a row
    StraightProfile dp = DEFAULT_STRAIGHT;
    dp.breakDistance          = 30.0;   // cm before target to begin decel
    dp.minSpeed               = 30.0;   // % minimum approach speed
    dp.maxSpeed               = 80.0;   // % peak cruise speed
    dp.distanceTolerance      = 1.0;    // cm exit bubble
    dp.timeout                = 5.0;    // seconds 5 sec default
    dp.brakeMode              = pros::E_MOTOR_BRAKE_HOLD;
    dp.kp_heading             = 2.0;    // heading PID proportional
    dp.ki_heading             = 0.0;    // heading PID integral
    dp.kd_heading             = 5.0;    // heading PID derivative
    dp.accelHeadingScaling    = 0.2;    // correction weight during accel
    dp.decelHeadingScaling    = 0.1;    // correction weight during decel
    dp.approachHeadingScaling = 0.1;    // correction weight during approach
    dp.headingLockDistance    = 3.0;    // cm — freeze heading near target
    dp.launchVoltage          = 3.0;    // V — initial kick voltage
    dp.accelFactor            = 1.2;    // traction ramp multiplier
    dp.slipThreshold          = 0.3;   // RPM slip before traction cuts in
    dp.decelStepPercent       = 0.30;   // ABS voltage reduction per step
    dp.lockThreshold          = 0.3;   // wheel lockup ratio
    dp.maxCurrentA            = 4.0;   // amps — wall stall trip threshold
    dp.overcurrentDurationMs  = 300; // ms — how long before breaker fires

    TurnProfile tp = DEFAULT_TURN;
    tp.breakDistance  = 5.0;    // degrees before target to begin decel
    tp.minSpeed       = 10.0;   // % minimum approach speed
    tp.maxSpeed       = 80.0;  // % peak turn speed
    tp.exitTolerance  = 3;    // degrees — stop when within this
    tp.timeout        = 3.0;    // seconds — release if stuck

    VisionProfile vp = DEFAULT_VISION;
    vp.drive.breakDistance          = 90.0;   // cm before target to begin decel
    vp.drive.minSpeed               = 20.0;   // % minimum approach speed
    vp.drive.maxSpeed               = 80.0;   // % peak cruise speed
    vp.drive.distanceTolerance      = 1.0;    // cm exit bubble
    vp.drive.timeout                = 5.0;    // seconds
    vp.drive.brakeMode              = pros::E_MOTOR_BRAKE_HOLD;
    vp.drive.kp_heading             = 0.4;    // low gain — vision correction is noisy
    vp.drive.ki_heading             = 0.0;
    vp.drive.kd_heading             = 0.1;
    vp.drive.accelHeadingScaling    = 0.2;    // correction weight during accel
    vp.drive.decelHeadingScaling    = 0.1;    // correction weight during decel
    vp.drive.approachHeadingScaling = 0.1;    // correction weight during approach
    vp.drive.headingLockDistance    = 3.0;   // cm — wider than odom; vision may shift near target
    vp.drive.launchVoltage          = 3.0;    // V — initial kick voltage
    vp.drive.accelFactor            = 1.2;    // traction ramp multiplier
    vp.drive.slipThreshold          = 0.3;    // RPM slip before traction cuts in
    vp.drive.decelStepPercent       = 0.30;    // ABS voltage reduction per step
    vp.drive.lockThreshold          = 0.3;    // wheel lockup ratio
    vp.drive.maxCurrentA            = 4.0;    // amps — wall stall trip threshold
    vp.drive.overcurrentDurationMs  = 300;    // ms — how long before breaker fires
    vp.kp_distToHeadScaling         = 1.75;    // vision correction aggressiveness
    vp.minObjectWidth               = 10;     // pixels — ignore detections smaller than this
    vp.minX                         = 0;      // detection zone left bound (pixels)
    vp.maxX                         = 320;    // detection zone right bound (pixels)
    vp.minY                         = 0;      // detection zone top bound (pixels)
    vp.maxY                         = 240;    // detection zone bottom bound (pixels)
    // int proximity = opticalSensor.get_proximity();


    pros::Task blockCounter(blockCountTask);

    if (teamColor == "RED") {
        blueColorSortStart();
    } else {
        redColorSortStart();
    }

    while (!intakeFull) {
        intakeMotor.move(127);
        colorSortMotor.move(127);
        pros::lcd::print(1, "Blocks: %d | Retries: %d", blockCount, retryCount);
        
        // Try vision approach with a shorter timeout
        VisionProfile vpShort = vp;
        vpShort.drive.timeout = 2.0;  // Shorter timeout to avoid long hangs
        
        uint32_t visionStart = pros::millis();
        visionDriveForward((teamColor == "RED") ? aiVision_redCube : aiVision_blueCube,
                           40, 150.0, globalRotation, vpShort, false);
        uint32_t visionElapsed = pros::millis() - visionStart;
        
        // If vision timeout occurred (elapsed >= 2000ms), we likely hit an obstacle
        if (visionElapsed >= 1900) {
            retryCount++;
            pros::lcd::print(2, "Vision timeout - backtracking");
            
            if (retryCount >= MAX_RETRIES_PER_BLOCK) {
                // Give up on this block location, back up and reset
                // intakeMotor.move(0);
                driveForward(-30, globalRotation, DEFAULT_STRAIGHT);  // Back up 30cm
                retryCount = 0;  // Reset retry counter for next block
                turnRight(globalRotation + 90, DEFAULT_TURN);  // Slight turn to try a new angle
                pros::delay(500);
                continue;
            }
        } else {
            retryCount = 0;  // Vision succeeded, reset retry counter
        }
        
        if (blockCount >= 2) {
            intakeFull = true;
            blockCount = 0; // Reset count after scoring
        }
    }
    
    blockCountTaskRunning.store(false);
    pros::delay(50);
    // // Only count if an object is close enough to the sensor
    // if (proximity > 50) {
    //     if (hue >= RED_HUE_MIN_2 && hue <= RED_HUE_MAX_2) {
    //         redCount++;
    //         pros::lcd::print(7, "Red Count: %d", redCount);
    //     } else if (hue >= BLUE_HUE_MIN && hue <= BLUE_HUE_MAX) {
    //         blueCount++;
    //         pros::lcd::print(8, "Blue Count: %d", blueCount);
    //     }
    // }
    if (intakeFull) {
        intakeMotor.move(127);
        colorSortMotor.move(127);
        // Trigger scoring mechanism here (e.g., activate pneumatics, run motors)
        // GPS reset
        double headingOffset = 90.0; // Adjust for cartesian vs VEX if needed
        double rawGpsX = gpsSensor.get_position().x;
        double rawGpsY = gpsSensor.get_position().y;
        double gpsError = gpsSensor.get_error();

        double xPosition = rawGpsX * 100.0;
        double yPosition = rawGpsY * 100.0;
        double heading = gpsSensor.get_heading() - headingOffset;
        while (heading < 0.0) heading += 360.0;
        while (heading >= 360.0) heading -= 360.0;

        setStartPosition(xPosition, yPosition, heading);

        pros::lcd::print(1, "GPS raw X: %.3f m Y: %.3f m", rawGpsX, rawGpsY);
        pros::lcd::print(2, "GPS err: %.3f m heading: %.1f", gpsError, gpsSensor.get_heading());
        pros::lcd::print(3, "SET X: %.1f cm Y: %.1f cm H: %.1f", xPosition, yPosition, heading);
        pros::lcd::print(4, "ODOM X: %.1f cm Y: %.1f cm H: %.1f", globalX, globalY, globalRotation);
        // pros::delay(5000);
        // turnRight(90, DEFAULT_TURN);
        // pros::lcd::print(5, "Post-turn GPS heading: %.1f", gpsSensor.get_heading());
        // pros::delay(50000);
        NavResult 
        result = navigateTo(LONG_GOAL_SE);
        pros::delay(500);
        setStartPosition(globalX, globalY, gpsSensor.get_heading()-headingOffset);
        // turnRight(90, DEFAULT_TURN);
        pros::delay(500);
        scorePiston.set_value(true);
        driveBackward(30, globalRotation, dp);
        scoreFlap.set_value(true);
        pros::delay(200);
        lever.move(127);
        pros::delay(2000);
        lever.move(-127);
        pros::delay(2000);
        lever.move(0);
        // turnRight(270, DEFAULT_TURN);

        pros::delay(50000);
        turnToPoint(-60, 120, DEFAULT_TURN); //47,47 in inches
        pros::delay(1000);
        forwardToPoint(-60, 120, dp); //47,47 in inches
        turnRight(270, DEFAULT_TURN);
        // pros::delay(500000);
        // turnToPoint(0 , 0 , DEFAULT_TURN); //47,47 in inches
        // pros::delay(500);
        // turnToPoint(180, 180, DEFAULT_TURN); //47,47 in inches
        // pros::delay(5000);
        // turnToPoint(290, 120, DEFAULT_TURN); //47,47 in inches
        // // pros::delay(500000);
        pros::delay(7000);
        scorePiston.set_value(true);
        forwardToPoint(-60, 100, dp); //47,47 in inches
        // xPos = 100*(gpsSensor.get_position().x - gpsXOffset);
        // yPos = 100*(gpsSensor.get_position().y - gpsYOffset);
        // pros::lcd::print(4, "X: %.2f | Y: %.2f | Heading: %.2f", xPos, yPos, heading);
        // pros::lcd::print(5, "X: %.2f | Y: %.2f | Heading: %.2f", globalX, globalY, globalRotation);
        // pros::delay(70000);
        turnToPoint(-45, 100, DEFAULT_TURN);
        forwardToPoint(-45, 100, dp);
        // scorePiston.set_value(true);
        turnToPoint(-45, 400, DEFAULT_TURN);
        driveForward(-50, globalRotation, dp);
        pros::delay(50000);
        //score
        scoreFlap.set_value(true);
        lever.move(127);
        pros::delay(2000);
        lever.move(-127);
        pros::delay(2000);
        lever.move(0);
        //reset lever
        // pros::lcd::print(9, "Scoring Red Ball!");
        driveForward(30, globalRotation, dp); // Back away from the wall after scoring
        scoreFlap.set_value(false);
        scorePiston.set_value(false);
        intakeFull = false; // Reset for next ball
    }
    // Task will exit cleanly when blockCountTaskRunning is set to false
    blockCounter.remove();
}
