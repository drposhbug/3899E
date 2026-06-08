/*----------------------------------------------------------------------------
 * auton.cpp — VAIRC autonomous routines for Team 3899E (Push Back 2025-2026)
 *
 * Replaces the V5RC legacy auton.cpp. All old routines preserved in auton.txt.
 * PROS hooks (initialize, autonomous, opcontrol) live in main.cpp — not here.
 * main.cpp autonomous() calls autonSelector().
 *
 * Auton selector: Left/Right on controller to cycle, A to confirm.
 *
 * Route functions:
 *   runAIMatchRoute()   — full VAIRC autonomous (Jetson-driven, ai.cpp)
 *   routeTest()         — field test: plans and drives a route manually
 *   routeGridTest()     — prints obstacle grid to brain screen
 *   systemTest()        — spins each motor briefly to verify hardware
 *   coordinateFinder()  — push robot around, prints live GPS coordinates
 *----------------------------------------------------------------------------*/

#include "main.h"
#include "robot_config.h"
#include "navigation.h"
#include "odometry.h"
#include "autontasks.h"
#include "ai.h"
#include "route_planner.h"
#include "robot_geometry.h"
#include "field_targets.h"
// ══════════════════════════════════════════════════════════════════════════════
// ROUTE FUNCTIONS
// ══════════════════════════════════════════════════════════════════════════════

// Full VAIRC match — Jetson drives strategy, V5 executes via ai.cpp
void runAIMatchRoute() {
    startOdometryTask();
    setStartPosition(0.0, 0.0, 0.0);  // update start coords before competition
    setAllianceRed(true);              // set to false for blue alliance
    runAIMatch();
}

// Manual route test — plans a path from current position to a target and drives it.
// Use this to verify route_planner and moveOdometry are working on the real field.
void routeTest() {
    pros::screen::erase();
    pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Route Test");

    startOdometryTask();
    setStartPosition(0.0, 0.0, 0.0);
    pros::delay(200);

    // Plan route from center field to top long goal approach
    RoutePath path = routePlan(globalX, globalY, 0.0, 97.0);  // 25cm short of goal face

    if (path.count == 0) {
        pros::screen::print(pros::E_TEXT_MEDIUM, 2, "No path found!");
        return;
    }

    pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Waypoints: %d  Est: %.1fs",
                        path.count, path.estimatedTimeSec);
    pros::delay(1000);

    bool reached = routeExecute(path);

    pros::screen::print(pros::E_TEXT_MEDIUM, 3, reached ? "Reached target!" : "Blocked mid-route");
    Controller.rumble(reached ? "." : "---");
}

void navTest() {
    startOdometryTask();
    setStartPosition(0.0, 0.0, 0.0);
    pros::delay(200);

    StraightProfile driveProfile = DEFAULT_STRAIGHT;
    driveProfile.breakDistance          = 85.0;
    driveProfile.minSpeed               = 13.0;
    driveProfile.maxSpeed               = 80.0;
    driveProfile.distanceTolerance      = 1.0;
    driveProfile.timeout                = 5.0;
    driveProfile.brakeMode              = pros::E_MOTOR_BRAKE_BRAKE;
    driveProfile.kp_heading             = 1.0;
    driveProfile.ki_heading             = 0.0;
    driveProfile.kd_heading             = 0.0;
    driveProfile.accelHeadingScaling    = 0.2;
    driveProfile.decelHeadingScaling    = 0.1;
    driveProfile.approachHeadingScaling = 0.1;
    driveProfile.headingLockDistance    = 3.0;
    driveProfile.launchVoltage          = 6.0;
    driveProfile.accelFactor            = 1.2;
    driveProfile.slipThreshold          = 0.3;
    driveProfile.decelStepPercent       = 2.0;
    driveProfile.lockThreshold          = 0.3;
    driveProfile.maxCurrentA            = 4.0;
    driveProfile.overcurrentDurationMs  = 300;

    TurnProfile turnProfile = DEFAULT_TURN;
    turnProfile.breakDistance  = 5.0;
    turnProfile.minSpeed       = 10.0;
    turnProfile.maxSpeed       = 20.0;
    turnProfile.exitTolerance  = 3;
    turnProfile.timeout        = 3.0;
}

void visionTest() {
    startOdometryTask();
    setStartPosition(0.0, 0.0, 0.0);
    pros::delay(200);

    VisionProfile vp = DEFAULT_VISION;
    vp.drive.breakDistance          = 85.0;
    vp.drive.minSpeed               = 10.0;
    vp.drive.maxSpeed               = 60.0;
    vp.drive.distanceTolerance      = 1.0;
    vp.drive.timeout                = 5.0;
    vp.drive.brakeMode              = pros::E_MOTOR_BRAKE_BRAKE;
    vp.drive.kp_heading             = .1;
    vp.drive.ki_heading             = 0.0;
    vp.drive.kd_heading             = 0.0;
    vp.drive.accelHeadingScaling    = 0.2;
    vp.drive.decelHeadingScaling    = 0.1;
    vp.drive.approachHeadingScaling = 0.1;
    vp.drive.headingLockDistance    = 15.0;
    vp.drive.launchVoltage          = 6.0;
    vp.drive.accelFactor            = 1.2;
    vp.drive.slipThreshold          = 0.3;
    vp.drive.decelStepPercent       = 2.0;
    vp.drive.lockThreshold          = 0.3;
    vp.drive.maxCurrentA            = 4.0;
    vp.drive.overcurrentDurationMs  = 500;
    vp.kp_distToHeadScaling         = 2.0;
    vp.minObjectWidth               = 10;
    vp.minX                         = 0;
    vp.maxX                         = 320;
    vp.minY                         = 0;
    vp.maxY                         = 240;

    visionDriveForward(aiVision_blueCube, 80, 150.0, 0.0, vp);
}

// Prints the route planner obstacle grid to brain screen.
// Use before competition to verify goal and park zone positions are correct.
void routeGridTest() {
    // Clear LCD text lines to minimize green overlay interference
    for (int i = 0; i < 8; i++) pros::lcd::set_text(i, "");
    pros::delay(50);
    routePrintGrid();
    // Hold forever — prevents program exit and keeps grid visible
    while (true) {
        pros::delay(100);
    }
}

// Spins each drive and mechanism motor for 500ms to verify hardware.
void systemTest() {
    pros::screen::erase();
    pros::screen::print(pros::E_TEXT_MEDIUM, 1, "System Test");

    InertialSensor.reset(true);

    pros::Motor* allMotors[] = {
        &LeftMotor1, &LeftMotor2, &LeftMotor3,
        &RightMotor1, &RightMotor2, &RightMotor3,
        &intakeMotor1, &intakeMotor2
    };
    const int numMotors = 8;

    for (int i = 0; i < numMotors; i++) {
        pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Testing motor %d...", i + 1);
        allMotors[i]->move(80);
        pros::delay(500);
        bool alive = (allMotors[i]->get_actual_velocity() != 0);
        allMotors[i]->brake();
        if (alive) Controller.rumble(".");
        pros::delay(300);
    }

    pros::screen::print(pros::E_TEXT_MEDIUM, 2, "System test complete");
    Controller.rumble(". .");
}

// Push robot around and watch live GPS/odometry coordinates on brain screen.
void coordinateFinder() {
    startOdometryTask();
    setStartPosition(0.0, 0.0, 0.0);
    startCoordinateFinder();
}

// ══════════════════════════════════════════════════════════════════════════════
// FIELD TARGETS TEST
// ══════════════════════════════════════════════════════════════════════════════
void fieldTargetsTest() {
    for (int i = 0; i < 8; i++) pros::lcd::clear_line(i);
    pros::screen::erase();

    startOdometryTask();
    setStartPosition(-124.5, -54.5, 180.0);
    pros::delay(200);

    pros::lcd::print(0, "START X:%.0f Y:%.0f H:%.0f",
                     globalX, globalY, getContinuousStandardHeading());

    NavResult result = navigateTo(LONG_GOAL_NW);

    const char* resultStr =
        result == NavResult::SUCCESS       ? "SUCCESS"  :
        result == NavResult::BLIND_CONTACT ? "CONTACT"  :
        result == NavResult::BLIND_TIMEOUT ? "TIMEOUT"  :
        result == NavResult::VISION_LOST   ? "VIS LOST" : "BLOCKED";

    pros::screen::print(pros::E_TEXT_MEDIUM, 1, "RESULT: %s", resultStr);
    pros::screen::print(pros::E_TEXT_MEDIUM, 2, "END X:%.0f Y:%.0f H:%.0f",
                        globalX, globalY, getContinuousStandardHeading());

    Controller.rumble(result == NavResult::SUCCESS ||
                      result == NavResult::BLIND_CONTACT ? "." : "---");

    // Hold screen forever so RESULT and END position stay visible after run
    while (true) {
        pros::delay(100);
    }
}

// ══════════════════════════════════════════════════════════════════════════════
// AUTON SELECTOR
// Left/Right on controller to cycle options, A to confirm and run.
// ══════════════════════════════════════════════════════════════════════════════
void autonSelector() {
    const char* autonNames[] = {
        "AI Match (Red)",
        "AI Match (Blue)",
        "Route Test",
        "Grid Test",
        "System Test",
        "Coordinate Finder",
        "Vision Test",
    };
    const int numAutons = 7;
    int autonMode = 0;

    pros::screen::erase();

    while (true) {
        pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Selected: %s          ", autonNames[autonMode]);
        pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Left/Right to cycle");
        pros::screen::print(pros::E_TEXT_MEDIUM, 4, "A to run");

        if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            autonMode = (autonMode - 1 + numAutons) % numAutons;
            pros::delay(200);
        } else if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            autonMode = (autonMode + 1) % numAutons;
            pros::delay(200);
        } else if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            pros::screen::erase();
            pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Running: %s", autonNames[autonMode]);
            pros::delay(300);

            switch (autonMode) {
                case 0:  setAllianceRed(true);  runAIMatchRoute();      break;
                case 1:  setAllianceRed(false); runAIMatchRoute();      break;
                case 2:  routeTest();                                   break;
                case 3:  routeGridTest();                               break;
                case 4:  systemTest();                                  break;
                case 5:  coordinateFinder();                            break;
                case 6:  visionTest();                                  break;
            }
            break;
        }

        pros::delay(20);
    }
}