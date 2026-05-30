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
//straightOdometryV3(100, 70, -20, 10, 5, 1, 0, 0, .2, .2, 40, 50);

moveOdometry(
    0,         // targetX — hold current X
    100,         // targetY — 100cm forward (was targetDistance)
    70.0,            // breakDistance
    10.0,            // minSpeed
    5.0,             // distanceTolerance
    .1,             // kp_heading
    0.0,             // ki_heading
    0.0,             // kd_heading
    pros::E_MOTOR_BRAKE_BRAKE,
    1000,             // accelHeadingScaling
    0.0,             // decelHeadingScaling
    0.0,             // approachHeadingScaling (no equivalent in straight, defaulting)
    50.0,            // maxSpeed
    5.0,            // headingLockDistance (no equivalent, reasonable default)
    5.0              // timeout (no equivalent, reasonable default)
);
}


 

// Prints the route planner obstacle grid to brain screen.
// Use before competition to verify goal and park zone positions are correct.
void routeGridTest() {
    routePrintGrid();
    // Hold until screen tap
    while (pros::screen::touch_status().touch_status == pros::E_TOUCH_RELEASED)
        pros::delay(50);
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
// Useful for verifying field position mapping before a match.
void coordinateFinder() {
    startOdometryTask();
    setStartPosition(0.0, 0.0, 0.0);
    startCoordinateFinder();
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
    };
    const int numAutons = 6;
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
                case 0: setAllianceRed(true);  runAIMatchRoute(); break;
                case 1: setAllianceRed(false); runAIMatchRoute(); break;
                case 2: routeTest();            break;
                case 3: routeGridTest();        break;
                case 4: systemTest();           break;
                case 5: coordinateFinder();     break;
            }
            break;
        }

        pros::delay(20);
    }
}