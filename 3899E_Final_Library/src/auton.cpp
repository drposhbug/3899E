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
    startOdometryTask();
    setStartPosition(0.0, 0.0, 0.0);
    pros::delay(200);

    StraightProfile driveProfile = DEFAULT_STRAIGHT;
    driveProfile.breakDistance          = 58.0;   // cm before target to begin decel
    driveProfile.minSpeed               = 20.0;   // % minimum approach speed
    driveProfile.maxSpeed               = 80.0;   // % peak cruise speed
    driveProfile.distanceTolerance      = 2.0;    // cm exit bubble
    driveProfile.timeout                = 5.0;    // seconds 5 sec default
    driveProfile.brakeMode              = pros::E_MOTOR_BRAKE_BRAKE;
    driveProfile.kp_heading             = 0.5;    // heading PID proportional
    driveProfile.ki_heading             = 0.0;    // heading PID integral
    driveProfile.kd_heading             = 0.0;    // heading PID derivative
    driveProfile.accelHeadingScaling    = 0.2;    // correction weight during accel
    driveProfile.decelHeadingScaling    = 0.1;    // correction weight during decel
    driveProfile.approachHeadingScaling = 0.1;    // correction weight during approach
    driveProfile.headingLockDistance    = 3.0;    // cm — freeze heading near target
    driveProfile.launchVoltage          = 3.0;    // V — initial kick voltage
    driveProfile.accelFactor            = 1.2;    // traction ramp multiplier
    driveProfile.slipThreshold          = 0.3;   // RPM slip before traction cuts in
    driveProfile.decelStepPercent       = 2.0;   // ABS voltage reduction per step
    driveProfile.lockThreshold          = 0.3;   // wheel lockup ratio
    driveProfile.maxCurrentA            = 4.0;   // amps — wall stall trip threshold
    driveProfile.overcurrentDurationMs  = 300; // ms — how long before breaker fires

    TurnProfile turnProfile = DEFAULT_TURN;
    turnProfile.breakDistance  = 5.0;    // degrees before target to begin decel
    turnProfile.minSpeed       = 10.0;   // % minimum approach speed
    turnProfile.maxSpeed       = 20.0;  // % peak turn speed
    turnProfile.exitTolerance  = 3;    // degrees — stop when within this
    turnProfile.timeout        = 3.0;    // seconds — release if stuck

    /*
    // ── Full square backward: (0,0) → (0,100) → (100,100) → (100,0) → (0,0) ──
    // Front faces opposite direction of travel so rear closes on target.
    turnLeft(-180.0, turnProfile);             // face south — back faces north
    pros::delay(300);
    backwardToPoint(  0.0, 100.0, driveProfile);  // leg 1: back north to (0,100)
    pros::delay(300);
    turnLeft(-270.0, turnProfile);             // face west — back faces east
    pros::delay(300);
    backwardToPoint(-100.0, 100.0, driveProfile);  // leg 2: back east to (100,100)
    pros::delay(300);
    turnLeft(0.0, turnProfile);             // face north — back faces south
    pros::delay(300);
    backwardToPoint(-100.0,   0.0, driveProfile);  // leg 3: back south to (100,0)
    pros::delay(300);
    turnLeft( -90.0, turnProfile);             // face east — back faces west
    pros::delay(300);
    backwardToPoint(  0.0,   0.0, driveProfile);  // leg 4: back west to origin
    */

    // ── Stage 2: straight + turn + straight ──────────────────────────
    // forwardToPoint(0.0, 100.0, driveProfile);
    // pros::delay(300);
    // turnToPoint(100.0, 100.0, turnProfile);
    // pros::delay(300);
    // forwardToPoint(100.0, 100.0, driveProfile);

    // ── Stage 1: forward + backward (closed-loop) ────────────────────
    // forwardToPoint(0.0, 100.0, driveProfile);
    // pros::delay(500);
    // setStartPosition(0.0, 100.0, 0.0);
    // backwardToPoint(0.0, 0.0, driveProfile);

    // ── driveForward/driveBackward test (open-loop) ───────────────────
    driveForward(150.0, 0.0, driveProfile);
    pros::delay(500);
    // setStartPosition(0.0, 150.0, 0.0);
    // driveBackward(150.0, 0.0, driveProfile);
    // pros::delay(500);
    // setStartPosition(0,0,0);
    // driveForward(50,0,driveProfile);
    // pros::delay(500);
    // driveBackward(50,0,driveProfile);
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
// void systemTest() {
//     pros::screen::erase();
//     pros::screen::print(pros::E_TEXT_MEDIUM, 1, "System Test");

//     InertialSensor.reset(true);

//     pros::Motor* allMotors[] = {
//         &LeftMotor1, &LeftMotor2, &LeftMotor3,
//         &RightMotor1, &RightMotor2, &RightMotor3,
//         &intakeMotor1, &intakeMotor2
//     };
//     const int numMotors = 8;

//     for (int i = 0; i < numMotors; i++) {
//         pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Testing motor %d...", i + 1);
//         allMotors[i]->move(80);
//         pros::delay(500);
//         bool alive = (allMotors[i]->get_actual_velocity() != 0);
//         allMotors[i]->brake();
//         if (alive) Controller.rumble(".");
//         pros::delay(300);
//     }

//     pros::screen::print(pros::E_TEXT_MEDIUM, 2, "System test complete");
//     Controller.rumble(". .");
// }

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
// void autonSelector() {
//     const char* autonNames[] = {
//         "AI Match (Red)",
//         "AI Match (Blue)",
//         "Route Test",
//         "Grid Test",
//         "System Test",
//         "Coordinate Finder",
//     };
//     const int numAutons = 6;
//     int autonMode = 0;

//     pros::screen::erase();

//     while (true) {
//         pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Selected: %s          ", autonNames[autonMode]);
//         pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Left/Right to cycle");
//         pros::screen::print(pros::E_TEXT_MEDIUM, 4, "A to run");

//         if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
//             autonMode = (autonMode - 1 + numAutons) % numAutons;
//             pros::delay(200);
//         } else if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
//             autonMode = (autonMode + 1) % numAutons;
//             pros::delay(200);
//         } else if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
//             pros::screen::erase();
//             pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Running: %s", autonNames[autonMode]);
//             pros::delay(300);

//             switch (autonMode) {
//                 case 0: setAllianceRed(true);  runAIMatchRoute(); break;
//                 case 1: setAllianceRed(false); runAIMatchRoute(); break;
//                 case 2: routeTest();            break;
//                 case 3: routeGridTest();        break;
//                 case 4: systemTest();           break;
//                 case 5: coordinateFinder();     break;
//             }
//             break;
//         }

//         pros::delay(20);
//     }
// }

void autonLeft15(){
    startOdometryTask();
    setStartPosition(-124.262, 11.548,0);

    StraightProfile driveProfile = DEFAULT_STRAIGHT;
    driveProfile.breakDistance          = 50.0;   // cm before target to begin decel
    driveProfile.minSpeed               = 20.0;   // % minimum approach speed
    driveProfile.maxSpeed               = 80.0;   // % peak cruise speed
    driveProfile.distanceTolerance      = 2.0;    // cm exit bubble
    driveProfile.timeout                = 5.0;    // seconds 5 sec default
    driveProfile.brakeMode              = pros::E_MOTOR_BRAKE_BRAKE;
    driveProfile.kp_heading             = 1.5;    // heading PID proportional
    driveProfile.ki_heading             = 0.0;    // heading PID integral
    driveProfile.kd_heading             = 0.0;    // heading PID derivative
    driveProfile.accelHeadingScaling    = 0.1;    // correction weight during accel
    driveProfile.decelHeadingScaling    = 0.0;    // correction weight during decel
    driveProfile.approachHeadingScaling = 0.0;    // correction weight during approach
    driveProfile.headingLockDistance    = 3.0;    // cm — freeze heading near target
    driveProfile.launchVoltage          = 6.0;    // V — initial kick voltage
    driveProfile.accelFactor            = 1.0;    // traction ramp multiplier
    driveProfile.slipThreshold          = 1.0;   // RPM slip before traction cuts in
    driveProfile.decelStepPercent       = 0.45;   // ABS voltage reduction per step
    driveProfile.lockThreshold          = 0.0;   // wheel lockup ratio
    driveProfile.maxCurrentA            = 4.0;   // amps — wall stall trip threshold
    driveProfile.overcurrentDurationMs  = 300; // ms — how long before breaker fires



    driveForward(60, 0.0, driveProfile);
    TurnProfile turnProfile = DEFAULT_TURN;
    turnProfile.breakDistance  = 10.0;    // degrees before target to begin decel
    turnProfile.minSpeed       = 15.0;   // % minimum approach speed
    turnProfile.maxSpeed       = 30.0;  // % peak turn speed
    turnProfile.exitTolerance  = 3;    // degrees — stop when within this
    turnProfile.timeout        = 3.0;    // seconds — release if stuck
    turnLeft(-135.0, turnProfile);
    // movement 2
    StraightProfile driveProfile1 = DEFAULT_STRAIGHT;
    driveProfile1.breakDistance          = 10.0;   // cm before target to begin decel
    driveProfile1.minSpeed               = 10.0;   // % minimum approach speed
    driveProfile1.maxSpeed               = 40.0;   // % peak cruise speed
    driveProfile1.distanceTolerance      = 2.0;    // cm exit bubble
    driveProfile1.timeout                = 5.0;    // seconds 5 sec default
    driveProfile1.brakeMode              = pros::E_MOTOR_BRAKE_BRAKE;
    driveProfile.kp_heading             = 0.3;    // heading PID proportional
    driveProfile.ki_heading             = 0.0;    // heading PID integral
    driveProfile.kd_heading             = 2;    // heading PID derivative
    driveProfile1.accelHeadingScaling    = 0.1;    // correction weight during accel
    driveProfile1.decelHeadingScaling    = 0.0;    // correction weight during decel
    driveProfile1.approachHeadingScaling = 0.0;    // correction weight during approach
    driveProfile1.headingLockDistance    = 3.0;    // cm — freeze heading near target
    driveProfile1.launchVoltage          = 6.0;    // V — initial kick voltage
    driveProfile1.accelFactor            = 1.0;    // traction ramp multiplier
    driveProfile1.slipThreshold          = 1.0;   // RPM slip before traction cuts in
    driveProfile1.decelStepPercent       = 0.45;   // ABS voltage reduction per step
    driveProfile1.lockThreshold          = 0.0;   // wheel lockup ratio
    driveProfile1.maxCurrentA            = 4.0;   // amps — wall stall trip threshold
    driveProfile1.overcurrentDurationMs  = 300; // ms — how long before breaker fires
    // forwardToPoint(-54.246,55.192,driveProfile1);

}
