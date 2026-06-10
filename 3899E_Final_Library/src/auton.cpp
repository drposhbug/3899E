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
#include "motion_config.h"  // DEFAULT_TURN and all named motion profiles
#include "navigation.h"
#include "odometry.h"
#include "autontasks.h"
#include "ai.h"
#include "route_planner.h"
#include "robot_geometry.h"
#include "field_targets.h"

// ── Field orientation flag ────────────────────────────────────────────────────
// HOME FIELD ONLY — set true when field strips are rotated 180° from standard.
// Standard: GPS North (+Y) = red alliance wall.
// 180°: GPS North is physically the blue alliance wall — negate both axes.
// SET TO FALSE at competition venue where field is correctly oriented.
// Each robot sets this independently in their own auton.cpp.
#define FIELD_ROTATION_180  true

#if FIELD_ROTATION_180
    #define APPLY_FIELD_ROTATION() do { \
        globalX = -globalX;             \
        globalY = -globalY;             \
    } while(0)
#else
    #define APPLY_FIELD_ROTATION() do {} while(0)
#endif
// ══════════════════════════════════════════════════════════════════════════════
// ROUTE FUNCTIONS
// ══════════════════════════════════════════════════════════════════════════════

// Full VAIRC match — Jetson drives strategy, V5 executes via ai.cpp
void runAIMatchRoute() {
    setStartPosition(0.0, 0.0, 0.0);  // update start coords before competition
    startOdometryTask();
    requestGpsReset();
    pros::delay(200);
    APPLY_FIELD_ROTATION();
    setAllianceRed(true);              // set to false for blue alliance
    runAIMatch();
}

// Manual route test — plans a path from current position to a target and drives it.
// Use this to verify route_planner and moveOdometry are working on the real field.
void routeTest() {
    pros::screen::erase();
    pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Route Test");

    setStartPosition(0.0, 0.0, 0.0);
    startOdometryTask();

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
    setStartPosition(0.0, 0.0, 0.0);
    startOdometryTask();

    StraightProfile driveProfile = LOADED_MID_FWD_80;
    driveProfile.breakDistance          = 60.0;
    driveProfile.minSpeed               = 15.0;
    driveProfile.maxSpeed               = 100.0;
    driveProfile.distanceTolerance      = 2.0;
    driveProfile.timeout                = 5.0;
    driveProfile.brakeMode              = pros::E_MOTOR_BRAKE_BRAKE;
    driveProfile.kp_heading             = 0.5;
    driveProfile.ki_heading             = 0.01;
    driveProfile.kd_heading             = 0.0;
    driveProfile.accelHeadingScaling    = 0.1;
    driveProfile.decelHeadingScaling    = 0.075;
    driveProfile.approachHeadingScaling = 0.075;
    driveProfile.headingLockDistance    = 5.0;
    driveProfile.launchVoltage          = 6.0;
    driveProfile.accelFactor            = 1.2;
    driveProfile.slipThreshold          = 0.3;
    driveProfile.decelStepPercent       = 2.0;
    driveProfile.lockThreshold          = 0.50;
    driveProfile.maxCurrentA            = 8.0;
    driveProfile.overcurrentDurationMs  = 500;

    //forwardToPoint(0.0, 80.0, driveProfile);
    //driveForward(150.0, 0.0, LOADED_MID_FWD_80);
    //turnToPoint(80.0, 0.0, DEFAULT_TURN);
    turnOdometry(100.0, DEFAULT_TURN);
}

void visionTest() {
    setStartPosition(-124.5, -34.5, 0.0);
    startOdometryTask();

    requestGpsReset();
    pros::delay(200);

    // ── Tune all parameters here, then copy final values to VISION_LONG_GOAL_FWD ──
    VisionProfile vp = VISION_LONG_GOAL_FWD;  // start from current named profile

    // ── Motion shape ──────────────────────────────────────────────────────────
    vp.drive.breakDistance          = 40.0;   // %
    vp.drive.minSpeed               = 15.0;   // %
    vp.drive.maxSpeed               = 40.0;   // %
    vp.drive.distanceTolerance      = 2.0;    // cm
    vp.drive.timeout                = 5.0;    // s
    vp.drive.brakeMode              = pros::E_MOTOR_BRAKE_BRAKE;

    // ── Heading PID ───────────────────────────────────────────────────────────
    vp.drive.kp_heading             = 0.05;
    vp.drive.ki_heading             = 0.0;
    vp.drive.kd_heading             = 0.0;

    // ── Phase heading scaling ─────────────────────────────────────────────────
    vp.drive.accelHeadingScaling    = 0.2;
    vp.drive.decelHeadingScaling    = 0.1;
    vp.drive.approachHeadingScaling = 0.1;
    vp.drive.headingLockDistance    = 5.0;   // cm

    // ── Traction / ABS ────────────────────────────────────────────────────────
    vp.drive.launchVoltage          = 6.0;    // V
    vp.drive.accelFactor            = 1.2;
    vp.drive.slipThreshold          = 0.3;
    vp.drive.decelStepPercent       = 2.0;    // %
    vp.drive.lockThreshold          = 0.50;

    // ── Circuit breaker ───────────────────────────────────────────────────────
    vp.drive.maxCurrentA            = 4.0;    // A — trips on goal contact
    vp.drive.overcurrentDurationMs  = 250;    // ms

    // ── Vision fusion ─────────────────────────────────────────────────────────
    vp.kp_distToHeadScaling         = 5.0;

    // ── Object detection filter ───────────────────────────────────────────────
    vp.minObjectWidth               = 10;     // px — minimum valid detection
    vp.minX                         = 0;
    vp.maxX                         = 320;
    vp.minY                         = 0;
    vp.maxY                         = 240;

    visionForwardToPoint(aiVision_orangeBase, 200, 60.0, 120.0, vp);
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
    setStartPosition(0.0, 0.0, 0.0);
    startOdometryTask();
    startCoordinateFinder();
}

// ══════════════════════════════════════════════════════════════════════════════
// FIELD TARGETS TEST
// ══════════════════════════════════════════════════════════════════════════════
void fieldTargetsTest() {
    for (int i = 0; i < 8; i++) pros::lcd::clear_line(i);
    pros::screen::erase();

    setStartPosition(-124.5, -34.5, 180.0);
    startOdometryTask();

    // GPS reset before routing — wait up to 200ms for task to finish
    // so A* starts from a corrected position rather than setStartPosition estimate
    requestGpsReset();
    pros::delay(200);  // 8 samples × 15ms = 120ms; 200ms gives it room to complete
    //APPLY_FIELD_ROTATION();


    pros::lcd::print(0, "START X:%.0f Y:%.0f H:%.0f",
                     globalX, globalY, getContinuousStandardHeading());

    NavResult result = navigateTo(LONG_GOAL_NE);

     const char* resultStr =
         result == NavResult::SUCCESS       ? "SUCCESS"  :
         result == NavResult::BLIND_CONTACT ? "CONTACT"  :
         result == NavResult::BLIND_TIMEOUT ? "TIMEOUT"  :
         result == NavResult::VISION_LOST   ? "VIS LOST" : "BLOCKED";

     pros::screen::print(pros::E_TEXT_MEDIUM, 1, "RESULT: %s", resultStr);

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
