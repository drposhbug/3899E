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

// Helper macro — applies 180° coordinate transform to globalX/globalY
// after a GPS reset. Call immediately after pros::delay() post-requestGpsReset().
// No-op when FIELD_ROTATION_180 is false.
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
    vp.drive.breakDistance          = 35.0;   // %
    vp.drive.minSpeed               = 15.0;   // %
    vp.drive.maxSpeed               = 50.0;   // %
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
    vp.drive.headingLockDistance    = 15.0;   // cm

    // ── Traction / ABS ────────────────────────────────────────────────────────
    vp.drive.launchVoltage          = 6.0;    // V
    vp.drive.accelFactor            = 1.2;
    vp.drive.slipThreshold          = 0.3;
    vp.drive.decelStepPercent       = 2.0;    // %
    vp.drive.lockThreshold          = 0.3;

    // ── Circuit breaker ───────────────────────────────────────────────────────
    vp.drive.maxCurrentA            = 4.0;    // A — trips on goal contact
    vp.drive.overcurrentDurationMs  = 500;    // ms

    // ── Vision fusion ─────────────────────────────────────────────────────────
    vp.kp_vision_heading            = 0.05;   // heading PID gain after vision locks
    vp.kp_distToHeadScaling         = 5.0;

    // ── Object detection filter ───────────────────────────────────────────────
    vp.minObjectWidth               = 10;     // px — minimum valid detection
    vp.minX                         = 0;
    vp.maxX                         = 320;
    vp.minY                         = 0;
    vp.maxY                         = 240;

    visionForwardToPoint(aiVision_orangeBase, 220, 60.0, 120.0, vp);
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

// Push robot around to compare live GPS readings vs odometry.
// GPS raw X/Y shown alongside odometry X/Y, error, and reset status.
// Press A on controller to trigger a GPS reset at any time.
void gpsTest() {
    setStartPosition(0.0, 0.0, 0.0);
    startOdometryTask();

    pros::screen::erase();
    pros::screen::print(pros::E_TEXT_MEDIUM, 0, "GPS TEST — push robot around");
    pros::screen::print(pros::E_TEXT_MEDIUM, 1, "A = request GPS reset");

    while (true) {
        pros::gps_position_s_t gpsPos = gpsSensor.get_position();
        double gpsErr = gpsSensor.get_error();
        bool gpsOk = (gpsErr != PROS_ERR_F && gpsErr < GPS_MAX_ERROR_M);

        if (Controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            requestGpsReset();
        }

        pros::screen::print(pros::E_TEXT_MEDIUM, 2, "GPS raw  X:%.1f Y:%.1f cm",
            gpsPos.x * 100.0, gpsPos.y * 100.0);
        pros::screen::print(pros::E_TEXT_MEDIUM, 3, "GPS err: %.4fm  %s",
            gpsErr, gpsOk ? "OK" : "WEAK");
        pros::screen::print(pros::E_TEXT_MEDIUM, 4, "Odom     X:%.1f Y:%.1f cm",
            globalX, globalY);
        pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Heading: %.1f deg",
            getContinuousStandardHeading());
        pros::screen::print(pros::E_TEXT_MEDIUM, 6, "Reset: %s  inProg:%d",
            gpsResetSucceeded.load() ? "OK" : "FAIL",
            (int)gpsResetInProgress.load());

        pros::delay(100);
    }
}

// ══════════════════════════════════════════════════════════════════════════════
// FIELD TARGETS TEST
// ══════════════════════════════════════════════════════════════════════════════
void fieldTargetsTest() {
    for (int i = 0; i < 8; i++) pros::lcd::clear_line(i);
    pros::screen::erase();

    setStartPosition(124.5, 34.5, 0.0);
    startOdometryTask();

    // GPS reset before routing — wait up to 200ms for task to finish
    // so A* starts from a corrected position rather than setStartPosition estimate
    requestGpsReset();
    pros::delay(200);  // 8 samples × 15ms = 120ms; 200ms gives it room to complete
    //APPLY_FIELD_ROTATION();

    pros::lcd::print(0, "START X:%.0f Y:%.0f H:%.0f",
                     globalX, globalY, getContinuousStandardHeading());

    NavResult 
    result = navigateTo(LOADER_NW);
    result = navigateTo(LOADER_NE);
    result = navigateTo(LOADER_SW);
    result = navigateTo(LOADER_SE);

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
// SWEEP TEST
// Runs visionSweepNorth() standalone — no match timer, no Jetson required.
// Alliance set via setAllianceRed() in the selector before calling this.
// ══════════════════════════════════════════════════════════════════════════════
void sweepTest(bool isRed) {
    for (int i = 0; i < 8; i++) pros::lcd::clear_line(i);
    pros::screen::erase();

    setStartPosition(0.0, 0.0, 0.0);
    startOdometryTask();

    requestGpsReset();
    pros::delay(200);

    pros::screen::print(pros::E_TEXT_MEDIUM, 0, "SWEEP TEST — 30s");
    pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Alliance: %s", isRed ? "RED" : "BLUE");

    visionSweepNorth();

    pros::screen::print(pros::E_TEXT_MEDIUM, 2, "SWEEP DONE — at nearest goal");
    Controller.rumble(".");

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
        "GPS Test",
        "Sweep Test (Red)",
        "Sweep Test (Blue)",
    };
    const int numAutons = 10;
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
                case 7:  gpsTest();                                     break;
                case 8:  setAllianceRed(true);  sweepTest(true);             break;
                case 9:  setAllianceRed(false); sweepTest(false);            break;
            }
            break;
        }

        pros::delay(20);
    }
}


void rightSideAuton(){
    setStartPosition(124.8, 37, -90);
    startOdometryTask();
    // intakeMotor1.move_voltage(12000);
    // intakeMotor2.move_voltage(-12000);
    StraightProfile driveProfile = LOADED_MID_FWD_80;
    driveProfile.breakDistance          = 20.0;
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
    // driveForward(33,-90,driveProfile);


    TurnProfile turnProfile1 = DEFAULT_TURN;
        turnProfile1.breakDistance    = 70.0,   // % of total turn angle
        turnProfile1.minSpeed         = 22.0,
        turnProfile1.maxSpeed         = 70.0,
        turnProfile1.exitTolerance    = 3.0,    // degrees
        turnProfile1.timeout          =1.0,

        // ── Internal motion constants ─────────────────────────────────────────────
        turnProfile1.accelFactor      = 1.2,
        turnProfile1.slipThreshold    = 1.0,    // never triggers
        turnProfile1.decelStepPercent = 10.0,
        turnProfile1.lockThreshold    = 1.0,    // never triggers
        turnProfile1.maxCurrentA      = 8.0,
        turnProfile1.overcurrentDurationMs = 500;
        
    // turnRight(30, turnProfile1);
    StraightProfile driveProfile1 = LOADED_MID_FWD_80;
        driveProfile1.breakDistance          = 30.0;
        driveProfile1.minSpeed               = 30.0;
        driveProfile1.maxSpeed               = 40.0;
        driveProfile1.distanceTolerance      = 2.0;
        driveProfile1.timeout                = 5.0;
        driveProfile1.brakeMode              = pros::E_MOTOR_BRAKE_BRAKE;
        driveProfile1.kp_heading             = 0.1;
        driveProfile1.ki_heading             = 0.0;
        driveProfile1.kd_heading             = 0.0;
        driveProfile1.accelHeadingScaling    = 0.1;
        driveProfile1.decelHeadingScaling    = 0.075;
        driveProfile1.approachHeadingScaling = 0.075;
        driveProfile1.headingLockDistance    = 5.0;
        driveProfile1.launchVoltage          = 6.0;
        driveProfile1.accelFactor            = 1.2;
        driveProfile1.slipThreshold          = 0.3;
        driveProfile1.decelStepPercent       = 2.0;
        driveProfile1.lockThreshold          = 0.50;
        driveProfile1.maxCurrentA            = 8.0;
        driveProfile1.overcurrentDurationMs  = 500;
    // driveForward(50,35,driveProfile1);
    StraightProfile driveProfile2 = LOADED_MID_FWD_80;
        driveProfile2.breakDistance          = 15.0;
        driveProfile2.minSpeed               = 15.0;
        driveProfile2.maxSpeed               = 30.0;
        driveProfile2.distanceTolerance      = 2.0;
        driveProfile2.timeout                = 1.5;
        driveProfile2.brakeMode              = pros::E_MOTOR_BRAKE_BRAKE;
        driveProfile2.kp_heading             = 0.2;
        driveProfile2.ki_heading             = 0.0;
        driveProfile2.kd_heading             = 0.0;
        driveProfile2.accelHeadingScaling    = 0.1;
        driveProfile2.decelHeadingScaling    = 0.075;
        driveProfile2.approachHeadingScaling = 0.075;
        driveProfile2.headingLockDistance    = 5.0;
        driveProfile2.launchVoltage          = 6.0;
        driveProfile2.accelFactor            = 1.2;
        driveProfile2.slipThreshold          = 0.3;
        driveProfile2.decelStepPercent       = 2.0;
        driveProfile2.lockThreshold          = 0.50;
        driveProfile2.maxCurrentA            = 8.0;
        driveProfile2.overcurrentDurationMs  = 500;

    turnLeft(-96,turnProfile1);
    driveForward(37,-96,driveProfile);
    turnRight(10, turnProfile1);
    driveForward(35,10,driveProfile1);
    turnRight(45, turnProfile1);
    driveForward(85,45,driveProfile1);
    turnLeft(0, turnProfile1);
    driveForward(10,0,driveProfile);
    // driveBackward(5,0,driveProfile1);
    turnRight(75, turnProfile1);
    driveForward(30,90,driveProfile2);
    // pros::delay(600);
    driveBackward(12,90,driveProfile);
    turnRight(180, turnProfile1);
    driveForward(20,180,driveProfile);
    // turnRight(-90, turnProfile1);
    navigateTo(LONG_GOAL_NE);
    // driveForward(40,-90,driveProfile2);
    //score

    pros::delay(1000000);
    driveForward(80,-150,driveProfile2);
    pros::delay(1000);
    driveBackward(30,-143,driveProfile);
    pros::delay(1000);
    turnLeft(-165,turnProfile1);
    pros::delay(1000);
    driveForward(130,-165,driveProfile);
    

    // turnRight(37,turnProfile1);
    // driveForward(60,37,driveProfile);
    // driveForward(30,90,driveProfile2);

    


}
