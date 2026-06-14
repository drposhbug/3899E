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

// Helper macro — applies 180° coordinate transform to globalX/globalY
// after a GPS reset. Call immediately after pros::delay() post-requestGpsReset().
// ══════════════════════════════════════════════════════════════════════════════
// ROUTE FUNCTIONS
// ══════════════════════════════════════════════════════════════════════════════

// Full VAIRC match — Jetson drives strategy, V5 executes via ai.cpp
void runAIMatchRoute() {
    setStartPosition(0.0, 0.0, 0.0);  // update start coords before competition
    requestGpsReset();
    pros::delay(200);
    setAllianceRed(true);              // set to false for blue alliance
    runAIMatch();
}

// Manual route test — plans a path from current position to a target and drives it.
// Use this to verify route_planner and moveOdometry are working on the real field.
void routeTest() {
    pros::screen::erase();
    pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Route Test");

    setStartPosition(0.0, 0.0, 0.0);

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
    setStartPosition(0, -176, 0.0);
    //setStartPosition(-124.5, -34.5, 0.0);

    //requestGpsReset();
    pros::delay(200);

    // ── Tune all parameters here, then copy final values to VISION_LONG_GOAL_FWD ──
    VisionProfile vp = VISION_LONG_GOAL_FWD;  // start from current named profile

    // ── Motion shape ──────────────────────────────────────────────────────────
    vp.drive.breakDistance          = 35.0;   // %
    vp.drive.minSpeed               = 15.0;   // %
    vp.drive.maxSpeed               = 40.0;   // %
    vp.drive.distanceTolerance      = 2.0;    // cm
    vp.drive.timeout                = 3.0;    // s
    vp.drive.brakeMode              = pros::E_MOTOR_BRAKE_BRAKE;

    // ── Heading PID ───────────────────────────────────────────────────────────
    vp.drive.kp_heading             = 0.1;
    vp.drive.ki_heading             = 0.0;
    vp.drive.kd_heading             = 0.0;

    // ── Phase heading scaling ─────────────────────────────────────────────────
    vp.drive.accelHeadingScaling    = 1;
    vp.drive.decelHeadingScaling    = 1;
    vp.drive.approachHeadingScaling = 1;
    vp.drive.headingLockDistance    = 1;   // cm

    // ── Traction / ABS ────────────────────────────────────────────────────────
    vp.drive.launchVoltage          = 3.0;    // V
    vp.drive.accelFactor            = 1.2;
    vp.drive.slipThreshold          = 0.3;
    vp.drive.decelStepPercent       = 2.0;    // %
    vp.drive.lockThreshold          = 0.3;

    // ── Circuit breaker ───────────────────────────────────────────────────────
    vp.drive.maxCurrentA            = 4.0;    // A — trips on goal contact
    vp.drive.overcurrentDurationMs  = 500;    // ms

    // ── Vision fusion ─────────────────────────────────────────────────────────
    vp.kp_vision_heading            = 0.1;   // heading PID gain after vision locks
    vp.kp_distToHeadScaling         = 3.0;

    // ── Object detection filter ───────────────────────────────────────────────
    vp.minObjectWidth               = 10;     // px — minimum valid detection
    vp.minX                         = 0;
    vp.maxX                         = 320;
    vp.minY                         = 0;
    vp.maxY                         = 300;

    visionForwardToPoint(aiVision_redCube, 60, 32, -62, vp);
    
    //visionDriveForward(aiVision_blueCube, 80, 150.0);
    //visionOnly(aiVision_blueCube, 80, 150.0, vp);
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
    startCoordinateFinder();
}

// Push robot around to compare live GPS readings vs odometry.
// GPS raw X/Y shown alongside odometry X/Y, error, and reset status.
// Press A on controller to trigger a GPS reset at any time.
void gpsTest() {
    setStartPosition(0.0, 0.0, 0.0);

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

    // GPS reset before routing — wait up to 200ms for task to finish
    // so A* starts from a corrected position rather than setStartPosition estimate
    requestGpsReset();
    pros::delay(200);  // 8 samples × 15ms = 120ms; 200ms gives it room to complete

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
// 

// ═════════════════════════════════════════════════════════════════════════════
void sweepTest(bool isRed) {
    for (int i = 0; i < 8; i++) pros::lcd::clear_line(i);
    pros::screen::erase();

    setStartPosition(0.0, 0.0, 0.0);

    requestGpsReset();
    pros::delay(200);

    pros::screen::print(pros::E_TEXT_MEDIUM, 0, "SWEEP TEST — 30s");
    pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Alliance: %s", isRed ? "RED" : "BLUE");

    sweepAndScore(80, 500, 8, 0, 25.0, 0.02, 0.8, 500.0, 20.0, 0);

    pros::screen::print(pros::E_TEXT_MEDIUM, 2, "SWEEP DONE — at nearest goal");
    Controller.rumble(".");

    while (true) {
        pros::delay(100);
    }
}

// ══════════════════════════════════════════════════════════════════════════════
// SWEEP AND RETURN
// Sweeps the field collecting blocks, scores at nearest goal, then A* navigates
// back to the starting position.
// ══════════════════════════════════════════════════════════════════════════════
void sweepAndReturn(bool isRed) {
    for (int i = 0; i < 8; i++) pros::lcd::clear_line(i);
    pros::screen::erase();

    setStartPosition(0.0, 0.0, 0.0);  // TODO: update to actual start coords
    setAllianceRed(isRed);

 //   requestGpsReset();
   // pros::delay(200);

    // Snapshot position after GPS settle — this is where we return to.
    double homeX = globalX;
    double homeY = globalY;

    pros::screen::print(pros::E_TEXT_MEDIUM, 0, "SWEEP — home (%.0f, %.0f)", homeX, homeY);

    // ── Sweep ─────────────────────────────────────────────────────────────────
    sweepAndScore(80, 500, 8, isRed ? 0 : 1, 25.0, 0.02, 0.8, 500.0, 20.0, 30000);

    pros::screen::print(pros::E_TEXT_MEDIUM, 1, "SWEEP DONE — returning home");

    // ── A* back to start ──────────────────────────────────────────────────────
 //   requestGpsReset();
   // pros::delay(200);

    RoutePath returnPath = routePlan(globalX, globalY, homeX, homeY);
    if (returnPath.count > 0) {
        bool reached = routeExecute(returnPath);
        pros::screen::print(pros::E_TEXT_MEDIUM, 2, reached ? "HOME — done" : "HOME — blocked");
        Controller.rumble(reached ? ". ." : "---");
    } else {
        pros::screen::print(pros::E_TEXT_MEDIUM, 2, "HOME — no path found");
        Controller.rumble("---");
    }

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
                case 8:  setAllianceRed(true);  sweepAndScore(80, 500, 8, 0, 25.0, 0.02, 0.8, 500.0, 20.0, 0); break;
                case 9:  setAllianceRed(false); sweepAndScore(80, 500, 8, 1, 25.0, 0.02, 0.8, 500.0, 20.0, 0); break;
            }
            break;
        }

        pros::delay(20);
    }
}


void rightSideAuton(){
    setStartPosition(124.8, 37, -90);

    // Colour sort task — runs entire auton, sorts blocks as they pass the optical sensor.
    // Blue right side auton: sorts blue blocks into right bay, red into left bay.
    static ColorTaskParams colorParams;
    colorParams.isRunning = true;
    colorParams.delayMs   = 0;
    pros::Task(colorDetectionTask, &colorParams, "colourSort");
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

    intakeHopperStart(10000, -100.0, 0.0, true);
    
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

    scoreBlueStart(4000);

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


// ══════════════════════════════════════════════════════════════════════════════
// DEMO: 15-SECOND LONG GOAL AUTONOMOUS
//
// PURPOSE: Shows how to combine all major auton building blocks in one routine.
//          Read the comments — every function call is explained.
//
// CALL FROM main.cpp autonomous():
//   longGoalAuto15s(true);   // red alliance
//   longGoalAuto15s(false);  // blue alliance
//
// WHAT IT DOES:
//   1. Set starting position + start odometry
//   2. GPS reset for accurate field position
//   3. Start intake (async — runs while robot drives)
//   4. A* navigate to the long goal approach point
//   5. Delay to let blocks collect, then start scoring (async — non-blocking)
//   6. Score red or blue blocks into correct bay depending on alliance
//
// ALLIANCE LOGIC:
//   Red  → approaches east long goal (LONG_GOAL_NE), scores red  (left bay)
//   Blue → approaches west long goal (LONG_GOAL_NW), scores blue (right bay)
//
// TUNING NOTES:
//   - Adjust setStartPosition() to match your actual starting tile
//   - intakeHopperStart timeMs should cover transit + scoring duration
//   - scoreRedStart/scoreBlueStart timeMs = how long to run scoring motors
//   - Add requestGpsReset() + pros::delay(200) before navigateTo for better accuracy
// ══════════════════════════════════════════════════════════════════════════════
void longGoalAuto15s(bool isRedAlliance) {

    // ── STEP 1: Set starting position ─────────────────────────────────────────
    // TODO: replace 0.0 with your actual starting heading.
    setStartPosition(124.0, 37.0, 270);

    // ── STEP 2: Start odometry task ───────────────────────────────────────────

    // ── STEP 3: GPS reset ─────────────────────────────────────────────────────
 requestGpsReset();
   // pros::delay(200);

    // ── STEP 4: Alliance ──────────────────────────────────────────────────────
    setAllianceRed(false);   // blue alliance

    // ── STEP 5: Start colour sort + intake before moving ─────────────────────
    // Eject red (opponent colour on blue alliance).
    ColorTaskParams colorParams = {
        true,
        Color::RED,
        0
    };
    pros::Task colorSortTask(colorDetectionTask, &colorParams, "Color Sort");
    intakeHopperStart(10000, -100.0, 0.0, true);
 // turnRight(, DEFAULT_TURN);

    StraightProfile customFwd = MID_FWD;
    customFwd.breakDistance     = 70.0;
    customFwd.minSpeed          = 13.0;
    customFwd.maxSpeed          = 70.0;
    customFwd.distanceTolerance = 1.0;
    customFwd.timeout           = 8.0;
    customFwd.brakeMode         = pros::E_MOTOR_BRAKE_BRAKE;
    customFwd.kp_heading        = 0.6;
    customFwd.ki_heading        = 0.0;
    customFwd.kd_heading        = 0.0;

    // ── STEP 6: Manual waypoints (odometry point-to-point) ───────────────────
    forwardToPoint(75.0,  37.0,  customFwd);  // heading ~82

    turnRight(30, TURN_45);
      pros::delay(300);

    forwardToPoint(117.0,  96.0, customFwd);  // heading ~200

    turnLeft(2, TURN_45);
    forwardToPoint(117.0, 151.0,  customFwd);  // heading ~275

    //turnLeft(180, DEFAULT_TURN);
    //forwardToPoint(-119.0, -175.0, LOADED_MID_FWD_80);  // heading ~180

    // ── STEP 7: A* to south-west long goal ───────────────────────────────────
   if (!arrivedAt(navigateTo(LONG_GOAL_NE))) {
        colorParams.isRunning = false;
       intakeHopperStop();

       return;
    }

    // Stop colour sort before scoring — don't want it firing during score run.
    colorParams.isRunning = false;
    pros::delay(20);  // let task exit its 10ms loop cleanly

    // ── STEP 8: Brief settle delay, then score ────────────────────────────────
    pros::delay(500);

    scoreBlueStart(4000);  // right bay

    // ── STEP 9: GPS reset while scoring ──────────────────────────────────────
  //  requestGpsReset();
    //pros::delay(200);

    // ── STEP 10: Wait for scoring to finish ───────────────────────────────────
    pros::delay(4000);
    
}

void TestTurn(bool isRedAlliance) {
/* 30 profle 
    TurnProfile customTurn = MID_TURN;
    customTurn.breakDistance         = 80.0;  // was 80 — start braking earlier
    customTurn.minSpeed              = 19.5;  // was 22 — crawl into target
    customTurn.maxSpeed              = 60.0;  // was 60 — less momentum to shed
    customTurn.exitTolerance         = 3.0;
    customTurn.timeout               = 5.0;
    customTurn.accelFactor           = 1.2;
    customTurn.slipThreshold         = 1.0;
    customTurn.decelStepPercent      = 5.0;   // was 10 — smoother decel curve
    customTurn.lockThreshold         = 1.0;
    customTurn.maxCurrentA           = 8.0;
    customTurn.overcurrentDurationMs = 500;
    */
   /*45 degree turn profile */
   /*
 TurnProfile customTurn = MID_TURN;
 
    customTurn.breakDistance         = 80.0;  // was 80 — start braking earlier
    customTurn.minSpeed              = 22.0;  // was 22 — crawl into target
    customTurn.maxSpeed              = 70.0;  // was 60 — less momentum to shed
    customTurn.exitTolerance         = 3.0;
    customTurn.timeout               = 5.0;
    customTurn.accelFactor           = 1.2;
    customTurn.slipThreshold         = 1.0;
    customTurn.decelStepPercent      = 5.0;   // was 10 — smoother decel curve
    customTurn.lockThreshold         = 1.0;
    customTurn.maxCurrentA           = 8.0;
*/
/*turn 90
 TurnProfile customTurn = MID_TURN;
 
    customTurn.breakDistance         = 74.0;  // was 80 — start braking earlier
    customTurn.minSpeed              = 24.2;  // was 22 — crawl into target
    customTurn.maxSpeed              = 80.0;  // was 60 — less momentum to shed
    customTurn.exitTolerance         = 3.0;
    customTurn.timeout               = 5.0;
    customTurn.accelFactor           = 1.2;
    customTurn.slipThreshold         = 1.0;
    customTurn.decelStepPercent      = 5.0;   // was 10 — smoother decel curve
    customTurn.lockThreshold         = 1.0;
    customTurn.maxCurrentA           = 8.0;
    */
//turn 180
/*
 TurnProfile customTurn = MID_TURN;
 
    customTurn.breakDistance         = 71.0;  // was 80 — start braking earlier
    customTurn.minSpeed              = 25;  // was 22 — crawl into target
    customTurn.maxSpeed              = 100.0;  // was 60 — less momentum to shed
    customTurn.exitTolerance         = 3.0;
    customTurn.timeout               = 5.0;
    customTurn.accelFactor           = 1.2;
    customTurn.slipThreshold         = 1.0;
    customTurn.decelStepPercent      = 5.0;   // was 10 — smoother decel curve
    customTurn.lockThreshold         = 1.0;
    customTurn.maxCurrentA           = 8.0;
    
*/
//turn 270

 TurnProfile customTurn = TURN_270;
 
    customTurn.breakDistance         = 58;  // was 80 — start braking earlier
    customTurn.minSpeed              = 25;  // was 22 — crawl into target
    customTurn.maxSpeed              = 100.0;  // was 60 — less momentum to shed
    customTurn.exitTolerance         = 3.0;
    customTurn.timeout               = 5.0;
    customTurn.accelFactor           = 1.3;
    customTurn.slipThreshold         = 1.0;
    customTurn.decelStepPercent      = 5.0;   // was 10 — smoother decel curve
    customTurn.lockThreshold         = 1.0;
    customTurn.maxCurrentA           = 8.0;
    

    turnRight(270, customTurn);
    

    // Stop colour sort before scoring — don't want it firing during score run.
  //  colorParams.isRunning = false;
  //  pros::delay(20);  // let task exit its 10ms loop cleanly

    // ── STEP 8: Brief settle delay, then score ────────────────────────────────
    pros::delay(500);

    scoreBlueStart(4000);  // right bay

    // ── STEP 9: GPS reset while scoring ──────────────────────────────────────
  //  requestGpsReset();
    //pros::delay(200);

    // ── STEP 10: Wait for scoring to finish ───────────────────────────────────
    pros::delay(4000);
    
}
/*

void longGoalAuto15sOg(bool isRedAlliance) {

    // ── Setup ─────────────────────────────────────────────────────────────────
    if (isRedAlliance) {
        setStartPosition(122.0, -60.0, 0.0);
    } else {
        setStartPosition(-122.0, -60.0, 180.0);
    }
    requestGpsReset();
    pros::delay(200);
    setAllianceRed(isRedAlliance);

    // Start colour sort for the whole match.
    ColorTaskParams colorParams = {
        true,
        isRedAlliance ? Color::BLUE : Color::RED,
        0
    };
    pros::Task colorSortTask(colorDetectionTask, &colorParams, "Color Sort");

    // ── Full-match loop (1 min 45 sec = 105 000 ms) ───────────────────────────
    const uint32_t MATCH_MS  = 105000;
    const uint32_t matchStart = pros::millis();

    auto timeUp = [&]() {
        return pros::millis() - matchStart >= MATCH_MS;
    };

    while (!timeUp()) {

        // ── Goal 1: (81.442, 117.722) ─────────────────────────────────────────
        intakeHopperStart(15000, -100.0, 0.0, true);
        RoutePath p1 = routePlan(globalX, globalY, 81.442, 117.722);
        if (p1.count > 0) routeExecute(p1);

        intakeHopperStop();
        if (timeUp()) break;

        pros::delay(300);
        if (isRedAlliance) scoreRedStart(4000); else scoreBlueStart(4000);
        requestGpsReset();
        pros::delay(200);
            pros::delay(3800);  // total ~4 s scoring wait

        if (timeUp()) break;

        // ── Goal 2: (82.528, -131.304) ────────────────────────────────────────
        intakeHopperStart(15000, -100.0, 0.0, true);
        RoutePath p2 = routePlan(globalX, globalY, 82.528, -131.304);
        if (p2.count > 0) routeExecute(p2);

        intakeHopperStop();
        if (timeUp()) break;

        pros::delay(300);
        if (isRedAlliance) scoreRedStart(4000); else scoreBlueStart(4000);
        requestGpsReset();
        pros::delay(200);
            pros::delay(3800);
    }

    // ── Cleanup ───────────────────────────────────────────────────────────────
    colorParams.isRunning = false;
    pros::delay(20);
    intakeHopperStop();
}

// ══════════════════════════════════════════════════════════════════════════════
// REPEAT NE / SE GOAL SWEEP — 1 min 45 sec
// Assumes odometry is already running (call after another routine, or add
// setup steps if calling standalone).
// Alternates A* between LONG_GOAL_NE and LONG_GOAL_SE for 105 seconds.
// ══════════════════════════════════════════════════════════════════════════════

*/


// ══════════════════════════════════════════════════════════════════════════════
// BLUE ALLIANCE — RIGHT SIDE AUTONOMOUS
//
// ── COORDINATE SYSTEM REFERENCE ──────────────────────────────────────────────
// VEX GPS origin (0,0) = field center. +X = East, +Y = North.
//
//   Quadrant II  (-X, +Y) | Quadrant I  (+X, +Y)
//   West-North              | East-North
//   ─────────────────────────────────────────────
//   Quadrant III (-X, -Y) | Quadrant IV (+X, -Y)
//   West-South              | East-South
//
// !! BLUE ALLIANCE RIGHT SIDE STARTS IN QUADRANT I: X POSITIVE, Y POSITIVE !!
// Blue alliance is on the EAST side of the field (+X).
// "Right side" from blue's perspective = NORTH of center (+Y).
// All starting coordinates are POSITIVE on both axes.
//
// ── HEADING REFERENCE ────────────────────────────────────────────────────────
// VEX heading: North = 0°, clockwise positive. -90° = 270° (same thing).
//   North =   0°  (+Y direction)
//   East  =  90°  (+X direction)
//   South = 180°  (-Y direction)
//   West  = -90°  (-X direction) ← robot faces opponent (red/west) at start
//
// Blue right-side robot faces WEST (toward red alliance) at start.
// Starting heading = -90° (West). Equivalent to 270°, -90° preferred for clarity.
//
// ── ISOLATION vs INTERACTION ─────────────────────────────────────────────────
// ISOLATION  (first 15s):  Robot must stay on east side (X > 0). Collect blocks
//                          on own side only. NO crossing center line (X must stay > 0).
// INTERACTION (next 1m45s): Full field. Sweep activates, scores at nearest goal.
//
// ── SWEEPANDSCORE PARAMS ─────────────────────────────────────────────────────
// sweepAndScore(targetPixelWidth, debounceMs, maxCubes, colourMode,
//               sweepSpeed, kpVisionHeading, kpDistToHead,
//               backupMs, scanTurnSpeed [, timeoutMs])
//   colourMode: 0=red only, 1=blue only, 2=largest cluster
//   For BLUE alliance: use colourMode=1 (chase blue blocks only)
//   maxCubes: once reached, sweepScoreNearest() navigates A* to nearest
//             long goal and scores automatically — built into sweepAndScore.
//
// ── CALL FROM autonomous() in main.cpp ───────────────────────────────────────
//   Isolation:   blueRightIsolation();
//   Interaction: blueRightInteraction();
// ══════════════════════════════════════════════════════════════════════════════

// ── ISOLATION PERIOD (15 seconds) ────────────────────────────────────────────
// Stay on east side (X > 0). Collect blue blocks on own side only.
// No sweep — sweep is interaction-only to avoid crossing center line.
void blueRightIsolation() {
    // East side (+X), slightly south of center (-Y), facing West (270°) toward opponent.
    setStartPosition(122.0, 36.0, 270.0);

    // Alliance must be set before any navigation or sweep calls.
    setAllianceRed(false);  // BLUE alliance

    requestGpsReset();
    pros::delay(200);

    // Colour sort task — blue blocks → right bay. Red blocks → left bay.
    static ColorTaskParams colorParams;
    colorParams.isRunning = true;
    colorParams.delayMs   = 0;
    pros::Task(colorDetectionTask, &colorParams, "colourSort");

    // Intake — start collecting blocks immediately from match start.
    intakeHopperStart(15000, -100.0, 0.0, true);
    startIntakeIndexer();

    // Profile for forward moves — tuned for loaded mid-distance legs
    StraightProfile customFwd = MID_FWD;
    customFwd.breakDistance     = 70.0;
    customFwd.minSpeed          = 13.0;
    customFwd.maxSpeed          = 70.0;
    customFwd.distanceTolerance = 1.0;
    customFwd.timeout           = 2.0;
    customFwd.brakeMode         = pros::E_MOTOR_BRAKE_BRAKE;
    customFwd.kp_heading        = 0.6;
    customFwd.ki_heading        = 0.0;
    customFwd.kd_heading        = 0.0;

    // Move 1 — drive west toward centre field, collecting blocks along south corridor
    forwardToPoint(37.0, -64.0, customFwd);

    // Backup 1 — reverse east to wall corridor, sweeping blocks back
    backwardToPoint(120.0, -64.0, MID_BWD);

    // Turn to face south toward south wall
    turnToPoint(120.0, -180.0, TURN_90);
    pros::delay(300);

    // Move 2 — drive south along east wall into corner, collecting everything
    forwardToPoint(120.0, -180.0, customFwd);

    // Backup 2 — reverse north to NE long goal approach point
    backwardToPoint(120.0, 120.0, MID_BWD);

    // A* to NE long goal — stays X > 0, safe during isolation
    navigateTo(LONG_GOAL_NE);
    sweepScore(false);  // full sequence — hood + indexers + right gate (blue)

    // Stop intake at end of isolation — interaction period relaunches it.
    stopIntakeIndexer();
    intakeHopperStop();

    // Kill colour sort task cleanly — interaction period relaunches after setAllianceRed().
    colorParams.isRunning = false;
    pros::delay(50);
}

// ── INTERACTION PERIOD (1 minute 45 seconds) ─────────────────────────────────
// Full field open. Sweep activates, vision chases blue blocks.
// When maxCubes reached, sweepScoreNearest() A* navigates to nearest
// available long goal and scores — this is built into sweepAndScore().
//
// PARK SAFETY: sweepAndScore runs for 85s (not 105s), leaving the last 20s
// for strategyPark() to navigate to the east wall park zone.
// Total: 85s sweep + 20s park = 105s interaction period.
void blueRightInteraction() {
    // Alliance MUST be set first — sweepAndScore and colour sort both read this.
    setAllianceRed(false);  // BLUE alliance

    // GPS reset at start of interaction — corrects any drift from isolation.
    requestGpsReset();
    pros::delay(200);

    // Colour sort — relaunch now that alliance is set. Isolation killed the
    // previous instance via colorParams.isRunning = false, so no duplicate.
    static ColorTaskParams colorParams;
    colorParams.isRunning = true;
    colorParams.delayMs   = 0;
    pros::Task(colorDetectionTask, &colorParams, "colourSort");

    // ── PHASE 1: NE quadrant sweep (~40s) ────────────────────────────────────
    // Blue blocks only. Scores internally at nearest long goal when maxCubes hit.
    sweepAndScore(
        80,       // targetPixelWidth
        500,      // debounceMs
        8,        // maxCubes
        1,        // colourMode: 1 = blue blocks only (BLUE alliance)
        25.0,     // sweepSpeed
        0.02,     // kpVisionHeading
        0.8,      // kpDistToHead
        500.0,    // backupMs
        20.0,     // scanTurnSpeed
        40000,    // timeoutMs — half of 85s budget minus 5s for crossing
        0.0       // xBoundary — field center X; stay in NE quadrant (X > 0)
    );

    // ── CROSS TO NW QUADRANT ─────────────────────────────────────────────────
    // Drive to center of NW quadrant (-91, 91). Intake stays running.
    // 2s timeout at 80% — gets there from anywhere in NE quadrant.
    StraightProfile crossProfile = MID_FWD;
    crossProfile.maxSpeed = 80.0;
    crossProfile.timeout  = 2.0;
    forwardToPoint(-91.0, 91.0, crossProfile);

    // ── PHASE 2: NW quadrant sweep (~40s) ────────────────────────────────────
    // Same params — blue blocks only, scores at nearest long goal when full.
    sweepAndScore(
        80,       // targetPixelWidth
        500,      // debounceMs
        8,        // maxCubes
        1,        // colourMode: 1 = blue blocks only (BLUE alliance)
        25.0,     // sweepSpeed
        0.02,     // kpVisionHeading
        0.8,      // kpDistToHead
        500.0,    // backupMs
        20.0,     // scanTurnSpeed
        40000,    // timeoutMs
        0.0       // xBoundary — field center X; stay in NW quadrant (X < 0)
    );

    // ── STOP & PARK ──────────────────────────────────────────────────────────
    leftDrive.move(0);
    rightDrive.move(0);
    intakeHopperStop();
    stopIntakeIndexer();

    // strategyPark() reads g_isRedAlliance (set to false above) and navigates
    // to PARK_OPPONENT (east wall) — correct zone for blue alliance.
    strategyPark();
}

// ══════════════════════════════════════════════════════════════════════════════
// RED RIGHT — Slot 3
// Starts west side (X negative), north of center (+Y), facing East (90°).
//   Isolation:   redRightIsolation();
//   Interaction: redRightInteraction();
// ══════════════════════════════════════════════════════════════════════════════

// ── ISOLATION PERIOD (15 seconds) ────────────────────────────────────────────
// Stay on west side (X < 0). Collect red blocks on own side only.
void redRightIsolation() {
    // West side (-X), slightly south of center (-Y), facing East (90°) toward opponent.
    setStartPosition(-122.0, -36.0, 90.0);
    requestGpsReset();
    pros::delay(200);

    // Alliance must be set before any navigation or sweep calls.
    setAllianceRed(true);  // RED alliance

    // Colour sort task — red blocks → left bay. Blue blocks → right bay.
    static ColorTaskParams colorParams;
    colorParams.isRunning = true;
    colorParams.delayMs   = 0;
    pros::Task(colorDetectionTask, &colorParams, "colourSort");

    // Intake — start collecting blocks immediately from match start.
    intakeHopperStart(15000, -100.0, 0.0, true);
    startIntakeIndexer();

    // Profile for forward moves — tuned for loaded mid-distance legs
    StraightProfile customFwd = MID_FWD;
    customFwd.breakDistance     = 70.0;
    customFwd.minSpeed          = 13.0;
    customFwd.maxSpeed          = 70.0;
    customFwd.distanceTolerance = 1.0;
    customFwd.timeout           = 2.0;
    customFwd.brakeMode         = pros::E_MOTOR_BRAKE_BRAKE;
    customFwd.kp_heading        = 0.6;
    customFwd.ki_heading        = 0.0;
    customFwd.kd_heading        = 0.0;

    // Move 1 — drive east toward centre field, collecting blocks along south corridor
    forwardToPoint(-37.0, -64.0, customFwd);

    // Backup 1 — reverse west to wall corridor, sweeping blocks back
    backwardToPoint(-120.0, -64.0, MID_BWD);

    // Turn to face south toward south wall
    turnToPoint(-120.0, -180.0, TURN_90);
    pros::delay(300);

    // Move 2 — drive south along west wall into corner, collecting everything
    forwardToPoint(-120.0, -180.0, customFwd);

    // Backup 2 — reverse north to SW long goal approach point
    backwardToPoint(-120.0, -120.0, MID_BWD);

    // A* to SW long goal — stays X < 0, safe during isolation
    navigateTo(LONG_GOAL_SW);
    sweepScore(true);  // full sequence — hood + indexers + left gate (red)

    // Stop intake at end of isolation — interaction period relaunches it.
    stopIntakeIndexer();
    intakeHopperStop();

    // Kill colour sort task cleanly — interaction period relaunches after setAllianceRed().
    colorParams.isRunning = false;
    pros::delay(50);
}

// ── INTERACTION PERIOD (1 minute 45 seconds) ─────────────────────────────────
void redRightInteraction() {
    // Alliance MUST be set first — sweepAndScore and colour sort both read this.
    setAllianceRed(true);  // RED alliance

    // GPS reset at start of interaction — corrects any drift from isolation.
    requestGpsReset();
    pros::delay(200);

    // Colour sort — relaunch now that alliance is set.
    static ColorTaskParams colorParams;
    colorParams.isRunning = true;
    colorParams.delayMs   = 0;
    pros::Task(colorDetectionTask, &colorParams, "colourSort");

    // ── PHASE 1: SW quadrant sweep (~40s) ────────────────────────────────────
    // Red blocks only. Scores internally at nearest long goal when maxCubes hit.
    sweepAndScore(
        80,       // targetPixelWidth
        500,      // debounceMs
        8,        // maxCubes
        0,        // colourMode: 0 = red blocks only (RED alliance)
        25.0,     // sweepSpeed
        0.02,     // kpVisionHeading
        0.8,      // kpDistToHead
        500.0,    // backupMs
        20.0,     // scanTurnSpeed
        40000,    // timeoutMs — half of 85s budget minus 5s for crossing
        0.0       // xBoundary — field center X; stay in SW quadrant (X < 0)
    );

    // ── CROSS TO SE QUADRANT ─────────────────────────────────────────────────
    // Drive to center of SE quadrant (91, -91). Intake stays running.
    // 2s timeout at 80% — gets there from anywhere in SW quadrant.
    StraightProfile crossProfile = MID_FWD;
    crossProfile.maxSpeed = 80.0;
    crossProfile.timeout  = 2.0;
    forwardToPoint(91.0, -91.0, crossProfile);

    // ── PHASE 2: SE quadrant sweep (~40s) ────────────────────────────────────
    // Same params — red blocks only, scores at nearest long goal when full.
    sweepAndScore(
        80,       // targetPixelWidth
        500,      // debounceMs
        8,        // maxCubes
        0,        // colourMode: 0 = red blocks only (RED alliance)
        25.0,     // sweepSpeed
        0.02,     // kpVisionHeading
        0.8,      // kpDistToHead
        500.0,    // backupMs
        20.0,     // scanTurnSpeed
        40000,    // timeoutMs
        0.0       // xBoundary — field center X; stay in SE quadrant (X > 0)
    );

    // ── STOP & PARK ──────────────────────────────────────────────────────────
    leftDrive.move(0);
    rightDrive.move(0);
    intakeHopperStop();
    stopIntakeIndexer();

    // strategyPark() reads g_isRedAlliance (set to true above) and navigates
    // to PARK_ALLIANCE (west wall) — correct zone for red alliance.
    strategyPark();
}

// ══════════════════════════════════════════════════════════════════════════════
// testSweepScore — fires all scoring components simultaneously as a background task.
//
// Reads alliance from getIsRedAlliance() — call setAllianceRed() before this.
// Main thread returns immediately; score sequence runs in background:
//   hood extends + all motors spin + correct gate opens → 3s → everything resets
//
// Call from Slot 6 (dev/test only).
// ══════════════════════════════════════════════════════════════════════════════
void testSweepScore(bool isRed) {
    setAllianceRed(isRed);  // ensure global is set before task reads it

    pros::screen::erase();
    pros::screen::print(pros::E_TEXT_MEDIUM, 0, "testSweepScore — %s — FIRING",
                        isRed ? "RED (left gate)" : "BLUE (right gate)");

    pros::Task([]{
        pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Scoring...");
        sweepScore(getIsRedAlliance());
        pros::screen::print(pros::E_TEXT_MEDIUM, 2, "DONE");
    }, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "testSweepScore");
}

// ══════════════════════════════════════════════════════════════════════════════
// testSweepIntake — runs intake motors + upper indexer exactly as during sweep.
// Hood stays down. Runs for 5 seconds then stops.
// Use to verify intake direction before a match.
// ══════════════════════════════════════════════════════════════════════════════
void testSweepIntake() {
    pros::screen::erase();
    pros::screen::print(pros::E_TEXT_MEDIUM, 0, "testSweepIntake — running 5s");

    intakeHopperStart(5000, -100.0, 0.0, true);  // intake motors
    startIntakeIndexer();                         // upper indexer -12000, hood down

    pros::delay(5000);

    stopIntakeIndexer();
    intakeHopperStop();

    pros::screen::print(pros::E_TEXT_MEDIUM, 1, "DONE");
}
// ══════════════════════════════════════════════════════════════════════════════
// skillsPark — Skills run: park on blue (east) wall.
//
// Starts facing East (90°). Sets alliance to blue so strategyPark() navigates
// to PARK_OPPONENT (east wall, X = +161cm).
// Slot 8 — Skills.
// ══════════════════════════════════════════════════════════════════════════════
void skillsPark() {
    // Start center field facing east
    setStartPosition(0.0, 0.0, 90.0);

    // Blue alliance → strategyPark() goes to east wall (PARK_OPPONENT)
    setAllianceRed(false);

    requestGpsReset();
    pros::delay(200);

    // Navigate to east wall park zone
    strategyPark();
}