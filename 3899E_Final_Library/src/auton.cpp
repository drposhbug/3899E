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
}


// Spins each drive and mechanism motor for 500ms to verify hardware.
void systemTest() {
    pros::screen::erase();
    pros::screen::print(pros::E_TEXT_MEDIUM, 1, "System Test");

    InertialSensor.reset(true);

    pros::Motor* allMotors[] = {
        &LeftMotor1, &LeftMotor2, &LeftMotor3,
        &RightMotor1, &RightMotor2, &RightMotor3,
        &intakeMotor, &colorSortMotor
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

// ══════════════════════════════════════════════════════════════════════════════
// FIELD TARGETS TEST
// ══════════════════════════════════════════════════════════════════════════════
void fieldTargetsTest() {
    for (int i = 0; i < 8; i++) pros::lcd::clear_line(i);
    pros::screen::erase();

    setStartPosition(60.96, 60.96, 0.0);
    startOdometryTask();

    // GPS reset before routing — wait up to 200ms for task to finish
    // so A* starts from a corrected position rather than setStartPosition estimate
    requestGpsReset();
    pros::delay(200);  // 8 samples × 15ms = 120ms; 200ms gives it room to complete
    //APPLY_FIELD_ROTATION();

    pros::lcd::print(0, "START X:%.0f Y:%.0f H:%.0f",
                     globalX, globalY, getContinuousStandardHeading());

    NavResult 
    result = navigateTo(LONG_GOAL_SE);

    const char* resultStr =
        result == NavResult::SUCCESS       ? "SUCCESS"  :
        result == NavResult::BLIND_CONTACT ? "CONTACT"  :
        result == NavResult::BLIND_TIMEOUT ? "TIMEOUT"  :
        result == NavResult::VISION_LOST   ? "VIS LOST" : "BLOCKED";

    pros::screen::print(pros::E_TEXT_MEDIUM, 1, "RESULT: %s", resultStr);

    Controller.rumble(result == NavResult::SUCCESS ||
                      result == NavResult::BLIND_CONTACT ? "." : "---");

    // Hold screen forever so RESULT and END position stay visible after run
    // while (true) {
    //     pros::delay(100);
    // }

    turnRight(90.0, DEFAULT_TURN);

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

void navTest() {
    startOdometryTask();
    setStartPosition(0.0, 0.0, 0.0);
    pros::delay(200);

    StraightProfile driveProfile = DEFAULT_STRAIGHT;
    driveProfile.breakDistance          = 90.0;   // cm before target to begin decel
    driveProfile.minSpeed               = 20.0;   // % minimum approach speed
    driveProfile.maxSpeed               = 80.0;   // % peak cruise speed
    driveProfile.distanceTolerance      = 1.0;    // cm exit bubble
    driveProfile.timeout                = 5.0;    // seconds 5 sec default
    driveProfile.brakeMode              = pros::E_MOTOR_BRAKE_BRAKE;
    driveProfile.kp_heading             = 2.0;    // heading PID proportional
    driveProfile.ki_heading             = 0.0;    // heading PID integral
    driveProfile.kd_heading             = 5.0;    // heading PID derivative
    driveProfile.accelHeadingScaling    = 0.2;    // correction weight during accel
    driveProfile.decelHeadingScaling    = 0.1;    // correction weight during decel
    driveProfile.approachHeadingScaling = 0.1;    // correction weight during approach
    driveProfile.headingLockDistance    = 3.0;    // cm — freeze heading near target
    driveProfile.launchVoltage          = 3.0;    // V — initial kick voltage
    driveProfile.accelFactor            = 1.2;    // traction ramp multiplier
    driveProfile.slipThreshold          = 0.3;   // RPM slip before traction cuts in
    driveProfile.decelStepPercent       = 0.30;   // ABS voltage reduction per step
    driveProfile.lockThreshold          = 0.3;   // wheel lockup ratio
    driveProfile.maxCurrentA            = 4.0;   // amps — wall stall trip threshold
    driveProfile.overcurrentDurationMs  = 300; // ms — how long before breaker fires

    // TurnProfile turnProfile = DEFAULT_TURN;
    // turnProfile.breakDistance  = 5.0;    // degrees before target to begin decel
    // turnProfile.minSpeed       = 10.0;   // % minimum approach speed
    // turnProfile.maxSpeed       = 20.0;  // % peak turn speed
    // turnProfile.exitTolerance  = 3;    // degrees — stop when within this
    // turnProfile.timeout        = 3.0;    // seconds — release if stuck

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
    // driveForward(150.0, 0.0, LONG_FWD);
    // turnOdometry(90, DEFAULT_TURN); //selectTurnProfile(180)
    turnRight(90, DEFAULT_TURN);
    pros::delay(500);
    // setStartPosition(0.0, 150.0, 0.0);
    // driveBackward(150.0, 0.0, driveProfile);
    // pros::delay(500);
    // setStartPosition(0,0,0);
    // driveForward(50,0,driveProfile);
    // pros::delay(500);
    // driveBackward(50,0,driveProfile);
}


void visionTest() {
    startOdometryTask();
    setStartPosition(0.0, 0.0, 0.0);
    pros::delay(200);

    // Signature: aiVision_redCube (from robot_config.h)
    // TODO: tune targetPixelWidth — 80px ≈ ~30 cm from a 3" block
    VisionProfile vp = DEFAULT_VISION;
    vp.drive.breakDistance          = 90.0;   // cm before target to begin decel
    vp.drive.minSpeed               = 20.0;   // % minimum approach speed
    vp.drive.maxSpeed               = 80.0;   // % peak cruise speed
    vp.drive.distanceTolerance      = 1.0;    // cm exit bubble
    vp.drive.timeout                = 5.0;    // seconds
    vp.drive.brakeMode              = pros::E_MOTOR_BRAKE_BRAKE;
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

    intakeMotor.move(127);
    colorSortMotor.move(127);
    // ── Stage 1: visionDriveForward — open-loop encoder distance + vision steering ──
    // Robot drives 150 cm forward; vision steers once target acquired.
    // visionDriveForward(aiVision_blueCube, 40, 150.0, 0.0, vp);

    // ── Stage 2: visionForwardToPoint — closed-loop odometry + vision steering ──
    // Robot drives to (0, 150) using live odometry; vision corrects heading.
    // visionForwardToPoint(aiVision_redCube, 60, 0.0, 150.0, vp);

    // ── Stage 3: visionBackwardToPoint — closed-loop backward + vision steering ──
    // Robot drives backward to (0, 0) using live odometry; vision corrects heading.
    // visionBackwardToPoint(aiVision_redCube, 80, 0.0, 0.0, vp);

    pros::lcd::print(1, "which ai sensor:" )
    // ── Stage 4: visionOnly — pure vision approach, no odometry position updates ──
    // Robot drives toward target until pixel width >= 80, encoder safety, or timeout.
    visionOnlyOG(aiVision_redCube, 40, 200.0, vp);
}

void redColorSortTest() {
    opticalSensor.set_led_pwm(100);
    intakeMotor.move(100);
    colorSortMotor.move(127);
    while (true) {
        if (opticalSensor.get_hue() < 60) { // blue is > 150 to 230
            colorSortMotor.move(-100);
            intakeMotor.move(127);
            pros::delay(20);
            intakeMotor.move(100);
            pros::delay(10);
        } else {
            intakeMotor.move(100);
            colorSortMotor.move(127);
        }
        pros::delay(10);
    }
}

void blueColorSortTest() {
    opticalSensor.set_led_pwm(100);
    intakeMotor.move(100);
    colorSortMotor.move(127);
    while (true) {
        if (opticalSensor.get_hue() < 240 && opticalSensor.get_hue() > 140) { // blue is > 150 to 230
            colorSortMotor.move(-100);
            intakeMotor.move(127);
            pros::delay(20);
            intakeMotor.move(100);
            pros::delay(10);
        } else {
            intakeMotor.move(100);
            colorSortMotor.move(127);
        }
        pros::delay(10);
    }
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

        //     switch (autonMode) {
        //         case 0:  setAllianceRed(true);  runAIMatchRoute();      break;
        //         case 1:  setAllianceRed(false); runAIMatchRoute();      break;
        //         case 2:  routeTest();                                   break;
        //         case 3:  routeGridTest();                               break;
        //         case 4:  systemTest();                                  break;
        //         case 5:  coordinateFinder();  break;
        //         case 6:  visionTest();        break;
        //     }
        //     break;
        // }

//         pros::delay(20);
//     }
}

void autonLeft15(){
    startOdometryTask();
    setStartPosition(-31.123,-119.262,0);
    intakeMotor.move(127);
    colorSortMotor.move(127);
    StraightProfile driveProfile = DEFAULT_STRAIGHT;
    driveProfile.breakDistance          = 30.0;   // cm before target to begin decel
    driveProfile.minSpeed               = 20.0;   // % minimum approach speed
    driveProfile.maxSpeed               = 80.0;   // % peak cruise speed
    driveProfile.distanceTolerance      = 1.0;    // cm exit bubble
    driveProfile.timeout                = 5.0;    // seconds 5 sec default
    driveProfile.brakeMode              = pros::E_MOTOR_BRAKE_BRAKE;
    driveProfile.kp_heading             = 2.0;    // heading PID proportional
    driveProfile.ki_heading             = 0.0;    // heading PID integral
    driveProfile.kd_heading             = 5.0;    // heading PID derivative
    driveProfile.accelHeadingScaling    = 0.2;    // correction weight during accel
    driveProfile.decelHeadingScaling    = 0.1;    // correction weight during decel
    driveProfile.approachHeadingScaling = 0.1;    // correction weight during approach
    driveProfile.headingLockDistance    = 3.0;    // cm — freeze heading near target
    driveProfile.launchVoltage          = 3.0;    // V — initial kick voltage
    driveProfile.accelFactor            = 1.2;    // traction ramp multiplier
    driveProfile.slipThreshold          = 0.3;   // RPM slip before traction cuts in
    driveProfile.decelStepPercent       = 0.30;   // ABS voltage reduction per step
    driveProfile.lockThreshold          = 0.3;   // wheel lockup ratio
    driveProfile.maxCurrentA            = 4.0;   // amps — wall stall trip threshold
    driveProfile.overcurrentDurationMs  = 300; // ms — how long before breaker fires



    driveForward(60, 0.0, driveProfile);
    TurnProfile turnProfile = DEFAULT_TURN;
    turnProfile.breakDistance  = 10.0;    // degrees before target to begin decel
    turnProfile.minSpeed       = 15.0;   // % minimum approach speed
    turnProfile.maxSpeed       = 30.0;  // % peak turn speed
    turnProfile.exitTolerance  = 3;    // degrees — stop when within this
    turnProfile.timeout        = 3.0;    // seconds — release if stuck
    turnLeft(-60.0, turnProfile);
    // movement 2
    StraightProfile driveProfile1 = DEFAULT_STRAIGHT;
    driveProfile1.breakDistance          = 10.0;   // cm before target to begin decel
    driveProfile1.minSpeed               = 20.0;   // % minimum approach speed
    driveProfile1.maxSpeed               = 90.0;   // % peak cruise speed
    driveProfile1.distanceTolerance      = 1.0;    // cm exit bubble
    driveProfile1.timeout                = 5.0;    // seconds 5 sec default
    driveProfile1.brakeMode              = pros::E_MOTOR_BRAKE_BRAKE;
    driveProfile1.kp_heading             = 2.0;    // heading PID proportional
    driveProfile1.ki_heading             = 0.0;    // heading PID integral
    driveProfile1.kd_heading             = 5.0;    // heading PID derivative
    driveProfile1.accelHeadingScaling    = 0.2;    // correction weight during accel
    driveProfile1.decelHeadingScaling    = 0.1;    // correction weight during decel
    driveProfile1.approachHeadingScaling = 0.1;    // correction weight during approach
    driveProfile1.headingLockDistance    = 3.0;    // cm — freeze heading near target
    driveProfile1.launchVoltage          = 3.0;    // V — initial kick voltage
    driveProfile1.accelFactor            = 1.2;    // traction ramp multiplier
    driveProfile1.slipThreshold          = 0.3;   // RPM slip before traction cuts in
    driveProfile1.decelStepPercent       = 0.30;   // ABS voltage reduction per step
    driveProfile1.lockThreshold          = 0.3;   // wheel lockup ratio
    driveProfile1.maxCurrentA            = 4.0;   // amps — wall stall trip threshold
    driveProfile1.overcurrentDurationMs  = 300; // ms — how long before breaker fires
    forwardToPoint(-60.192,-54.246,driveProfile1);
    StraightProfile driveProfile2 = DEFAULT_STRAIGHT;
    driveProfile2.breakDistance          = 2.0;   // cm before target to begin decel
    driveProfile2.minSpeed               = 20.0;   // % minimum approach speed
    driveProfile2.maxSpeed               = 90.0;   // % peak cruise speed
    driveProfile2.distanceTolerance      = 1.0;    // cm exit bubble
    driveProfile2.timeout                = 5.0;    // seconds 5 sec default
    driveProfile2.brakeMode              = pros::E_MOTOR_BRAKE_BRAKE;
    driveProfile2.kp_heading             = 2.0;    // heading PID proportional
    driveProfile2.ki_heading             = 0.0;    // heading PID integral
    driveProfile2.kd_heading             = 5.0;    // heading PID derivative
    driveProfile2.accelHeadingScaling    = 0.2;    // correction weight during accel
    driveProfile2.decelHeadingScaling    = 0.1;    // correction weight during decel
    driveProfile2.approachHeadingScaling = 0.1;    // correction weight during approach
    driveProfile2.headingLockDistance    = 3.0;    // cm — freeze heading near target
    driveProfile2.launchVoltage          = 3.0;    // V — initial kick voltage
    driveProfile2.accelFactor            = 1.2;    // traction ramp multiplier
    driveProfile2.slipThreshold          = 0.3;   // RPM slip before traction cuts in
    driveProfile2.decelStepPercent       = 0.30;   // ABS voltage reduction per step
    driveProfile2.lockThreshold          = 0.3;   // wheel lockup ratio
    driveProfile2.maxCurrentA            = 4.0;   // amps — wall stall trip threshold
    driveProfile2.overcurrentDurationMs  = 300; // ms — how long before breaker fires
    forwardToPoint(-55.192,-53.246,driveProfile2);



    TurnProfile turnProfile1 = DEFAULT_TURN;
    turnProfile1.breakDistance  = 5.0;    // degrees before target to begin decel
    turnProfile1.minSpeed       = 15.0;   // % minimum approach speed
    turnProfile1.maxSpeed       = 30.0;  // % peak turn speed
    turnProfile1.exitTolerance  = 3;    // degrees — stop when within this
    turnProfile1.timeout        = 1.0;    // seconds — release if stuck
    turnOdometry(-40,turnProfile1);
    matchloader.set_value(true);

    StraightProfile driveProfile3 = DEFAULT_STRAIGHT;
    driveProfile3.breakDistance          = 10.0;   // cm before target to begin decel
    driveProfile3.minSpeed               = 15.0;   // % minimum approach speed
    driveProfile3.maxSpeed               = 70.0;   // % peak cruise speed
    driveProfile3.distanceTolerance      = 1.0;    // cm exit bubble
    driveProfile3.timeout                = 3.0;    // seconds 5 sec default
    driveProfile3.brakeMode              = pros::E_MOTOR_BRAKE_BRAKE;
    driveProfile3.kp_heading             = 2.0;    // heading PID proportional
    driveProfile3.ki_heading             = 0.0;    // heading PID integral
    driveProfile3.kd_heading             = 5.0;    // heading PID derivative
    driveProfile3.accelHeadingScaling    = 0.2;    // correction weight during accel
    driveProfile3.decelHeadingScaling    = 0.1;    // correction weight during decel
    driveProfile3.approachHeadingScaling = 0.1;    // correction weight during approach
    driveProfile3.headingLockDistance    = 3.0;    // cm — freeze heading near target
    driveProfile3.launchVoltage          = 3.0;    // V — initial kick voltage
    driveProfile3.accelFactor            = 1.2;    // traction ramp multiplier
    driveProfile3.slipThreshold          = 0.3;   // RPM slip before traction cuts in
    driveProfile3.decelStepPercent       = 0.30;   // ABS voltage reduction per step
    driveProfile3.lockThreshold          = 0.3;   // wheel lockup ratio
    driveProfile3.maxCurrentA            = 4.0;   // amps — wall stall trip threshold
    driveProfile3.overcurrentDurationMs  = 300; // ms — how long before breaker fires

    driveBackward(30.0,0.0,driveProfile3);
    TurnProfile turnProfile2 = DEFAULT_TURN;
    turnProfile2.breakDistance  = 10.0;    // degrees before target to begin decel
    turnProfile2.minSpeed       = 15.0;   // % minimum approach speed
    turnProfile2.maxSpeed       = 30.0;  // % peak turn speed
    turnProfile2.exitTolerance  = 3;    // degrees — stop when within this
    turnProfile2.timeout        = 3.0;    // seconds — release if stuck
}

void skills() {
    //flliped y values
    matchloader.set_value(false);
    setStartPosition(-162, 0, 180);
    forwardToPoint(-148, 117,DEFAULT_STRAIGHT);
    turnToPoint(-188, 117, DEFAULT_TURN);
    intakeMotor.move(127);
    matchloader.set_value(true);
    forwardToPoint(-150, 120, DEFAULT_STRAIGHT);
    pros::delay(1000);
    intakeMotor.move(0);
    backwardToPoint(-75, 120, DEFAULT_STRAIGHT);
    matchloader.set_value(false);
    scoreFlap.set_value(true);
    lever.move(127);
    pros::delay(400);
    lever.move(-127);
    pros::delay(400);
    forwardToPoint(-90, 90, DEFAULT_STRAIGHT);
    lever.move(0);
    scoreFlap.set_value(false);
    backwardToPoint(-120, 90, DEFAULT_STRAIGHT);
    matchloader.set_value(true);
    intakeMotor.move(127);
    forwardToPoint(-50, 119, DEFAULT_STRAIGHT);
    pros::delay(1000);
    intakeMotor.move(0);
    matchloader.set_value(false);
    backwardToPoint(-120, 90, DEFAULT_STRAIGHT);
    // quadrant change, idk whats after, trying flipping both now
    backwardToPoint(120, 90, DEFAULT_STRAIGHT);
    forwardToPoint(-90, -120, DEFAULT_STRAIGHT);
    forwardToPoint(-80, -120, DEFAULT_STRAIGHT);
    turnToPoint(0, 120, DEFAULT_TURN);
    scoreFlap.set_value(true);
    lever.move(127);
    pros::delay(400);
    lever.move(-127);
    pros::delay(400);
    lever.move(0);
    scoreFlap.set_value(false); 
    matchloader.set_value(true);
    intakeMotor.move(127);
    forwardToPoint(-150, -120, DEFAULT_STRAIGHT);
    pros::delay(1000);
    intakeMotor.move(0);
    scoreFlap.set_value(true);
    backwardToPoint(-75, -120, DEFAULT_STRAIGHT);
    matchloader.set_value(false);
    lever.move(127);
    pros::delay(400);
    lever.move(-127);
    pros::delay(400);
    lever.move(0);
    forwardToPoint(-90, -90, DEFAULT_STRAIGHT);
    scoreFlap.set_value(false);
    turnToPoint(0, 90, DEFAULT_TURN);
    //another quadrant change idk what to do
    forwardToPoint(-120, 100, DEFAULT_STRAIGHT);
    matchloader.set_value(true);
    intakeMotor.move(127);
    forwardToPoint(-150, 120, DEFAULT_STRAIGHT);
    pros::delay(1000);
    intakeMotor.move(0);
    scoreFlap.set_value(true);
    backwardToPoint(-75, 120, DEFAULT_STRAIGHT);
    matchloader.set_value(false);
    lever.move(127);
    pros::delay(400);
    lever.move(-127);
    pros::delay(400);   
    lever.move(0);
    forwardToPoint(-90, 90, DEFAULT_STRAIGHT);
    backwardToPoint(162, 0, DEFAULT_STRAIGHT);
}   