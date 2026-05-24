// #include "main.h"
// #include "pid.h"
// #include "utils.h"
// #include "robot_config.h"
// #include "vision_follow.h"
// #include "navigation.h"
// #include "odometry.h"
// #include "autontasks.h"
// #include <cmath>
// // NOTE: <functional> excluded — it pulls in <format> which is incompatible
// //       with the PROS ARM newlib + GCC 14.3.1 toolchain combination.

// // ── Vision tracking task state (defined in vision_follow.cpp) ─────────────────
// extern std::atomic<double> visionHorizontalNormalizedOffset;
// extern std::atomic<int>    visionCurrentObjectWidth;
// extern std::atomic<bool>   visionTargetTracked;
// extern const pros::vision_signature_s_t* currentVisionSignature;
// extern int currentMinObjectWidth;
// extern std::atomic<bool> visionTaskShouldRun;

// // ══════════════════════════════════════════════════════════════════════════════
// // DEVELOPMENT / DIAGNOSTIC ROUTINES
// // ══════════════════════════════════════════════════════════════════════════════

// // ── Navigation test sequence ──────────────────────────────────────────────────
// // Runs each navigation primitive in order with a pause between steps.
// // Brain screen shows which test is active so you can observe each move.
// // Place the robot in open space with ~1.5 m clearance in all directions.
// void test()
// {
//     pros::screen::erase();
//     startOdometryTask();
//     setStartPosition(0.0, 0.0, 0.0);
//     pros::delay(500);

//     // Drive forward East 60cm
//     // forwardToPoint(targetX, targetY, breakDistance, minSpeed, distanceTolerance,
//     //                kp_heading, ki_heading, kd_heading,
//     //                accelHeadingScaling, decelHeadingScaling, approachHeadingScaling, maxSpeed)
//     pros::screen::print(pros::E_TEXT_MEDIUM, 1, "forwardToPoint (60,0)");
//     forwardToPoint(60, 0, 10.0, 17.0, 5.0, 0.8, 0.0, 0.0, 0.1, 0.1, 0.3, 20.0);
//     pros::delay(2000);

//     // Drive backward back to origin
//     // backwardToPoint(targetX, targetY, breakDistance, minSpeed, distanceTolerance,
//     //                 kp_heading, ki_heading, kd_heading,
//     //                 accelHeadingScaling, decelHeadingScaling, approachHeadingScaling, maxSpeed)
//     pros::screen::print(pros::E_TEXT_MEDIUM, 1, "backwardToPoint (0,0)");
//     backwardToPoint(0, 0, 10.0, 17.0, 5.0, 0.8, 0.0, 0.0, 0.1, 0.1, 0.3, 20.0);
//     pros::delay(2000);

//     pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Done!");
//     Controller.rumble(".");
// }

// // Displays live vision sensor readings for red and blue cubes on the brain screen.
// // Press screen to exit.
// void visionSensorTest()
// {
//     pros::screen::erase();
//     pros::screen::print(pros::E_TEXT_MEDIUM, 1, "VISION TEST MODE");
//     pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Use Full Frame (0-320, 0-240)");

//     while (true) {
//         // Query red cube signature.
//         pros::vision_object_s_t redObj  = aiVision.get_by_sig(0, aiVision_redCube.id);
//         bool redFound  = (redObj.signature == (unsigned)aiVision_redCube.id  && redObj.width > 0);
//         int  redX = redFound  ? redObj.x_middle_coord : 0;
//         int  redW = redFound  ? redObj.width          : 0;

//         // Query blue cube signature.
//         pros::vision_object_s_t blueObj = aiVision.get_by_sig(0, aiVision_blueCube.id);
//         bool blueFound = (blueObj.signature == (unsigned)aiVision_blueCube.id && blueObj.width > 0);
//         int  blueX = blueFound ? blueObj.x_middle_coord : 0;
//         int  blueW = blueFound ? blueObj.width          : 0;

//         if (redFound) {
//             pros::screen::print(pros::E_TEXT_MEDIUM, 3, "RED : FOUND! X=%3d W=%3d  ", redX, redW);
//         } else {
//             pros::screen::print(pros::E_TEXT_MEDIUM, 3, "RED : SEARCHING...        ");
//         }

//         if (blueFound) {
//             pros::screen::print(pros::E_TEXT_MEDIUM, 4, "BLUE: FOUND! X=%3d W=%3d  ", blueX, blueW);
//         } else {
//             pros::screen::print(pros::E_TEXT_MEDIUM, 4, "BLUE: SEARCHING...        ");
//         }

//         pros::screen::print(pros::E_TEXT_MEDIUM, 6, "If 0 found: Check IDs in Utility");
//         pros::delay(100);
//     }
// }

// // Continuously prints the robot's odometry coordinates to the brain screen.
// void CoordinateFinderTask()
// {
//     setStartPosition(0, 36.2, 0.0);
//     while (true) {
//         startCoordinateFinder();
//         pros::delay(500);
//     }
// }

// // Launches the vision tracking task and displays live offset/width data.
// // Tap the brain screen to exit.
// void testVisionOnly()
// {
//     // Configure and start the vision tracking background task.
//     currentVisionSignature = &aiVision_orangeGoal;
//     currentMinObjectWidth  = 10;
//     visionTargetTracked    = false;
//     visionTaskShouldRun    = true;

//     pros::Task(visionTrackingTask, nullptr, "visionTracking");
//     pros::delay(50);

//     pros::screen::erase();

//     while (pros::screen::touch_status().touch_status == pros::E_TOUCH_RELEASED) {
//         pros::screen::erase();
//         pros::screen::print(pros::E_TEXT_MEDIUM, 1, "=== VISION TEST ===");
//         pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Press to exit");
//         pros::screen::print(pros::E_TEXT_MEDIUM, 4, "Tracked: %s",  visionTargetTracked.load() ? "**YES**" : "NO");
//         pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Width: %d px", visionCurrentObjectWidth.load());
//         pros::screen::print(pros::E_TEXT_MEDIUM, 6, "Offset: %.3f", visionHorizontalNormalizedOffset.load());
//         pros::screen::print(pros::E_TEXT_MEDIUM, 8, "Raw Count: %d", aiVision.get_object_count());
//         pros::screen::print(pros::E_TEXT_MEDIUM, 9, "Min Filter: %d", currentMinObjectWidth);
//         pros::delay(200);
//     }

//     // Signal task to stop and allow it to exit cleanly.
//     visionTaskShouldRun = false;
//     pros::delay(50);
//     pros::screen::erase();
// }

// // Directly queries the vision sensor (no tracking task) and displays results.
// // Tap the brain screen to exit.
// void testVisionDirect()
// {
//     pros::screen::erase();
//     pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Direct Vision Test");
//     pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Press screen to exit");

//     while (pros::screen::touch_status().touch_status == pros::E_TOUCH_RELEASED) {
//         pros::vision_object_s_t obj = aiVision.get_by_sig(0, aiVision_orangeGoal.id);
//         bool found = (obj.signature == (unsigned)aiVision_orangeGoal.id && obj.width > 0);

//         pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Object Count: %d   ", aiVision.get_object_count());

//         if (found) {
//             pros::screen::print(pros::E_TEXT_MEDIUM, 4, "X: %d  Y: %d        ", obj.x_middle_coord, obj.y_middle_coord);
//             pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Width: %d  Height: %d   ", obj.width, obj.height);
//         } else {
//             pros::screen::print(pros::E_TEXT_MEDIUM, 4, "NO OBJECTS DETECTED     ");
//         }

//         pros::delay(100);
//     }

//     pros::screen::erase();
// }

// // Minimal vision query — shows object count and dimensions without tracking task.
// // Tap the brain screen to exit.
// void testVisionBasic()
// {
//     pros::screen::erase();
//     pros::screen::print(pros::E_TEXT_MEDIUM, 1, "BASIC VISION TEST");
//     pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Press to exit");

//     while (pros::screen::touch_status().touch_status == pros::E_TOUCH_RELEASED) {
//         pros::vision_object_s_t obj = aiVision.get_by_sig(0, aiVision_orangeGoal.id);
//         bool found = (obj.signature == (unsigned)aiVision_orangeGoal.id && obj.width > 0);

//         pros::screen::print(pros::E_TEXT_MEDIUM, 4, "Count: %d   ", aiVision.get_object_count());

//         if (found) {
//             pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Width: %d   ", obj.width);
//             pros::screen::print(pros::E_TEXT_MEDIUM, 6, "X: %d Y: %d   ", obj.x_middle_coord, obj.y_middle_coord);
//         }

//         pros::delay(100);
//     }
// }

// // Sandbox for testing motion primitives. Uncomment calls as needed.
// void autonTest()
// {
//     setStartPosition(0, 0, 0);
//     // Uncomment to test specific routines:
//     // startOdometryTask();
//     // testVisionOnly();
//     // testVisionDirect();
//     // testVisionBasic();
//     // forwardToPoint(0, -75, 20, 16, 2.0, 2, 0.0, 0.0, 0.1, 0.1, 0.1, 60);
//     // startCoordinateFinder();
// }

// // ── Hardware diagnostic ───────────────────────────────────────────────────────
// // Spins each drive and intake motor for 500 ms and rumbles the controller
// // if the motor responds. Then cycles all pneumatics.
// // Select via auton selector for pre-match verification.
// void systemTest()
// {
//     initializeOpticalSensor();
//     InertialSensor.set_rotation(0);

//     // Stack-local pointer array — pointers are never stored or returned,
//     // so this is safe and avoids pulling in <functional> which breaks
//     // the PROS ARM newlib + GCC 14.3.1 toolchain.
//     pros::Motor* allMotors[] = {
//         &LeftMotor1, &LeftMotor2, &LeftMotor3,
//         &RightMotor1, &RightMotor2, &RightMotor3,
//         &intakeMotor1, &intakeMotor2
//     };
//     const int numMotors = 8;

//     // Spin each motor briefly and confirm it responds.
//     for (int i = 0; i < numMotors; i++) {
//         allMotors[i]->move(127);
//         pros::delay(500);
//         if (allMotors[i]->get_actual_velocity() != 0) {
//             Controller.rumble(".");  // confirms motor is alive
//         }
//         pros::delay(500);
//         allMotors[i]->brake();
//     }

//     // Cycle each pneumatic solenoid to verify actuation.
//     wingPneumatics.set_value(true);   pros::delay(500);
//     wingPneumatics.set_value(false);  pros::delay(500);

//     matchLoadPneumatics.set_value(true);   pros::delay(500);
//     matchLoadPneumatics.set_value(false);  pros::delay(500);

//     frontHoodPneumatics.set_value(true);   pros::delay(500);
//     frontHoodPneumatics.set_value(false);  pros::delay(500);

//     leftGatePneumatics.set_value(true);    pros::delay(500);
//     leftGatePneumatics.set_value(false);   pros::delay(500);

//     rightGatePneumatics.set_value(true);   pros::delay(500);
//     rightGatePneumatics.set_value(false);  pros::delay(500);

//     // Check optical sensors for proximity response.
//     bool lNear = leftLaneOptical.get_proximity()  > 50;
//     bool rNear = rightLaneOptical.get_proximity() > 50;
//     if (lNear) { Controller.rumble("."); }
//     pros::delay(500);
//     if (rNear) { Controller.rumble("."); }
//     pros::delay(500);
// }

// // Calibrates the IMU and blocks until complete. Displays status on brain screen.
// void Inertial_Calib()
// {
//     pros::screen::erase();
//     pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Calibrating...");
//     InertialSensor.reset();
//     while (InertialSensor.is_calibrating()) {
//         pros::delay(100);
//     }
//     pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Calibration Complete!");
//     pros::delay(500);
//     pros::screen::erase();
// }

// // ══════════════════════════════════════════════════════════════════════════════
// // ODOMETRY TEST ROUTINES
// // ══════════════════════════════════════════════════════════════════════════════

// // Drives a simple L-shaped path using odometry point navigation.
// void odomTest()
// {
//     initializeOpticalSensor();
//     startOdometryTask();
//     setStartPosition(0.0, 0.0, -90.0);
//     forwardToPoint(0, 70, 40);
//     turnRightToPoint(70, 0, 70);
//     stopOdometryTask();
// }

// // ══════════════════════════════════════════════════════════════════════════════
// // MATCH AUTONOMOUS ROUTINES
// // ══════════════════════════════════════════════════════════════════════════════

// // Left alliance starting position auton using motion profile (MP) functions.
// void autonLeft()
// {
//     initializeOpticalSensor();
//     setStartPosition(0.0, 0.0, 180.0);

//     forwardMP(79, 38, 180, 20, 0.815, 0.0, 0.0, 0.0, 0.55, 0.3, 80);
//     pros::delay(200);
//     pros::delay(200);
//     forwardMP(15, 10, 90, 15, 0.815, 0.0, 0.0, 0.0, 0.55, 0.3, 70);
//     pros::delay(200);
//     backwardMP(20, 15, 90, 10, 0.815, 0.00, 0.00, 0.0, 0.55, 0.3, 50);
//     pros::delay(200);
//     pros::delay(200);
//     forwardMP(35, 18, 0, 15, 0.815, 0.0, 0.0, 0.0, 0, 0, 80);
// }

// // Right alliance starting position auton — mirror of autonLeft.
// void autonRight()
// {
//     initializeOpticalSensor();
//     setStartPosition(0.0, 0.0, 0.0);

//     forwardMP(79, 38, 0, 20, 0.815, 0.0, 0.0, 0.0, 0.55, 0.3, 80);
//     pros::delay(200);
//     pros::delay(200);
//     forwardMP(15, 10, 270, 15, 0.815, 0.0, 0.0, 0.0, 0.55, 0.3, 70);
//     pros::delay(200);
//     backwardMP(20, 15, 270, 10, 0.815, 0.00, 0.00, 0.0, 0.55, 0.3, 50);
//     pros::delay(200);
//     pros::delay(200);
//     forwardMP(35, 18, 180, 15, 0.815, 0.0, 0.0, 0.0, 0, 0, 80);
// }

// // Left-side long auton — drives far to collect and score multiple objects.
// void leftSideLong()
// {
//     initializeOpticalSensor();
//     InertialSensor.set_rotation(0);
//     robotStartingHeading = 16;
//     ptoPneumatics.set_value(false);
//     frontHoodPneumatics.set_value(false);
//     wingPneumatics.set_value(false);

//     pros::delay(50);
//     matchloadStart(6000, 100, 1050, true);
//     driveForward(83, 60, 16);
//     pros::delay(250);
//     turnLeft(147, 118, 26, 80, 16);
//     pros::delay(200);

//     driveForward(102, 70, 147, 24, 0.3, 0.002, 0, 0.1, 1, 0.3, 90);
//     pros::delay(200);
//     turnLeft(175, 25, 26, 80, 13);
//     pros::delay(200);
//     smartStraight(50, 26, 180, 15, 150);
//     pros::delay(270);

//     driveBackward(26, 14, 176);
//     pros::delay(200);
//     turnRight(0, 145, 25, 90, 16);
//     pros::delay(200);
//     smartStraight(40, 19, 0, 24, 200);

//     score(3200, 100);
//     driveBackward(20, 14, -10);
//     pros::delay(100);
//     turnLeft(40, 18, 25, 90, 16);
//     pros::delay(100);

//     driveForward(35, 25, 40, 24, 0.3, 0.002, 0, 0.1, 1, 0.3, 90);
//     pros::delay(100);

//     turnRight(4, 30, 26, 80);
//     pros::delay(100);
//     driveForward(12, 0, 4, 24, 0.3, 0.002, 0, 0.1, 1, 0.3, 90);
//     wingPneumatics.set_value(true);

//     driveForward(40, 0, 4, 24, 0.3, 0.002, 0, 0.1, 1, 0.3, 40);
// }

// // Left-side middle auton — shorter path targeting the center field objects.
// void leftSidemiddle()
// {
//     initializeOpticalSensor();
//     InertialSensor.set_rotation(0);
//     robotStartingHeading = 16;
//     ptoPneumatics.set_value(false);
//     frontHoodPneumatics.set_value(false);
//     wingPneumatics.set_value(false);

//     pros::delay(50);
//     matchloadStart(1200, 100, 750, true);
//     matchLoadPneumatics.set_value(true);
//     driveForward(70, 60, 16, 26);
//     pros::delay(200);

//     turnRight(-50, 50, 25, 90, 16);

//     driveForward(25, 13, -50, 24, 0.3, 0.002, 0, 0.1, 1, 0.3, 90);
// }

// // Speedway event left-side auton using PTO and hood pneumatics.
// void SpeedwayAutonLeft()
// {
//     initializeOpticalSensor();
//     InertialSensor.set_rotation(0);
//     robotStartingHeading = 0;
//     ptoPneumatics.set_value(true);
//     frontHoodPneumatics.set_value(true);

//     pros::delay(400);
//     intake(true, 100);
//     forwardMP(80, 60, -18, 20, 0.615, 0, 0, 0.1, 0.05, 0.05, 50);
//     pros::delay(400);
//     backwardMP(7, 4, -20, 15);
//     intake(false, 0);
//     forwardMP(18, 10, 29, 30, 0.5, 0, 0, 0.1, 0.05, 0.05, 100);
//     ptoPneumatics.set_value(true);
//     frontHoodPneumatics.set_value(false);
//     score(400, 95);
//     score(1000, 65);
//     score(1000, 65);
//     backwardMP(116, 67, 45, 15);
//     pros::delay(200);
//     matchLoadPneumatics.set_value(true);
//     pros::delay(800);
//     rightDrive.brake();
//     leftDrive.brake();
//     intake(true, 100);
//     pros::delay(700);
//     intake(false, 0);
// }

// // Right-side 7-ball auton — uses match loader to stage balls before driving.
// void SevenBallRight()
// {
//     matchLoadPneumatics.set_value(true);
//     initializeOpticalSensor();
//     InertialSensor.set_rotation(0);
//     robotStartingHeading = -12;
//     ptoPneumatics.set_value(false);
//     frontHoodPneumatics.set_value(false);

//     // Brief delay for match load pneumatic to engage fully before retracting.
//     pros::delay(50);
//     matchLoadPneumatics.set_value(false);
//     driveForward(49, 38, -12);
//     pros::delay(300);
//     matchLoadPneumatics.set_value(true);
//     driveForward(30, 24, -12);
//     pros::delay(200);
//     driveBackward(25, 18, -12);
//     pros::delay(100);
// }

// // Left-side 7-ball auton — mirror of SevenBallRight with scoring pass.
// void SevenBallLeft()
// {
//     matchLoadPneumatics.set_value(true);
//     initializeOpticalSensor();
//     InertialSensor.set_rotation(0);
//     robotStartingHeading = 12;
//     ptoPneumatics.set_value(false);
//     frontHoodPneumatics.set_value(false);

//     pros::delay(50);
//     matchLoadPneumatics.set_value(false);
//     driveForward(40, 30, 16);
//     pros::delay(300);
//     driveForward(21, 21, 16);
//     pros::delay(100);
//     driveBackward(25, 15, 16);
//     pros::delay(100);
//     turnLeft(88, 65);
//     driveForward(72, 35, 88);
//     pros::delay(100);
//     turnRight(2, 65);
//     pidlessForward(600, 20);
//     score(7500, 75);
// }

// // Right-side middle auton — collects from center and scores at goal.
// void rightMiddleAuto()
// {
//     initializeOpticalSensor();
//     InertialSensor.set_rotation(0);
//     robotStartingHeading = -16;
//     ptoPneumatics.set_value(false);
//     frontHoodPneumatics.set_value(false);
//     wingPneumatics.set_value(false);

//     pros::delay(50);
//     matchloadStart(2000, 40, 630, true);
//     driveForward(91, 58, -16);
//     pros::delay(200);
//     driveBackward(19, 12, -16);
//     pros::delay(100);
//     turnLeft(45, 50, 25, 90, 16);
//     pros::delay(200);
//     smartStraight(70, 50, 45, 15, 150);
//     outtake(500, 100);
//     pros::delay(100);

//     driveBackward(130, 110, 45);
//     pros::delay(100);
//     turnLeft(180, 110, 25, 90, 16);
//     smartStraight(30, 21, 180, 15, 150);
//     pros::delay(270);

//     driveBackward(26, 14, 176);
//     pros::delay(200);
//     turnLeft(10, 145, 25, 90, 16);
//     pros::delay(200);
//     smartStraight(40, 19, 10, 24, 150);

//     score(3200, 100);
//     driveBackward(20, 14, 10);
//     pros::delay(200);
//     turnLeft(40, 18, 25, 90, 16);
//     pros::delay(100);

//     driveForward(35, 25, 40, 24, 0.3, 0.002, 0, 0.1, 1, 0.3, 90);
//     pros::delay(100);

//     turnRight(4, 30, 26, 80);
//     pros::delay(100);
//     driveForward(12, 0, 4, 24, 0.3, 0.002, 0, 0.1, 1, 0.3, 90);
//     wingPneumatics.set_value(true);

//     driveForward(40, 0, 4, 24, 0.3, 0.002, 0, 0.1, 1, 0.3, 40);
// }

// // ══════════════════════════════════════════════════════════════════════════════
// // SKILLS AUTONOMOUS ROUTINES
// // ══════════════════════════════════════════════════════════════════════════════

// // Full skills run — visits all four quadrants using vision + odometry navigation.
// void skillsAuton()
// {
//     // Start at East (0° Standard Cartesian), 36 cm up from south wall.
//     setStartPosition(0, 36, 0);
//     startOdometryTask();
//     leftGatePneumatics.set_value(true);
//     rightGatePneumatics.set_value(true);
//     initializeOpticalSensor();
//     leftLaneOptical.set_led_pwm(100);
//     rightLaneOptical.set_led_pwm(100);
//     rudderPneumatics.set_value(true);

//     // ── Stop 1: South East MatchLoader (6 Blocks + 1 Preload) ────────────────
//     turnRight(-42, 20);
//     smartStop(5, 5, 200, false);
//     intakeColourStart(9000, 100, false, false);
//     matchloadPneumaticStart(9000, 0, true);

//     visionDriveMinimal(
//         aiVision_redCube,
//         100,
//         0.0,
//         24.0, 65.0,
//         pros::E_MOTOR_BRAKE_COAST,
//         .5, 0.0, 0.0,
//         1.2,
//         1.0, 0.0, 0.0
//     );

//     smartStraight(47, 0, -90, 15, 100, 0.4, 0.0, 0.0, 0.2, 0.2, 0.2, 50);
//     pros::delay(2000);

//     // ── Stop 2: South West Middle (4 Blocks) ─────────────────────────────────
//     matchloadPneumaticStop();
//     turnRight(-175, 40);
//     smartStop(5, 5, 400, false);

//     moveVisionOdometry(
//         aiVision_redCube, 60,
//         48.1, 67.4,
//         30.0, pros::E_MOTOR_BRAKE_COAST,
//         60.0, 0.43, 0.0, 0.04, 1.05,
//         10, 50, 260, 0, 240,
//         25.0, 1.0, 0.22, 0.2, 0.25, 15, 5.0
//     );

//     matchloadPneumaticStart(400, 800, true);

//     moveOdometry(
//         37.71, 100,
//         0.0, 25.0, 2.0,
//         0.75, 0.0, 0.04,
//         pros::E_MOTOR_BRAKE_HOLD,
//         0.22, 0.2, 0.25,
//         60.0, 20.0, 5.0
//     );

//     moveVisionOdometry(
//         aiVision_redCube, 60,
//         31.61, 180,
//         30.0, pros::E_MOTOR_BRAKE_HOLD,
//         60.0, 0.43, 0.0, 0.04, 1.05,
//         10, 50, 260, 0, 240,
//         25.0, 1.0, 0.22, 0.2, 0.25, 15, 5.0
//     );

//     pros::delay(30000);

//     moveOdometry(
//         30.31, 210,
//         0.0, 25.0, 2.0,
//         0.75, 0.0, 0.04,
//         pros::E_MOTOR_BRAKE_HOLD,
//         0.22, 0.2, 0.25,
//         60.0, 20.0, 5.0
//     );

//     pros::delay(30000);
//     smartStop(5, 5, 300, false);
//     pros::delay(30000);

//     // ── Stop 3: North West (2 Blocks) ────────────────────────────────────────
//     turnRight(45, 20);
//     wingPneumatics.set_value(true);
//     smartStop(5, 5, 200, false);
//     matchloadPneumaticStop();

//     moveOdometry(
//         59.32, 153.35,
//         30, 25.0, 2.0,
//         0.85, 0.0, 0.04,
//         pros::E_MOTOR_BRAKE_HOLD,
//         0.22, 0.2, 0.25,
//         60.0, 20.0, 5.0
//     );

//     matchloadPneumaticStart(1000, 0, true);

//     // ── Stop 4: North East (2 Blocks) ────────────────────────────────────────
//     turnRight(-185, 40);
//     smartStop(5, 5, 400, false);

//     moveVisionOdometry(
//         aiVision_redCube, 60,
//         48.1, 67.4,
//         30.0, pros::E_MOTOR_BRAKE_COAST,
//         60.0, 0.43, 0.0, 0.04, 1.05,
//         10, 50, 260, 0, 240,
//         25.0, 1.0, 0.22, 0.2, 0.25, 15, 5.0
//     );

//     matchloadPneumaticStart(2500, 425, true);

//     moveOdometry(
//         30.5, 125.99,
//         0.0, 25.0, 2.0,
//         0.75, 0.0, 0.04,
//         pros::E_MOTOR_BRAKE_HOLD,
//         0.22, 0.2, 0.25,
//         60.0, 20.0, 5.0
//     );

//     smartStop(5, 5, 300, false);
// }

// // Alternate skills run — same stops as skillsAuton with different routing.
// void skillsAuton2()
// {
//     setStartPosition(0, 36, 0);
//     startOdometryTask();
//     leftGatePneumatics.set_value(true);
//     rightGatePneumatics.set_value(true);
//     initializeOpticalSensor();
//     leftLaneOptical.set_led_pwm(100);
//     rightLaneOptical.set_led_pwm(100);
//     rudderPneumatics.set_value(false);

//     // ── Stop 1: South East MatchLoader ───────────────────────────────────────
//     turnRight(-42, 20);
//     smartStop(5, 5, 200, false);
//     intakeColourStart(9000, 100, false, false);
//     matchloadPneumaticStart(9000, 0, true);

//     visionDriveMinimal(
//         aiVision_redCube,
//         100, 0.0,
//         24.0, 65.0,
//         pros::E_MOTOR_BRAKE_COAST,
//         .5, 0.0, 0.0,
//         1.2,
//         1.0, 0.0, 0.0
//     );

//     smartStraight(47, 0, -90, 15, 100, 0.4, 0.0, 0.0, 0.2, 0.2, 0.2, 50);
//     pros::delay(1000);

//     // ── Stop 2: South West Middle ─────────────────────────────────────────────
//     matchloadPneumaticStop();
//     turnLeft(90, 140, 25, 100, 4);
//     smartStop(5, 5, 400, false);
//     pros::delay(5000);
//     smartStraight(47, 0, -90, 15, 100, 0.4, 0.0, 0.0, 0.2, 0.2, 0.2, 50);
//     pros::delay(50000);

//     moveVisionOdometry(
//         aiVision_orangeGoal, 100,
//         70, 53.5,
//         15.0, pros::E_MOTOR_BRAKE_COAST,
//         60.0, 0.75, 0.0, 0.04, 1.05,
//         10, 0, 260, 0, 240,
//         25.0, 1.0, 0.22, 0.2, 0.25, 20, 5.0
//     );

//     smartStop(5, 5, 400, false);

//     moveVisionOdometry(
//         aiVision_redCube, 60,
//         48.1, 67.4,
//         30.0, pros::E_MOTOR_BRAKE_COAST,
//         60.0, 0.43, 0.0, 0.04, 1.05,
//         10, 50, 260, 0, 240,
//         25.0, 1.0, 0.22, 0.2, 0.25, 15, 5.0
//     );

//     matchloadPneumaticStart(400, 800, true);

//     moveOdometry(
//         37.71, 100,
//         0.0, 25.0, 2.0,
//         0.75, 0.0, 0.04,
//         pros::E_MOTOR_BRAKE_HOLD,
//         0.22, 0.2, 0.25,
//         60.0, 20.0, 5.0
//     );

//     moveVisionOdometry(
//         aiVision_redCube, 60,
//         31.61, 180,
//         30.0, pros::E_MOTOR_BRAKE_HOLD,
//         60.0, 0.43, 0.0, 0.04, 1.05,
//         10, 50, 260, 0, 240,
//         25.0, 1.0, 0.22, 0.2, 0.25, 15, 5.0
//     );

//     pros::delay(30000);

//     moveOdometry(
//         30.31, 210,
//         0.0, 25.0, 2.0,
//         0.75, 0.0, 0.04,
//         pros::E_MOTOR_BRAKE_HOLD,
//         0.22, 0.2, 0.25,
//         60.0, 20.0, 5.0
//     );

//     pros::delay(30000);
//     smartStop(5, 5, 300, false);

//     // ── Stop 3: North West ────────────────────────────────────────────────────
//     turnRight(45, 20);
//     wingPneumatics.set_value(true);
//     smartStop(5, 5, 200, false);
//     matchloadPneumaticStop();

//     moveOdometry(
//         59.32, 153.35,
//         30, 25.0, 2.0,
//         0.85, 0.0, 0.04,
//         pros::E_MOTOR_BRAKE_HOLD,
//         0.22, 0.2, 0.25,
//         60.0, 20.0, 5.0
//     );

//     matchloadPneumaticStart(1000, 0, true);

//     // ── Stop 4: North East ────────────────────────────────────────────────────
//     turnRight(-185, 40);
//     smartStop(5, 5, 400, false);

//     moveVisionOdometry(
//         aiVision_redCube, 60,
//         48.1, 67.4,
//         30.0, pros::E_MOTOR_BRAKE_COAST,
//         60.0, 0.43, 0.0, 0.04, 1.05,
//         10, 50, 260, 0, 240,
//         25.0, 1.0, 0.22, 0.2, 0.25, 15, 5.0
//     );

//     matchloadPneumaticStart(2500, 425, true);

//     moveOdometry(
//         30.5, 125.99,
//         0.0, 25.0, 2.0,
//         0.75, 0.0, 0.04,
//         pros::E_MOTOR_BRAKE_HOLD,
//         0.22, 0.2, 0.25,
//         60.0, 20.0, 5.0
//     );

//     smartStop(5, 5, 300, false);
// }

// // Solo AWP using vision drive to collect blocks on both sides of the field.
// void soloAWP()
// {
//     ptoPneumatics.set_value(false);
//     frontHoodPneumatics.set_value(false);
//     wingPneumatics.set_value(true);

//     forwardToPoint(-47, 0, 70, 15, 1, 0.05, 0, 0, 0.2, 0.2, 0.2, 40);
//     turnRight(-135, 0, 24, 100, 20);
//     smartStop(5, 10, 300, false);
//     smartStraight(47, 0, -180, 15, 200, 0, 0, 0.05, 0.2, 0.2, 0.2, 40);

//     // Bottom middle right — 4 cubes.
//     turnRight(45, 0, 25, 100, 70);
//     pros::delay(350);

//     visionDriveV2(
//         aiVision_redCube, 60,
//         45,
//         pros::E_MOTOR_BRAKE_HOLD,
//         75.0,
//         0.1, 0.0, 0.0,
//         0.3,
//         10, 0, 320, 0, 240,
//         24.0, 100.0,
//         1.5, 0.0, 0.0
//     );

//     matchLoadPneumatics.set_value(true);

//     // Top middle right — 4 cubes.
//     setStartPosition(0.0, 0, 45);
//     turnRight(-45, 0, 25, 100, 70);

//     visionDriveV2(
//         aiVision_blueCube, 60,
//         45,
//         pros::E_MOTOR_BRAKE_HOLD,
//         75.0,
//         0.1, 0.0, 0.0,
//         0.3,
//         10, 0, 320, 0, 240,
//         24.0, 100.0,
//         1.5, 0.0, 0.0
//     );
// }

// // Colour sort intake test — optical sensor only, no movement.
// void colourTest()
// {
//     initializeOpticalSensor();
//     InertialSensor.set_rotation(0);
// }

// // Solo AWP using odometry — middle-field approach with wing deploy.
// void soloAWPMiddle()
// {
//     initializeOpticalSensor();
//     InertialSensor.set_rotation(0);
//     robotStartingHeading = -90;
//     ptoPneumatics.set_value(false);
//     frontHoodPneumatics.set_value(false);
//     wingPneumatics.set_value(true);

//     driveForwardV2(60, 45, -90, 24, 3, 0.3, 0.005, 0, 0.1, 1, 0.3, 100);

//     leftDrive.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
//     rightDrive.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
//     leftDrive.brake();
//     rightDrive.brake();
//     turnRight(-179, 65, 25, 80, 28);
// }

// // Placeholder skills entry — sets start position only.
// void skills()
// {
//     initializeOpticalSensor();
//     setStartPosition(0.0, 0.0, 0.0);
// }

// // Nothing auton — runs intake only, used for testing ball feed.
// void nothing()
// {
//     initializeOpticalSensor();
//     InertialSensor.set_rotation(0);
//     robotStartingHeading = 0;
//     intakeStart(5000, 60, false, false);
//     pidlessForward(1000, 100);
// }

// // Solo AWP using vision + odometry to collect and score.
// void soloAwp2()
// {
//     setStartPosition(0, 36, 0);
//     startOdometryTask();
//     initializeOpticalSensor();

//     intakeHopperStart(2000, 100, 500, true);  // delayed intake start, non-blocking

//     moveVisionOdometry(
//         aiVision_redCube, 60,
//         110.0, 36,
//         70.0, pros::E_MOTOR_BRAKE_HOLD,
//         100.0, 0.43, 0.0, 0.04, 1.05,
//         10, 0, 320, 0, 240,
//         16.0, 1.0, 0.22, 0.2, 0.25, 15, 5.0
//     );

//     pros::delay(20000);
//     turnRight(-29, 0);
//     smartStop(5, 0, 200, false);
//     smartStraight(47, 0, -90, 15, 100, 0.4, 0.0, 0.0, 0.2, 0.2, 0.2, 80);
//     pros::delay(500);
//     matchLoadPneumatics.set_value(false);
//     pros::delay(200);
//     turnLeft(22, 0, 20, 100);
//     pros::delay(500);
//     smartStraight(67, 57, 92, 24, 150, 0.05, 0, 0.0, 0.2, 0.2, 0.2, 40);
//     leftGatePneumatics.set_value(false);
//     rightGatePneumatics.set_value(false);
//     score(30000, 100);
//     startCoordinateFinder();
// }

// // Runs intake and scoring mechanisms sequentially for system verification.
// void runEverything()
// {
//     initializeOpticalSensor();
//     InertialSensor.set_rotation(0);
//     matchloadStart(1000, 50, 0, true);
//     score(1000, 50);
//     outtake(1000, 50);
//     wingPneumatics.set_value(true);
// }

// // Gateway skills run — vision-guided approach with match loader and scoring.
// void skillsAutonGateway()
// {
//     setStartPosition(0.0, 0, -90.0);
//     initializeOpticalSensor();

//     matchLoadPneumatics.set_value(true);
//     pros::delay(200);
//     turnRight(-142, 5, 2, 85, 22);
//     smartStop(5, 10, 300, false);
//     matchloadStart(2700, 100, 0, true);

//     visionDriveMinimal(
//         aiVision_redCube,
//         70, 0.0,
//         24.0, 40.0,
//         pros::E_MOTOR_BRAKE_HOLD,
//         .06, 0.0, 0.0,
//         0.3,
//         1.50, 0.0, 0.0
//     );

//     smartStraight(47, 40, -180, 15, 200, 0.4, 0.01, 0.05, 0.2, 0.2, 0.2, 40);
//     pros::delay(2000);
//     driveBackward(26, 14, -180, 24, 1.1, 0, 0, 0.1, 0.2, 0.3, 70);
//     pros::delay(400);

//     turnLeft(-4, 0, 25, 85, 80);
//     pros::delay(400);
//     smartStraight(50, 48, -4, 24, 150, 0.6, 0.01, 0.05, 0.2, 0.2, 0.2, 50);
//     score(320000, 100);
//     driveBackward(5, -4, -180, 24, 1.1, 0, 0, 0.1, 0.2, 0.3, 70);

//     matchloadPneumaticStart(500, 0, true);   // drop piston, hold 500 ms, retract
//     intakeHopperStart(500, 100, 0, true);    // run intake concurrently

//     turnLeft(-330, 0, 25, 100, 20);
//     pros::delay(200);

//     visionDriveMinimal(
//         aiVision_blueCube,
//         100, 0.0,
//         24.0, 75.0,
//         pros::E_MOTOR_BRAKE_HOLD,
//         .1, 0.0, 0.0,
//         1.75,
//         1.50, 0.0, 0.0
//     );
// }

// // Odometry solo AWP — sets position and starts coordinate finder only.
// void soloAwpOdom()
// {
//     setStartPosition(0, 0, 0);
//     startOdometryTask();
//     startCoordinateFinder();
//     initializeOpticalSensor();
// }

// // Provincial right-side auto — vision match load followed by scoring run.
// void provsAuto()
// {
//     setStartPosition(0, 36, 0);
//     startOdometryTask();
//     leftGatePneumatics.set_value(true);
//     rightGatePneumatics.set_value(true);
//     initializeOpticalSensor();
//     leftLaneOptical.set_led_pwm(100);
//     rightLaneOptical.set_led_pwm(100);
//     rudderPneumatics.set_value(false);

//     // ── Stop 1: South East MatchLoader ───────────────────────────────────────
//     turnRight(-42, 36);
//     smartStop(5, 5, 200, false);
//     matchloadPneumaticStart(9000, 0, true);

//     visionDriveMinimal(
//         aiVision_redCube,
//         100, 0.0,
//         24.0, 65.0,
//         pros::E_MOTOR_BRAKE_COAST,
//         .5, 0.0, 0.0,
//         1.2,
//         1.0, 0.0, 0.0
//     );

//     intakeStart(1500, 100, false, true);
//     smartStraight(30, 15, -90, 15, 200, 0.4, 0.0, 0.0, 0.2, 0.2, 0.2, 50);

//     matchloadPneumaticStop();
//     turnLeft(87, 50, 25, 50, 4);
//     smartStop(5, 5, 400, false);
//     smartStraight(15, 12, 87, 10, 10, 0.4, 0.01, 0.05, 0.2, 0.2, 0.2, 30);
//     rightScore(3000, 100);
//     turnLeft(140, 30, 20, 80);

//     visionDriveMinimal(
//         aiVision_redCube,
//         100, 0.0,
//         24.0, 65.0,
//         pros::E_MOTOR_BRAKE_COAST,
//         .5, 0.0, 0.0,
//         1.2,
//         1.0, 0.0, 0.0
//     );
// }

// // Provincial left-side auto — mirror of provsAuto using blue cube signature.
// void provsAutoLeft()
// {
//     setStartPosition(0, 36, 180);
//     startOdometryTask();
//     leftGatePneumatics.set_value(true);
//     rightGatePneumatics.set_value(true);
//     initializeOpticalSensor();
//     leftLaneOptical.set_led_pwm(100);
//     rightLaneOptical.set_led_pwm(100);
//     rudderPneumatics.set_value(false);

//     // ── Stop 1: South East MatchLoader ───────────────────────────────────────
//     turnLeft(267, 36);
//     smartStop(5, 5, 200, false);
//     matchloadPneumaticStart(9000, 0, true);

//     visionDriveMinimal(
//         aiVision_blueCube,
//         100, 0.0,
//         24.0, 65.0,
//         pros::E_MOTOR_BRAKE_COAST,
//         .5, 0.0, 0.0,
//         1.2,
//         1.0, 0.0, 0.0
//     );

//     intakeStart(1500, 100, false, true);
//     smartStraight(30, 15, -90, 15, 200, 0.4, 0.0, 0.0, 0.2, 0.2, 0.2, 50);

//     matchloadPneumaticStop();
//     turnRight(93, 50, 25, 50, 4);
//     smartStop(5, 5, 400, false);
//     smartStraight(15, 12, 93, 10, 10, 0.4, 0.01, 0.05, 0.2, 0.2, 0.2, 30);
//     rightScore(3000, 100);
// }

// // Vision following demo — follows a red ball for 10 seconds then stops.
// void visionDemo()
// {
//     followRedBall();
//     pros::delay(10000);
//     stopFollowing();
// }

// // ══════════════════════════════════════════════════════════════════════════════
// // AUTON SELECTOR
// // ══════════════════════════════════════════════════════════════════════════════
// // Displays a menu on the brain screen. Use controller Left/Right to cycle
// // through options and A to confirm. Runs the selected auton immediately.
// void autonSelector()
// {
//     int autonMode = 0;
//     const char* autonNames[] = {
//         "Auton Left",
//         "Auton Right",
//         "Solo AWP",
//         "Skills",
//         "Odom Test",
//         "Vision Test",
//         "Auton Test"
//     };
//     const int numAutons = 7;

//     pros::screen::erase();

//     while (true) {
//         pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Selected: %s          ", autonNames[autonMode]);
//         pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Press Left/Right to change");
//         pros::screen::print(pros::E_TEXT_MEDIUM, 4, "Press A to confirm");

//         if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
//             autonMode = (autonMode - 1 + numAutons) % numAutons;
//             pros::delay(200);
//         }
//         else if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
//             autonMode = (autonMode + 1) % numAutons;
//             pros::delay(200);
//         }
//         else if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
//             pros::screen::erase();
//             pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Running: %s", autonNames[autonMode]);
//             pros::delay(500);

//             switch (autonMode) {
//                 case 0: autonLeft();       break;
//                 case 1: autonRight();      break;
//                 case 2: soloAWP();         break;
//                 case 3: skills();          break;
//                 case 4: odomTest();        break;
//                 case 5: visionSensorTest(); break;
//                 case 6: autonTest();       break;
//             }
//             break;
//         }

//         pros::delay(20);
//     }
// }