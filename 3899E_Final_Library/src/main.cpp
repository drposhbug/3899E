#include "main.h"
#include "robot_config.h"
#include "driver.h"
#include "auton.h"
#include "utils.h"
#include "navigation.h"
#include "autontasks.h"
#include "odometry.h"
#include "ai.h"

// std::pmr::string teamColor = "RED";  // default to red, set to "BLUE" if blue alliance

void initialize()
{
    passiveEncoderLeft.set_reversed(true);
    passiveEncoderRight.set_reversed(false);
    passiveEncoderX.set_reversed(true);

    // // Reset and enable color blob detection on the front camera
    // aiVision_Front.reset();
    // aiVision_Front.enable_detection_types(pros::AivisionModeType::colors);

    // // Re-create your VEXcode parameters inside the PROS struct format:
    // // (id, red/Cr, green/Cb, blue/hue, hue_range, sat_range)
    // pros::AIVision::Color redBlock  = {1, 167, 29, 70, 16, 0.38};
    // pros::AIVision::Color blueBlock = {2, 31, 69, 115, 17, 0.42};
    // pros::AIVision::Color mlModel   = {3, 223, 137, 51, 7, 0.45};

    // // Upload the signatures to the front camera hardware
    // aiVision_Front.set_color(redBlock);
    // aiVision_Front.set_color(blueBlock);
    // aiVision_Front.set_color(mlModel);

    // // If your back camera uses identical color tracking, send them there too:
    // aiVision_Back.reset();
    // aiVision_Back.enable_detection_types(pros::AivisionModeType::colors);
    // aiVision_Back.set_color(redBlock);
    // aiVision_Back.set_color(blueBlock);
    // aiVision_Back.set_color(mlModel);

    robotInit();

    pros::lcd::initialize();

    // GPS live display — disabled for now
    // pros::screen::erase();
    // while (!Controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
    //     pros::screen::print(pros::E_TEXT_MEDIUM, 1, "GPS err:%.3fm", gpsSensor.get_error());
    //     pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Raw X:%.3f Y:%.3fm",
    //         gpsSensor.get_position().x, gpsSensor.get_position().y);
    //     bool gpsOk = gpsReset();
    //     pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Reset: %s", gpsOk ? "OK" : "FAIL");
    //     pros::screen::print(pros::E_TEXT_MEDIUM, 4, "Odom X:%.1f Y:%.1fcm", globalX, globalY);
    //     pros::delay(100);
    // }

    // Jetson diagnostic display — disabled during field targets debugging
    // Uncomment after debugging is complete
    // pros::Task([]{
    //     while (true) {
    //         pros::lcd::print(0, "pkts:%d err:%d tmo:%d",
    //             g_jetson.get_packets(),
    //             g_jetson.get_errors(),
    //             g_jetson.get_timeouts());
    //         int32_t strat = getStrategy();
    //         JetsonDetection det;
    //         if (getLatestDetection(CLASS_FWD_RED_BLOCK, 0.4f, &det)) {
    //             pros::lcd::print(1, "strat:%d hOff:%.2f dist:%.1fcm",
    //                 strat, det.hOffset, det.distanceCm);
    //         } else {
    //             pros::lcd::print(1, "strat:%d  no target", strat);
    //         }
    //         pros::delay(100);
    //     }
    // }, TASK_PRIORITY_MIN, TASK_STACK_DEPTH_DEFAULT, "JetsonDisplay");
}

void disabled() {}

void competition_initialize() {
    // autonSelector();
}

static bool posDisplayRunning = true;

void autonomous()
{
    pros::screen::erase();
    // matchloader.set_value(true);

    // PosDisplay disabled for routeGridTest — conflicts with routePrintGrid on line 0
    // Restore after grid debugging is done
    // pros::Task([]{
    //     while (posDisplayRunning) {
    //         pros::screen::print(pros::E_TEXT_MEDIUM, 0,
    //             "X:%.1f Y:%.1f H:%.1f",
    //             globalX, globalY, getContinuousStandardHeading());
    //         pros::delay(100);
    //     }
    // }, TASK_PRIORITY_MIN, TASK_STACK_DEPTH_DEFAULT, "PosDisplay");

    // DEBUG velocity display task — remove before competition
    // Comment back in after fieldTargetsTest debugging is done
    // pros::Task([&]{
    //     while (true) {
    //         double mRPM = leftDrive.get_actual_velocity() * DRIVE_MOTOR_RPM_ADJ;
    //         double eRPM = globalLeftEncoderRPM * (encoderWheelCircumferenceCM / wheelCircumferenceCM);
    //         if (std::fabs(mRPM) > 1.0 || std::fabs(eRPM) > 0.1) {
    //             dispMotorRPM   = mRPM;
    //             dispEncoderRPM = eRPM;
    //         }
    //         pros::lcd::print(2, "mRPM:%7.1f eRPM:%7.2f", dispMotorRPM, dispEncoderRPM);
    //         pros::lcd::print(3, "slip: mRPM/eRPM ratio");
    //         pros::delay(50);
    //     }
    // }, TASK_PRIORITY_MIN, TASK_STACK_DEPTH_DEFAULT, "VelDebug");

    // ── Uncomment one to test at home ─────────────────────────────────────────
    //fieldTargetsTest();
    visionTest();
    // navTest();
    // blueColorSortTest();
    //runAIMatchRoute();
    //routeTest();
    //routeGridTest();
    //systemTest();
    //coordinateFinder();
    // autonLeft15();
    //skills();
    // ─────────────────────────────────────────────────────────────────────────

    // Hold for remainder of autonomous period — prevents task cleanup killing the screen
    while (true) {
        pros::delay(100);
    }
}

void opcontrol() {

    while (true) {
        
        AITracking("RED"); // put alliance colour here as argument: "RED" or "BLUE"
        pros::delay(50);
    }
    // pros::screen::erase();
    // Controller.clear();
    driverControl();
    // coordinateFinder();
}