#include "main.h"
#include "robot_config.h"
#include "driver.h"
#include "auton.h"
#include "utils.h"
#include "navigation.h"
#include "autontasks.h"
#include "odometry.h"
#include "ai.h"

void initialize()
{
    passiveEncoderLeft.set_reversed(true);
    passiveEncoderRight.set_reversed(false);
    passiveEncoderX.set_reversed(true);

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

    // Jetson diagnostic display — live packet stats on LCD at 10Hz
    pros::Task([]{
        while (true) {
            pros::lcd::print(0, "pkts:%d err:%d tmo:%d",
                g_jetson.get_packets(),
                g_jetson.get_errors(),
                g_jetson.get_timeouts());
            int32_t strat = getStrategy();
            JetsonDetection det;
            if (getLatestDetection(CLASS_FWD_RED_BLOCK, 0.4f, &det)) {
                pros::lcd::print(1, "strat:%d hOff:%.2f dist:%.1fcm",
                    strat, det.hOffset, det.distanceCm);
            } else {
                pros::lcd::print(1, "strat:%d  no target", strat);
            }
            pros::delay(100);
        }
    }, TASK_PRIORITY_MIN, TASK_STACK_DEPTH_DEFAULT, "JetsonDisplay");
}

void disabled() {}

void competition_initialize() {
    autonSelector();
}

void autonomous()
{
    pros::screen::erase();

    // DEBUG velocity display task — remove before competition
    // Reads motor and encoder velocity every 50ms, holds last non-zero value on screen.
    static double dispMotorRPM   = 0.0;
    static double dispEncoderRPM = 0.0;
    pros::Task([&]{
        while (true) {
            double mRPM = leftDrive.get_actual_velocity() * DRIVE_MOTOR_RPM_ADJ;
            double eRPM = globalLeftEncoderRPM * (encoderWheelCircumferenceCM / wheelCircumferenceCM);
            // Only update display if robot is moving — holds last value when stopped
            if (std::fabs(mRPM) > 1.0 || std::fabs(eRPM) > 0.1) {
                dispMotorRPM   = mRPM;
                dispEncoderRPM = eRPM;
            }
            pros::lcd::print(2, "mRPM:%7.1f eRPM:%7.2f", dispMotorRPM, dispEncoderRPM);
            pros::lcd::print(3, "slip: mRPM/eRPM ratio");
            pros::delay(50);
        }
    }, TASK_PRIORITY_MIN, TASK_STACK_DEPTH_DEFAULT, "VelDebug");

    // ── Uncomment one to test at home ─────────────────────────────────────────
    visionTest();
    //navTest();
    //runAIMatchRoute();
    //routeTest();
    //routeGridTest();
    //systemTest();
    //coordinateFinder();
    // ─────────────────────────────────────────────────────────────────────────
    
}

void opcontrol()
{
    pros::screen::erase();
    Controller.clear();
    driverControl();
}