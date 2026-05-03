#include "main.h"
#include "robot_config.h"
#include "driver.h"
#include "auton.h"
#include "utils.h"
#include "navigation.h"
#include "autontasks.h"

// Called once when the robot powers on (before competition starts)
void initialize() {
    passiveEncoderLeft.set_reversed(true);
    passiveEncoderRight.set_reversed(false);
    passiveEncoderX.set_reversed(true);
    robotInit();  // calibrates IMU, registers vision sigs, zeros encoders
}

// Called when the robot is disabled by the field controller
void disabled() {}

// Called during the pre-autonomous phase of a competition
void competition_initialize() {}

// Called when the autonomous period begins
void autonomous() {
    pros::screen::erase();
    wingPneumatics.set_value(false);
    test();
}

// Called when the driver control period begins
void opcontrol() {
    pros::screen::erase();
    Controller.clear();  // stop controller prints during driver control
    driverControl();
}
