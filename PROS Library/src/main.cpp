#include "main.h"
#include "robot_config.hpp"
#include "auton.hpp"
#include "driver.hpp"
#include "autontasks.hpp"
#include "utils.hpp"
#include "odometry.hpp"

void initialize() {
    wingPneumatics.set_value(false);
    vexcodeInit();
    
    pros::lcd::initialize();
    pros::lcd::set_text(1, "Robot Initialized");
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    pros::lcd::clear();
    pros::lcd::set_text(1, "Running Autonomous...");

    // Start the heading display task safely using the wrapper
    startHeadingDisplay();
    
    wingPneumatics.set_value(false);
    
    // Uncomment the routine you want to run
    // autonTest();    
    // autonLeft(); 
    // autonRight();
    soloAWP();
    
    pros::lcd::set_text(1, "Autonomous Complete");
}

void opcontrol() {
    pros::lcd::clear();
    pros::lcd::set_text(1, "Running Driver Control...");
    
    // Stop the autonomous heading display when driver control starts
    stopHeadingDisplay();
    
    driverControl(); 
}