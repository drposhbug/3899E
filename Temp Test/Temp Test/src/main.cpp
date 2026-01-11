#include "main.h"
#include "robot_config.hpp"
#include "driver.hpp"
#include "auton.hpp"
#include "utils.hpp"       // FIX: Added this so it finds initializeOpticalSensor
#include "navigation.hpp"  // FIX: Added this so it finds smartMove
#include "autontasks.hpp"  // FIX: Added this so it finds matchloadStart

// =============================================================
// INITIALIZATION
// =============================================================
void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(1, "NEW CODE LOADED!...");

    // SAFETY CHECK: Motors
    if (leftMotor[0] == nullptr || rightMotor[0] == nullptr) {
        pros::lcd::clear();
        pros::lcd::set_text(1, "CRITICAL ERROR:");
        pros::lcd::set_text(2, "Motors are NULL!");
        pros::lcd::set_text(3, "Rebuild Project Required");
        return; 
    }

    // Initialize Hardware
    //wingPneumatics.set_value(false);
    
    pros::lcd::set_text(1, "NEW CODE LOADED!");
    pros::lcd::set_text(2, "Battery: " + std::to_string(pros::battery::get_capacity()) + "%");
}

//void disabled() {}
//void competition_initialize() {}

// =============================================================
// AUTONOMOUS (DEBUG MODE)
// =============================================================
void autonomous() {
autonTest();
}

// =============================================================
// DRIVER CONTROL
// =============================================================
void opcontrol() {
    pros::lcd::clear();
    pros::lcd::print(0, "Driver Control Active");
/*
    if (leftMotor[0] == nullptr) {
        pros::lcd::print(1, "STOP: Null Motors in Driver");
        return;
    }
        */

    driverControl(); 
}