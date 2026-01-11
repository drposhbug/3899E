#include "main.h"
#include "robot_config.hpp"

void driverControl() {
    while (true) {
        // Get joystick values
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        
        // Apply deadband
        if (abs(leftY) < 10) leftY = 0;
        if (abs(rightY) < 10) rightY = 0;
        
        // Move left side
        leftMotor1.move(leftY);
        leftMotor2.move(leftY);
        leftMotor3.move(leftY);
        
        // Move right side
        rightMotor1.move(rightY);
        rightMotor2.move(rightY);
        rightMotor3.move(rightY);
        
        pros::delay(20);
    }
}