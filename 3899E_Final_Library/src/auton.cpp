#include "pid.h"          // Include the PID header file
#include "vex.h"          // Include the VEX library
#include "utils.h"        // Include the utility header for normalizeHeading
#include "robot_config.h" // Include the robot configuration
#include "navigation.h"
#include "odometry.h"
#include <cmath> // Include math library for M_PI

using namespace vex; // Use the VEX namespace

    void autonTest()
    {
        
        initializeOpticalSensor();
        
        headingOffset = 0; 
        
        
       // forwardMP(100); // Mirrored angle from 138 to 42 (180 - 138 = 42)
      forwardMP(200, 120, 0, 0.1, 0.0, 0.00, 0.00, 0.1, 0.1, 0.1, 100);
           
        wait(200, msec);
       /* 
        LeftMotor1.setBrake(brakeType::coast);
        LeftMotor2.setBrake(brakeType::coast);
        LeftMotor3.setBrake(brakeType::coast);
        RightMotor1.setBrake(brakeType::coast);
        RightMotor2.setBrake(brakeType::coast);
        RightMotor3.setBrake(brakeType::coast);
        */
    }


