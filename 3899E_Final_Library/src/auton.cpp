#include "pid.h"          // Include the PID header file
#include "vex.h"          // Include the VEX library
#include "utils.h"        // Include the utility header for normalizeHeading
#include "robot_config.h" // Include the robot configuration
#include "navigation.h"
#include "odometry.h"
#include <cmath> // Include math library for M_PI

using namespace vex; // Use the VEX namespace

/*straightOdometry(targetDistance, breakDistance, targetHeading, minSpeed,
                    kp_heading, ki_heading, kd_heading,
                    accelHeadingScaling, decelHeadingScaling,
                    approachHeadingScaling, maxSpeed);
*/                    

//turnOdometry(turnAmount, breakDistance, minSpeed, maxSpeed)
   void autonTest()
{
    initializeOpticalSensor();

    // Reset gyro to ensure clean starting state
    InertialSensor.setRotation(0, degrees);
    InertialSensor.setHeading(0, degrees);
    
    headingOffset = 0; 
    
    //move(300, 80, forward);
    forwardMP(200, 80, 0, 20, 0.9, 0.002, 0.00, 0.1, 100, 0.15, 100);
    //leftMP(45,30,10);   
    //turnOdometry(180, 20, 10, 100);
    //wait(300, msec);
    //turnOdometry(0, 20, 10, 100);
    //wait(200, msec);
    //turnOdometry(180, 20, 10, 100);
    //wait(200, msec);
    
    /*
    // Explicitly ensure all motors are braked at the end
    for (int i = 0; i < 3; i++) {
        leftMotor[i].setBrake(brake);
        rightMotor[i].setBrake(brake);
        leftMotor[i].stop();
        rightMotor[i].stop();
    }
    */

    //wait(10000, msec);  // Wait to test resistance
}


