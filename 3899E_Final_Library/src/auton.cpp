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
   void autonTest()
{
    initializeOpticalSensor();
    
    headingOffset = 0; 
    
    forwardMP(200, 80, -25, 25, 0.5, 0.00, 0.00, 0.1, 0.1, 0.05, 100);
       
    wait(200, msec);
    
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


