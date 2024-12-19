#include "vex.h"
#include "vision_tracking.h"
#include "robot-config.h"  // Include this to use motor definitions and vision sensor

using namespace vex;

const int VISION_SENSOR_WIDTH = 320;  // Correct width for the AI vision sensor

void followObject(aivision &visionSensor, double maxVelocity, double distanceThreshold) {
    while (true) {
        Brain.Screen.clearScreen();
        Brain.Screen.printAt(10, 20, "Attempting to take snapshot");
        visionSensor.takeSnapshot(red1);  // Use the red1 for taking snapshot
        Brain.Screen.printAt(10, 40, "Snapshot taken, object count: %d", visionSensor.objectCount);

        if (visionSensor.objectCount > 0) {
            // Get object information
            double objX = visionSensor.largestObject.centerX;
            double objArea = visionSensor.largestObject.width * visionSensor.largestObject.height;

            // Debugging: Print object information
            Brain.Screen.printAt(10, 60, "Object X: %f", objX);
            Brain.Screen.printAt(10, 80, "Object Area: %f", objArea);
            Brain.Screen.printAt(10, 100, "Target Area: %f", distanceThreshold);

            // Calculate error from center
            double errorX = objX - (VISION_SENSOR_WIDTH / 2);

            // Debugging: Print error
           // Brain.Screen.printAt(10, 100, "Error X: %f", errorX);

            // Calculate drive velocity based on error
            double turnVelocity = errorX * .05; // Adjust the 0.2 for turning sensitivity
            double forwardVelocity = objArea < distanceThreshold ? maxVelocity : 0; // Move forward if close

            // Debugging: Print calculated velocities
            Brain.Screen.printAt(10, 120, "Turn Velocity: %f", turnVelocity);
            Brain.Screen.printAt(10, 140, "Forward Velocity: %f", forwardVelocity);

            if (forwardVelocity == 0) {
                // Stop the robot as it has reached the distance threshold
                // Stop all left and right motors using the array and a loop
                for (int i = 0; i < 3; i++) {
                leftMotor[i].stop(brake);  // Stop left motors
                rightMotor[i].stop(brake); // Stop right motors
                }

                Brain.Screen.printAt(10, 160, "Reached distance threshold");
                break;
            }

            // Drive the robot using the updated motor configuration
            double leftMotorSpeed = forwardVelocity + turnVelocity;
            double rightMotorSpeed = forwardVelocity - turnVelocity;

            // Debugging: Print motor speeds
          //  Brain.Screen.printAt(10, 160, "Left Motor Speed: %f", leftMotorSpeed);
           // Brain.Screen.printAt(10, 180, "Right Motor Speed: %f", rightMotorSpeed);

            // Use a loop to spin all left and right motors
        for (int i = 0; i < 3; i++) {
        // Spin left motors
        leftMotor[i].spin(fwd, leftMotorSpeed, pct);

        // Spin right motors
        rightMotor[i].spin(fwd, rightMotorSpeed, pct);
        }

        } else {
            // Object not found, drive forward
            Brain.Screen.printAt(10, 200, "Object not found, driving forward");

            // Use a loop to spin all left and right motors
        for (int i = 0; i < 3; i++) {
        // Spin left motors
        leftMotor[i].spin(fwd, 5, pct);

        // Spin right motors
        rightMotor[i].spin(fwd, 5, pct);
        }

        // Add a short delay to avoid overloading the CPU
        wait(20, msec);
    }
}
}
