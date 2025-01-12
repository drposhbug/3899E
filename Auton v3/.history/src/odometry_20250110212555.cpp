#include "odometry.h"
#include "robot-config.h"
#include "utils.h"
#include <cmath>

using namespace vex;

// Global Odometry Variables
double globalX = 0.0;           // Tracks the robot's X-coordinate on the field
double globalY = 0.0;           // Tracks the robot's Y-coordinate on the field
double globalHeading = 0.0;     // Tracks the robot's orientation (in degrees)

// Previous Encoder Readings (used to calculate delta changes)
double prevLeftEncoder = 0.0;   // Stores the last reading from the left encoder
double prevRightEncoder = 0.0;  // Stores the last reading from the right encoder
double prevXEncoder = 0.0;      // Stores the last reading from the X encoder (if enabled)
double prevHeading = 0.0;       // Stores the last heading from the inertial sensor

// Encoder State Management
bool xEncoderEnabled = true;   // Determines whether the X encoder is active (disabled during spot turns)

// Reset function for zeroing odometry
void resetOdometry() {
    globalX = 0.0;
    globalY = 0.0;
    globalHeading = 0.0;
    
    // Store current encoder values as new reference points
    prevLeftEncoder = passiveEncoderLeft.position(vex::rotationUnits::deg);
    prevRightEncoder = passiveEncoderRight.position(vex::rotationUnits::deg);
    prevXEncoder = passiveEncoderX.position(vex::rotationUnits::deg);
    prevHeading = InertialSensor.heading();
}

// Function to Update Odometry Readings
void updateOdometry() {
    // Step 1: Read Encoder and Inertial Sensor Values
    double leftEncoder = passiveEncoderLeft.position(vex::rotationUnits::deg);   // Current left encoder value in degrees
    double rightEncoder = passiveEncoderRight.position(vex::rotationUnits::deg); // Current right encoder value in degrees
    double xEncoder = xEncoderEnabled ? passiveEncoderX.position(vex::rotationUnits::deg) : prevXEncoder; // Read from X encoder if enabled
    double currentHeading = InertialSensor.heading(); // Current heading from the inertial sensor in degrees

    // Step 2: Calculate Delta Changes
    double deltaLeft = leftEncoder - prevLeftEncoder;   // Change in left encoder value
    double deltaRight = rightEncoder - prevRightEncoder; // Change in right encoder value
    double deltaX = xEncoder - prevXEncoder;           // Change in X encoder value (if enabled)
    double deltaHeading = normalizeHeading(currentHeading - prevHeading); // Change in heading (normalized)

    // Step 3: Update Previous Values for the Next Cycle
    prevLeftEncoder = leftEncoder;    // Store current left encoder value for the next update
    prevRightEncoder = rightEncoder;  // Store current right encoder value for the next update
    prevXEncoder = xEncoder;          // Store current X encoder value for the next update
    prevHeading = currentHeading;     // Store current heading for the next update

    // Step 4: Calculate Average Distance Traveled
    // Average distance moved by the robot based on both left and right encoders
    double avgDeltaDistance = ((deltaLeft + deltaRight) / 2.0) * (encoderWheelCircumferenceCM / 360.0);

    // Step 5: Calculate Movement Components (ΔX and ΔY)
    // Cache trigonometric calculations for better performance
    double headingRad = globalHeading * (M_PI / 180.0);
    double cosHeading = cos(headingRad);
    double sinHeading = sin(headingRad);

    // Convert average distance into X and Y changes using cached trig values
    double deltaXPos = avgDeltaDistance * cosHeading; // X-axis movement component
    double deltaYPos = avgDeltaDistance * sinHeading; // Y-axis movement component

    // Step 6: Update Global Position
    globalX += deltaXPos; // Update global X position
    globalY += deltaYPos; // Update global Y position
    globalHeading = normalizeHeading(currentHeading); // Update global heading and ensure it stays normalized

    // Step 7: Debugging Information (Displayed on Brain Screen)
    Brain.Screen.printAt(10, 20, "X: %.2f, Y: %.2f, Heading: %.2f", globalX, globalY, globalHeading);
    Brain.Screen.printAt(10, 40, "DeltaLeft: %.2f, DeltaRight: %.2f", deltaLeft, deltaRight);
    Brain.Screen.printAt(10, 60, "DeltaHeading: %.2f", deltaHeading);
}