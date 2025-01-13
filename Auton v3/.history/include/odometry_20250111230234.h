#include "vex.h"  
#ifndef ODOMETRY_H
#define ODOMETRY_H

// Global Position Variables
extern double globalX;           // X-coordinate on the field
extern double globalY;           // Y-coordinate on the field
extern double globalHeading;     // Orientation in degrees

// Previous Encoder Values
extern double prevLeftEncoder;   // Previous left encoder reading
extern double prevRightEncoder;  // Previous right encoder reading
extern double prevXEncoder;      // Previous X encoder reading
extern double prevHeading;       // Previous heading reading

// Encoder State
extern bool xEncoderEnabled;    // X encoder enable/disable flag

// Basic Odometry Functions
void resetOdometry();           // Resets all odometry values to zero
void updateOdometry();          // Updates the robot's position and orientation

// Navigation Helper Functions
void calculatePathToTarget(double currentX, double currentY, 
                         double targetX, double targetY, 
                         double& distance, double& heading);  // Calculates path parameters to target

// Navigation Movement Functions
void turnToPoint(double targetX, double targetY,            // Turn to face a point
                double turnSpeed = 50, double tolerance = 5.0, 
                double minSpeed = 5.0, double slipThreshold = 1.1, 
                double accelFactor = 1.4);

void straightTo(double targetX, double targetY,             // Move straight to a point
               double straightSpeed = 70, double stopTolerance = 1.0,
               double kp_heading = 1.0, double ki_heading = 0.0, 
               double kd_heading = 0.0, double accelHeadingScaling = 0.5,
               double decelHeadingScaling = 1.0, double minSpeed = 10.0);

#endif // ODOMETRY_H