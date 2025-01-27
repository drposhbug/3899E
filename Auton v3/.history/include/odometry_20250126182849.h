#include "vex.h"  
#ifndef ODOMETRY_H
#define ODOMETRY_H
#include "navigation.h"

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

// Function Declarations
void resetOdometry();           // Resets all odometry values to zero
void updateOdometry();          // Updates the robot's position and orientation

// Add these new functions:
// Navigation Helper Functions
void calculatePathToTarget(double currentX, double currentY, 
                         double targetX, double targetY, 
                         double& distance, double& heading);  // Calculates path parameters to target

void turnToPoint(double targetX, double targetY,            
                double breakDistanceInDegrees = 5.0,
                double minSpeed = 5.0,  
                double maxSpeed = 50);            

void straightToPoint(double targetX, double targetY,             
               double breakDistance = 1.0,                 // breakDistance
               double minSpeed = 10.0,  
               double kp_heading = 1.0,                    // kp_heading
               double ki_heading = 0.0,                    // ki_heading
               double kd_heading = 0.0,                    // kd_heading
               double accelHeadingScaling = 0.5,          // accelHeadingScaling
               double decelHeadingScaling = 1.0,          // decelHeadingScaling
               double approachHeadingScaling = 1.0,       // approachHeadingScaling                 
               double maxSpeed = 100);                    
               

#endif // ODOMETRY_H