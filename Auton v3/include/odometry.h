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

// Function Declarations
void resetOdometry();           // Resets all odometry values to zero
void updateOdometry();          // Updates the robot's position and orientation

#endif // ODOMETRY_H