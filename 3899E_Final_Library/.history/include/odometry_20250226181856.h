#include "vex.h"  
#ifndef ODOMETRY_H
#define ODOMETRY_H
#include "navigation.h"
#include <thread>
#include <chrono>

// Global Position Variables
extern double globalX;           // X-coordinate on the field
extern double globalY;           // Y-coordinate on the field
extern double globalHeading;     // Orientation in degrees

// Previous Encoder Values
extern double prevLeftEncoder;   // Previous left encoder reading
extern double prevRightEncoder;  // Previous right encoder reading
extern double prevXEncoder;      // Previous X encoder reading
extern double prevHeading;       // Previous heading reading

// Add these function prototypes
void setStartPosition(double startX = 0, double startY = 0, double startHeading = 0);

// Encoder State
extern bool xEncoderEnabled;    // X encoder enable/disable flag

// Function Declarations
void updateOdometry();          // Updates the robot's position and orientation

// Add these new functions:
// Navigation Helper Functions
void calculatePathToTarget(double currentX, double currentY, 
                         double targetX, double targetY, 
                         double& distance, double& heading);  // Calculates path parameters to target

void turnToPoint(double targetX, double targetY,            
                double breakDistanceInDegrees,
                double minSpeed = 15.0,  
                double maxSpeed = 100);                    

void forwardToPoint(double targetX, double targetY,             
               double breakDistance,                 // breakDistance
               double minSpeed = 15.0,  
               double kp_heading = 0.2,                    // kp_heading
               double ki_heading = 0.0,                    // ki_heading
               double kd_heading = 0.0,                    // kd_heading
               double accelHeadingScaling = 0.4,          // accelHeadingScaling
               double decelHeadingScaling = 0.25,          // decelHeadingScaling
               double approachHeadingScaling = 0.25,       // approachHeadingScaling                 
               double maxSpeed = 100);    

void backwardToPoint(double targetX, double targetY,             
               double breakDistance,                 // breakDistance
               double minSpeed = 15.0,  
               double kp_heading = 100,                    // kp_heading
               double ki_heading = 0.0,                    // ki_heading
               double kd_heading = 0.0,                    // kd_heading
               double accelHeadingScaling = 0.0,          // accelHeadingScaling
               double decelHeadingScaling = 0.0,          // decelHeadingScaling
               double approachHeadingScaling = 0.0,       // approachHeadingScaling                 
               double maxSpeed = 100);                

// Struct for odometry task parameters
struct OdometryTaskParams {
    bool isRunning;  // Flag to control task execution
};

// Function declaration for odometry task
int odometryTask(void *params);

// Global odometry task parameters
extern OdometryTaskParams odometryParams;


              

#endif // ODOMETRY_H