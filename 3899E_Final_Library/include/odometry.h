#include "vex.h"  
#ifndef ODOMETRY_H
#define ODOMETRY_H
#include "navigation.h"
#include <thread>
#include <chrono>

// Global Position Variables
extern double globalX;           // X-coordinate on the field
extern double globalY;           // Y-coordinate on the field
extern double globalRotation;    // Cumulative rotation in degrees

// Previous Encoder Values
extern double prevLeftEncoder;   // Previous left encoder reading
extern double prevRightEncoder;  // Previous right encoder reading
extern double prevXEncoder;      // Previous X encoder reading
extern double prevRotation;      // Previous rotation reading

// Function declarations for odometry task management
void startOdometryTask();
void stopOdometryTask();

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
                double breakDistanceInDegrees = 25.0,
                double minSpeed = 17.0,  
                double maxSpeed = 100.0);

void turnLeftToPoint(double targetX, double targetY,            
                double breakDistanceInDegrees = 25.0,
                double minSpeed = 17.0,  
                double maxSpeed = 100.0);

void turnRightToPoint(double targetX, double targetY,            
                double breakDistanceInDegrees = 25.0,
                double minSpeed = 17.0,  
                double maxSpeed = 100.0);

void forwardToPoint(double targetX, double targetY,             
               double breakDistance = 35.0,
               double minSpeed = 16.0,  
               double kp_heading = 0.615,
               double ki_heading = 0.0,
               double kd_heading = 0.0,
               double accelHeadingScaling = 0.10,
               double decelHeadingScaling = 0.05,
               double approachHeadingScaling = 0.05,
               double maxSpeed = 100.0);

void backwardToPoint(double targetX, double targetY,             
               double breakDistance = 30.0,
               double minSpeed = 15.0,  
               double kp_heading = 0.8,
               double ki_heading = 0.0,
               double kd_heading = 0.0,
               double accelHeadingScaling = 0.08,
               double decelHeadingScaling = 0.06,
               double approachHeadingScaling = 0.06,
               double maxSpeed = 80.0);

// Struct for odometry task parameters
struct OdometryTaskParams {
    bool isRunning;  // Flag to control task execution
};

// Function declaration for odometry task
int odometryTask(void *params);

// Global odometry task parameters
extern OdometryTaskParams odometryParams;


              

#endif // ODOMETRY_H