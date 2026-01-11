#ifndef ODOMETRY_HPP
#define ODOMETRY_HPP

#include "main.h"

// Global Position Variables
extern double globalX;           // X-coordinate on the field
extern double globalY;           // Y-coordinate on the field
extern double globalRotation;    // Cumulative rotation in degrees

// Previous Encoder Values
extern double prevLeftEncoder;   // Previous left encoder reading
extern double prevRightEncoder;  // Previous right encoder reading
extern double prevXEncoder;      // Previous X encoder reading
extern double prevRotation;      // Previous rotation reading

// Encoder State
extern bool xEncoderEnabled;    // X encoder enable/disable flag

// Function Declarations
void updateOdometry();          // Updates the robot's position and orientation

// Navigation Helper Functions
void calculatePathToTarget(double currentX, double currentY, 
                         double targetX, double targetY, 
                         double& distance, double& heading);  // Calculates path parameters to target

void turnToPoint(double targetX, double targetY,            
                double breakDistanceInDegrees,
                double minSpeed,  
                double maxSpeed);

void turnLeftToPoint(double targetX, double targetY,
                    double breakDistanceInDegrees,
                    double minSpeed,
                    double maxSpeed);

void turnRightToPoint(double targetX, double targetY,
                     double breakDistanceInDegrees,
                     double minSpeed,
                     double maxSpeed);

void forwardToPoint(double targetX, double targetY,             
               double breakDistance,
               double minSpeed,  
               double kp_heading,
               double ki_heading,
               double kd_heading,
               double accelHeadingScaling,
               double decelHeadingScaling,
               double approachHeadingScaling,
               double maxSpeed);    

void backwardToPoint(double targetX, double targetY,             
               double breakDistance,
               double minSpeed,  
               double kp_heading,
               double ki_heading,
               double kd_heading,
               double accelHeadingScaling,
               double decelHeadingScaling,
               double approachHeadingScaling,
               double maxSpeed);

// Struct for odometry task parameters
struct OdometryTaskParams {
    bool isRunning;  // Flag to control task execution
};

// Function declaration for odometry task
void odometryTask(void* params);

// Global odometry task parameters
extern OdometryTaskParams odometryParams;

// Function to set custom starting position and heading
void setStartPosition(double startX = 0, double startY = 0, double startHeading = 0);

// Task management functions
void startOdometryTask();
void stopOdometryTask();

#endif // ODOMETRY_HPP
