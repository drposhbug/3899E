#include "vex.h"  
#ifndef ODOMETRY_H
#define ODOMETRY_H
#include "navigation.h"
#include "robot_config.h"
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

// Encoder State
extern bool xEncoderEnabled;    // X encoder enable/disable flag

// Function Declarations
void updateOdometry();          // Updates the robot's position and orientation

// Initialization
void setStartPosition(double startX = 0, double startY = 0, double startHeading = 0);

// Navigation Helper Functions
void calculatePathToTarget(double currentX, double currentY, 
                         double targetX, double targetY, 
                         double& distance, double& heading);

void turnToPoint(double targetX, double targetY,            
                double breakDistanceInDegrees = MotionDefaults::TurningLeft::BREAK_DISTANCE,
                double minSpeed = MotionDefaults::TurningLeft::MIN_SPEED,  
                double maxSpeed = MotionDefaults::TurningLeft::MAX_SPEED);

void turnLeftToPoint(double targetX, double targetY,
                    double breakDistanceInDegrees = MotionDefaults::TurningLeft::BREAK_DISTANCE,
                    double minSpeed = MotionDefaults::TurningLeft::MIN_SPEED,
                    double maxSpeed = MotionDefaults::TurningLeft::MAX_SPEED,
                    double exitTolerance = 0.5);

void turnRightToPoint(double targetX, double targetY,
                     double breakDistanceInDegrees = MotionDefaults::TurningRight::BREAK_DISTANCE,
                     double minSpeed = MotionDefaults::TurningRight::MIN_SPEED,
                     double maxSpeed = MotionDefaults::TurningRight::MAX_SPEED,
                        double exitTolerance = 0.5);


// In include/odometry.h

// In odometry.h

void forwardToPoint(double targetX, double targetY,             
               double breakDistance = 10,
               double minSpeed = 24,  
               double distanceTolerance = 5,
               double kp_heading = 1.1,
               double ki_heading = 0.0,
               double kd_heading = 0,
               double accelHeadingScaling = 0.1,
               double decelHeadingScaling = 0.1,
               double approachHeadingScaling = 0.3,
               double maxSpeed = 100);    

void backwardToPoint(double targetX, double targetY,             
               double breakDistance = 10,
               double minSpeed = 24,  
               double distanceTolerance = 5,
               double kp_heading = 1.1,
               double ki_heading = 0.0,
               double kd_heading = 0,
               double accelHeadingScaling = 0.1,
               double decelHeadingScaling = 0.1,
               double approachHeadingScaling = 0.3,
               double maxSpeed = 100);
               
// Odometry Task Management
struct OdometryTaskParams {
    bool isRunning;  // Flag to control task execution
};

int odometryTask(void *params);
extern OdometryTaskParams odometryParams;

void startOdometryTask();
void stopOdometryTask();

#endif // ODOMETRY_H