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
                    double maxSpeed = MotionDefaults::TurningLeft::MAX_SPEED);

void turnRightToPoint(double targetX, double targetY,
                     double breakDistanceInDegrees = MotionDefaults::TurningRight::BREAK_DISTANCE,
                     double minSpeed = MotionDefaults::TurningRight::MIN_SPEED,
                     double maxSpeed = MotionDefaults::TurningRight::MAX_SPEED);

void forwardToPoint(double targetX, double targetY,             
               double breakDistance = MotionDefaults::StraightForward::BREAK_DISTANCE,
               double minSpeed = MotionDefaults::StraightForward::MIN_SPEED,  
               double kp_heading = MotionDefaults::StraightForward::KP_HEADING,
               double ki_heading = MotionDefaults::StraightForward::KI_HEADING,
               double kd_heading = MotionDefaults::StraightForward::KD_HEADING,
               double accelHeadingScaling = MotionDefaults::StraightForward::ACCEL_HEADING_SCALING,
               double decelHeadingScaling = MotionDefaults::StraightForward::DECEL_HEADING_SCALING,
               double approachHeadingScaling = MotionDefaults::StraightForward::APPROACH_HEADING_SCALING,
               double maxSpeed = MotionDefaults::StraightForward::MAX_SPEED);    

void backwardToPoint(double targetX, double targetY,             
               double breakDistance = MotionDefaults::StraightBackward::BREAK_DISTANCE,
               double minSpeed = MotionDefaults::StraightBackward::MIN_SPEED,  
               double kp_heading = MotionDefaults::StraightBackward::KP_HEADING,
               double ki_heading = MotionDefaults::StraightBackward::KI_HEADING,
               double kd_heading = MotionDefaults::StraightBackward::KD_HEADING,
               double accelHeadingScaling = MotionDefaults::StraightBackward::ACCEL_HEADING_SCALING,
               double decelHeadingScaling = MotionDefaults::StraightBackward::DECEL_HEADING_SCALING,
               double approachHeadingScaling = MotionDefaults::StraightBackward::APPROACH_HEADING_SCALING,
               double maxSpeed = MotionDefaults::StraightBackward::MAX_SPEED);

// Odometry Task Management
struct OdometryTaskParams {
    bool isRunning;  // Flag to control task execution
};

int odometryTask(void *params);
extern OdometryTaskParams odometryParams;

void startOdometryTask();
void stopOdometryTask();

#endif // ODOMETRY_H