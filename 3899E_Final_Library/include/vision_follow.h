#ifndef VISION_FOLLOW_H
#define VISION_FOLLOW_H

#include "vex.h"

// Constants for vision following
namespace VisionFollow {
    // Screen center for the AI Vision sensor (320x240 resolution)
    constexpr double SCREEN_CENTER_X = 160.0;
    constexpr double SCREEN_CENTER_Y = 120.0;
    
    // PID tuning constants for following
    constexpr double TURN_KP = 0.4;  // Proportional gain for turning
    constexpr double TURN_KI = 0.0;  // Integral gain for turning
    constexpr double TURN_KD = 0.2;  // Derivative gain for turning
    
    constexpr double MOVE_KP = 0.3;  // Proportional gain for forward/backward
    constexpr double MOVE_KI = 0.0;  // Integral gain for moving
    constexpr double MOVE_KD = 0.1;  // Derivative gain for moving
    
    // Thresholds
    constexpr double TURN_DEADZONE = 10.0;    // Pixels - don't turn if error is less than this
    constexpr double MOVE_DEADZONE = 5.0;     // Pixels - don't move if error is less than this
    constexpr double MIN_OBJECT_SIZE = 50;    // Minimum object width to track
    constexpr double MAX_OBJECT_SIZE = 220;   // Maximum object width before backing up
    constexpr double STOP_DISTANCE = 80;      // Desired distance from object (pixels)
    
    constexpr double MAX_TURN_SPEED = 80.0;   // Max speed for turning (percentage)
    constexpr double MAX_MOVE_SPEED = 60.0;   // Max speed for moving (percentage)
    constexpr double MIN_MOTOR_SPEED = 10.0;  // Minimum motor speed to overcome friction
}

// Function declarations
void followRedBall();
void stopFollowing();
bool isRedBallDetected();

#endif // VISION_FOLLOW_H
