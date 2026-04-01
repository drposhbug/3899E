#ifndef VISION_FOLLOW_H
#define VISION_FOLLOW_H

#include "main.h"   // PROS entry point

// ══════════════════════════════════════════════════════════════════════════════
// VISION FOLLOW — PID-based object tracking using the AI Vision Sensor
// Drives the robot toward a detected color object, keeping it centered.
// ══════════════════════════════════════════════════════════════════════════════

namespace VisionFollow {

    // AI Vision sensor resolution: 320 × 240 pixels.
    constexpr double SCREEN_CENTER_X = 160.0;
    constexpr double SCREEN_CENTER_Y = 120.0;

    // ── Turn PID (lateral centering) ─────────────────────────────────────────
    // Corrects horizontal offset between the detected object and screen center.
    constexpr double TURN_KP = 0.4;
    constexpr double TURN_KI = 0.0;
    constexpr double TURN_KD = 0.2;

    // ── Move PID (distance control) ──────────────────────────────────────────
    // Drives toward/away from the object to maintain a target pixel width.
    constexpr double MOVE_KP = 0.3;
    constexpr double MOVE_KI = 0.0;
    constexpr double MOVE_KD = 0.1;

    // ── Deadzones ─────────────────────────────────────────────────────────────
    constexpr double TURN_DEADZONE     = 10.0;  // px — ignore small horizontal errors
    constexpr double MOVE_DEADZONE     =  5.0;  // px — ignore small distance errors

    // ── Object size gates ─────────────────────────────────────────────────────
    constexpr double MIN_OBJECT_SIZE   =  50.0; // px — ignore detections smaller than this
    constexpr double MAX_OBJECT_SIZE   = 220.0; // px — back up if object is larger than this
    constexpr double STOP_DISTANCE     =  80.0; // px — desired object pixel-width (target distance)

    // ── Speed limits ──────────────────────────────────────────────────────────
    constexpr double MAX_TURN_SPEED    = 80.0;  // % — cap turn speed
    constexpr double MAX_MOVE_SPEED    = 60.0;  // % — cap forward/backward speed
    constexpr double MIN_MOTOR_SPEED   = 10.0;  // % — floor to overcome static friction

} // namespace VisionFollow

// Start following a red ball detected by the AI Vision sensor.
// Runs until stopFollowing() is called or the ball is lost.
void followRedBall();

// Signal the follow loop to stop on its next iteration.
void stopFollowing();

// Returns true if the AI Vision sensor currently detects a red ball.
bool isRedBallDetected();

#endif // VISION_FOLLOW_H
