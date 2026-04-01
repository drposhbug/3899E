#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "main.h"          // PROS entry point
#include "navigation.h"    // MotionDefaults namespace
#include "robot_config.h"

// ══════════════════════════════════════════════════════════════════════════════
// GLOBAL POSITION STATE
// Updated continuously by the odometry background task.
// Read from motion functions and auton routines; write only from updateOdometry().
// Units: centimeters for X/Y, degrees for rotation.
// ══════════════════════════════════════════════════════════════════════════════
extern double globalX;         // robot X position on the field (cm)
extern double globalY;         // robot Y position on the field (cm)
extern double globalRotation;  // cumulative (unbounded) rotation in degrees

// ── Previous encoder snapshots (used to compute deltas each iteration) ────────
extern double prevLeftEncoder;   // left tracking wheel position, previous cycle (degrees)
extern double prevRightEncoder;  // right tracking wheel position, previous cycle (degrees)
extern double prevXEncoder;      // lateral encoder position, previous cycle (degrees)
extern double prevRotation;      // gyro/encoder heading, previous cycle (degrees)

// ── Encoder configuration ─────────────────────────────────────────────────────
extern bool xEncoderEnabled;  // false = skip the lateral encoder in the odometry update

// ══════════════════════════════════════════════════════════════════════════════
// CORE ODOMETRY UPDATE
// Called each iteration of the odometry task (~10 ms).
// Reads current encoder values, computes delta, and updates globalX/Y/Rotation.
// ══════════════════════════════════════════════════════════════════════════════
void updateOdometry();

// ══════════════════════════════════════════════════════════════════════════════
// INITIALIZATION
// Set the robot's known position and heading before autonomous starts.
// Defaults to the field origin (0, 0) facing East (0°).
// ══════════════════════════════════════════════════════════════════════════════
void setStartPosition(double startX = 0.0, double startY = 0.0, double startHeading = 0.0);

// ══════════════════════════════════════════════════════════════════════════════
// NAVIGATION HELPERS
// Used internally by odometry-based motion functions.
// ══════════════════════════════════════════════════════════════════════════════

// Given current position and a target, compute straight-line distance (cm)
// and the absolute heading required to reach it (Standard Cartesian degrees).
void calculatePathToTarget(double currentX, double currentY,
                           double targetX,  double targetY,
                           double& distance, double& heading);

// ══════════════════════════════════════════════════════════════════════════════
// POINT-TO-POINT TURN HELPERS
// Turn the robot to face a field coordinate rather than an absolute heading.
// All three functions use the same motion profile (MotionDefaults::TurningLeft).
//
// turnToPoint   – chooses the shorter direction automatically.
// turnLeftToPoint  – forces a counter-clockwise turn.
// turnRightToPoint – forces a clockwise turn.
// ══════════════════════════════════════════════════════════════════════════════
void turnToPoint(double targetX, double targetY,
                 double breakDistanceInDegrees = MotionDefaults::TurningLeft::BREAK_DISTANCE,
                 double minSpeed               = MotionDefaults::TurningLeft::MIN_SPEED,
                 double maxSpeed               = MotionDefaults::TurningLeft::MAX_SPEED);

void turnLeftToPoint(double targetX, double targetY,
                     double breakDistanceInDegrees = MotionDefaults::TurningLeft::BREAK_DISTANCE,
                     double minSpeed               = MotionDefaults::TurningLeft::MIN_SPEED,
                     double maxSpeed               = MotionDefaults::TurningLeft::MAX_SPEED,
                     double exitTolerance          = 0.5);

void turnRightToPoint(double targetX, double targetY,
                      double breakDistanceInDegrees = MotionDefaults::TurningRight::BREAK_DISTANCE,
                      double minSpeed               = MotionDefaults::TurningRight::MIN_SPEED,
                      double maxSpeed               = MotionDefaults::TurningRight::MAX_SPEED,
                      double exitTolerance          = 0.5);

// ══════════════════════════════════════════════════════════════════════════════
// POINT-TO-POINT STRAIGHT DRIVE HELPERS
// Drive from the current position to a target field coordinate.
// forwardToPoint  – approach facing the target.
// backwardToPoint – approach with the robot reversed.
// ══════════════════════════════════════════════════════════════════════════════
void forwardToPoint(double targetX, double targetY,
                    double breakDistance          = 10.0,
                    double minSpeed               = 24.0,
                    double distanceTolerance      = 5.0,
                    double kp_heading             = 1.1,
                    double ki_heading             = 0.0,
                    double kd_heading             = 0.0,
                    double accelHeadingScaling    = 0.1,
                    double decelHeadingScaling    = 0.1,
                    double approachHeadingScaling = 0.3,
                    double maxSpeed               = 100.0);

void backwardToPoint(double targetX, double targetY,
                     double breakDistance          = 10.0,
                     double minSpeed               = 24.0,
                     double distanceTolerance      = 5.0,
                     double kp_heading             = 1.1,
                     double ki_heading             = 0.0,
                     double kd_heading             = 0.0,
                     double accelHeadingScaling    = 0.1,
                     double decelHeadingScaling    = 0.1,
                     double approachHeadingScaling = 0.3,
                     double maxSpeed               = 100.0);

// ══════════════════════════════════════════════════════════════════════════════
// ODOMETRY BACKGROUND TASK
// The task runs updateOdometry() in a tight loop (~10 ms period).
// OdometryTaskParams.isRunning is the stop flag: set false to cleanly exit.
// ══════════════════════════════════════════════════════════════════════════════
struct OdometryTaskParams {
    bool isRunning;  // task reads this each iteration; set false to stop
};

// PROS task function — void(void*) signature.
void odometryTask(void* params);

extern OdometryTaskParams odometryParams;

// Convenience wrappers: create / delete the PROS task.
void startOdometryTask();
void stopOdometryTask();

#endif // ODOMETRY_H
