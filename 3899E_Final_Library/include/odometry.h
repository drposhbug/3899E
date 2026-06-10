#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "main.h"          // PROS entry point
#include "robot_config.h"

// ══════════════════════════════════════════════════════════════════════════════
// GLOBAL POSITION STATE
// Updated continuously by the odometry background task.
// Read from motion functions and auton routines; write only from updateOdometry().
// Convention: VEX Coordinates (North=0degrees, Clockwise positive, origin at field center)
// Units: centimeters for X/Y, degrees for continuous rotation.
// ══════════════════════════════════════════════════════════════════════════════
extern double globalX;         // robot X position (cm)
extern double globalY;         // robot Y position (cm)
extern double globalRotation;  // continuous rotation (degrees)

// Encoder velocity computed from position deltas in updateOdometry().
// Use these instead of pros::Rotation::get_velocity() which is unreliable in PROS.
extern double globalLeftEncoderRPM;   // left tracking wheel velocity (RPM)
extern double globalRightEncoderRPM;  // right tracking wheel velocity (RPM)

// ── Previous encoder snapshots (used to compute deltas each iteration) ────────
extern double prevLeftEncoder;   // previous left encoder reading (degrees)
extern double prevRightEncoder;  // previous right encoder reading (degrees)
extern double prevXEncoder;      // previous lateral encoder reading (degrees)
extern double prevRotation;      // previous heading (degrees)

// ── Encoder configuration ─────────────────────────────────────────────────────
extern bool xEncoderEnabled;  // enable/disable lateral encoder

// ── GPS reset atomic handoff ──────────────────────────────────────────────────
// The GPS reset task writes here; updateOdometry() applies the correction safely
// between ticks to avoid racing with globalX += deltaXPos accumulation.
#include <atomic>
extern std::atomic<bool> pendingGpsReset;  // set by applyGpsReset(), cleared by updateOdometry()
void applyGpsReset(double newX_cm, double newY_cm);  // called by GPS task — do not call directly

// ══════════════════════════════════════════════════════════════════════════════
// CORE ODOMETRY UPDATE
// Called each iteration of the odometry task (~10 ms).
// Reads current encoder values, computes delta, and updates globalX/Y/Rotation.
// ══════════════════════════════════════════════════════════════════════════════
void updateOdometry();

// ══════════════════════════════════════════════════════════════════════════════
// INITIALIZATION
// Set the robot's known position and heading before autonomous starts.
// Defaults to the field origin (0, 0) facing North (0degrees).
// ══════════════════════════════════════════════════════════════════════════════
void setStartPosition(double startX = 0.0, double startY = 0.0, double startHeading = 0.0);

// ══════════════════════════════════════════════════════════════════════════════
// NAVIGATION HELPERS
// Used internally by odometry-based motion functions.
// ══════════════════════════════════════════════════════════════════════════════

// Given current position and a target, compute straight-line distance (cm)
// and the absolute heading required to reach it (VEX degrees, North=0degrees, CW+).
void calculatePathToTarget(double currentX, double currentY,
                           double targetX,  double targetY,
                           double& distance, double& heading);

// ── All point-to-point navigation functions (turnToPoint, forwardToPoint, etc.)
// are declared in navigation.h and implemented in navigation.cpp.
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