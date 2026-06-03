#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "main.h"          // PROS entry point
#include "utils.h"         // Color enum and heading helpers
#include "motion_config.h" // StraightProfile, TurnProfile, VisionProfile
#include <atomic>

// ══════════════════════════════════════════════════════════════════════════════
// DRIVE PHASE — global readable by auto-calibration system.
// Written by each core motion function on every phase transition.
// Reset to PHASE_IDLE on function exit.
// ══════════════════════════════════════════════════════════════════════════════
enum DrivePhase { PHASE_IDLE, PHASE_LAUNCH, PHASE_CRUISE, PHASE_DECEL, PHASE_APPROACH };
extern DrivePhase currentDrivePhase;

// ══════════════════════════════════════════════════════════════════════════════
// VISION-ODOMETRY FUSION — SHARED STATE
// These atomics are written by the vision background task and read by motion
// functions, allowing lock-free data sharing between tasks.
// ══════════════════════════════════════════════════════════════════════════════
extern std::atomic<double> visionHorizontalNormalizedOffset;  // -1 to +1, left to right
extern std::atomic<int>    visionCurrentObjectWidth;          // pixels, 0 if not detected
extern std::atomic<bool>   visionTargetTracked;               // true if object in frame

double getCurrentEncoderDistanceCM();

// Background task that grabs AI Vision snapshots at ~50 Hz and updates the
// above atomics. Start with pros::Task before calling any visionDrive* function.
void visionTrackingTask(void* param);

// ══════════════════════════════════════════════════════════════════════════════
// OPEN-LOOP MOVEMENT
// ══════════════════════════════════════════════════════════════════════════════

// Drive at fixed speed for a set distance. No heading correction.
// reversed = true drives backward.
void move(double distanceCM, double maxSpeed, bool reversed = false);

// Same as move() but aborts early if both sides detect a wall stall.
// wallStalledTimeMs < 0 disables stall detection.
void smartMove(double distanceCM, double maxSpeed, bool reversed = false, double wallStalledTimeMs = -1);

// ══════════════════════════════════════════════════════════════════════════════
// PID-CORRECTED STRAIGHT DRIVE
// pidStraight holds a fixed heading via PID while driving a set distance.
// ══════════════════════════════════════════════════════════════════════════════
void pidStraight(double targetHeading,
                 double targetDistanceCM,
                 double speed,
                 double kp_heading     = 0.6,
                 double ki_heading     = 0.0,
                 double kd_heading     = 0.0,
                 double distanceOffset = 5.0,
                 pros::motor_brake_mode_e_t brakeMode = pros::E_MOTOR_BRAKE_BRAKE);

// ══════════════════════════════════════════════════════════════════════════════
// MOTION-PROFILED TURN
// Accelerates to maxSpeed then decelerates to minSpeed as the heading
// approaches the target, stopping within breakDistanceInDegrees.
// ══════════════════════════════════════════════════════════════════════════════
void turn(double targetHeading,
          double breakDistanceInDegrees,
          double minSpeed = 17.0,
          double maxSpeed = 100.0);

// ══════════════════════════════════════════════════════════════════════════════
// ARC TURN
// Drives both sides at different speeds to trace a radius-based arc.
// turnLeft = true → left side drives slower (arc left).
// ══════════════════════════════════════════════════════════════════════════════
void arcTurn(double targetDistance,
             double breakDistance,
             double minSpeed,
             double maxSpeed,
             double turnRadius,
             bool   turnLeft);

// ══════════════════════════════════════════════════════════════════════════════
// CORE STRAIGHT ENGINE
// Open-loop, encoder-relative. Does not use field position.
// All straight wrappers call this.
// ══════════════════════════════════════════════════════════════════════════════

// straightDistance — motion-profiled straight drive.
// targetDistance: cm to travel (positive = forward, negative = backward).
// targetHeading:  heading to hold (VEX Coordinates, North = 0°, CW+).
void straightDistance(double targetDistance,
                      double targetHeading,
                      const StraightProfile& p = DEFAULT_STRAIGHT);

// smartStraight — straightDistance with optional wall-stall abort.
// wallStalledTimeMs: −1 = disabled; > 0 = exit after this many ms of stall.
void smartStraight(double targetDistance,
                   double breakDistance,
                   double targetHeading     = 0.0,
                   double wallStalledTimeMs = 100.0,
                   const StraightProfile& p = DEFAULT_STRAIGHT);

// ══════════════════════════════════════════════════════════════════════════════
// STRAIGHT WRAPPERS
// driveForward / driveBackward — coder-friendly names for straightDistance.
// heading defaults to 0.0 (hold current heading).
// ══════════════════════════════════════════════════════════════════════════════
void driveForward(double targetDistance,
                  double targetHeading     = 0.0,
                  const StraightProfile& p = DEFAULT_STRAIGHT);

void driveBackward(double targetDistance,
                   double targetHeading     = 0.0,
                   const StraightProfile& p = BACKWARD_STRAIGHT);

// ══════════════════════════════════════════════════════════════════════════════
// POINT-TO-POINT TURN HELPERS
// Turn the robot to face a field coordinate rather than an absolute heading.
// All use TurnProfile — single source of truth for turn tuning.
//
// turnToPoint      — shortest path automatically
// turnLeftToPoint  — forces CCW turn
// turnRightToPoint — forces CW turn
// ══════════════════════════════════════════════════════════════════════════════
void turnToPoint(double targetX, double targetY,
                 const TurnProfile& p = DEFAULT_TURN);

void turnLeftToPoint(double targetX, double targetY,
                     const TurnProfile& p = DEFAULT_TURN);

void turnRightToPoint(double targetX, double targetY,
                      const TurnProfile& p = DEFAULT_TURN);

// ══════════════════════════════════════════════════════════════════════════════
// CORE ODOMETRY ENGINE
// Closed-loop — re-computes heading and distance to (x, y) each tick.
// forwardToPoint: forward approach.
// backwardToPoint: separate implementation — rear faces target, drives backward.
// ══════════════════════════════════════════════════════════════════════════════
void forwardToPoint(double targetX,
                    double targetY,
                    const StraightProfile& p = DEFAULT_STRAIGHT);

void backwardToPoint(double targetX,
                     double targetY,
                     const StraightProfile& p = BACKWARD_STRAIGHT);

// ══════════════════════════════════════════════════════════════════════════════
// CORE TURN ENGINE
// turnOdometry — point turn to absolute heading using motion profiling.
// Callers (turnLeft / turnRight) adjust targetHeading before passing it in.
// ══════════════════════════════════════════════════════════════════════════════
void turnOdometry(double targetHeading,
                  const TurnProfile& p = DEFAULT_TURN);

// ══════════════════════════════════════════════════════════════════════════════
// TURN WRAPPERS
// ══════════════════════════════════════════════════════════════════════════════

// turnRight — force a CW turn to an absolute field heading.
void turnRight(double absoluteTargetHeading,
               const TurnProfile& p = DEFAULT_TURN);

// turnLeft — force a CCW turn to an absolute field heading.
void turnLeft(double absoluteTargetHeading,
              const TurnProfile& p = DEFAULT_TURN);

// pivotTurnOdometryV2 — pivot turn (one side brakes, one side drives).
void pivotTurnOdometryV2(double targetHeading,
                          const TurnProfile& p = DEFAULT_PIVOT);

// pivotLeftMP / pivotRightMP — relative pivot turns.
// turnAmount: degrees to rotate from current heading.
void pivotLeftMP(double turnAmount,
                 const TurnProfile& p = DEFAULT_PIVOT);

void pivotRightMP(double turnAmount,
                  const TurnProfile& p = DEFAULT_PIVOT);

// ══════════════════════════════════════════════════════════════════════════════
// LAUNCH CONTROL (free function)
// ══════════════════════════════════════════════════════════════════════════════
double launchControl(double targetDriverSpeed, pros::Motor& motor, pros::Rotation& encoder);

// ══════════════════════════════════════════════════════════════════════════════
// tractionControl CLASS
// ══════════════════════════════════════════════════════════════════════════════
class tractionControl {
public:
    tractionControl(double minSpeedVoltage, double maxSpeedVoltage, double slipThreshold);
    double tractionControlSpeed(double tractionMotorVoltage,
                                double motorSpeed,
                                double robotSpeed,
                                double accelFactor);
private:
    double minSpeedVoltage;
    double maxSpeedVoltage;
    double slipThreshold;
};

// ══════════════════════════════════════════════════════════════════════════════
// adaptiveABS CLASS
// ══════════════════════════════════════════════════════════════════════════════
class adaptiveABS {
private:
    double lockThreshold;
    double decelStepVoltage;
    double lastAttemptedVoltage;
    bool   wasLockedLastCycle;
    pros::motor_brake_mode_e_t currentBrakeMode;

public:
    adaptiveABS(double decelStepPercent, double lockThreshold);
    void initialize(double startingVoltage);
    double decelControlSpeed(double wheelSpeed, double robotSpeed);
    pros::motor_brake_mode_e_t getBrakeMode() { return currentBrakeMode; }
};

// ══════════════════════════════════════════════════════════════════════════════
// driveToWall — drives toward a wall at low speed, detects stall, then stops.
// stalledSidePower: voltage to hold after stall (0 = release, >0 = keep pressing).
// ══════════════════════════════════════════════════════════════════════════════
void driveToWall(double targetDistance,
                 double targetHeading     = 0.0,
                 double minSpeed          = 15.0,
                 double wallStalledTimeMs = 150.0,
                 double stalledSidePower  = 0.0,
                 pros::motor_brake_mode_e_t brakeMode = pros::E_MOTOR_BRAKE_BRAKE,
                 double timeoutMs         = 3000.0,
                 double maxSpeed          = 40.0);

// ══════════════════════════════════════════════════════════════════════════════
// VISION — FORWARD/POINT VARIANTS
//
// visionForwardToPoint — closed-loop: fuses live odometry with vision heading.
//   reversed = true → drives backward toward target.
// visionBackwardToPoint — thin wrapper, calls visionForwardToPoint(reversed=true).
//
// visionDriveForward — open-loop: encoder distance + vision heading.
//   Vision steers once acquired; holds targetHeading before acquisition.
//   reversed = true → drives backward.
// visionDriveBackward — thin wrapper, calls visionDriveForward(reversed=true).
// ══════════════════════════════════════════════════════════════════════════════
void visionForwardToPoint(pros::vision_signature_s_t targetSignature,
                          int    targetPixelWidth,
                          double targetX,
                          double targetY,
                          const VisionProfile& p = DEFAULT_VISION,
                          bool   reversed        = false);

void visionBackwardToPoint(pros::vision_signature_s_t targetSignature,
                           int    targetPixelWidth,
                           double targetX,
                           double targetY,
                           const VisionProfile& p = DEFAULT_VISION);

void visionDriveForward(pros::vision_signature_s_t targetSignature,
                        int    targetPixelWidth,
                        double targetDistance,
                        double targetHeading     = 0.0,
                        const VisionProfile& p   = DEFAULT_VISION,
                        bool   reversed          = false);

void visionDriveBackward(pros::vision_signature_s_t targetSignature,
                         int    targetPixelWidth,
                         double targetDistance,
                         double targetHeading   = 0.0,
                         const VisionProfile& p = DEFAULT_VISION);

// ══════════════════════════════════════════════════════════════════════════════
// visionOnly — pure vision-guided approach, no odometry position updates.
//
// Pre-acquisition:  holds the entry gyro heading.
// Post-acquisition: corrects heading using vision lateral error only.
// Exit:             vision pixel-width >= targetPixelWidth (primary),
//                   encoder distance >= targetDistance (safety),
//                   or timeout elapsed.
// ══════════════════════════════════════════════════════════════════════════════
void visionOnly(pros::vision_signature_s_t targetSignature,
                int    targetPixelWidth,
                double targetDistance,
                const VisionProfile& p = DEFAULT_VISION);

// ══════════════════════════════════════════════════════════════════════════════
// visionDriveMinimal — simplified vision drive (individual params, no profile).
// Separate simpler architecture; not part of the profile system.
// Two overloads: color signature and color code.
// ══════════════════════════════════════════════════════════════════════════════
void visionDriveMinimal(
    pros::vision_signature_s_t targetSignature,
    int    targetPixelWidth,
    double targetHeading           = 0.0,
    double minSpeedPct             = 20.0,
    double maxSpeedPct             = 85.0,
    pros::motor_brake_mode_e_t brakeMode = pros::E_MOTOR_BRAKE_COAST,
    double kp_head                 = 0.20,
    double ki_head                 = 0.00,
    double kd_head                 = 0.00,
    double kp_distToHeadScaling    = 0.015,
    double kp_dist                 = 1.30,
    double ki_dist                 = 0.00,
    double kd_dist                 = 0.00);

void visionDriveMinimal(
    pros::vision_color_code_t targetSignature,
    int    targetPixelWidth,
    double targetHeading           = 0.0,
    double minSpeedPct             = 20.0,
    double maxSpeedPct             = 85.0,
    pros::motor_brake_mode_e_t brakeMode = pros::E_MOTOR_BRAKE_COAST,
    double kp_head                 = 0.20,
    double ki_head                 = 0.00,
    double kd_head                 = 0.00,
    double kp_distToHeadScaling    = 0.015,
    double kp_dist                 = 1.30,
    double ki_dist                 = 0.00,
    double kd_dist                 = 0.00);

// ══════════════════════════════════════════════════════════════════════════════
// visionDriveV2 — advanced AI Vision tracking with Priority Scaling.
// Separate architecture; not part of the profile system.
// ══════════════════════════════════════════════════════════════════════════════
void visionDriveV2(
    pros::vision_signature_s_t targetSignature,
    int    targetPixelWidth      = 60,
    double targetHeading         = 0.0,
    pros::motor_brake_mode_e_t brakeMode = pros::E_MOTOR_BRAKE_COAST,
    double maxSpeedPct           = 75.0,
    double kp_head               = 0.1,
    double ki_head               = 0.0,
    double kd_head               = 0.0,
    double kp_distToHeadScaling  = 0.3,
    int    minObjectWidth        = 10,
    int    minX                  = 0,
    int    maxX                  = 320,
    int    minY                  = 0,
    int    maxY                  = 240,
    double minSpeedPct           = 16.0,
    double timeoutDistanceCM     = 100.0,
    double kp_dist               = 1.50,
    double ki_dist               = 0.0,
    double kd_dist               = 0.0);

#endif // NAVIGATION_H