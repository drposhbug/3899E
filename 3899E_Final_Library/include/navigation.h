#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "main.h"    // PROS entry point
#include "utils.h"   // Color enum and heading helpers
#include <atomic>

// ══════════════════════════════════════════════════════════════════════════════
// VISION-ODOMETRY FUSION — SHARED STATE
// These atomics are written by the vision background task and read by motion
// functions, allowing lock-free data sharing between tasks.
// ══════════════════════════════════════════════════════════════════════════════
extern std::atomic<double> visionHorizontalNormalizedOffset; // −1.0 (left) … +1.0 (right)
extern std::atomic<int>    visionCurrentObjectWidth;         // pixels; 0 = not detected
extern std::atomic<bool>   visionTargetTracked;              // true while object is in frame

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
// brakeMode: COAST = roll to stop, BRAKE = immediate stop, HOLD = lock in place.
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
          double minSpeed  = 17.0,
          double maxSpeed  = 100.0);

// ──────────────────────────────────────────────────────────────────────────────
// straightOdometryV3 — motion-profiled straight drive using odometry distance
// tracking, with configurable distance tolerance, brake mode, and timeout guard.
// ──────────────────────────────────────────────────────────────────────────────
void straightOdometryV3(double targetDistance,
                        double breakDistance,
                        double targetHeading,
                        double minSpeed               = 16.0,
                        double distanceTolerance      = 2.0,
                        double kp_heading             = 0.4,
                        double ki_heading             = 0.01,
                        double kd_heading             = 0.05,
                        double accelHeadingScaling    = 0.2,
                        double decelHeadingScaling    = 0.2,
                        double approachHeadingScaling = 0.2,
                        double maxSpeed               = 100.0,
                        pros::motor_brake_mode_e_t brakeMode = pros::E_MOTOR_BRAKE_BRAKE,
                        double timeout                = 3.0);

// smartStraight — straightOdometry with optional wall-stall abort.
// wallStalledTimeMs: −1 = disabled; > 0 = exit after this many ms of stall.
void smartStraight(double targetDistance,
                   double breakDistance,
                   double targetHeading          = 0.0,
                   double minSpeed               = 16.0,
                   double wallStalledTimeMs       = 100.0,
                   double kp_heading             = 0.4,
                   double ki_heading             = 0.01,
                   double kd_heading             = 0.05,
                   double accelHeadingScaling    = 0.2,
                   double decelHeadingScaling    = 0.2,
                   double approachHeadingScaling = 0.2,
                   double maxSpeed               = 100.0);

// ══════════════════════════════════════════════════════════════════════════════
// ARC TURN
// Drives both sides at different speeds to trace a radius-based arc.
// turnLeft = true → left side drives slower (arc left).
// ══════════════════════════════════════════════════════════════════════════════
void arcTurn(double targetDistance,
             double breakDistance,
             double minSpeed,
             double maxSpeed,
             double turnRadius,  // arc radius in cm
             bool   turnLeft);

// ══════════════════════════════════════════════════════════════════════════════
// ODOMETRY TURN
// Turns to an absolute heading using the IMU and odometry for feedback.
// exitTolerance: degrees of error acceptable to declare turn complete.
// ══════════════════════════════════════════════════════════════════════════════
void turnOdometry(double targetHeading,
                  double breakDistanceInDegrees,
                  double minSpeed       = 25.0,
                  double maxSpeed       = 100.0,
                  double exitTolerance  = 16.0);

// ══════════════════════════════════════════════════════════════════════════════
// LAUNCH CONTROL (free function)
// Adjusts motor voltage to maintain targetDriverSpeed despite load.
// Returns the corrected voltage to apply.
// ══════════════════════════════════════════════════════════════════════════════
double launchControl(double targetDriverSpeed, pros::Motor& motor, pros::Rotation& encoder);

// ══════════════════════════════════════════════════════════════════════════════
// LaunchControl CLASS
// Per-motor slip-prevention controller: reduces voltage when the wheel spins
// faster than the robot (loss of traction).
// ══════════════════════════════════════════════════════════════════════════════
class LaunchControl {
public:
    // slipThresholdValue: ratio of wheel RPM to chassis RPM above which slip is declared.
    LaunchControl(pros::Motor& motor, pros::Rotation& encoder, double slipThresholdValue = 1.1);

    // Call each loop iteration with the requested power (0–100%).
    // Returns a potentially reduced power value that prevents wheelspin.
    double adjustSpeed(double targetPower);

private:
    pros::Motor&    motor;
    pros::Rotation& encoder;

    const double slipThreshold;   // e.g. 1.1 = allow 10% slip before correcting
    double motorRPM;              // last measured motor RPM
    double encoderRPMScaled;      // last measured encoder RPM (scaled to match motor units)
};

// ══════════════════════════════════════════════════════════════════════════════
// tractionControl CLASS
// Voltage-based traction limiter: reduces drive voltage proportionally when
// the wheel speed exceeds the estimated chassis speed by the slip threshold.
// ══════════════════════════════════════════════════════════════════════════════
class tractionControl {
public:
    tractionControl(double minSpeedVoltage, double maxSpeedVoltage, double slipThreshold);

    // Returns a corrected motor voltage.
    // tractionMotorVoltage – requested voltage
    // motorSpeed           – current wheel RPM
    // robotSpeed           – estimated chassis RPM from passive encoders
    // accelFactor          – scaling multiplier during the acceleration phase
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
// Anti-lock braking for the deceleration phase: progressively reduces voltage
// when wheels are locking up, guaranteeing minimum braking force.
// ══════════════════════════════════════════════════════════════════════════════
class adaptiveABS {
private:
    double lockThreshold;          // wheel/chassis speed ratio below which lockup is declared
    double decelStepVoltage;       // voltage reduction per step when lockup detected
    double lastAttemptedVoltage;   // voltage commanded last cycle
    bool   wasLockedLastCycle;     // true if lockup was detected in the previous cycle
    pros::motor_brake_mode_e_t currentBrakeMode;

public:
    adaptiveABS(double decelStepPercent, double lockThreshold);

    // Set the starting voltage at the beginning of a decel phase.
    void initialize(double startingVoltage);

    // Returns the voltage to apply this cycle.
    // wheelSpeed  – current wheel RPM
    // robotSpeed  – chassis (encoder) RPM
    double decelControlSpeed(double wheelSpeed, double robotSpeed);

    pros::motor_brake_mode_e_t getBrakeMode() { return currentBrakeMode; }
};

// ══════════════════════════════════════════════════════════════════════════════
// HIGH-LEVEL MOTION PROFILE WRAPPERS
// These wrap straightOdometryV3 / turnOdometry with robot-specific tuning so
// most autonomous routines only need one parameter: the distance or angle.
// ══════════════════════════════════════════════════════════════════════════════

// Forward / backward with default tuning from MotionDefaults::StraightForward.
void forwardMP(double targetDistance,
               double breakDistance          = 35.0,
               double targetHeading          = 0.0,
               double minSpeed               = 16.0,
               double kp_heading             = 0.615,
               double ki_heading             = 0.0,
               double kd_heading             = 0.0,
               double accelHeadingScaling    = 0.10,
               double decelHeadingScaling    = 0.05,
               double approachHeadingScaling = 0.05,
               double maxSpeed               = 100.0);

void backwardMP(double targetDistance,
                double breakDistance          = 35.0,
                double targetHeading          = 0.0,
                double minSpeed               = 16.0,
                double kp_heading             = 0.615,
                double ki_heading             = 0.0,
                double kd_heading             = 0.0,
                double accelHeadingScaling    = 0.10,
                double decelHeadingScaling    = 0.05,
                double approachHeadingScaling = 0.05,
                double maxSpeed               = 100.0);

// ══════════════════════════════════════════════════════════════════════════════
// PIVOT TURNS (one side stationary)
// V2: continuous headings, target-snapping, motion profiling — recommended.
// ══════════════════════════════════════════════════════════════════════════════
// V2 — exits when within exitTolerance degrees of the target.
void pivotTurnOdometryV2(double targetHeading,
                          double breakDistanceInDegrees,
                          double minSpeed,
                          double maxSpeed,
                          double exitTolerance = 2.0);

void pivotLeftMP(double turnAmount, double breakDistance, double minSpeed, double maxSpeed);
void pivotRightMP(double turnAmount, double breakDistance, double minSpeed, double maxSpeed);

// ══════════════════════════════════════════════════════════════════════════════
// driveForward / driveBackward — odometry-closed-loop straight drive
// V1: basic version.  V2: adds distanceTolerance + integral.  V3: zero integral.
// ══════════════════════════════════════════════════════════════════════════════
void driveForward(double targetDistance,
                  double breakDistance          = 10.0,
                  double targetHeading          = 0.0,
                  double minSpeed               = 24.0,
                  double kp_heading             = 1.1,
                  double ki_heading             = 0.0,
                  double kd_heading             = 0.0,
                  double accelHeadingScaling    = 0.1,
                  double decelHeadingScaling    = 0.2,
                  double approachHeadingScaling = 0.3,
                  double maxSpeed               = 100.0);

void driveBackward(double targetDistance,
                   double breakDistance          = 10.0,
                   double targetHeading          = 0.0,
                   double minSpeed               = 24.0,
                   double kp_heading             = 1.1,
                   double ki_heading             = 0.0,
                   double kd_heading             = 0.0,
                   double accelHeadingScaling    = 0.1,
                   double decelHeadingScaling    = 0.2,
                   double approachHeadingScaling = 0.3,
                   double maxSpeed               = 100.0);

// Absolute-heading turn wrappers (targetHeading is a field-frame heading, not relative).
void turnRight(double absoluteTargetHeading,
               double breakDistance,
               double minSpeed      = 25.0,
               double maxSpeed      = 100.0,
               double exitTolerance = 2.0);

void turnLeft(double absoluteTargetHeading,
              double breakDistance,
              double minSpeed      = 25.0,
              double maxSpeed      = 100.0,
              double exitTolerance = 2.0);

// Open-loop forward drive for a fixed duration (no sensors — use sparingly).
void pidlessForward(double timeMs, double speedPct);

// ── V2 ────────────────────────────────────────────────────────────────────────
void driveForwardV2(double targetDistance,
                    double breakDistance          = 10.0,
                    double targetHeading          = 0.0,
                    double minSpeed               = 24.0,
                    double distanceTolerance      = 5.0,
                    double kp_heading             = 1.1,
                    double ki_heading             = 0.005,
                    double kd_heading             = 0.0,
                    double accelHeadingScaling    = 0.1,
                    double decelHeadingScaling    = 1.0,
                    double approachHeadingScaling = 0.3,
                    double maxSpeed               = 100.0);

void driveBackwardV2(double targetDistance,
                     double breakDistance          = 10.0,
                     double targetHeading          = 0.0,
                     double minSpeed               = 24.0,
                     double distanceTolerance      = 5.0,
                     double kp_heading             = 1.1,
                     double ki_heading             = 0.005,
                     double kd_heading             = 0.0,
                     double accelHeadingScaling    = 0.1,
                     double decelHeadingScaling    = 1.0,
                     double approachHeadingScaling = 0.3,
                     double maxSpeed               = 100.0);

// ══════════════════════════════════════════════════════════════════════════════
// visionDriveMinimal — simplified vision drive (no timeout distance parameter).
// Two overloads: one for color signatures, one for color codes (multi-signature).
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
    double kp_distToHeadScaling    = 0.015,  // blend factor: how much pixel-width error feeds into heading
    double kp_dist                 = 1.30,
    double ki_dist                 = 0.00,
    double kd_dist                 = 0.00
);

void visionDriveMinimal(
    pros::vision_color_code_t targetSignature,   // multi-color code variant
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
    double kd_dist                 = 0.00
);

// ══════════════════════════════════════════════════════════════════════════════
// visionDriveV2 — streamlined vision drive using heading-scaling instead of
// separate distance PID; better for objects with irregular pixel-width curves.
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
    double kd_dist               = 0.0
);

// ══════════════════════════════════════════════════════════════════════════════
// moveOdometry — closed-loop point-to-point drive using live odometry.
// Re-computes heading and remaining distance each loop iteration.
//
// headingLockDistance: within this many cm, heading is frozen to reduce
//   oscillation during the final approach.
// ══════════════════════════════════════════════════════════════════════════════
void moveOdometry(double targetX,
                  double targetY,
                  double breakDistance,
                  double minSpeed               = 16.0,
                  double distanceTolerance      = 2.0,
                  double kp_heading             = 0.4,
                  double ki_heading             = 0.01,
                  double kd_heading             = 0.05,
                  pros::motor_brake_mode_e_t brakeMode = pros::E_MOTOR_BRAKE_BRAKE,
                  double accelHeadingScaling    = 0.2,
                  double decelHeadingScaling    = 0.2,
                  double approachHeadingScaling = 0.2,
                  double maxSpeed               = 100.0,
                  double headingLockDistance    = 8.0,
                  double timeout                = 3.0);

// ══════════════════════════════════════════════════════════════════════════════
// moveVisionOdometry — fuses live odometry with AI Vision heading correction.
// Uses odometry for distance/phase gating and vision for lateral alignment.
// Vision exit: object pixel-width >= targetPixelWidth.
// Odometry exit: robot reaches targetX/Y within distanceTolerance.
// ══════════════════════════════════════════════════════════════════════════════
void moveVisionOdometry(
    pros::vision_signature_s_t targetSignature,
    int    targetPixelWidth,
    double targetX,
    double targetY,
    double breakDistance,
    pros::motor_brake_mode_e_t brakeMode      = pros::E_MOTOR_BRAKE_COAST,
    double maxSpeed                            = 100.0,
    double kp_head                             = 0.1,
    double ki_head                             = 0.0,
    double kd_head                             = 0.0,
    double kp_distToHeadScaling                = 0.3,
    int    minObjectWidth                      = 10,
    int    minX                                = 0,
    int    maxX                                = 320,
    int    minY                                = 0,
    int    maxY                                = 240,
    double minSpeed                            = 16.0,
    double distanceTolerance                   = 2.0,
    double accelHeadingScaling                 = 0.2,
    double decelHeadingScaling                 = 0.2,
    double approachHeadingScaling              = 0.2,
    double headingLockDistance                 = 15.0,
    double timeout                             = 3.0);

// ══════════════════════════════════════════════════════════════════════════════
// driveToWall — drives toward a wall at low speed, detects stall, then stops.
// stalledSidePower: voltage to hold after stall (0 = release, >0 = continue pressing).
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
// moveVisionOdometryOpen — like moveVisionOdometry but uses open-loop encoder
// distance (computed once from targetX/Y at entry) instead of re-querying
// odometry each iteration. Lighter CPU load; useful when odometry drifts.
// ══════════════════════════════════════════════════════════════════════════════
void moveVisionOdometryOpen(
    pros::vision_signature_s_t targetSignature,
    int    targetPixelWidth,
    double targetX,
    double targetY,
    double breakDistance,
    pros::motor_brake_mode_e_t brakeMode      = pros::E_MOTOR_BRAKE_COAST,
    double maxSpeed                            = 100.0,
    double kp_head                             = 0.1,
    double ki_head                             = 0.0,
    double kd_head                             = 0.0,
    double kp_distToHeadScaling                = 0.3,
    int    minObjectWidth                      = 10,
    int    minX                                = 0,
    int    maxX                                = 320,
    int    minY                                = 0,
    int    maxY                                = 240,
    double minSpeed                            = 16.0,
    double distanceTolerance                   = 2.0,
    double accelHeadingScaling                 = 0.2,
    double decelHeadingScaling                 = 0.2,
    double approachHeadingScaling              = 0.2,
    double headingLockDistance                 = 15.0,
    double timeout                             = 3.0);

// ══════════════════════════════════════════════════════════════════════════════
// visionOnly — pure vision-guided approach, no odometry position updates.
//
// Pre-acquisition:  holds the entry gyro heading.
// Post-acquisition: corrects heading using vision lateral error only.
// Exit:             vision pixel-width >= targetPixelWidth (primary),
//                   encoder distance >= targetDistance (safety),
//                   or timeout elapsed.
// ══════════════════════════════════════════════════════════════════════════════
void visionOnly(
    pros::vision_signature_s_t targetSignature,
    int    targetPixelWidth,
    double targetDistance,       // max encoder travel (cm) — safety exit if vision never locks
    double breakDistance,        // encoder distance at which decel phase begins (cm)
    pros::motor_brake_mode_e_t brakeMode     = pros::E_MOTOR_BRAKE_COAST,
    double maxSpeed                           = 100.0,
    double kp_head                            = 0.1,
    double ki_head                            = 0.0,
    double kd_head                            = 0.0,
    double kp_distToHeadScaling               = 0.3,  // 0.0 = gentle arc, 1.0 = snap to target
    int    minObjectWidth                     = 10,
    int    minX                               = 0,
    int    maxX                               = 320,
    int    minY                               = 0,
    int    maxY                               = 240,
    double minSpeed                           = 16.0,
    double accelHeadingScaling                = 0.2,
    double decelHeadingScaling                = 0.2,
    double approachHeadingScaling             = 0.2,
    double timeout                            = 3.0);

#endif // NAVIGATION_H
