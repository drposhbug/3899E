/*----------------------------------------------------------------------------
 * ai.h — VAIRC-specific AI functions for the V5 Brain (Team 3899E)
 *
 * This file is VAIRC competition-specific. Contains everything the V5 Brain
 * needs to participate in a fully autonomous VAIRC match:
 *
 *   SECTION 1 — Strategy codes & class IDs
 *   SECTION 2 — JetsonDetection struct
 *   SECTION 3 — Jetson data accessors (getLatestDetection, getStrategy)
 *   SECTION 4 — Vision navigation (moveVisionOdometryAI)
 *   SECTION 5 — Navigation result types (NavResult, TargetType)
 *   SECTION 6 — Three-phase navigation wrapper (navigateToTarget)
 *   SECTION 7 — Strategy functions (one per game action)
 *   SECTION 8 — Match dispatch (runAIMatch, setStrategyCode)
 *   SECTION 9 — Behavior stubs (visionSweepNorth, etc.)
 *
 * Dependencies:
 *   jetson_comms.h  — AI_RECORD, DETECTION_OBJECT, g_jetson
 *   route_planner.h — RoutePath, routePlan(), routeExecute()
 *   robot_geometry.h — field constants, robot dimensions
 *----------------------------------------------------------------------------*/

#ifndef AI_H
#define AI_H

#include "jetson_comms.h"   // AI_RECORD, DETECTION_OBJECT, g_jetson
#include "route_planner.h"  // RoutePath, routePlan, routeExecute
#include "robot_geometry.h" // field constants
#include "robot_config.h"   // aiVision_orangeCap, aiVision_redCube, aiVision_blueCube

// ============================================================================
// SECTION 1 — Strategy codes & class IDs
// ============================================================================

// Strategy codes — dispatched by Jetson field-state model.
// Must match STRATEGY_* constants in jetson_ai.py on the Jetson side.
#define STRATEGY_IDLE             0
#define STRATEGY_SCORE_TOP        1   // Score into top long goal
#define STRATEGY_SCORE_BOTTOM     2   // Score into bottom long goal
#define STRATEGY_SCORE_CENTER     3   // Score into center goal
#define STRATEGY_DESCORE_TOP      4   // Descore opponent blocks from top goal
#define STRATEGY_DESCORE_BOTTOM   5   // Descore opponent blocks from bottom goal
#define STRATEGY_BLOCK_TOP        6   // Block opponent from top goal
#define STRATEGY_BLOCK_BOTTOM     7   // Block opponent from bottom goal
#define STRATEGY_USE_MATCH_LOADER 8   // Navigate to match loader and intake
#define STRATEGY_PARK             9   // Navigate to park zone
// Legacy codes — kept for backwards compatibility with existing jetson_ai.py
#define STRATEGY_SCORE_BLUE       1
#define STRATEGY_SCORE_RED        2
#define STRATEGY_DESCORE          4
#define STRATEGY_DEFEND           6
#define STRATEGY_SURVEY           0
#define STRATEGY_SKILLS_SEQ       100

// Class IDs — match YOLO label ordering in vaic_protocol.py
// Forward camera (e-CAM25_CUONX):
#define CLASS_FWD_BLUE_BLOCK    0
#define CLASS_FWD_RED_BLOCK     1
#define CLASS_FWD_GOAL          2
#define CLASS_FWD_ROBOT         3
#define CLASS_FWD_PARK_ZONE     4
// Survey camera (RealSense D435, Phase 2 only):
#define CLASS_SURVEY_OPP_ROBOT  10
#define CLASS_SURVEY_BLUE_BLOCK 11
#define CLASS_SURVEY_RED_BLOCK  12

// ============================================================================
// SECTION 2 — JetsonDetection struct
// ============================================================================

// Simplified detection result for navigation functions.
// Decouples callers from AI_RECORD internals.
struct JetsonDetection {
    float   hOffset;      // Normalized horizontal offset: -1.0=left, 0=center, +1.0=right
    float   distanceCm;   // Estimated distance to target (cm)
    int32_t classID;      // Object class (CLASS_FWD_* or CLASS_SURVEY_*)
    float   confidence;   // YOLO confidence 0.0–1.0
    float   mapX;         // Field X position (meters, Jetson map — VEX Coordinates, field center = 0,0)
    float   mapY;         // Field Y position (meters, Jetson map — VEX Coordinates, field center = 0,0)
    int32_t seqNum;       // Jetson frame counter — use for dedup in nav loops
};

// ============================================================================
// SECTION 3 — Jetson data accessors
// ============================================================================

/**
 * getLatestDetection — find best matching detection in latest AI_RECORD.
 * Returns true and fills *out on success.
 * Returns false if no packet received or no match found.
 *
 * @param classID        Target class (CLASS_FWD_RED_BLOCK etc.)
 * @param minConfidence  Minimum YOLO confidence to accept (0.0–1.0)
 * @param out            Filled on success — caller allocates
 */
bool getLatestDetection(int classID, float minConfidence, JetsonDetection* out);

/**
 * getStrategy — returns strategyCode from latest AI_RECORD.
 * Returns STRATEGY_IDLE (0) if no packet received yet.
 */
int32_t getStrategy(void);

// Jetson serial state accessors — populated by serial receive task
bool   jetsonObstacleDetected();   // true if Jetson flags an obstacle in forward path
double jetsonTargetDistance();     // distance estimate to tracked target (cm), -1 if none
bool   jetsonTargetTracked();      // true if Jetson currently has a target lock

// ============================================================================
// SECTION 4 — Vision navigation
// ============================================================================

/**
 * moveVisionOdometryAI — vision-guided autonomous movement using Jetson YOLO.
 *
 * Open-loop encoder distance tracking with Jetson YOLO heading correction.
 * Same LAUNCH→CRUISE→DECEL→APPROACH architecture as moveVisionOdometryOpen.
 * VEX AI Vision sensor replaced by g_jetson data.
 *
 * Key differences from moveVisionOdometryOpen:
 *   - Exit: Jetson distanceCm <= targetStopDistanceCm (not pixel width)
 *   - Frame dedup via seqNum (not centerX/width cache)
 *   - Bounding box filtering done on Jetson — no min/maxX/Y params
 *   - FOV: AI_CAM_DEG_PER_UNIT = 35° (not 25.5° VEX AI Vision)
 */
void moveVisionOdometryAI(int    objectClassID,
                          float  targetStopDistanceCm,
                          double targetX,
                          double targetY,
                          double breakDistance,
                          pros::motor_brake_mode_e_t brakeMode        = pros::E_MOTOR_BRAKE_COAST,
                          double maxSpeed                              = 75.0,
                          double kp_head                              = 0.1,
                          double ki_head                              = 0.0,
                          double kd_head                              = 0.0,
                          double kp_distToHeadScaling                 = 0.3,
                          double minSpeed                             = 16.0,
                          double accelHeadingScaling                  = 0.2,
                          double decelHeadingScaling                  = 0.2,
                          double approachHeadingScaling               = 0.2,
                          double headingLockDistance                  = 15.0,
                          double timeout                              = 3.0);

// ============================================================================
// SECTION 5 — Navigation result types
// ============================================================================

// What type of target the robot is navigating to.
// Controls blind approach behavior (power, stall detection, timeout).
enum class TargetType {
    LONG_GOAL,     // Rigid, fixed. Stall detection. Moderate blind power.
    CENTER_GOAL,   // Rigid, fixed. Same as long goal.
    MATCH_LOADER,  // Wall-mounted. Stall at wall = confirmed position.
    BLOCK,         // Light, moveable. Lower blind power, no stall detection.
    PARK_ZONE,     // Position only — no vision or blind phase.
};

// Result of a navigateToTarget() call.
// Strategy functions switch on this to decide the next action.
enum class NavResult {
    SUCCESS,          // Reached target cleanly
    BLIND_CONTACT,    // Stall detected — physically at target
    BLIND_TIMEOUT,    // Blind phase timed out — probably close enough
    VISION_LOST,      // Vision dropped before blind threshold
    BLOCKED_REROUTE,  // Timed out mid-navigation — obstacle, replan needed
};

// ============================================================================
// SECTION 6 — Three-phase navigation wrapper
// ============================================================================

/**
 * navigateToTarget — low-level three-phase navigation to raw XY coordinates.
 * Called internally by navigateTo() and strategy functions.
 * Can use a pre-planned RoutePath (pass count > 0) or plans internally.
 */
NavResult navigateToTarget(double goalX, double goalY,
                           TargetType target,
                           const RoutePath& precomputedPath = RoutePath{});

// Blind approach only — for when already in position for final push.
NavResult blindApproach(TargetType target);

// ============================================================================
// SECTION 7 — Strategy functions
// ============================================================================

// Each function is one complete robot action:
//   navigate to target → execute game mechanism → return

void strategyScoreTopGoal();       // Score blocks into top long goal
void strategyScoreBottomGoal();    // Score blocks into bottom long goal
void strategyScoreCenterGoal();    // Score blocks into center goal
void strategyDescoreTopGoal();     // Descore opponent blocks from top goal
void strategyDescoreBottomGoal();  // Descore opponent blocks from bottom goal
void strategyBlockTopGoal();       // Block opponent from top goal
void strategyBlockBottomGoal();    // Block opponent from bottom goal
void strategyUseMatchLoader();     // Navigate to match loader and intake
void strategyPark();               // Navigate to park zone

// Set alliance color at match start — determines which park zone to use
void setAllianceRed(bool isRed);

// Returns park zone X coordinate for active alliance
double getParkX();

// ============================================================================
// SECTION 8 — Match dispatch
// ============================================================================

/**
 * runAIMatch — top-level VAIRC match loop. Call from autonomous() in auton.cpp.
 *
 * Priority hierarchy evaluated each cycle:
 *   1. Match timer — dynamic park trigger (always wins, no override)
 *   2. Rules/safety — illegal strategy codes rejected locally (VAIG3/SG7)
 *   3. Jetson strategy code — normal operation
 *   4. Last known strategy — serial dropout fallback (V5 never goes idle)
 */
void runAIMatch();

// Update current strategy code — called by serial receive task (thread-safe)
void setStrategyCode(int code);

// Read current strategy code
int getStrategyCode();

// ============================================================================
// SECTION 9 — Behavior stubs
// ============================================================================

void visionSweepNorth();   // North-side block hoarding (to be implemented)

#endif // AI_H