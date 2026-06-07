// ======================================================================
// field_targets.cpp — Named target table for VAIRC Push Back (Team 3899E)
//
// TWO TABLES — selected at compile time via ACTIVE_BOT in robot_geometry.h:
//   BOT_24INCH: front-facing for all targets (Jetson YOLO, front AI Vision)
//   BOT_15INCH: rear-facing for goals, front-facing for loaders
//
// To switch bots: change ACTIVE_BOT in robot_geometry.h, run pros make.
//
// All field coordinates in VEX GPS cm (origin = field center,
// X east positive, Y north positive). Verified against VEX Field Ref
// Spec 276-9142 via pushback_field_map_v15.html.
//
// Approach headings use VEX convention: North = 0°, clockwise positive.
//   North=0°  East=90°  South=180°  West=270°
//
// ⚠ Standoff distances are estimates — tune after field testing.
// ⚠ Verify all coordinates against game manual Appendix A before competition.
// ======================================================================

#include "field_targets.h"
#include "ai.h"              // TargetType full definition
#include "robot_geometry.h"  // RobotGeometry namespace constants
#include "odometry.h"        // gpsReset
#include "robot_config.h"    // gpsSensor, GPS_MAX_ERROR_M
#include <cmath>

using namespace RobotGeometry;

// ══════════════════════════════════════════════════════════════════════════════
// APPROACH STANDOFF DISTANCES (cm)
// Tune after field testing — must clear A* obstacle buffer + robot half-depth.
// ══════════════════════════════════════════════════════════════════════════════
static const double LONG_GOAL_STANDOFF_CM    = 50.0;
static const double CENTER_GOAL_STANDOFF_CM  = 55.0;
static const double MATCH_LOADER_STANDOFF_CM = 45.0;

// ══════════════════════════════════════════════════════════════════════════════
// FIELD ELEMENT COORDINATES — from robot_geometry.h (authoritative)
//
// Long goals run along X axis:
//   Y = ±122cm (LONG_GOAL_Y_TOP / LONG_GOAL_Y_BOT)
//   X span: ±62cm (LONG_GOAL_HALF_W)
//
// Match loader posts at X=±183cm, Y=±122cm (ML_POSTS[])
// ══════════════════════════════════════════════════════════════════════════════
static const double LONG_GOAL_X_EAST  =  LONG_GOAL_HALF_W;   //  62.0cm
static const double LONG_GOAL_X_WEST  = -LONG_GOAL_HALF_W;   // -62.0cm
static const double LONG_GOAL_Y_NORTH =  LONG_GOAL_Y_TOP;    // +122.0cm
static const double LONG_GOAL_Y_SOUTH =  LONG_GOAL_Y_BOT;    // -122.0cm

static const double ML_X_EAST  =  ML_POSTS[3].x;   // +183.0cm
static const double ML_X_WEST  =  ML_POSTS[0].x;   // -183.0cm
static const double ML_Y_NORTH =  ML_POSTS[0].y;   // +122.0cm
static const double ML_Y_SOUTH =  ML_POSTS[1].y;   // -122.0cm

// Center goal diagonal offset: standoff / sqrt(2)
static const double CG_DIAG = CENTER_GOAL_STANDOFF_CM * 0.70710678;

// ══════════════════════════════════════════════════════════════════════════════
// FIELD TARGET TABLE
//
// Two tables — one per bot. Selected at compile time via ACTIVE_BOT.
// To switch bots: change ACTIVE_BOT in robot_geometry.h, recompile.
//
// 24" bot — front-facing for everything (Jetson YOLO, front AI Vision).
//   Long goals: approach along Y axis, face N or S into goal opening.
//   Match loaders: approach from infield, face E or W toward post.
//   Center goals: diagonal approach, Jetson steers final phase.
//
// 15" bot — front for loaders, rear for goals (rear AI Vision).
//   Long goals: approach along X axis offset, face E or W so rear faces goal.
//   Match loaders: same as 24" bot — front faces post.
//   Center goals: diagonal approach, GPS reset + odometry.
//
// Order must match TargetID enum exactly — indexed directly by ID.
// ══════════════════════════════════════════════════════════════════════════════

#if ACTIVE_BOT == BOT_24INCH

// ── 24" BOT TABLE ─────────────────────────────────────────────────────────────
// Long goals: robot approaches from infield (center side), front faces into opening.
//   GOAL_NE/NW — north end of goal, approach from south (infield), face South (180°)
//   GOAL_SE/SW — south end of goal, approach from north (infield), face North (0°)
//
// Match loaders: front faces post.
//   East posts  → face East  (90°)
//   West posts  → face West  (270°)
const NamedTarget FIELD_TARGETS[TARGET_COUNT] = {

    // ── East long goal ────────────────────────────────────────────────────────
    {   // GOAL_NE — north end, approach from south (infield), face South into opening
        GOAL_NE, TargetType::LONG_GOAL,
        LONG_GOAL_X_EAST, LONG_GOAL_Y_NORTH - LONG_GOAL_STANDOFF_CM, 180.0,
        LONG_GOAL_X_EAST, LONG_GOAL_Y_NORTH
    },
    {   // GOAL_SE — south end, approach from north (infield), face North into opening
        GOAL_SE, TargetType::LONG_GOAL,
        LONG_GOAL_X_EAST, LONG_GOAL_Y_SOUTH + LONG_GOAL_STANDOFF_CM, 0.0,
        LONG_GOAL_X_EAST, LONG_GOAL_Y_SOUTH
    },

    // ── West long goal ────────────────────────────────────────────────────────
    {   // GOAL_NW — north end, approach from south (infield), face South into opening
        GOAL_NW, TargetType::LONG_GOAL,
        LONG_GOAL_X_WEST, LONG_GOAL_Y_NORTH - LONG_GOAL_STANDOFF_CM, 180.0,
        LONG_GOAL_X_WEST, LONG_GOAL_Y_NORTH
    },
    {   // GOAL_SW — south end, approach from north (infield), face North into opening
        GOAL_SW, TargetType::LONG_GOAL,
        LONG_GOAL_X_WEST, LONG_GOAL_Y_SOUTH + LONG_GOAL_STANDOFF_CM, 0.0,
        LONG_GOAL_X_WEST, LONG_GOAL_Y_SOUTH
    },

    // ── Center goals — diagonal approach, Jetson steers ──────────────────────
    {   // CENTER_NE
        CENTER_NE, TargetType::CENTER_GOAL,
        CG_DIAG,  CG_DIAG, 45.0,
        CENTER_GOAL_HALF_EXTENT * 0.70710678,  CENTER_GOAL_HALF_EXTENT * 0.70710678
    },
    {   // CENTER_SE
        CENTER_SE, TargetType::CENTER_GOAL,
        CG_DIAG, -CG_DIAG, 135.0,
        CENTER_GOAL_HALF_EXTENT * 0.70710678, -CENTER_GOAL_HALF_EXTENT * 0.70710678
    },
    {   // CENTER_SW
        CENTER_SW, TargetType::CENTER_GOAL,
        -CG_DIAG, -CG_DIAG, 225.0,
        -CENTER_GOAL_HALF_EXTENT * 0.70710678, -CENTER_GOAL_HALF_EXTENT * 0.70710678
    },
    {   // CENTER_NW
        CENTER_NW, TargetType::CENTER_GOAL,
        -CG_DIAG,  CG_DIAG, 315.0,
        -CENTER_GOAL_HALF_EXTENT * 0.70710678,  CENTER_GOAL_HALF_EXTENT * 0.70710678
    },

    // ── Match loaders — front faces post ─────────────────────────────────────
    {   // LOADER_NE — east post, north
        LOADER_NE, TargetType::MATCH_LOADER,
        ML_X_EAST - MATCH_LOADER_STANDOFF_CM, ML_Y_NORTH, 90.0,
        ML_X_EAST, ML_Y_NORTH
    },
    {   // LOADER_SE — east post, south
        LOADER_SE, TargetType::MATCH_LOADER,
        ML_X_EAST - MATCH_LOADER_STANDOFF_CM, ML_Y_SOUTH, 90.0,
        ML_X_EAST, ML_Y_SOUTH
    },
    {   // LOADER_SW — west post, south
        LOADER_SW, TargetType::MATCH_LOADER,
        ML_X_WEST + MATCH_LOADER_STANDOFF_CM, ML_Y_SOUTH, 270.0,
        ML_X_WEST, ML_Y_SOUTH
    },
    {   // LOADER_NW — west post, north
        LOADER_NW, TargetType::MATCH_LOADER,
        ML_X_WEST + MATCH_LOADER_STANDOFF_CM, ML_Y_NORTH, 270.0,
        ML_X_WEST, ML_Y_NORTH
    },

    // ── Parking ───────────────────────────────────────────────────────────────
    {   PARK_ALLIANCE, TargetType::PARK_ZONE, RED_PARK_X,  PARK_Y, 270.0, RED_PARK_X,  PARK_Y },
    {   PARK_OPPONENT, TargetType::PARK_ZONE, BLUE_PARK_X, PARK_Y,  90.0, BLUE_PARK_X, PARK_Y },

};  // FIELD_TARGETS — 24" bot

#else  // BOT_15INCH

// ── 15" BOT TABLE ─────────────────────────────────────────────────────────────
// Long goals: rear scores, so robot faces AWAY from goal (rear faces goal).
//   East goal → face West (270°), approach from west side of goal
//   West goal → face East  (90°), approach from east side of goal
//
// Match loaders: front faces post (same as 24" bot).
// Center goals: diagonal approach, GPS reset + odometry (no Jetson).
const NamedTarget FIELD_TARGETS[TARGET_COUNT] = {

    // ── East long goal — approach from west, face West so rear faces East ─────
    {   // GOAL_NE
        GOAL_NE, TargetType::LONG_GOAL,
        LONG_GOAL_X_EAST - LONG_GOAL_STANDOFF_CM, LONG_GOAL_Y_NORTH, 270.0,
        LONG_GOAL_X_EAST, LONG_GOAL_Y_NORTH
    },
    {   // GOAL_SE
        GOAL_SE, TargetType::LONG_GOAL,
        LONG_GOAL_X_EAST - LONG_GOAL_STANDOFF_CM, LONG_GOAL_Y_SOUTH, 270.0,
        LONG_GOAL_X_EAST, LONG_GOAL_Y_SOUTH
    },

    // ── West long goal — approach from east, face East so rear faces West ─────
    {   // GOAL_NW
        GOAL_NW, TargetType::LONG_GOAL,
        LONG_GOAL_X_WEST + LONG_GOAL_STANDOFF_CM, LONG_GOAL_Y_NORTH, 90.0,
        LONG_GOAL_X_WEST, LONG_GOAL_Y_NORTH
    },
    {   // GOAL_SW
        GOAL_SW, TargetType::LONG_GOAL,
        LONG_GOAL_X_WEST + LONG_GOAL_STANDOFF_CM, LONG_GOAL_Y_SOUTH, 90.0,
        LONG_GOAL_X_WEST, LONG_GOAL_Y_SOUTH
    },

    // ── Center goals — diagonal approach, GPS reset + odometry ───────────────
    {   CENTER_NE, TargetType::CENTER_GOAL,  CG_DIAG,  CG_DIAG,  45.0,  CENTER_GOAL_HALF_EXTENT * 0.70710678,  CENTER_GOAL_HALF_EXTENT * 0.70710678 },
    {   CENTER_SE, TargetType::CENTER_GOAL,  CG_DIAG, -CG_DIAG, 135.0,  CENTER_GOAL_HALF_EXTENT * 0.70710678, -CENTER_GOAL_HALF_EXTENT * 0.70710678 },
    {   CENTER_SW, TargetType::CENTER_GOAL, -CG_DIAG, -CG_DIAG, 225.0, -CENTER_GOAL_HALF_EXTENT * 0.70710678, -CENTER_GOAL_HALF_EXTENT * 0.70710678 },
    {   CENTER_NW, TargetType::CENTER_GOAL, -CG_DIAG,  CG_DIAG, 315.0, -CENTER_GOAL_HALF_EXTENT * 0.70710678,  CENTER_GOAL_HALF_EXTENT * 0.70710678 },

    // ── Match loaders — front faces post (same as 24" bot) ───────────────────
    {   LOADER_NE, TargetType::MATCH_LOADER, ML_X_EAST - MATCH_LOADER_STANDOFF_CM, ML_Y_NORTH,  90.0, ML_X_EAST, ML_Y_NORTH },
    {   LOADER_SE, TargetType::MATCH_LOADER, ML_X_EAST - MATCH_LOADER_STANDOFF_CM, ML_Y_SOUTH,  90.0, ML_X_EAST, ML_Y_SOUTH },
    {   LOADER_SW, TargetType::MATCH_LOADER, ML_X_WEST + MATCH_LOADER_STANDOFF_CM, ML_Y_SOUTH, 270.0, ML_X_WEST, ML_Y_SOUTH },
    {   LOADER_NW, TargetType::MATCH_LOADER, ML_X_WEST + MATCH_LOADER_STANDOFF_CM, ML_Y_NORTH, 270.0, ML_X_WEST, ML_Y_NORTH },

    // ── Parking ───────────────────────────────────────────────────────────────
    {   PARK_ALLIANCE, TargetType::PARK_ZONE, RED_PARK_X,  PARK_Y, 270.0, RED_PARK_X,  PARK_Y },
    {   PARK_OPPONENT, TargetType::PARK_ZONE, BLUE_PARK_X, PARK_Y,  90.0, BLUE_PARK_X, PARK_Y },

};  // FIELD_TARGETS — 15" bot

#endif  // ACTIVE_BOT

// ══════════════════════════════════════════════════════════════════════════════
// getTarget
// ══════════════════════════════════════════════════════════════════════════════
const NamedTarget& getTarget(TargetID id) {
    return FIELD_TARGETS[id];
}

// ══════════════════════════════════════════════════════════════════════════════
// waitAndResetGPS
// ══════════════════════════════════════════════════════════════════════════════
bool waitAndResetGPS(uint32_t waitMs) {
    uint32_t start = pros::millis();
    while (pros::millis() - start < waitMs) {
        if (gpsSensor.get_error() < GPS_MAX_ERROR_M) {
            gpsReset();
            return true;
        }
        pros::delay(20);
    }
    return false;
}