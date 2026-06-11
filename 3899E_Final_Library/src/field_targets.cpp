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
#include "odometry.h"        // applyGpsReset, pendingGpsReset
#include "navigation.h"      // requestGpsReset
#include "robot_config.h"    // gpsSensor, GPS_MAX_ERROR_M
#include <cmath>

using namespace RobotGeometry;

// ══════════════════════════════════════════════════════════════════════════════
// APPROACH STANDOFF DISTANCES (cm)
// Tune after field testing — must clear A* obstacle buffer + robot half-depth.
// ══════════════════════════════════════════════════════════════════════════════
static const double LONG_GOAL_STANDOFF_CM    = 50.0;   // unused — goals share approach with ML
static const double CENTER_GOAL_STANDOFF_CM  = 80.0;   // 80/√2=56.57cm, 1 cell beyond 6x6 block edge
static const double MATCH_LOADER_STANDOFF_CM = 45.0;   // unused — ML shares approach with goals

// Shared approach point for long goals and match loaders.
// X=±122cm is the midpoint between goal end (±62cm) and ML post (±183cm).
// Goals face inward (toward field center), ML posts face the wall.
static const double SHARED_APPROACH_X = 122.0;

// Loader approach is 30cm further infield than the shared goal/loader approach point.
// Gives more room for the forwardToPoint final drive-in to the post.
static const double LOADER_APPROACH_X = 92.0;  // SHARED_APPROACH_X - 30cm

// Park zone approach: 3 cells (45.72cm) infield of park target center.
// RED_PARK_X=-161.41 → approach at -161.41+45.72=-115.69cm
// BLUE_PARK_X=+161.41 → approach at +161.41-45.72=+115.69cm
static const double PARK_APPROACH_X   = 45.72;  // offset from target center

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
// Long goals: robot scores by pushing blocks east→west (or west→east) into the
// side face of the goal bar. Approach from outside the goal end on X axis.
//   East goals → approach from east (X=+114), face West (270°)
//   West goals → approach from west (X=−114), face East  (90°)
//
// Approach X=±114cm is just outside the long goal block (col 17/6, passable).
// Standoff from goal end (X=±62): 114−62 = 52cm.
//
// Match loaders: front faces post.
//   East posts  → face East  (90°)
//   West posts  → face West  (270°)
const NamedTarget FIELD_TARGETS[TARGET_COUNT] = {

    // ── East long goal — shared approach with LOADER_NE/SE ──────────────────────
    // Approach X=+122cm (midpoint between goal end +62 and ML post +183).
    // Goal faces West (into field), ML faces East (toward post). Same XY, diff heading.
    {   // LONG_GOAL_NE — approach from east corridor, face West into goal
        LONG_GOAL_NE, TargetType::LONG_GOAL,
        SHARED_APPROACH_X, ML_Y_NORTH, 270.0,
        LONG_GOAL_X_EAST, LONG_GOAL_Y_NORTH
    },
    {   // LONG_GOAL_SE — approach from east corridor, face West into goal
        LONG_GOAL_SE, TargetType::LONG_GOAL,
        SHARED_APPROACH_X, ML_Y_SOUTH, 270.0,
        LONG_GOAL_X_EAST, LONG_GOAL_Y_SOUTH
    },

    // ── West long goal — shared approach with LOADER_NW/SW ───────────────────
    {   // LONG_GOAL_NW — approach from west corridor, face East into goal
        LONG_GOAL_NW, TargetType::LONG_GOAL,
        -SHARED_APPROACH_X, ML_Y_NORTH, 90.0,
        LONG_GOAL_X_WEST, LONG_GOAL_Y_NORTH
    },
    {   // LONG_GOAL_SW — approach from west corridor, face East into goal
        LONG_GOAL_SW, TargetType::LONG_GOAL,
        -SHARED_APPROACH_X, ML_Y_SOUTH, 90.0,
        LONG_GOAL_X_WEST, LONG_GOAL_Y_SOUTH
    },

    // ── Center goals — diagonal approach, face origin (0,0) ──────────────────
    // Approach 1 cell beyond 6x6 block edge. Face SW/NW/NE/SE toward center.
    {   // CENTER_GOAL_NE — approach from NE, face SW (225°) toward origin
        CENTER_GOAL_NE, TargetType::CENTER_GOAL,
        CG_DIAG,  CG_DIAG, 225.0,
        CENTER_GOAL_HALF_EXTENT * 0.70710678,  CENTER_GOAL_HALF_EXTENT * 0.70710678
    },
    {   // CENTER_GOAL_SE — approach from SE, face NW (315°) toward origin
        CENTER_GOAL_SE, TargetType::CENTER_GOAL,
        CG_DIAG, -CG_DIAG, 315.0,
        CENTER_GOAL_HALF_EXTENT * 0.70710678, -CENTER_GOAL_HALF_EXTENT * 0.70710678
    },
    {   // CENTER_GOAL_SW — approach from SW, face NE (45°) toward origin
        CENTER_GOAL_SW, TargetType::CENTER_GOAL,
        -CG_DIAG, -CG_DIAG, 45.0,
        -CENTER_GOAL_HALF_EXTENT * 0.70710678, -CENTER_GOAL_HALF_EXTENT * 0.70710678
    },
    {   // CENTER_GOAL_NW — approach from NW, face SE (135°) toward origin
        CENTER_GOAL_NW, TargetType::CENTER_GOAL,
        -CG_DIAG,  CG_DIAG, 135.0,
        -CENTER_GOAL_HALF_EXTENT * 0.70710678,  CENTER_GOAL_HALF_EXTENT * 0.70710678
    },

    // ── Match loaders — approach 30cm further back than long goal approach ──────
    // Loader approach X=±92cm, Y=±121cm (matches target Y)
    {   // LOADER_NE — east post north, face East toward post
        LOADER_NE, TargetType::MATCH_LOADER,
        LOADER_APPROACH_X, 121.0, 90.0,
        ML_X_EAST, 121.0
    },
    {   // LOADER_SE — east post south, face East toward post
        LOADER_SE, TargetType::MATCH_LOADER,
        LOADER_APPROACH_X, -121.0, 90.0,
        ML_X_EAST, -121.0
    },
    {   // LOADER_SW — west post south, face West toward post
        LOADER_SW, TargetType::MATCH_LOADER,
        -LOADER_APPROACH_X, -121.0, 270.0,
        ML_X_WEST, -121.0
    },
    {   // LOADER_NW — west post north, face West toward post
        LOADER_NW, TargetType::MATCH_LOADER,
        -LOADER_APPROACH_X, 121.0, 270.0,
        ML_X_WEST, 121.0
    },

    // ── Parking — approach 1 cell outside park zone, face wall ───────────────
    {   PARK_ALLIANCE, TargetType::PARK_ZONE, RED_PARK_X  + 45.72, PARK_Y, 270.0, RED_PARK_X,  PARK_Y },
    {   PARK_OPPONENT, TargetType::PARK_ZONE, BLUE_PARK_X - 45.72, PARK_Y,  90.0, BLUE_PARK_X, PARK_Y },

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
    {   // LONG_GOAL_NE
        LONG_GOAL_NE, TargetType::LONG_GOAL,
        LONG_GOAL_X_EAST + LONG_GOAL_STANDOFF_CM, LONG_GOAL_Y_NORTH, 90.0,
        LONG_GOAL_X_EAST, LONG_GOAL_Y_NORTH
    },
    {   // LONG_GOAL_SE
        LONG_GOAL_SE, TargetType::LONG_GOAL,
        LONG_GOAL_X_EAST + LONG_GOAL_STANDOFF_CM, LONG_GOAL_Y_SOUTH, 90.0,
        LONG_GOAL_X_EAST, LONG_GOAL_Y_SOUTH
    },

    // ── West long goal — approach from east, face East so rear faces West ─────
    {   // LONG_GOAL_NW
        LONG_GOAL_NW, TargetType::LONG_GOAL,
        LONG_GOAL_X_WEST - LONG_GOAL_STANDOFF_CM, LONG_GOAL_Y_NORTH, 270.0,
        LONG_GOAL_X_WEST, LONG_GOAL_Y_NORTH
    },
    {   // LONG_GOAL_SW
        LONG_GOAL_SW, TargetType::LONG_GOAL,
        LONG_GOAL_X_WEST - LONG_GOAL_STANDOFF_CM, LONG_GOAL_Y_SOUTH, 270.0,
        LONG_GOAL_X_WEST, LONG_GOAL_Y_SOUTH
    },

    // ── Center goals — diagonal approach, GPS reset + odometry ───────────────
    {   CENTER_GOAL_NE, TargetType::CENTER_GOAL,  CG_DIAG,  CG_DIAG,  45.0,  CENTER_GOAL_HALF_EXTENT * 0.70710678,  CENTER_GOAL_HALF_EXTENT * 0.70710678 },
    {   CENTER_GOAL_SE, TargetType::CENTER_GOAL,  CG_DIAG, -CG_DIAG, 135.0,  CENTER_GOAL_HALF_EXTENT * 0.70710678, -CENTER_GOAL_HALF_EXTENT * 0.70710678 },
    {   CENTER_GOAL_SW, TargetType::CENTER_GOAL, -CG_DIAG, -CG_DIAG, 225.0, -CENTER_GOAL_HALF_EXTENT * 0.70710678, -CENTER_GOAL_HALF_EXTENT * 0.70710678 },
    {   CENTER_GOAL_NW, TargetType::CENTER_GOAL, -CG_DIAG,  CG_DIAG, 315.0, -CENTER_GOAL_HALF_EXTENT * 0.70710678,  CENTER_GOAL_HALF_EXTENT * 0.70710678 },

    // ── Match loaders — front faces post (same as 24" bot) ───────────────────
    {   LOADER_NE, TargetType::MATCH_LOADER, ML_X_EAST - MATCH_LOADER_STANDOFF_CM, ML_Y_NORTH,  270.0, ML_X_EAST, ML_Y_NORTH },
    {   LOADER_SE, TargetType::MATCH_LOADER, ML_X_EAST - MATCH_LOADER_STANDOFF_CM, ML_Y_SOUTH,  270.0, ML_X_EAST, ML_Y_SOUTH },
    {   LOADER_SW, TargetType::MATCH_LOADER, ML_X_WEST + MATCH_LOADER_STANDOFF_CM, ML_Y_SOUTH, 270.0, ML_X_WEST, ML_Y_SOUTH },
    {   LOADER_NW, TargetType::MATCH_LOADER, ML_X_WEST + MATCH_LOADER_STANDOFF_CM, ML_Y_NORTH, 270.0, ML_X_WEST, ML_Y_NORTH },

    // ── Parking ───────────────────────────────────────────────────────────────
    {   PARK_ALLIANCE, TargetType::PARK_ZONE, RED_PARK_X  + 45.72, PARK_Y, 270.0, RED_PARK_X,  PARK_Y },
    {   PARK_OPPONENT, TargetType::PARK_ZONE, BLUE_PARK_X - 45.72, PARK_Y,  90.0, BLUE_PARK_X, PARK_Y },

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
            requestGpsReset();  // non-blocking — task runs in background
            return true;
        }
        pros::delay(20);
    }
    return false;
}