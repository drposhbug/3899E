#ifndef ROBOT_GEOMETRY_H
#define ROBOT_GEOMETRY_H

// ======================================================================
// robot_geometry.h — Physical dimensions and field geometry constants
//
// Single source of truth for chassis dimensions, robot clearances,
// navigation timing, and Push Back field layout.
//
// Coordinate system: VEX GPS (matches V5 GPS sensor exactly)
//   Origin (0,0) = field center
//   X: east positive, west negative (cm)
//   Y: north positive, south negative (cm)
//
// Field geometry verified from pushback_field_map_v15.html
// (VEX Field Ref Spec 276-9142)
// ======================================================================

// ─────────────────────────────────────────────
// Active robot selection
// Change this one line before deploying to each bot
// ─────────────────────────────────────────────
#define BOT_24INCH 0
#define BOT_15INCH 1
#define ACTIVE_BOT BOT_15INCH

namespace RobotGeometry {

    // ------------------------------------------------------------------
    // Chassis dimensions
    // ------------------------------------------------------------------

    namespace Bot24 {
        constexpr double WIDTH_CM     = 33.02;   // 13" — widest axis (E-W)
        constexpr double DEPTH_CM     = 45.72;   // 18" — front-to-back (N-S)
        // Diagonal half-extent: sqrt((13/2)^2 + (18/2)^2) = 28.2cm → 29.0cm clearance
        // Rounded up from true diagonal. 6" cell discretization adds ~7.6cm
        // implicit buffer on top, giving ~36.6cm effective physical clearance.
        constexpr double CLEARANCE_CM = 29.0;
    }

    namespace Bot15 {
        constexpr double WIDTH_CM     = 33.02;   // 12.25"
        constexpr double DEPTH_CM     = 48.26;   // 19" (with intake down and aligner extended)
        // Diagonal half-extent: sqrt((12.25/2)^2 + (19/2)^2) = 28.71cm → 29.0cm with margin
        constexpr double CLEARANCE_CM = 29.0;
    }

    // Active robot — used by navigation code
#if ACTIVE_BOT == BOT_24INCH
    constexpr double ROBOT_WIDTH_CM  = Bot24::WIDTH_CM;
    constexpr double ROBOT_DEPTH_CM  = Bot24::DEPTH_CM;
#else
    constexpr double ROBOT_WIDTH_CM  = Bot15::WIDTH_CM;
    constexpr double ROBOT_DEPTH_CM  = Bot15::DEPTH_CM;
#endif

    // ------------------------------------------------------------------
    // Navigation speed constants
    // Tune ROBOT_CRUISE_SPEED_CMS from a real timed run:
    //   drive 200cm at 80% power, measure seconds → speed = 200 / seconds
    // ------------------------------------------------------------------

    constexpr double ROBOT_CRUISE_SPEED_CMS  = 90.0;  // cm/s at ~80% power — tune from real run
    constexpr double ROBOT_CRUISE_POWER_PCT  = 80.0;  // power level assumption for above constant

    // Approach speed used for final vision-guided phase
    constexpr double ROBOT_APPROACH_SPEED_CMS = 50.0; // cm/s during vision approach

    // ------------------------------------------------------------------
    // Park timing constants
    // ------------------------------------------------------------------

    // Safety buffer added on top of A* estimated travel time to park
    // Accounts for acceleration, deceleration, heading corrections
    constexpr double PARK_TIME_BUFFER_SEC = 3.0;

    // How far before park zone center to stop (edge of ramp)
    constexpr double PARK_APPROACH_DISTANCE_CM = 10.0;

    // ------------------------------------------------------------------
    // Navigation phase thresholds
    // ------------------------------------------------------------------

    // Switch from A*/odometry to vision approach when within this distance
    constexpr double VISION_HANDOFF_DISTANCE_CM = 80.0;

    // Switch from vision approach to blind final push when within this distance
    constexpr double BLIND_HANDOFF_DISTANCE_CM  = 20.0;

    // Blind approach: motor velocity below this = stall/contact detected (RPM)
    constexpr double STALL_DETECTION_RPM        = 10.0;

    // Blind approach: how long stall must persist before confirmed contact (ms)
    constexpr double STALL_CONFIRM_MS           = 150.0;

    // Blind approach: max time before BLIND_TIMEOUT returned (ms)
    constexpr double BLIND_TIMEOUT_MS           = 1500.0;

    // Blind approach power (% of max voltage) — reduced to avoid scattering objects
    constexpr double BLIND_APPROACH_POWER_PCT   = 35.0;

    // ------------------------------------------------------------------
    // Field dimensions (VEX Field Ref Spec 276-9142)
    // ------------------------------------------------------------------

    constexpr double FIELD_SIZE_CM = 365.76;   // 12ft exact
    constexpr double FIELD_HALF_CM = 182.88;   // ±axis extent

    // ------------------------------------------------------------------
    // A* grid obstacle layout — 24x24 grid, 15.24cm (6") cells
    //
    // All obstacles defined directly in cell coordinates.
    // ROBOT_CLEARANCE_CM = 0 — buffering is baked into cell ranges below,
    // not applied globally. This keeps corridors open for navigation.
    //
    // Grid orientation:
    //   col 0  = west wall,  col 23 = east wall
    //   row 0  = south wall, row 23 = north wall
    //   outer ring (row/col 0 and 23) always blocked = 1-cell wall border
    // ------------------------------------------------------------------

    constexpr double ROBOT_CLEARANCE_CM = 0.0;  // no global inflation — see note above

    // ------------------------------------------------------------------
    // Long goals
    //   Physical: 50" long x 6" wide, centered at Y=±122cm
    //   Grid:     10 cells wide x 4 cells tall
    //             1 passable row left between outer edge and wall border
    //   North: cols 7–16, rows 18–21  (row 22 = passable corridor, row 23 = wall)
    //   South: cols 7–16, rows  2– 5  (row  1 = passable corridor, row  0 = wall)
    // ------------------------------------------------------------------

    constexpr double LONG_GOAL_Y_TOP  =  122.0;  // cm — kept for field_targets.cpp reference
    constexpr double LONG_GOAL_Y_BOT  = -122.0;
    constexpr double LONG_GOAL_HALF_W =   62.0;  // cm half-length of goal bar — used by field_targets.cpp

    // Cell ranges used by buildStaticGrid()
    constexpr int LONG_GOAL_COL_MIN =  7;
    constexpr int LONG_GOAL_COL_MAX = 16;
    constexpr int LONG_GOAL_ROW_N_MIN = 18;  // north goal bottom row
    constexpr int LONG_GOAL_ROW_N_MAX = 21;  // north goal top row
    constexpr int LONG_GOAL_ROW_S_MIN =  2;  // south goal bottom row
    constexpr int LONG_GOAL_ROW_S_MAX =  5;  // south goal top row

    // ------------------------------------------------------------------
    // Center goals — diagonal X crossing at (0,0)
    //   Grid: 6x6 block (4x4 physical + 1 cell buffer all sides)
    //   Cols 9–14, rows 9–14
    // ------------------------------------------------------------------

    constexpr double CENTER_GOAL_HALF_EXTENT = 45.72;  // 3 cells × 15.24cm — for reference only
    constexpr int CENTER_GOAL_COL_MIN =  9;
    constexpr int CENTER_GOAL_COL_MAX = 14;
    constexpr int CENTER_GOAL_ROW_MIN =  9;
    constexpr int CENTER_GOAL_ROW_MAX = 14;

    // ------------------------------------------------------------------
    // Match loader posts — ø10.6cm at (±183cm, ±122cm)
    //   Physical radius only — no buffer added (robot approaches to score)
    // ------------------------------------------------------------------

    constexpr double ML_POST_RADIUS = 5.3;

    struct Post { double x, y; };
    constexpr Post ML_POSTS[4] = {
        { -183.0,  122.0 },  // ML1 — top-left
        { -183.0, -122.0 },  // ML2 — bottom-left
        {  183.0,  122.0 },  // ML3 — top-right
        {  183.0, -122.0 },  // ML4 — bottom-right
    };
    constexpr int NUM_ML_POSTS = 4;

    // ------------------------------------------------------------------
    // Park zones — flush with east/west walls, centered at Y=0
    //   Physical: 19" N-S (48.26cm) x 17" E-W depth (43.18cm)
    //   No buffer — robot must enter these zones to park
    //   Stored in dynamic grid, opened via routeOpenParkZones() at 20s mark
    // ------------------------------------------------------------------

    struct Zone { double xMin, yMin, xMax, yMax; };
    constexpr Zone PARK_ZONES[2] = {
        { -183.0, -24.13, -137.16,  24.13 },  // Red  park (west wall) — cols 1-2
        {  137.16, -24.13,  183.0,  24.13 },  // Blue park (east wall) — cols 21-22
    };
    constexpr int NUM_PARK_ZONES = 2;

    // Park zone center coordinates (used by strategy functions)
    constexpr double RED_PARK_X  = -161.41;  // midpoint of west park zone
    constexpr double BLUE_PARK_X =  161.41;  // midpoint of east park zone
    constexpr double PARK_Y      =    0.0;

} // namespace RobotGeometry

#endif // ROBOT_GEOMETRY_H