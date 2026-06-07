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
#define ACTIVE_BOT BOT_24INCH

namespace RobotGeometry {

    // ------------------------------------------------------------------
    // Chassis dimensions
    // ------------------------------------------------------------------

    namespace Bot24 {
        constexpr double WIDTH_CM     = 55.88;   // 22" — widest axis (E-W)
        constexpr double DEPTH_CM     = 35.56;   // 14" — front-to-back (N-S)
        // Diagonal half-extent: sqrt((22/2)^2 + (14/2)^2) = 32.9cm → 33.5cm with margin
        constexpr double CLEARANCE_CM = 0.0;
    }

    namespace Bot15 {
        constexpr double WIDTH_CM     = 38.10;   // 15"
        constexpr double DEPTH_CM     = 30.48;   // 12"
        // Diagonal half-extent: sqrt((15/2)^2 + (12/2)^2) = 24.4cm → 25.0cm with margin
        constexpr double CLEARANCE_CM = 25.0;
    }

    // Active robot — used by A* and navigation code
#if ACTIVE_BOT == BOT_24INCH
    constexpr double ROBOT_WIDTH_CM     = Bot24::WIDTH_CM;
    constexpr double ROBOT_DEPTH_CM     = Bot24::DEPTH_CM;
    constexpr double ROBOT_CLEARANCE_CM = Bot24::CLEARANCE_CM;
#else
    constexpr double ROBOT_WIDTH_CM     = Bot15::WIDTH_CM;
    constexpr double ROBOT_DEPTH_CM     = Bot15::DEPTH_CM;
    constexpr double ROBOT_CLEARANCE_CM = Bot15::CLEARANCE_CM;
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
    // Long goals — Y=±122cm, X=±62cm ends, ~5cm thick
    // ------------------------------------------------------------------

    constexpr double LONG_GOAL_HALF_W = 62.0;
    constexpr double LONG_GOAL_HALF_H =  5.0;
    constexpr double LONG_GOAL_Y_TOP  =  122.0;
    constexpr double LONG_GOAL_Y_BOT  = -122.0;

    // ------------------------------------------------------------------
    // Center goals — X-shape, hub at (0,0), tips at (±20cm, ±20cm)
    // ------------------------------------------------------------------

    constexpr double CENTER_GOAL_HALF_EXTENT = 20.0;

    // ------------------------------------------------------------------
    // Match loader posts — ø10.6cm at (±183cm, ±122cm)
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
    // Park zones — flush with east/west walls
    //   47.9cm (N-S) x 42.8cm (E-W) + 1" ramp field-facing edge
    //   Red: west wall, center (-159, 0)
    //   Blue: east wall, center (+159, 0)
    // ------------------------------------------------------------------

    struct Zone { double xMin, yMin, xMax, yMax; };
    constexpr Zone PARK_ZONES[2] = {
        { -183.0, -23.95, -140.2,  23.95 },  // Red  park (west wall)
        {  140.2, -23.95,  183.0,  23.95 },  // Blue park (east wall)
    };
    constexpr int NUM_PARK_ZONES = 2;

    // Park zone center coordinates (used by strategy functions)
    constexpr double RED_PARK_X  = -159.0;
    constexpr double BLUE_PARK_X =  159.0;
    constexpr double PARK_Y      =    0.0;

} // namespace RobotGeometry

#endif // ROBOT_GEOMETRY_H