#ifndef FIELD_TARGETS_H
#define FIELD_TARGETS_H

// ======================================================================
// field_targets.h — Named field targets for VAIRC Push Back (Team 3899E)
//
// Pure data: target table, coordinates, approach points, headings.
// Navigation logic lives in ai.cpp (navigateTo, navigateToTarget).
//
// Coordinate system: VEX GPS — origin (0,0) at field center,
//   X east positive, Y north positive (cm).
//
// Field geometry verified from pushback_field_map_v15.html
// (VEX Field Ref Spec 276-9142).
// ======================================================================

#include "main.h"
#include "ai.h"              // TargetType, NavResult — ai.h does not include field_targets.h
#include "robot_config.h"    // gpsSensor, GPS_MAX_ERROR_M

// ══════════════════════════════════════════════════════════════════════════════
// TARGET IDs
//
// Compass convention: direction = which side of the field element the robot
// approaches from. East goal north end = GOAL_NE (robot comes from the north).
//
// Long goals:    GOAL_NE/SE (east goal)  GOAL_NW/SW (west goal)
// Center goals:  CENTER_NE/SE/SW/NW      (diagonal X arms)
// Match loaders: LOADER_NE/SE/SW/NW      (corner posts)
// Parking:       PARK_ALLIANCE / PARK_OPPONENT
// ══════════════════════════════════════════════════════════════════════════════
enum TargetID : int {
    // East long goal (X = +120cm)
    GOAL_NE,        // north approach
    GOAL_SE,        // south approach

    // West long goal (X = -120cm)
    GOAL_NW,        // north approach
    GOAL_SW,        // south approach

    // Center goal — diagonal X structure
    CENTER_NE,
    CENTER_SE,
    CENTER_SW,
    CENTER_NW,

    // Match loader posts
    LOADER_NE,
    LOADER_SE,
    LOADER_SW,
    LOADER_NW,

    // Parking zones
    PARK_ALLIANCE,  // west wall, X = -159cm
    PARK_OPPONENT,  // east wall, X = +159cm

    TARGET_COUNT    // sentinel — array size, keep last
};

// ══════════════════════════════════════════════════════════════════════════════
// NAMED TARGET STRUCT
//
// approachX/Y:    where A* terminates (outside obstacle buffer)
// approachHeading: robot faces this heading at the approach point (VEX degrees,
//                  North=0°, CW+). Cardinal except center goals (diagonal).
// targetX/Y:      field element center — vision aim point / GPS reset reference
// type:           TargetType from ai.h — controls sensor and approach strategy
// ══════════════════════════════════════════════════════════════════════════════
struct NamedTarget {
    TargetID   id;
    TargetType type;        // TargetType from ai.h

    double approachX;       // cm, VEX GPS
    double approachY;       // cm, VEX GPS
    double approachHeading; // degrees, North=0°, CW+

    double targetX;         // cm, VEX GPS
    double targetY;         // cm, VEX GPS
};

// ══════════════════════════════════════════════════════════════════════════════
// TARGET TABLE — defined in field_targets.cpp
// ══════════════════════════════════════════════════════════════════════════════
extern const NamedTarget FIELD_TARGETS[TARGET_COUNT];

// Returns a const reference to the NamedTarget for a given ID.
const NamedTarget& getTarget(TargetID id);

// ══════════════════════════════════════════════════════════════════════════════
// GPS RESET HELPER
//
// Spins up to waitMs for gpsSensor.get_error() < GPS_MAX_ERROR_M,
// then calls gpsReset(). Used by 15" bot center goal approach.
// Returns true if GPS settled and reset was accepted within waitMs.
// ══════════════════════════════════════════════════════════════════════════════
bool waitAndResetGPS(uint32_t waitMs = 500);

// ══════════════════════════════════════════════════════════════════════════════
// navigateTo — single-call navigation to a named field target.
//
// Reads approach coordinates, heading, and type from FIELD_TARGETS table,
// then sequences all three phases via navigateToTarget() in ai.cpp:
//   Phase 1: 25° turn threshold + forwardToPoint per A* waypoint
//   Phase 2: vision approach (sensor selected by target type and ACTIVE_BOT)
//   Phase 3: blind approach until stall or timeout
//
// matchLoaderSig:
//   Default: aiVision_orangeCap (tournament after 15 sec, always)
//   Override: aiVision_redCube / aiVision_blueCube for first 15 sec or skills.
//
// Returns NavResult from ai.h.
// ══════════════════════════════════════════════════════════════════════════════
NavResult navigateTo(TargetID id,
                     pros::AIVision::Color matchLoaderSig = aiVision_orangeCap);

#endif // FIELD_TARGETS_H