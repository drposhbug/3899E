#ifndef ROUTE_PLANNER_H
#define ROUTE_PLANNER_H

// ======================================================================
// route_planner.h — Field path planner for VAIRC Push Back (Team 3899E)
//
// Plans obstacle-avoiding routes across the Push Back field using an
// 8-directional grid search (A* algorithm internally).
//
// Coordinate system: VEX GPS — origin (0,0) at field center,
//   X east positive, Y north positive (cm). Matches V5 GPS sensor exactly.
//
// Static obstacles (goals, park zones, ML posts) sourced from
// robot_geometry.h. Dynamic obstacles (detected robots) added at runtime
// via routeAddObstacle() and cleared with routeClearObstacles().
//
// Can be called from anywhere — ai.cpp (Jetson-driven), auton.cpp
// (scripted/skills), or any other context.
//
// Usage:
//   RoutePath path = routePlan(globalX, globalY, targetX, targetY);
//   if (path.count > 0)
//       routeExecute(path);
// ======================================================================

#include "main.h"

// Max waypoints a path can hold
static const int ROUTE_MAX_WAYPOINTS = 24;

// Path returned by routePlan()
struct RoutePath {
    double x[ROUTE_MAX_WAYPOINTS];  // cm, VEX GPS coordinates
    double y[ROUTE_MAX_WAYPOINTS];
    int    count;                   // 0 = no path found
    double estimatedTimeSec;        // estimated travel time at cruise speed
};

// ── Planning ──────────────────────────────────────────────────────────

// Plan a route from (startX, startY) to (goalX, goalY) in VEX GPS cm.
// Returns path with count=0 if no route exists.
RoutePath routePlan(double startX, double startY,
                    double goalX,  double goalY);

// ── Execution ─────────────────────────────────────────────────────────

// Drive through a planned route waypoint-by-waypoint using moveOdometry.
// Returns true if all waypoints reached, false if any timed out (blocked).
bool routeExecute(const RoutePath& path,
                  double breakDistance     = 20.0,
                  double minSpeed          = 16.0,
                  double distanceTolerance = 3.0,
                  double maxSpeed          = 80.0,
                  double timeout           = 4.0);

// ── Dynamic obstacles ─────────────────────────────────────────────────

// Mark a ~24" (2-cell) temporary obstacle at (x, y) in VEX GPS cm.
// Persists until routeClearObstacles(). Call before replanning when a
// blockage is detected mid-route.
void routeAddObstacle(double x, double y);

// Clear all temporary obstacles.
// Call after replanning — robots move, stale obstacles corrupt future routes.
void routeClearObstacles();

// ── Utilities ─────────────────────────────────────────────────────────

// Print obstacle grid to V5 Brain screen (debug).
// X = static obstacle, D = dynamic obstacle, . = passable
void routePrintGrid();

#endif // ROUTE_PLANNER_H