// ======================================================================
// route_planner.cpp — Field path planner for VAIRC Push Back
//
// Implements 8-directional A* grid search on a 12x12 field grid.
// Grid cells are 30.48cm (12") — one half-tile per cell.
//
// All field geometry and robot dimensions from robot_geometry.h.
// ======================================================================

#include "route_planner.h"
#include "robot_geometry.h"
#include "navigation.h"    // forwardToPoint, StraightProfile
#include <cmath>
#include <cfloat>

using namespace RobotGeometry;

// ---------------------------------------------------------------------------
// Grid constants
// ---------------------------------------------------------------------------

static const int    GRID_N       = 12;
static const double CELL_SIZE_CM = 30.48;   // 12" per cell

static const double CARDINAL_COST = 1.0;
static const double DIAGONAL_COST = 1.41421356;

struct Direction { int dc; int dr; double cost; };
static const Direction DIRS[8] = {
    { 0,  1, CARDINAL_COST },  // N
    { 0, -1, CARDINAL_COST },  // S
    { 1,  0, CARDINAL_COST },  // E
    {-1,  0, CARDINAL_COST },  // W
    { 1,  1, DIAGONAL_COST },  // NE
    {-1,  1, DIAGONAL_COST },  // NW
    { 1, -1, DIAGONAL_COST },  // SE
    {-1, -1, DIAGONAL_COST },  // SW
};

// ---------------------------------------------------------------------------
// Coordinate helpers (VEX GPS cm ↔ grid cell)
//   col = X axis (East positive) — col 0 = west edge, col 11 = east edge
//   row = Y axis (North positive) — row 0 = south edge, row 11 = north edge
// ---------------------------------------------------------------------------

static void cmToCell(double x, double y, int& col, int& row) {
    col = (int)((x + FIELD_HALF_CM) / CELL_SIZE_CM);
    row = (int)((y + FIELD_HALF_CM) / CELL_SIZE_CM);
    if (col < 0) col = 0;  if (col >= GRID_N) col = GRID_N - 1;
    if (row < 0) row = 0;  if (row >= GRID_N) row = GRID_N - 1;
}

static void cellToCm(int col, int row, double& x, double& y) {
    x = (col + 0.5) * CELL_SIZE_CM - FIELD_HALF_CM;
    y = (row + 0.5) * CELL_SIZE_CM - FIELD_HALF_CM;
}

// ---------------------------------------------------------------------------
// Two-layer obstacle grid
//   g_staticGrid  — built once at startup from robot_geometry.h
//   g_dynamicGrid — runtime temp obstacles, cleared by routeClearObstacles()
// ---------------------------------------------------------------------------

static bool g_staticBuilt  = false;
static bool g_staticGrid [GRID_N][GRID_N];
static bool g_dynamicGrid[GRID_N][GRID_N];

static void buildStaticGrid() {
    if (g_staticBuilt) return;

    const double clr = ROBOT_CLEARANCE_CM;

    for (int r = 0; r < GRID_N; r++)
        for (int c = 0; c < GRID_N; c++) {
            g_staticGrid [r][c] = false;
            g_dynamicGrid[r][c] = false;
        }

    for (int r = 0; r < GRID_N; r++) {
        for (int c = 0; c < GRID_N; c++) {
            double cx, cy;
            cellToCm(c, r, cx, cy);

            // Long goals — signed rectangle distance
            for (int g = 0; g < 2; g++) {
                double gy   = (g == 0) ? LONG_GOAL_Y_TOP : LONG_GOAL_Y_BOT;
                double dx   = fabs(cx)      - LONG_GOAL_HALF_W;
                double dy   = fabs(cy - gy) - LONG_GOAL_HALF_H;
                double dist = (dx > 0 && dy > 0) ? sqrt(dx*dx + dy*dy)
                            : (dx > 0)            ? dx
                            : (dy > 0)            ? dy
                            :                       fmax(dx, dy);
                if (dist < clr) { g_staticGrid[r][c] = true; break; }
            }
            if (g_staticGrid[r][c]) continue;

            // Center goal bounding box
            if (fabs(cx) < CENTER_GOAL_HALF_EXTENT + clr &&
                fabs(cy) < CENTER_GOAL_HALF_EXTENT + clr) {
                g_staticGrid[r][c] = true; continue;
            }

            // Match loader posts
            for (int p = 0; p < NUM_ML_POSTS; p++) {
                double dx = cx - ML_POSTS[p].x;
                double dy = cy - ML_POSTS[p].y;
                if (sqrt(dx*dx + dy*dy) < ML_POST_RADIUS + clr) {
                    g_staticGrid[r][c] = true; break;
                }
            }
            if (g_staticGrid[r][c]) continue;

            // Park zones
            for (int z = 0; z < NUM_PARK_ZONES; z++) {
                if (cx >= PARK_ZONES[z].xMin - clr &&
                    cx <= PARK_ZONES[z].xMax + clr &&
                    cy >= PARK_ZONES[z].yMin - clr &&
                    cy <= PARK_ZONES[z].yMax + clr) {
                    g_staticGrid[r][c] = true; break;
                }
            }
        }
    }
    g_staticBuilt = true;
}

static bool passable(int col, int row) {
    if (col < 0 || col >= GRID_N || row < 0 || row >= GRID_N) return false;
    return !g_staticGrid[row][col] && !g_dynamicGrid[row][col];
}

// BFS outward to snap start/goal out of an obstacle cell
static bool nearestPassable(int col, int row, int& outCol, int& outRow) {
    for (int radius = 1; radius <= 3; radius++) {
        for (int dr = -radius; dr <= radius; dr++) {
            for (int dc = -radius; dc <= radius; dc++) {
                if (abs(dr) != radius && abs(dc) != radius) continue;
                int nc = col + dc, nr = row + dr;
                if (passable(nc, nr)) { outCol = nc; outRow = nr; return true; }
            }
        }
    }
    return false;
}

// ---------------------------------------------------------------------------
// Public: dynamic obstacle management
// ---------------------------------------------------------------------------

void routeAddObstacle(double x, double y) {
    buildStaticGrid();
    int col, row;
    cmToCell(x, y, col, row);
    // Mark a 2-cell (~24") block — conservative robot-sized footprint
    for (int dr = -1; dr <= 1; dr++)
        for (int dc = -1; dc <= 1; dc++) {
            int nc = col + dc, nr = row + dr;
            if (nc >= 0 && nc < GRID_N && nr >= 0 && nr < GRID_N)
                g_dynamicGrid[nr][nc] = true;
        }
}

void routeClearObstacles() {
    for (int r = 0; r < GRID_N; r++)
        for (int c = 0; c < GRID_N; c++)
            g_dynamicGrid[r][c] = false;
}

// ---------------------------------------------------------------------------
// Grid search — fixed-size working memory (no heap allocation on V5 Brain)
// ---------------------------------------------------------------------------

static const int GRID_CELLS = GRID_N * GRID_N;  // 144

static inline int idx(int col, int row) { return row * GRID_N + col; }

struct MinHeap {
    int   cells[GRID_CELLS];
    float fval[GRID_CELLS];
    int   size;

    void clear() { size = 0; }

    void push(int cell, float f) {
        int i = size++;
        cells[i] = cell;
        fval[cell] = f;
        while (i > 0) {
            int parent = (i - 1) / 2;
            if (fval[cells[parent]] <= fval[cells[i]]) break;
            int tmp = cells[parent]; cells[parent] = cells[i]; cells[i] = tmp;
            i = parent;
        }
    }

    int pop() {
        int top = cells[0];
        cells[0] = cells[--size];
        int i = 0;
        while (true) {
            int l = 2*i+1, rr = 2*i+2, s = i;
            if (l  < size && fval[cells[l]]  < fval[cells[s]]) s = l;
            if (rr < size && fval[cells[rr]] < fval[cells[s]]) s = rr;
            if (s == i) break;
            int tmp = cells[s]; cells[s] = cells[i]; cells[i] = tmp;
            i = s;
        }
        return top;
    }

    bool empty() { return size == 0; }
};

static MinHeap g_heap;
static float   g_gScore  [GRID_CELLS];
static int     g_cameFrom[GRID_CELLS];
static bool    g_inClosed[GRID_CELLS];

// ---------------------------------------------------------------------------
// routePlan
// ---------------------------------------------------------------------------

RoutePath routePlan(double startX, double startY,
                    double goalX,  double goalY) {
    RoutePath result;
    result.count            = 0;
    result.estimatedTimeSec = 0.0;

    buildStaticGrid();

    int startCol, startRow, goalCol, goalRow;
    cmToCell(startX, startY, startCol, startRow);
    cmToCell(goalX,  goalY,  goalCol,  goalRow);

    if (!passable(startCol, startRow))
        if (!nearestPassable(startCol, startRow, startCol, startRow)) return result;
    if (!passable(goalCol, goalRow))
        if (!nearestPassable(goalCol, goalRow, goalCol, goalRow)) return result;

    int startIdx = idx(startCol, startRow);
    int goalIdx  = idx(goalCol,  goalRow);

    if (startIdx == goalIdx) {
        result.x[0] = goalX; result.y[0] = goalY;
        result.count = 1; result.estimatedTimeSec = 0.0;
        return result;
    }

    for (int i = 0; i < GRID_CELLS; i++) {
        g_gScore[i]   = FLT_MAX;
        g_cameFrom[i] = -1;
        g_inClosed[i] = false;
    }
    g_heap.clear();
    g_gScore[startIdx] = 0.0f;

    // Euclidean heuristic
    auto h = [&](int col, int row) -> float {
        float dc = (float)(goalCol - col);
        float dr = (float)(goalRow - row);
        return sqrtf(dc*dc + dr*dr);
    };

    g_heap.push(startIdx, h(startCol, startRow));

    while (!g_heap.empty()) {
        int current = g_heap.pop();
        if (g_inClosed[current]) continue;
        g_inClosed[current] = true;
        if (current == goalIdx) break;

        int col = current % GRID_N;
        int row = current / GRID_N;

        for (int d = 0; d < 8; d++) {
            int nc = col + DIRS[d].dc;
            int nr = row + DIRS[d].dr;
            if (!passable(nc, nr)) continue;
            int nIdx = idx(nc, nr);
            if (g_inClosed[nIdx]) continue;
            float tg = g_gScore[current] + (float)DIRS[d].cost;
            if (tg < g_gScore[nIdx]) {
                g_gScore[nIdx]   = tg;
                g_cameFrom[nIdx] = current;
                g_heap.push(nIdx, tg + h(nc, nr));
            }
        }
    }

    if (g_cameFrom[goalIdx] == -1 && goalIdx != startIdx) return result;

    // Reconstruct path
    int pathCells[GRID_CELLS];
    int pathLen = 0;
    int current = goalIdx;
    while (current != startIdx && pathLen < GRID_CELLS)
        pathCells[pathLen++] = current, current = g_cameFrom[current];

    int writeIdx = 0;
    for (int i = pathLen - 1; i >= 0 && writeIdx < ROUTE_MAX_WAYPOINTS; i--) {
        int c  = pathCells[i] % GRID_N;
        int rr = pathCells[i] / GRID_N;
        if (pathCells[i] == goalIdx) {
            result.x[writeIdx] = goalX;
            result.y[writeIdx] = goalY;
        } else {
            cellToCm(c, rr, result.x[writeIdx], result.y[writeIdx]);
        }
        writeIdx++;
    }
    result.count = writeIdx;

    // Estimated travel time: path cost in cells × cm/cell ÷ cruise speed
    result.estimatedTimeSec =
        (g_gScore[goalIdx] * CELL_SIZE_CM) / ROBOT_CRUISE_SPEED_CMS;

    return result;
}

// ---------------------------------------------------------------------------
// routeExecute — drives path waypoint-by-waypoint using forwardToPoint
// Returns true if all waypoints reached, false if any timed out (blocked)
// ---------------------------------------------------------------------------

bool routeExecute(const RoutePath& path,
                  double breakDistance,
                  double minSpeed,
                  double distanceTolerance,
                  double maxSpeed,
                  double timeout) {

    StraightProfile wpProfile = DEFAULT_STRAIGHT;
    wpProfile.breakDistance          = breakDistance;
    wpProfile.minSpeed               = minSpeed;
    wpProfile.distanceTolerance      = distanceTolerance;
    wpProfile.kp_heading             = 0.4;
    wpProfile.ki_heading             = 0.01;
    wpProfile.kd_heading             = 0.05;
    wpProfile.brakeMode              = pros::E_MOTOR_BRAKE_BRAKE;
    wpProfile.accelHeadingScaling    = 0.2;
    wpProfile.decelHeadingScaling    = 0.2;
    wpProfile.approachHeadingScaling = 0.2;
    wpProfile.maxSpeed               = maxSpeed;
    wpProfile.headingLockDistance    = 8.0;
    wpProfile.timeout                = timeout;

    for (int i = 0; i < path.count; i++) {
        uint32_t wpStart = pros::millis();

        forwardToPoint(path.x[i], path.y[i], wpProfile);

        // Timeout means waypoint wasn't reached — something blocked the path
        if (pros::millis() - wpStart >= static_cast<uint32_t>(timeout * 1000.0 - 50.0))
            return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// routePrintGrid — debug display on V5 Brain screen
// ---------------------------------------------------------------------------

void routePrintGrid() {
    buildStaticGrid();
    pros::screen::erase();
    pros::screen::print(pros::E_TEXT_MEDIUM, 0, "Route grid  X=static D=dynamic .=open");
    pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Bot clearance: %.1fcm", ROBOT_CLEARANCE_CM);

    for (int r = GRID_N - 1; r >= 0; r--) {
        char line[GRID_N + 1];
        for (int c = 0; c < GRID_N; c++) {
            if      (g_dynamicGrid[r][c]) line[c] = 'D';
            else if (g_staticGrid [r][c]) line[c] = 'X';
            else                          line[c] = '.';
        }
        line[GRID_N] = '\0';
        int screenRow = 2 + (GRID_N - 1 - r);
        pros::screen::print(pros::E_TEXT_MEDIUM, screenRow, "%2d %s", r, line);
    }
}