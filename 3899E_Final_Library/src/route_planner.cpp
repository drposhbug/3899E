// ======================================================================
// route_planner.cpp — Field path planner for VAIRC Push Back
//
// Implements 8-directional A* grid search on a 24x24 field grid.
// Grid cells are 15.24cm (6") — one half-tile per cell.
//
// All field geometry and robot dimensions from robot_geometry.h.
//
// Fixes applied vs original:
//   - HEAP_CAPACITY = GRID_CELLS * 4 prevents MinHeap overflow on re-pushes
//   - Octile distance heuristic replaces sqrtf (exact for 8-dir, faster)
//   - String pulling (line-of-sight DDA) reduces waypoint count after A*
// ======================================================================

#include "route_planner.h"
#include "robot_geometry.h"
#include "navigation.h"    // forwardToPoint, StraightProfile
#include "motion_config.h"  // DEFAULT_STRAIGHT — do not hardcode profile values here
#include <cmath>
#include <cfloat>

using namespace RobotGeometry;

// ---------------------------------------------------------------------------
// Grid constants — 24x24 grid, 6" (15.24cm) cells
// ---------------------------------------------------------------------------

static const int    GRID_N       = 24;
static const double CELL_SIZE_CM = 15.24;   // 6" per cell

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
// Coordinate helpers (VEX GPS cm <-> grid cell)
//   col = X axis (East positive)  — col  0 = west edge, col 23 = east edge
//   row = Y axis (North positive) — row  0 = south edge, row 23 = north edge
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
//   g_staticGrid  — built once at startup from robot_geometry.h constants
//   g_dynamicGrid — runtime obstacles, cleared by routeClearObstacles()
// ---------------------------------------------------------------------------

static bool g_staticBuilt  = false;
static bool g_staticGrid [GRID_N][GRID_N];
static bool g_dynamicGrid[GRID_N][GRID_N];

static void buildStaticGrid() {
    if (g_staticBuilt) return;

    for (int r = 0; r < GRID_N; r++)
        for (int c = 0; c < GRID_N; c++) {
            g_staticGrid [r][c] = false;
            g_dynamicGrid[r][c] = false;
        }

    // Wall border — 1-cell ring on all sides
    for (int i = 0; i < GRID_N; i++) {
        g_staticGrid[0][i]        = true;  // south wall
        g_staticGrid[GRID_N-1][i] = true;  // north wall
        g_staticGrid[i][0]        = true;  // west wall
        g_staticGrid[i][GRID_N-1] = true;  // east wall
    }

    // Long goals — north: cols 7–16, rows 18–21
    for (int r = LONG_GOAL_ROW_N_MIN; r <= LONG_GOAL_ROW_N_MAX; r++)
        for (int c = LONG_GOAL_COL_MIN; c <= LONG_GOAL_COL_MAX; c++)
            g_staticGrid[r][c] = true;

    // Long goals — south: cols 7–16, rows 2–5
    for (int r = LONG_GOAL_ROW_S_MIN; r <= LONG_GOAL_ROW_S_MAX; r++)
        for (int c = LONG_GOAL_COL_MIN; c <= LONG_GOAL_COL_MAX; c++)
            g_staticGrid[r][c] = true;

    // Center goals — cols 9–14, rows 9–14
    for (int r = CENTER_GOAL_ROW_MIN; r <= CENTER_GOAL_ROW_MAX; r++)
        for (int c = CENTER_GOAL_COL_MIN; c <= CENTER_GOAL_COL_MAX; c++)
            g_staticGrid[r][c] = true;

    // Park zones — start blocked in dynamic grid, opened at 20s via routeOpenParkZones()
    // Red park (west): cols 1–2, rows 10–13
    // Blue park (east): cols 21–22, rows 10–13
    for (int r = 10; r <= 13; r++) {
        g_dynamicGrid[r][1] = true;
        g_dynamicGrid[r][2] = true;
        g_dynamicGrid[r][21] = true;
        g_dynamicGrid[r][22] = true;
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
    // Mark a 3-cell (~18") block — conservative robot-sized footprint at 6" resolution
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

void routeOpenParkZones() {
    // Clear only the park zone cells — leaves any other dynamic obstacles intact
    for (int r = 10; r <= 13; r++) {
        g_dynamicGrid[r][1]  = false;
        g_dynamicGrid[r][2]  = false;
        g_dynamicGrid[r][21] = false;
        g_dynamicGrid[r][22] = false;
    }
}

// ---------------------------------------------------------------------------
// Grid search — fixed-size working memory (no heap allocation on V5 Brain)
// ---------------------------------------------------------------------------

static const int GRID_CELLS    = GRID_N * GRID_N;   // 576 (24x24 @ 6" cells)
static const int HEAP_CAPACITY = GRID_CELLS * 4;     // nodes re-pushed on cost improvement;
                                                      // 4x headroom covers worst-case 8-dir

static inline int idx(int col, int row) { return row * GRID_N + col; }

struct MinHeap {
    int   cells[HEAP_CAPACITY];  // queue slots — larger than GRID_CELLS; nodes pushed multiple times
    float fval [GRID_CELLS];     // indexed by cell id (not heap position) — stays at GRID_CELLS
    int   size;

    void clear() { size = 0; }

    void push(int cell, float f) {
        if (size >= HEAP_CAPACITY) return;   // circuit breaker — never write out of bounds
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
// String pulling — line-of-sight check using DDA rasterization
//
// Returns true if the straight line from (c0,r0) to (c1,r1) passes through
// only passable cells. Used post-A* to collapse staircase paths into fewer,
// longer straight segments.
// ---------------------------------------------------------------------------

static bool lineOfSight(int c0, int r0, int c1, int r1) {
    int dc = abs(c1 - c0);
    int dr = abs(r1 - r0);
    int sc = (c0 < c1) ? 1 : -1;
    int sr = (r0 < r1) ? 1 : -1;
    int err = dc - dr;

    int c = c0, r = r0;
    while (true) {
        if (!passable(c, r)) return false;
        if (c == c1 && r == r1) break;
        int e2 = 2 * err;
        if (e2 > -dr) { err -= dr; c += sc; }
        if (e2 <  dc) { err += dc; r += sr; }
    }
    return true;
}

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

    // Octile distance heuristic — exact for 8-directional grid, no sqrtf needed
    auto h = [&](int col, int row) -> float {
        float dx = fabsf((float)(goalCol - col));
        float dy = fabsf((float)(goalRow - row));
        return (dx > dy) ? ((float)DIAGONAL_COST * dy + (float)CARDINAL_COST * (dx - dy))
                         : ((float)DIAGONAL_COST * dx + (float)CARDINAL_COST * (dy - dx));
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

    // Reconstruct raw path (goal -> start, then reverse)
    int pathCells[GRID_CELLS];
    int pathLen = 0;
    int current = goalIdx;
    while (current != startIdx && pathLen < GRID_CELLS)
        pathCells[pathLen++] = current, current = g_cameFrom[current];

    // Reverse into forward order; store raw waypoints
    double rawX[GRID_CELLS], rawY[GRID_CELLS];
    int rawCount = 0;
    for (int i = pathLen - 1; i >= 0 && rawCount < GRID_CELLS; i--) {
        int c  = pathCells[i] % GRID_N;
        int rr = pathCells[i] / GRID_N;
        if (pathCells[i] == goalIdx) {
            rawX[rawCount] = goalX;
            rawY[rawCount] = goalY;
        } else {
            cellToCm(c, rr, rawX[rawCount], rawY[rawCount]);
        }
        rawCount++;
    }

    // String pulling — walk forward; skip any waypoint with clear line-of-sight
    // to a further one. Collapses staircase diagonals into clean straight legs.
    int writeIdx = 0;
    int k = 0;
    while (k < rawCount && writeIdx < ROUTE_MAX_WAYPOINTS) {
        result.x[writeIdx] = rawX[k];
        result.y[writeIdx] = rawY[k];
        writeIdx++;

        if (k >= rawCount - 1) break;

        // Find furthest waypoint reachable in a straight line from k
        int skip = k + 1;
        for (int check = rawCount - 1; check > k + 1; check--) {
            int cc0, rr0, cc1, rr1;
            cmToCell(rawX[k],     rawY[k],     cc0, rr0);
            cmToCell(rawX[check], rawY[check], cc1, rr1);
            if (lineOfSight(cc0, rr0, cc1, rr1)) { skip = check; break; }
        }
        k = skip;
    }
    result.count = writeIdx;

    // Estimated travel time: path cost in cells x cm/cell / cruise speed
    result.estimatedTimeSec =
        (g_gScore[goalIdx] * CELL_SIZE_CM) / ROBOT_CRUISE_SPEED_CMS;

    return result;
}

// ---------------------------------------------------------------------------
// routeExecute — drives path waypoint-by-waypoint using forwardToPoint
//
// Profile selection uses DEFAULT_STRAIGHT from motion_config.h — all tuning
// belongs there, not here. Do not hardcode profile values in this file.
//
// TODO (post field validation): select profile based on leg distance and
// heading delta between waypoints:
//   short legs  (<30cm)  → tighter breakDistance, lower maxSpeed
//   long legs   (>90cm)  → full cruise profile
//   sharp turns (>45deg) → insert turnOdometry between waypoints
//
// Returns true if all waypoints reached, false if any timed out (blocked).
// ---------------------------------------------------------------------------

bool routeExecute(const RoutePath& path) {

    // Use calibrated default — tuned in motion_config.cpp, not here
    const StraightProfile& wpProfile = DEFAULT_STRAIGHT;

    for (int i = 0; i < path.count; i++) {
        uint32_t wpStart = pros::millis();

        forwardToPoint(path.x[i], path.y[i], wpProfile);

        // Timeout means waypoint wasn't reached — something blocked the path
        uint32_t elapsed = pros::millis() - wpStart;
        if (elapsed >= static_cast<uint32_t>(wpProfile.timeout * 1000.0 - 50.0))
            return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// routePrintGrid — debug display on V5 Brain screen
// 24 rows x 24 cols — uses E_TEXT_SMALL to fit on screen
// ---------------------------------------------------------------------------

void routePrintGrid() {
    buildStaticGrid();
    pros::screen::erase();
    pros::screen::print(pros::E_TEXT_SMALL, 0, "Grid 24x24 6\" X=static D=dyn .=open");
    pros::screen::print(pros::E_TEXT_SMALL, 1, "Bot clearance: %.1fcm", ROBOT_CLEARANCE_CM);

    for (int r = GRID_N - 1; r >= 0; r--) {
        char line[GRID_N + 1];
        for (int c = 0; c < GRID_N; c++) {
            if      (g_dynamicGrid[r][c]) line[c] = 'D';
            else if (g_staticGrid [r][c]) line[c] = 'X';
            else                          line[c] = '.';
        }
        line[GRID_N] = '\0';
        int screenRow = 2 + (GRID_N - 1 - r);
        pros::screen::print(pros::E_TEXT_SMALL, screenRow, "%2d %s", r, line);
    }
}