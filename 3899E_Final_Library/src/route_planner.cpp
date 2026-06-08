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

    for (int r = 0; r < GRID_N; r++) {
        for (int c = 0; c < GRID_N; c++) {

            // ── 1. Wall border — outer ring always blocked ──────────────
            // Keeps robot center ≥1 cell (15.24cm) from any field wall.
            if (r == 0 || r == GRID_N-1 || c == 0 || c == GRID_N-1) {
                g_staticGrid[r][c] = true;
                continue;
            }

            // ── 2. Long goals ───────────────────────────────────────────
            // 10 cells wide × 4 cells tall, 1 passable row between outer
            // edge and wall border on both north and south sides.
            // North: cols 7-16, rows 18-21
            // South: cols 7-16, rows  2- 5
            if (c >= LONG_GOAL_COL_MIN && c <= LONG_GOAL_COL_MAX) {
                if ((r >= LONG_GOAL_ROW_N_MIN && r <= LONG_GOAL_ROW_N_MAX) ||
                    (r >= LONG_GOAL_ROW_S_MIN && r <= LONG_GOAL_ROW_S_MAX)) {
                    g_staticGrid[r][c] = true;
                    continue;
                }
            }

            // ── 3. Center goals ─────────────────────────────────────────
            // 6×6 block (4×4 physical X + 1 cell buffer all sides)
            // Cols 9-14, rows 9-14
            if (c >= CENTER_GOAL_COL_MIN && c <= CENTER_GOAL_COL_MAX &&
                r >= CENTER_GOAL_ROW_MIN && r <= CENTER_GOAL_ROW_MAX) {
                g_staticGrid[r][c] = true;
                continue;
            }

            // ── 4. Match loader posts ───────────────────────────────────
            // Physical radius only — robot approaches these to score.
            // Circle check against cell center in cm.
            {
                double cx, cy;
                cellToCm(c, r, cx, cy);
                for (int p = 0; p < NUM_ML_POSTS; p++) {
                    double dx = cx - ML_POSTS[p].x;
                    double dy = cy - ML_POSTS[p].y;
                    if (dx*dx + dy*dy <= ML_POST_RADIUS * ML_POST_RADIUS) {
                        g_staticGrid[r][c] = true;
                        break;
                    }
                }
            }
        }
    }

    // ── 5. Park zones — dynamic layer ──────────────────────────────────
    // Blocked from match start. Opened at 20s mark via routeOpenParkZones().
    // Dynamic grid only — robot must be able to enter to park.
    // Skip outer wall ring — those cells are already statically blocked.
    for (int r = 1; r < GRID_N-1; r++) {
        for (int c = 1; c < GRID_N-1; c++) {
            double cx, cy;
            cellToCm(c, r, cx, cy);
            for (int z = 0; z < NUM_PARK_ZONES; z++) {
                if (cx >= PARK_ZONES[z].xMin && cx <= PARK_ZONES[z].xMax &&
                    cy >= PARK_ZONES[z].yMin && cy <= PARK_ZONES[z].yMax) {
                    g_dynamicGrid[r][c] = true;
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

void routeInitParkZones() {
    // Call once at match start — blocks park zones in dynamic grid so A*
    // routes around them during normal play.
    buildStaticGrid();
    for (int r = 1; r < GRID_N-1; r++) {
        for (int c = 1; c < GRID_N-1; c++) {
            double cx, cy;
            cellToCm(c, r, cx, cy);
            for (int z = 0; z < NUM_PARK_ZONES; z++) {
                if (cx >= PARK_ZONES[z].xMin && cx <= PARK_ZONES[z].xMax &&
                    cy >= PARK_ZONES[z].yMin && cy <= PARK_ZONES[z].yMax) {
                    g_dynamicGrid[r][c] = true;
                }
            }
        }
    }
}

void routeOpenParkZones() {
    // Call at 20-second mark — clears park zones so robot can navigate into them.
    for (int r = 1; r < GRID_N-1; r++) {
        for (int c = 1; c < GRID_N-1; c++) {
            double cx, cy;
            cellToCm(c, r, cx, cy);
            for (int z = 0; z < NUM_PARK_ZONES; z++) {
                if (cx >= PARK_ZONES[z].xMin && cx <= PARK_ZONES[z].xMax &&
                    cy >= PARK_ZONES[z].yMin && cy <= PARK_ZONES[z].yMax) {
                    g_dynamicGrid[r][c] = false;
                }
            }
        }
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
// lineOfSight — DDA rasterization obstacle check
//
// Returns true if the straight line from (c0,r0) to (c1,r1) passes through
// only passable cells. Used by the LOS forward scan to validate each proposed
// merged leg against both static and dynamic obstacle grids.
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

    // ── LOS forward scan — collapse A* path into minimum straight legs ────────
    //
    // Walk forward through raw waypoints extending a straight line from the
    // current anchor as far as possible. As long as lineOfSight(anchor→candidate)
    // is clear, keep going — no waypoint is emitted yet. The moment LOS blocks,
    // the previous candidate was the farthest safe point: emit it, make it the
    // new anchor, continue from there.
    //
    // Anchor is tracked in cm (not snapped to grid cell) so the first leg runs
    // from the robot's actual position, not a cell centre. cmToCell is called
    // only at check time for the Bresenham traversal.
    //
    // Result: one waypoint per genuine direction change or obstacle avoidance
    // turn. Open-field cross-field routes collapse to a single waypoint.

    // Prepend actual robot position as the scan origin so the first leg starts
    // from where the robot is, not the first A* cell centre.
    double allX[GRID_CELLS + 1], allY[GRID_CELLS + 1];
    int allCount = 0;
    allX[allCount] = startX;  allY[allCount] = startY;  allCount++;
    for (int i = 0; i < rawCount; i++) {
        allX[allCount] = rawX[i];  allY[allCount] = rawY[i];  allCount++;
    }

    int writeIdx = 0;
    int anchor   = 0;

    for (int candidate = 1; candidate < allCount; candidate++) {
        int ac, ar, cc, cr;
        cmToCell(allX[anchor],    allY[anchor],    ac, ar);
        cmToCell(allX[candidate], allY[candidate], cc, cr);

        if (!lineOfSight(ac, ar, cc, cr)) {
            // LOS blocked — candidate-1 was the last safe endpoint.
            // Emit it and restart the scan from there.
            int safe = candidate - 1;
            if (safe > anchor && writeIdx < ROUTE_MAX_WAYPOINTS) {
                result.x[writeIdx] = allX[safe];
                result.y[writeIdx] = allY[safe];
                writeIdx++;
            }
            anchor = (safe > anchor) ? safe : anchor + 1;
        }
        // LOS clear — keep extending, nothing emitted yet.
    }

    // Always emit the exact goal coordinate as the final waypoint.
    if (writeIdx < ROUTE_MAX_WAYPOINTS) {
        result.x[writeIdx] = allX[allCount - 1];
        result.y[writeIdx] = allY[allCount - 1];
        writeIdx++;
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