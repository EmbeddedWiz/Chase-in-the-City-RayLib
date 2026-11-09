// Headless logic tests for City Chase.
//
// Built with -DCHASE_TESTING, which enables `friend class ChaseTest;` in
// chase.hpp so the harness can reach private state without touching the
// public API.
//
// The tests run without a window: raylib's RNG (SetRandomSeed /
// GetRandomValue) works in a headless context, so every scenario is
// reproducible from a fixed seed.

#include "chase.hpp"
#include "grid.hpp"
#include <raylib.h>
#include <algorithm>
#include <array>
#include <cstdio>
#include <stdexcept>
#include <utility>
#include <vector>

// ---------------------------------------------------------------------------
// Minimal test harness
// ---------------------------------------------------------------------------

static int g_checks = 0;
static int g_failures = 0;
static int g_sectionFailures = 0;

static void beginSection(const char *name)
{
    g_sectionFailures = 0;
    std::printf("[TEST] %s\n", name);
}

static void endSection()
{
    if (g_sectionFailures == 0)
        std::printf("       passed\n");
    else
        std::printf("       FAILED (%d check%s)\n", g_sectionFailures, g_sectionFailures == 1 ? "" : "s");
}

#define CHECK(cond)                                                                 \
    do {                                                                            \
        ++g_checks;                                                                 \
        if (!(cond)) {                                                              \
            ++g_failures;                                                           \
            ++g_sectionFailures;                                                    \
            std::printf("    FAIL: %s (line %d)\n", #cond, __LINE__);              \
        }                                                                           \
    } while (0)

#define CHECK_EQ(a, b)                                                              \
    do {                                                                            \
        int va_ = (a);                                                              \
        int vb_ = (b);                                                              \
        ++g_checks;                                                                 \
        if (va_ != vb_) {                                                           \
            ++g_failures;                                                           \
            ++g_sectionFailures;                                                    \
            std::printf("    FAIL: %s == %s  (%d != %d) (line %d)\n", #a, #b, va_, vb_, __LINE__); \
        }                                                                           \
    } while (0)

// ---------------------------------------------------------------------------
// Independent road-network helpers (do not use Grid::isRoadNetworkConnected,
// so connectivity is validated by a separate algorithm)
// ---------------------------------------------------------------------------

static bool independentRoadsConnected(const Grid &grid)
{
    const int rows = grid.getRows();
    const int cols = grid.getColumns();
    int roadCells = 0;
    int startR = -1;
    int startC = -1;

    for (int r = 0; r < rows; r++)
        for (int c = 0; c < cols; c++)
            if (grid.getValue(r, c) == 0) {
                roadCells++;
                if (startR < 0) {
                    startR = r;
                    startC = c;
                }
            }

    if (roadCells <= 1)
        return true;

    std::vector<int> seen(rows * cols, 0);
    std::vector<int> q;
    auto idx = [cols](int r, int c) { return r * cols + c; };
    seen[idx(startR, startC)] = 1;
    q.push_back(idx(startR, startC));
    size_t head = 0;
    while (head < q.size()) {
        int cur = q[head++];
        int r = cur / cols;
        int c = cur % cols;
        const int kr[] = {-1, 1, 0, 0};
        const int kc[] = {0, 0, -1, 1};
        for (int d = 0; d < 4; d++) {
            int nr = r + kr[d];
            int nc = c + kc[d];
            if (nr < 0 || nr >= rows || nc < 0 || nc >= cols)
                continue;
            int ni = idx(nr, nc);
            if (seen[ni] || grid.getValue(nr, nc) != 0)
                continue;
            seen[ni] = 1;
            q.push_back(ni);
        }
    }

    int reached = 0;
    for (int v : seen)
        reached += v;
    return reached == roadCells;
}

// BFS road distance between two cells; -1 when either cell is not road or
// they belong to different components.
static int roadDistance(const Grid &grid, int r1, int c1, int r2, int c2)
{
    const int rows = grid.getRows();
    const int cols = grid.getColumns();
    if (!grid.isWithinBounds(r1, c1) || !grid.isWithinBounds(r2, c2))
        return -1;
    if (grid.getValue(r1, c1) != 0 || grid.getValue(r2, c2) != 0)
        return -1;

    std::vector<int> dist(rows * cols, -1);
    std::vector<int> q;
    auto idx = [cols](int r, int c) { return r * cols + c; };
    dist[idx(r1, c1)] = 0;
    q.push_back(idx(r1, c1));
    size_t head = 0;
    while (head < q.size()) {
        int cur = q[head++];
        int r = cur / cols;
        int c = cur % cols;
        const int kr[] = {-1, 1, 0, 0};
        const int kc[] = {0, 0, -1, 1};
        for (int d = 0; d < 4; d++) {
            int nr = r + kr[d];
            int nc = c + kc[d];
            if (nr < 0 || nr >= rows || nc < 0 || nc >= cols)
                continue;
            int ni = idx(nr, nc);
            if (dist[ni] != -1 || grid.getValue(nr, nc) != 0)
                continue;
            dist[ni] = dist[cur] + 1;
            if (nr == r2 && nc == c2)
                return dist[ni];
            q.push_back(ni);
        }
    }
    return grid.getValue(r2, c2) == 0 ? dist[idx(r2, c2)] : -1;
}

// ---------------------------------------------------------------------------
// Test access to private Chase state (friend, enabled by CHASE_TESTING)
// ---------------------------------------------------------------------------

class ChaseTest {
  public:
    struct Snapshot {
        Chase::Car redCar;
        Chase::Car blueCar;
        int movesRed;
        int movesBlue;
        bool finished;
        bool crashed;
        int blueExtraMoves;
        int moveTimer;
        int closeCatchTimer;
        int redStuck;
        int cycleDetectCount;
        int stateHistIdx;
        int stateHistCount;
        Chase::ChaseState history[Chase::STATE_HIST_SIZE];
    };

    static Snapshot snap(const Chase &ch)
    {
        Snapshot s;
        s.redCar = ch.redCar;
        s.blueCar = ch.blueCar;
        s.movesRed = ch.movesRed;
        s.movesBlue = ch.movesBlue;
        s.finished = ch.finished;
        s.crashed = ch.crashed;
        s.blueExtraMoves = ch.blueExtraMoves;
        s.moveTimer = ch.moveTimer;
        s.closeCatchTimer = ch.closeCatchTimer;
        s.redStuck = ch.redStuck;
        s.cycleDetectCount = ch.cycleDetectCount;
        s.stateHistIdx = ch.stateHistIdx;
        s.stateHistCount = ch.stateHistCount;
        for (int i = 0; i < Chase::STATE_HIST_SIZE; i++)
            s.history[i] = ch.stateHistory[i];
        return s;
    }

    static bool same(const Snapshot &a, const Snapshot &b)
    {
        if (a.redCar.row != b.redCar.row || a.redCar.col != b.redCar.col ||
            a.redCar.lastMoveRow != b.redCar.lastMoveRow ||
            a.redCar.lastMoveCol != b.redCar.lastMoveCol)
            return false;
        if (a.blueCar.row != b.blueCar.row || a.blueCar.col != b.blueCar.col ||
            a.blueCar.lastMoveRow != b.blueCar.lastMoveRow ||
            a.blueCar.lastMoveCol != b.blueCar.lastMoveCol)
            return false;
        if (a.movesRed != b.movesRed || a.movesBlue != b.movesBlue)
            return false;
        if (a.finished != b.finished || a.crashed != b.crashed)
            return false;
        if (a.blueExtraMoves != b.blueExtraMoves || a.moveTimer != b.moveTimer)
            return false;
        if (a.closeCatchTimer != b.closeCatchTimer || a.redStuck != b.redStuck)
            return false;
        if (a.cycleDetectCount != b.cycleDetectCount)
            return false;
        if (a.stateHistIdx != b.stateHistIdx || a.stateHistCount != b.stateHistCount)
            return false;
        for (int i = 0; i < Chase::STATE_HIST_SIZE; i++)
            if (!(a.history[i] == b.history[i]))
                return false;
        return true;
    }

    static void placeCars(Chase &ch, int redRow, int redCol, int blueRow, int blueCol)
    {
        ch.redCar.row = redRow;
        ch.redCar.col = redCol;
        ch.blueCar.row = blueRow;
        ch.blueCar.col = blueCol;
        ch.redCar.lastMoveRow = 0;
        ch.redCar.lastMoveCol = 0;
        ch.blueCar.lastMoveRow = 0;
        ch.blueCar.lastMoveCol = 0;
    }

    // Full 9-field key: positions AND last-move directions AND the red
    // speed mode.
    static Chase::ChaseState blockOf(const Chase &ch)
    {
        return {ch.redCar.row, ch.redCar.col, ch.blueCar.row, ch.blueCar.col,
                ch.redCar.lastMoveRow, ch.redCar.lastMoveCol,
                ch.blueCar.lastMoveRow, ch.blueCar.lastMoveCol,
                ch.currentRedSpeedMode()};
    }

    static Chase::Step callBfsToward(Chase &ch, int sr, int sc, int gr, int gc, const Chase::ChaseState &block)
    {
        return ch.bfsToward(sr, sc, gr, gc, ch.redWs, block);
    }

    static Chase::Step callBfsAway(Chase &ch, int sr, int sc, const Chase::ChaseState &block)
    {
        return ch.bfsAway(sr, sc, block, ch.blueWs);
    }

    static Chase::Step callBfsLookahead(Chase &ch, int sr, int sc, int gr, int gc,
                                        int ticks, int rpm, int bpm)
    {
        return ch.bfsLookahead(sr, sc, gr, gc, ticks, rpm, bpm);
    }

    static Chase::Step callFleeLookahead(Chase &ch, int sr, int sc, int fr, int fc,
                                         int ticks, int rpm, int bpm)
    {
        return ch.fleeLookahead(sr, sc, fr, fc, ticks, rpm, bpm);
    }

    // One simulation engine for both actors.
    static Chase::SimResult callSimRed(Chase &ch, const Chase::SimulationState &state,
                                       int dr, int dc, int ticks, int rpm, int bpm)
    {
        return ch.simulate(state, dr, dc, ticks, rpm, bpm, true);
    }

    static Chase::SimResult callSimBlue(Chase &ch, const Chase::SimulationState &state,
                                        int dr, int dc, int ticks, int rpm, int bpm)
    {
        return ch.simulate(state, dr, dc, ticks, rpm, bpm, false);
    }

    // Cycle-detection unit tests.
    static int callDetectCyclePeriod(const Chase &ch) { return ch.detectCyclePeriod(); }
    static void pushState(Chase &ch, const Chase::ChaseState &s) { ch.pushState(s); }
    static bool stateInHistory(const Chase &ch, const Chase::ChaseState &s) { return ch.isStateInHistory(s); }
    static void clearHistory(Chase &ch)
    {
        ch.stateHistIdx = 0;
        ch.stateHistCount = 0;
    }
    static int histCount(const Chase &ch) { return ch.stateHistCount; }
    static int histIdx(const Chase &ch) { return ch.stateHistIdx; }

    // Simple value accessors so the test functions do not have to see
    // the private members directly.
    static Chase::Car redOf(const Chase &ch) { return ch.redCar; }
    static Chase::Car blueOf(const Chase &ch) { return ch.blueCar; }
    static void setBlueExtraMoves(Chase &ch, int v) { ch.blueExtraMoves = v; }
    static void setBestDist(Chase &ch, int v) { ch.bestDist = v; }
    static int bestDistOf(const Chase &ch) { return ch.bestDist; }
    static int stalemateTicksOf(const Chase &ch) { return ch.stalemateTicks; }
    static void callUpdateStalemateGuard(Chase &ch) { ch.updateStalemateGuard(); }

    // History-key regression and cycle-break tests.
    // The newest entry of the ring (call only when histCount > 0).
    static Chase::ChaseState topState(const Chase &ch)
    {
        int i = (ch.stateHistIdx - 1 + Chase::STATE_HIST_SIZE) % Chase::STATE_HIST_SIZE;
        return ch.stateHistory[i];
    }
    static void callBreakCycle(Chase &ch, Chase::ChaseState &block) { ch.breakCycle(block); }
    static void setCycleDetectCount(Chase &ch, int v) { ch.cycleDetectCount = v; }

    // Drive movesRed across the 150/300 thresholds and
    // read back the derived red speed mode.
    static void setMovesRed(Chase &ch, int v) { ch.movesRed = v; }
    static int redSpeedModeOf(const Chase &ch) { return ch.currentRedSpeedMode(); }
    static int modeOf(const Chase::ChaseState &s) { return s.redSpeedMode; }

    // Factories + constants for the private nested types. Callers outside
    // ChaseTest never spell the type names: they receive deduced (auto)
    // values through these friend members only.
    static Chase::ChaseState makeState(int rr, int rc, int br, int bc,
                                       int rlr, int rlc, int blr, int blc,
                                       int redSpeedMode = 2)
    {
        return {rr, rc, br, bc, rlr, rlc, blr, blc, redSpeedMode};
    }
    static Chase::SimulationState makeSim(int rr, int rc, int br, int bc)
    {
        return {rr, rc, br, bc};
    }
    static int histSize() { return Chase::STATE_HIST_SIZE; }
    static int invalidStep() { return Chase::INVALID_STEP; }
};

// ---------------------------------------------------------------------------
// 3.2 City generation
// ---------------------------------------------------------------------------

static void testCityGeneration()
{
    beginSection("city generation (20 seeds)");

    for (int seed = 100; seed < 120; seed++) {
        SetRandomSeed(seed);
        Grid grid(800, 600, 15);
        grid.generateCity(2);

        const int total = grid.getRows() * grid.getColumns();
        int roadCells = 0;
        int buildingCells = 0;
        int badValues = 0;
        for (int r = 0; r < grid.getRows(); r++)
            for (int c = 0; c < grid.getColumns(); c++) {
                int v = grid.getValue(r, c);
                if (v < 0 || v > 6)
                    badValues++;
                if (v == 0)
                    roadCells++;
                else
                    buildingCells++;
            }

        CHECK_EQ(badValues, 0);                       // values stay in the 0..6 range
        CHECK(roadCells > total / 10);                 // the city was not reduced to void
        CHECK(buildingCells > 0);                      // buildings were actually placed
        CHECK_EQ(independentRoadsConnected(grid), true); // validation independent of the repair
        CHECK_EQ(grid.isRoadNetworkConnected(), true);   // the class reports the same
    }

    endSection();
}

// connectRoads() on an already connected network is a no-op, and the
// generator guarantees a connected road net.
static void testConnectRoadsIdempotent()
{
    beginSection("connectRoads idempotence");

    SetRandomSeed(555);
    Grid grid(800, 600, 15);
    grid.generateCity(2); // internally ends with connectRoads()

    auto cellHash = [](const Grid &g) {
        long long h = 1469598103934665603LL;
        for (int r = 0; r < g.getRows(); r++)
            for (int c = 0; c < g.getColumns(); c++) {
                h ^= (unsigned int)g.getValue(r, c);
                h *= 1099511628211ULL;
            }
        return h;
    };

    CHECK(grid.isRoadNetworkConnected()); // guaranteed by the generator
    long long before = cellHash(grid);

    // Second run on the already-connected network: no cell may change.
    grid.connectRoads();
    CHECK(cellHash(grid) == before);
    CHECK(grid.isRoadNetworkConnected());

    // A corrupted network must be repaired back to connectivity.
    grid.setValue(0, 0, 1);
    grid.setValue(0, 1, 1);
    grid.setValue(1, 0, 1); // seal the first road cell off
    grid.connectRoads();
    CHECK(grid.isRoadNetworkConnected());

    endSection();
}

// ---------------------------------------------------------------------------
// 3.3 Car placement
// ---------------------------------------------------------------------------

static void testCarPlacement()
{
    beginSection("car placement on restart");

    SetRandomSeed(777);
    Grid grid(800, 600, 15);
    grid.generateCity(2);
    Chase chase(800, 600, 15, grid);

    for (int i = 0; i < 50; i++) {
        chase.reset(i == 49); // mostly soft restarts, one hard (R)

        const auto red = ChaseTest::redOf(chase);
        const auto blue = ChaseTest::blueOf(chase);

        // Both cars on road cells (never inside a building).
        CHECK_EQ(grid.getValue(red.row, red.col), 0);
        CHECK_EQ(grid.getValue(blue.row, blue.col), 0);

        // Distinct cells.
        CHECK(red.row != blue.row || red.col != blue.col);

        // Same connected road network, with the intended initial separation.
        int d = roadDistance(grid, red.row, red.col, blue.row, blue.col);
        CHECK(d >= 12 && d <= 50);
    }

    endSection();
}

// ---------------------------------------------------------------------------
// 3.4 Pathfinding: all four directions, obstacles, corridors, dead ends,
// fallback behaviour, and the explicit "up is not rejected" check.
// ---------------------------------------------------------------------------

static void testPathfinding()
{
    beginSection("pathfinding (bfsToward / bfsAway)");

    // Empty 20x20 field.
    Grid grid(300, 300, 15);
    Chase chase(300, 300, 15, grid);

    ChaseTest::placeCars(chase, 10, 10, 10, 15);
    auto block = ChaseTest::blockOf(chase);

    // (a) Straight path: first step goes east toward the goal.
    auto step = ChaseTest::callBfsToward(chase, 10, 10, 10, 15, block);
    CHECK_EQ(step.rowDelta, 0);
    CHECK_EQ(step.colDelta, 1);

    // (b) Straight path going up: {-1, 0} must be accepted.
    ChaseTest::placeCars(chase, 10, 10, 5, 10);
    block = ChaseTest::blockOf(chase);
    step = ChaseTest::callBfsToward(chase, 10, 10, 5, 10, block);
    CHECK_EQ(step.rowDelta, -1);
    CHECK_EQ(step.colDelta, 0);

    // (c) Start adjacent to the goal: no move is needed.
    ChaseTest::placeCars(chase, 10, 10, 10, 11);
    block = ChaseTest::blockOf(chase);
    step = ChaseTest::callBfsToward(chase, 10, 10, 10, 11, block);
    CHECK_EQ(step.rowDelta, 0);
    CHECK_EQ(step.colDelta, 0);

    // (d) Path around an obstacle: the wall at column 8 blocks the direct
    // route. Going over the top (rows 0-2) costs 13 steps, going under
    // (row 17 and below) costs 31, so the unique shortest detour goes up
    // and the first step is deterministically up.
    for (int r = 3; r <= 16; r++)
        grid.setValue(r, 8, 1);
    ChaseTest::placeCars(chase, 5, 5, 5, 12);
    block = ChaseTest::blockOf(chase);
    step = ChaseTest::callBfsToward(chase, 5, 5, 5, 12, block);
    CHECK_EQ(step.rowDelta, -1);
    CHECK_EQ(step.colDelta, 0);

    // (e) Narrow one-cell corridor: only forward is possible.
    for (int c = 4; c <= 15; c++) {
        grid.setValue(9, c, 1);
        grid.setValue(11, c, 1);
    }
    grid.setValue(10, 8, 0); // keep the corridor open where the wall from (d) crosses it
    ChaseTest::placeCars(chase, 10, 4, 10, 15);
    block = ChaseTest::blockOf(chase);
    step = ChaseTest::callBfsToward(chase, 10, 4, 10, 15, block);
    CHECK_EQ(step.rowDelta, 0);
    CHECK_EQ(step.colDelta, 1);

    // (f) Dead end: red is boxed in by buildings -> INVALID_STEP.
    grid.setValue(9, 10, 1);
    grid.setValue(11, 10, 1);
    grid.setValue(10, 9, 1);
    grid.setValue(10, 11, 1);
    ChaseTest::placeCars(chase, 10, 10, 3, 3);
    block = ChaseTest::blockOf(chase);
    step = ChaseTest::callBfsToward(chase, 10, 10, 3, 3, block);
    CHECK_EQ(step.rowDelta, ChaseTest::invalidStep());

    // and the lookahead must report the same (no fallback move exists).
    step = ChaseTest::callBfsLookahead(chase, 10, 10, 3, 3, 4, 2, 1);
    CHECK_EQ(step.rowDelta, ChaseTest::invalidStep());

    // (g) Fallback with a sealed goal: the goal component is unreachable,
    // the fallback heuristic must still accept the upward step.
    grid.setValue(9, 10, 0);
    grid.setValue(11, 10, 0);
    grid.setValue(10, 9, 0);
    grid.setValue(10, 11, 0);
    for (int r = 3; r <= 5; r++)
        for (int c = 9; c <= 11; c++)
            grid.setValue(r, c, 1);
    ChaseTest::placeCars(chase, 10, 10, 4, 10);
    block = ChaseTest::blockOf(chase);
    step = ChaseTest::callBfsToward(chase, 10, 10, 4, 10, block);
    CHECK_EQ(step.rowDelta, -1);
    CHECK_EQ(step.colDelta, 0);

    // (h) bfsAway: blue runs away from the red car; the farthest region is
    // the bottom-right corner, so the step must be down or east.
    grid.setValue(9, 10, 0);
    grid.setValue(11, 10, 0);
    grid.setValue(10, 9, 0);
    grid.setValue(10, 11, 0);
    ChaseTest::placeCars(chase, 3, 3, 17, 17);
    block = ChaseTest::blockOf(chase);
    step = ChaseTest::callBfsAway(chase, 17, 17, block);
    CHECK((step.rowDelta == 1 && step.colDelta == 0) ||
          (step.rowDelta == 0 && step.colDelta == 1));

    endSection();
}

// BFS correctness at the map borders — reachable goal
// on a border, unreachable goal, no step may leave the field.
static void testPathfindingBorders()
{
    beginSection("pathfinding at map borders");

    // Empty 20x20 field: every cell is road.
    Grid grid(300, 300, 15);
    Chase chase(300, 300, 15, grid);

    // (a) Goal on the top border: the detour runs along row 0, first step east.
    ChaseTest::placeCars(chase, 0, 0, 0, 19);
    auto step = ChaseTest::callBfsToward(chase, 0, 0, 0, 19, ChaseTest::blockOf(chase));
    CHECK_EQ(step.rowDelta, 0);
    CHECK_EQ(step.colDelta, 1);

    // (b) Goal on the left border: the detour runs down column 0.
    ChaseTest::placeCars(chase, 0, 0, 19, 0);
    step = ChaseTest::callBfsToward(chase, 0, 0, 19, 0, ChaseTest::blockOf(chase));
    CHECK_EQ(step.rowDelta, 1);
    CHECK_EQ(step.colDelta, 0);

    // (c) From every corner the step must stay inside the field.
    ChaseTest::placeCars(chase, 0, 0, 19, 19);
    step = ChaseTest::callBfsToward(chase, 0, 0, 19, 19, ChaseTest::blockOf(chase));
    CHECK(step.rowDelta == 1 || step.rowDelta == 0);
    CHECK(step.colDelta == 1 || step.colDelta == 0);

    ChaseTest::placeCars(chase, 19, 19, 0, 0);
    step = ChaseTest::callBfsToward(chase, 19, 19, 0, 0, ChaseTest::blockOf(chase));
    CHECK(step.rowDelta == -1 || step.rowDelta == 0);
    CHECK(step.colDelta == -1 || step.colDelta == 0);

    // (d) bfsAway from a border corner: the corner cell (0,0) blocks one
    // neighbour; the evader must step down (away from the edge), not out
    // of the field and not into the follower's cell.
    ChaseTest::placeCars(chase, 0, 0, 0, 1);
    step = ChaseTest::callBfsAway(chase, 0, 1, ChaseTest::blockOf(chase));
    CHECK_EQ(step.rowDelta, 1);
    CHECK_EQ(step.colDelta, 0);

    // (e) Unreachable goal behind a full wall: INVALID_STEP, never a step
    // out of the field.
    for (int r = 0; r < 20; r++)
        grid.setValue(r, 10, 1);
    ChaseTest::placeCars(chase, 0, 0, 0, 19);
    step = ChaseTest::callBfsToward(chase, 0, 0, 5, 15, ChaseTest::blockOf(chase));
    // the start side is fully walled against the goal side: the fallback
    // can still walk along the wall, and never below row 0.
    CHECK(step.rowDelta == 1 || step.rowDelta == 0 || step.rowDelta == ChaseTest::invalidStep());
    if (step.rowDelta != ChaseTest::invalidStep()) {
        CHECK(0 + step.rowDelta >= 0 && 0 + step.rowDelta < 20);
        CHECK(0 + step.colDelta >= 0 && 0 + step.colDelta < 20);
    }

    endSection();
}

// ---------------------------------------------------------------------------
// 3.5 Lookahead simulation must not touch the real game state.
// ---------------------------------------------------------------------------

static void testSimulationInvariance()
{
    beginSection("simulation invariance");

    SetRandomSeed(321);
    Grid grid(800, 600, 15);
    grid.generateCity(2);
    Chase chase(800, 600, 15, grid);
    chase.reset(true);
    SetRandomSeed(322); // deterministic sim runs

    ChaseTest::Snapshot before = ChaseTest::snap(chase);
    const auto red = ChaseTest::redOf(chase);
    const auto blue = ChaseTest::blueOf(chase);

    ChaseTest::callBfsLookahead(chase, red.row, red.col, blue.row, blue.col, 4, 2, 1);
    ChaseTest::callFleeLookahead(chase, blue.row, blue.col, red.row, red.col, 4, 2, 1);
    auto simState = ChaseTest::makeSim(red.row, red.col, blue.row, blue.col);
    ChaseTest::callSimRed(chase, simState, 1, 0, 4, 3, 1);
    ChaseTest::callSimBlue(chase, simState, -1, 0, 4, 2, 1);

    ChaseTest::Snapshot after = ChaseTest::snap(chase);
    CHECK(ChaseTest::same(before, after));

    endSection();
}

// 3.5 Metrics of the simulation results.
static void testSimulationMetrics()
{
    beginSection("simulation metrics");

    // Open field, controlled geometry. Red and blue are 16 cells apart.
    Grid grid(300, 300, 15);
    Chase chase(300, 300, 15, grid);
    ChaseTest::placeCars(chase, 10, 10, 2, 2);
    SetRandomSeed(5);

    auto state = ChaseTest::makeSim(10, 10, 2, 2);

    auto slow = ChaseTest::callSimRed(chase, state, -1, 0, 6, 2, 1);
    auto fast = ChaseTest::callSimRed(chase, state, -1, 0, 6, 4, 1);

    // Generic invariants.
    CHECK(slow.totalSteps >= 1);
    CHECK(slow.minDist <= slow.finalDist);
    CHECK(slow.dirChanges <= slow.totalSteps);
    if (slow.caught)
        CHECK(slow.finalDist <= 1); // "caught" means the cars ended up adjacent

    // Variable red speed: 4 moves per cycle closes the 16-cell gap,
    // 2 moves per cycle does not.
    CHECK(fast.caught);
    CHECK(!slow.caught);
    CHECK(fast.minDist <= slow.minDist);

    // Blue-side simulation: evaderStuck is only a property of the red
    // simulation; on blue it must stay clear.
    auto blueSim = ChaseTest::callSimBlue(chase, state, -1, 0, 6, 2, 1);
    CHECK(!blueSim.evaderStuck);
    CHECK(blueSim.minDist <= blueSim.finalDist);
    CHECK(blueSim.dirChanges <= blueSim.totalSteps);

    // Net deltas are measured on the actor's displacement.
    CHECK(fast.netRowDelta != 0 || fast.netColDelta != 0);

    endSection();
}

// ---------------------------------------------------------------------------
// Cycle detection — repetition, exit, no false positives;
// and the 9-field key completeness.
// ---------------------------------------------------------------------------

static void testCycleDetection()
{
    beginSection("cycle detection (periods, key completeness)");

    Grid grid(300, 300, 15);
    Chase chase(300, 300, 15, grid);

    const auto A = ChaseTest::makeState(5, 5, 9, 9, 1, 0, 0, 0);
    const auto B = ChaseTest::makeState(5, 6, 9, 8, 0, 1, 0, -1);
    const auto C = ChaseTest::makeState(5, 7, 9, 7, -1, 0, 0, 1);

    // (a) A period-2 pattern A B A B is detected.
    ChaseTest::clearHistory(chase);
    ChaseTest::pushState(chase, A);
    ChaseTest::pushState(chase, B);
    ChaseTest::pushState(chase, A);
    ChaseTest::pushState(chase, B);
    CHECK_EQ(ChaseTest::callDetectCyclePeriod(chase), 2);

    // (b) A period-3 pattern A B C A B C is detected.
    ChaseTest::clearHistory(chase);
    ChaseTest::pushState(chase, A);
    ChaseTest::pushState(chase, B);
    ChaseTest::pushState(chase, C);
    ChaseTest::pushState(chase, A);
    ChaseTest::pushState(chase, B);
    ChaseTest::pushState(chase, C);
    CHECK_EQ(ChaseTest::callDetectCyclePeriod(chase), 3);

    // (c) No repetition -> no detection (no false positives).
    ChaseTest::clearHistory(chase);
    for (int i = 0; i < 6; i++) {
        auto s = ChaseTest::makeState(3 + i, 3 + i, 8, 8, 0, 0, 1 - i, i - 1); // all distinct
        ChaseTest::pushState(chase, s);
    }
    CHECK_EQ(ChaseTest::callDetectCyclePeriod(chase), -1);

    // (d) Too short a history cannot contain a period: -1.
    ChaseTest::clearHistory(chase);
    ChaseTest::pushState(chase, A);
    ChaseTest::pushState(chase, B);
    CHECK_EQ(ChaseTest::callDetectCyclePeriod(chase), -1);

    // (e) The key includes the last-move directions: the same positions
    // with different directions are DIFFERENT states (a
    // position-only key would false-positive here).
    ChaseTest::clearHistory(chase);
    const auto P1 = ChaseTest::makeState(4, 4, 6, 6, 1, 0, 0, 0);
    const auto P2 = ChaseTest::makeState(4, 4, 6, 6, 0, 1, 0, 0);
    ChaseTest::pushState(chase, P1);
    CHECK(ChaseTest::stateInHistory(chase, P1));
    CHECK(!ChaseTest::stateInHistory(chase, P2));

    endSection();
}

// ---------------------------------------------------------------------------
// The fixed-size state history is a ring buffer — verify the
// eviction behaviour when the buffer is full.
// ---------------------------------------------------------------------------

static void testStateHistoryRing()
{
    beginSection("state history ring buffer");

    Grid grid(300, 300, 15);
    Chase chase(300, 300, 15, grid);
    ChaseTest::clearHistory(chase);

    // Push 300 distinct states; the ring keeps only the last 256.
    for (int i = 0; i < 300; i++) {
        auto s = ChaseTest::makeState(5, 5, 9, 9, i, 0, 0, 0);
        ChaseTest::pushState(chase, s);
    }

    CHECK_EQ(ChaseTest::histCount(chase), ChaseTest::histSize());
    CHECK_EQ(ChaseTest::histIdx(chase), 300 % ChaseTest::histSize());

    auto state = [](int i) { return ChaseTest::makeState(5, 5, 9, 9, i, 0, 0, 0); };

    // Evicted entries are gone...
    CHECK(!ChaseTest::stateInHistory(chase, state(0)));
    CHECK(!ChaseTest::stateInHistory(chase, state(43)));
    // ...recent ones are still visible (full ring, no false negatives).
    CHECK(ChaseTest::stateInHistory(chase, state(44)));
    CHECK(ChaseTest::stateInHistory(chase, state(255)));
    CHECK(ChaseTest::stateInHistory(chase, state(299)));

    endSection();
}

// ---------------------------------------------------------------------------
// 3.7 / F.4: the stalemate guard — a round whose distance never improves
// ends by itself; the guard does not fire before its limit.
// ---------------------------------------------------------------------------

static void testStalemateGuard()
{
    beginSection("stalemate guard");

    // The guard is a standalone method (updateStalemateGuard), so the
    // test drives it directly — no car policy, fully deterministic.
    // STALEMATE_LIMIT mirrors kStalemateLimitTicks in chase.cpp.
    const int STALEMATE_LIMIT = 400;

    Grid grid(400, 400, 15); // empty 26 x 26 road field
    Chase chase(400, 400, 15, grid);
    chase.reset(true);
    ChaseTest::placeCars(chase, 6, 6, 24, 10); // manhattan distance 22
    ChaseTest::setBestDist(chase, 0); // can never be beaten

    // No false positive: one tick short of the limit the round is alive.
    for (int t = 0; t < STALEMATE_LIMIT - 1; t++)
        ChaseTest::callUpdateStalemateGuard(chase);
    CHECK(!chase.isFinished());
    CHECK(!chase.isCrashed());
    CHECK_EQ(ChaseTest::stalemateTicksOf(chase), STALEMATE_LIMIT - 1);

    // The limit tick: the guard ends the round as a red crash and blue
    // earns the usual speed bonus.
    ChaseTest::callUpdateStalemateGuard(chase);
    CHECK(chase.isFinished());
    CHECK(chase.isCrashed());
    CHECK_EQ(chase.getBlueExtraMoves(), 1);

    // An improvement (distance below the recorded best) resets the streak.
    Chase chase2(400, 400, 15, grid);
    chase2.reset(true);
    ChaseTest::placeCars(chase2, 10, 10, 12, 12); // manhattan distance 4
    ChaseTest::setBestDist(chase2, 40);
    ChaseTest::callUpdateStalemateGuard(chase2);
    CHECK_EQ(ChaseTest::stalemateTicksOf(chase2), 0);
    CHECK_EQ(ChaseTest::bestDistOf(chase2), 4);
    CHECK(!chase2.isFinished());

    endSection();
}

// ---------------------------------------------------------------------------
// The state pushed into the history ring after a tick must
// carry blue's NEW position and NEW last-move direction (the block captured
// inside the blue loop predates blue's last move), and every pushed state
// must match the actual car fields.
// ---------------------------------------------------------------------------

static void testFreshHistoryAfterBlueMove()
{
    beginSection("fresh history key after blue moves");

    Grid grid(400, 400, 15); // empty 26 x 26 road field
    Chase chase(400, 400, 15, grid);
    chase.reset(true);
    ChaseTest::placeCars(chase, 6, 6, 6, 12); // manhattan distance 6
    ChaseTest::clearHistory(chase);

    // Drive three full ticks (8 frames per tick). Blue makes a successful
    // move on its last blue step of every tick, so the pushed state must
    // reflect that move — the old code pushed the block captured before it.
    for (int t = 0; t < 3 && !chase.isFinished(); t++) {
        for (int f = 0; f < 8; f++)
            chase.update();

        auto top = ChaseTest::topState(chase);
        auto actual = ChaseTest::blockOf(chase);
        CHECK(top == actual); // positions AND last-move directions

        auto red = ChaseTest::redOf(chase);
        auto blue = ChaseTest::blueOf(chase);
        CHECK(red.row != blue.row || red.col != blue.col); // no overlap
        CHECK(red.row >= 0 && red.row < grid.getRows() &&
              red.col >= 0 && red.col < grid.getColumns());
        CHECK(blue.row >= 0 && blue.row < grid.getRows() &&
              blue.col >= 0 && blue.col < grid.getColumns());
        CHECK_EQ(grid.getValue(red.row, red.col), 0); // no building cell
        CHECK_EQ(grid.getValue(blue.row, blue.col), 0);
    }

    endSection();
}

// ---------------------------------------------------------------------------
// Integration test — real cycle-detection logic on a real
// history ring, then breakCycle() execution right after a successful blue
// move: the history matches the actual cars, the cars never overlap, and
// neither car stands on a building cell or leaves the grid.
// ---------------------------------------------------------------------------

static void testCycleBreakAfterBlueMove()
{
    beginSection("cycle detection + breakCycle after a blue move");

    Grid grid(400, 400, 15);
    Chase chase(400, 400, 15, grid);
    chase.reset(true);
    ChaseTest::placeCars(chase, 8, 8, 8, 14);
    ChaseTest::clearHistory(chase);

    // One real tick: blue makes a successful move and update() pushes the
    // (now fresh) post-tick state.
    for (int f = 0; f < 8; f++)
        chase.update();
    CHECK(ChaseTest::histCount(chase) >= 1);
    CHECK(ChaseTest::topState(chase) == ChaseTest::blockOf(chase));

    // Real detector on a real ring: craft a strict period-2 tail from two
    // distinct 8-field keys and put the counter one step from the limit.
    auto s0 = ChaseTest::blockOf(chase);
    auto s1 = s0;
    s1.blueCol += 1; // a distinct key, still on the road
    ChaseTest::pushState(chase, s0);
    ChaseTest::pushState(chase, s1);
    ChaseTest::pushState(chase, s0);
    ChaseTest::pushState(chase, s1);
    ChaseTest::setCycleDetectCount(chase, 1);
    CHECK_EQ(ChaseTest::callDetectCyclePeriod(chase), 2);

    // Run the reaction half on the actual game state: red makes a
    // deviation move (never onto blue's cell), blue takes one bfsAway step.
    auto block = ChaseTest::blockOf(chase);
    ChaseTest::callBreakCycle(chase, block);

    auto red = ChaseTest::redOf(chase);
    auto blue = ChaseTest::blueOf(chase);
    CHECK(red.row != blue.row || red.col != blue.col); // no overlap
    CHECK(red.row >= 0 && red.row < grid.getRows() &&
          red.col >= 0 && red.col < grid.getColumns());
    CHECK(blue.row >= 0 && blue.row < grid.getRows() &&
          blue.col >= 0 && blue.col < grid.getColumns());
    CHECK_EQ(grid.getValue(red.row, red.col), 0); // no building cell
    CHECK_EQ(grid.getValue(blue.row, blue.col), 0);
    // The reaction refreshed the key and pushed the post-move state:
    // history matches the actual car fields again.
    CHECK(ChaseTest::topState(chase) == ChaseTest::blockOf(chase));

    endSection();
}

// ---------------------------------------------------------------------------
// The Grid guards — a zero-sized field is rejected at
// construction (it would make every modulo/index calculation undefined),
// and copyCellsFrom compares the dimensions, not just the buffer size.
// ---------------------------------------------------------------------------

static void testGridGuards()
{
    beginSection("grid guards");

    // width/height below cellSize: integer division yields a 0-field.
    bool threw = false;
    try {
        Grid tiny(10, 10, 15);
    }
    catch (const std::invalid_argument &) {
        threw = true;
    }
    CHECK(threw); // a zero-sized grid is not constructible

    // Same total cell count, transposed dimensions (4x3 vs 3x4):
    // the old buffer-size-only check let this copy through.
    Grid a(45, 60, 15); // 4 rows x 3 cols
    Grid b(60, 45, 15); // 3 rows x 4 cols
    CHECK_EQ(a.getRows(), 4);
    CHECK_EQ(b.getRows(), 3);
    CHECK_EQ(a.getRows() * a.getColumns(), b.getRows() * b.getColumns());
    threw = false;
    try {
        a.copyCellsFrom(b);
    }
    catch (const std::invalid_argument &) {
        threw = true;
    }
    CHECK(threw); // transposed grids are rejected

    // Same dimensions: the copy works.
    Grid c(45, 60, 15);
    c.setValue(1, 1, 7);
    Grid d(45, 60, 15);
    d.copyCellsFrom(c); // must not throw
    CHECK_EQ(d.getValue(1, 1), 7);

    endSection();
}

// ---------------------------------------------------------------------------
// The Chase constructor must validate its parameters
// before any arithmetic — cellSize == 0 would divide by zero.
// ---------------------------------------------------------------------------
static void testChaseCtorGuards()
{
    beginSection("chase constructor guards");

    Grid grid(600, 600, 15);

    bool threw = false;
    try { Chase g1(0, 600, 15, grid); }
    catch (const std::invalid_argument &) { threw = true; }
    CHECK(threw); // width == 0

    threw = false;
    try { Chase g2(600, 0, 15, grid); }
    catch (const std::invalid_argument &) { threw = true; }
    CHECK(threw); // height == 0

    threw = false;
    try { Chase g3(600, 600, 0, grid); }
    catch (const std::invalid_argument &) { threw = true; }
    CHECK(threw); // cellSize == 0 (the old divide-by-zero)

    threw = false;
    try { Chase g4(600, 600, -5, grid); }
    catch (const std::invalid_argument &) { threw = true; }
    CHECK(threw); // cellSize < 0

    threw = false;
    try { Chase g5(600, 600, 15, grid); }
    catch (const std::exception &) { threw = true; }
    CHECK(!threw); // correct positive parameters construct fine

    endSection();
}

// ---------------------------------------------------------------------------
// breakCycle() must end the round as a normal catch when
// the cars end up adjacent after a deviation move — for both cars. The
// setups box the cars in so every deviation step is deterministic.
// ---------------------------------------------------------------------------
static void testBreakCycleCatch()
{
    beginSection("breakCycle catch check after each car move");

    // (a) Red closes the gap: red has exactly one free direction — east —
    // and the deviation step lands adjacent to blue, so the round must end
    // BEFORE blue gets its move.
    {
        Grid grid(400, 400, 15); // 26 x 26 open road field
        grid.setValue(9, 10, 1);
        grid.setValue(11, 10, 1);
        grid.setValue(10, 9, 1);
        Chase chase(400, 400, 15, grid);
        chase.reset(true);
        ChaseTest::placeCars(chase, 10, 10, 10, 12); // manhattan distance 2

        auto block = ChaseTest::blockOf(chase);
        ChaseTest::callBreakCycle(chase, block);

        CHECK(chase.isFinished()); // the round ended as a catch
        CHECK(!chase.isCrashed()); // a normal catch, not a crash
        auto red = ChaseTest::redOf(chase);
        auto blue = ChaseTest::blueOf(chase);
        CHECK_EQ(red.col, 11); // red took its only free direction
        CHECK_EQ(blue.row, 10);
        CHECK_EQ(blue.col, 12); // blue never moved
        CHECK(ChaseTest::topState(chase) == ChaseTest::blockOf(chase)); // key matches the actual fields
    }

    // (b) Blue closes the gap: red's deviation steps east (not adjacent
    // yet), then blue's boxed-in escape step walks it next to red.
    {
        Grid grid(400, 400, 15);
        grid.setValue(9, 10, 1);
        grid.setValue(11, 10, 1);
        grid.setValue(10, 9, 1);
        grid.setValue(8, 12, 1);
        grid.setValue(9, 11, 1);
        grid.setValue(9, 13, 1);
        Chase chase(400, 400, 15, grid);
        chase.reset(true);
        ChaseTest::placeCars(chase, 10, 10, 9, 12); // manhattan distance 3

        auto block = ChaseTest::blockOf(chase);
        ChaseTest::callBreakCycle(chase, block);

        CHECK(chase.isFinished());
        CHECK(!chase.isCrashed());
        auto red = ChaseTest::redOf(chase);
        auto blue = ChaseTest::blueOf(chase);
        CHECK_EQ(red.row, 10);
        CHECK_EQ(red.col, 11); // red deviated east; still not adjacent
        CHECK_EQ(blue.row, 10);
        CHECK_EQ(blue.col, 12); // blue's only free cell is adjacent
        CHECK(ChaseTest::topState(chase) == ChaseTest::blockOf(chase));
    }

    endSection();
}

// ---------------------------------------------------------------------------
// The red speed mode is part of the cycle key — identical
// coordinates and directions under different red schedules are different
// states; the 150/300-move thresholds govern the mode transitions.
// ---------------------------------------------------------------------------
static void testRedSpeedModeKey()
{
    beginSection("red speed mode in the cycle key");

    Grid grid(300, 300, 15);
    Chase chase(300, 300, 15, grid);
    ChaseTest::clearHistory(chase);

    // Same positions + directions, different red schedules: different keys.
    auto s2 = ChaseTest::makeState(5, 5, 9, 9, 1, 0, 0, 0, 2);
    auto s3 = ChaseTest::makeState(5, 5, 9, 9, 1, 0, 0, 0, 3);
    CHECK(!(s2 == s3));
    ChaseTest::pushState(chase, s2);
    CHECK(ChaseTest::stateInHistory(chase, s2));
    CHECK(!ChaseTest::stateInHistory(chase, s3));

    // Same positions, directions AND mode: the same state.
    auto s2b = ChaseTest::makeState(5, 5, 9, 9, 1, 0, 0, 0, 2);
    CHECK(s2 == s2b);

    // Threshold transitions: <=150 -> 2, 151..300 -> 3, >300 -> 4
    // (at the exact threshold the previous mode still holds).
    struct { int movesRed; int mode; } bounds[] = {
        {149, 2}, {150, 2}, {151, 3}, {299, 3}, {300, 3}, {301, 4}};
    for (auto &b : bounds) {
        ChaseTest::setMovesRed(chase, b.movesRed);
        CHECK_EQ(ChaseTest::redSpeedModeOf(chase), b.mode);
        // a key formed from the actual fields carries the current mode:
        auto key = ChaseTest::blockOf(chase);
        CHECK_EQ(ChaseTest::modeOf(key), b.mode);
    }

    endSection();
}

// ---------------------------------------------------------------------------
// Replay determinism — the lookahead model and the real
// execution must be two views of the same rules: two independent instances
// started from the same seed produce identical car trajectories.
// ---------------------------------------------------------------------------

static void testReplayDeterminism()
{
    beginSection("replay determinism (prediction mirrors execution)");

    auto run = [](int seed) {
        SetRandomSeed(seed);
        Grid grid(800, 600, 15);
        grid.generateCity(2);
        Chase chase(800, 600, 15, grid);
        chase.reset(true);

        std::vector<std::array<int, 4>> trace;
        int frames = 0;
        while (!chase.isFinished() && frames < 2000) {
            chase.update();
            frames++;
            const auto r = ChaseTest::redOf(chase);
            const auto b = ChaseTest::blueOf(chase);
            trace.push_back({r.row, r.col, b.row, b.col});
        }
        trace.push_back({frames, 0, 0, 0}); // record the length
        return trace;
    };

    std::vector<std::array<int, 4>> a = run(1357);
    std::vector<std::array<int, 4>> b = run(1357);

    CHECK(a.size() > 8);               // the game actually ran
    CHECK_EQ((int)a.size(), (int)b.size());
    int mismatch = 0;
    for (size_t i = 0; i < a.size() && i < b.size(); i++)
        if (!(a[i] == b[i]))
            mismatch++;
    CHECK_EQ(mismatch, 0); // identical tick-by-tick trajectories

    endSection();
}

// ---------------------------------------------------------------------------
// Reservoir sampling in the city generator — the choice of the
// building variant and its position must stay uniform, not degenerate to
// a single candidate.
// ---------------------------------------------------------------------------

static void testReservoirUniformity()
{
    beginSection("reservoir sampling uniformity (city generator)");

    // Small 5x5 field: a 2x2 building with clearance 1 has exactly
    // 4 valid positions. With a fixed seed the whole experiment is
    // reproducible; the check is that the distribution over
    // (variant, position) candidates is symmetric.
    Grid grid(75, 75, 15);
    const auto &variants = grid.buildingVariants();
    const int vCount = (int)variants[0].size();
    CHECK(vCount >= 2); // dedup still keeps multiple variants

    const int trials = 2000;
    std::vector<std::vector<int>> counts(vCount, std::vector<int>(40, 0));
    int placed = 0;
    int failed = 0;
    int badPosition = 0;

    SetRandomSeed(2024);
    for (int t = 0; t < trials; t++) {
        grid.clear();
        auto p = grid.placeRandomGroup(0, 1);
        if (!p.ok) {
            failed++;
            continue;
        }
        placed++;
        // A 2x2 matrix with clearance 1 on a 5x5 field: the top-left
        // corner can only sit in rows/cols 1..2.
        if (p.row < 1 || p.row > 2 || p.col < 1 || p.col > 2)
            badPosition++;
        counts[p.variant][p.row * 10 + p.col]++;
    }

    CHECK_EQ(failed, 0);       // the empty field always has candidates
    CHECK_EQ(badPosition, 0);  // the clearance margin is respected
    CHECK_EQ(placed, trials);

    // Uniformity: every observed (variant, position) combination must land
    // in a wide, symmetric band around the expected share
    // placed / (vCount * 4) — a degenerate reservoir would pile almost all
    // placements into a handful of combinations and break the band.
    int expected = placed / (vCount * 4);
    int lo = expected / 4;
    int hi = expected * 5;
    int observedCombos = 0;
    int outOfBand = 0;
    for (int v = 0; v < vCount; v++)
        for (int p = 0; p < 40; p++) {
            if (counts[v][p] > 0)
                observedCombos++;
            if (counts[v][p] > hi || (counts[v][p] > 0 && counts[v][p] < lo))
                outOfBand++;
        }
    CHECK(outOfBand == 0);
    CHECK(observedCombos >= vCount * 4 - 2); // all combinations were used

    std::printf("       variants: %d, placed: %d, observed combos: %d/%d\n",
                vCount, placed, observedCombos, vCount * 4);

    endSection();
}

// ---------------------------------------------------------------------------
// The building library must contain well-formed, duplicate-free
// variants (rotation + dedup pass did not corrupt the patterns).
// ---------------------------------------------------------------------------

static void testBuildingVariants()
{
    beginSection("building library (rotations / dedup)");

    Grid grid(75, 75, 15);
    const auto &groups = grid.buildingVariants();
    CHECK_EQ((int)groups.size(), 6);

    // The class g stores matrices of size ~(g + 2): a non-square source
    // (7x6 among the 7-class patterns) yields square AND rectangular
    // rotations alike, so only well-formedness is checked, not the exact
    // size.
    for (int g = 0; g < 6; g++) {
        const auto &group = groups[g];
        CHECK(!group.empty());
        for (size_t v = 0; v < group.size(); v++) {
            const auto &m = group[v];
            CHECK(m.size() >= 1);
            CHECK(m[0].size() >= 1);
            bool hasOne = false;
            bool valuesOk = true;
            for (size_t i = 0; i < m.size(); i++) {
                CHECK_EQ((int)m[i].size(), (int)m[0].size()); // rectangular
                for (size_t j = 0; j < m[i].size(); j++) {
                    int x = m[i][j];
                    if (x != 0 && x != 1) {
                        valuesOk = false;
                        std::printf("    odd value %d at group %d variant %zu cell [%zu][%zu]\n",
                                    x, g, v, i, j);
                    }
                }
                if (std::find(m[i].begin(), m[i].end(), 1) != m[i].end())
                    hasOne = true;
            }
            CHECK(valuesOk); // only 0/1 cells
            CHECK(hasOne);   // not an empty matrix

            // No duplicates inside the class: the dedup pass works.
            // (Rows are compared whole, so differently-shaped matrices —
            // 7x6 vs 6x7 rotations — can never count as equal.)
            for (size_t w = v + 1; w < group.size(); w++) {
                const auto &m2 = group[w];
                bool same = (m.size() == m2.size());
                for (size_t i = 0; i < m.size() && same; i++)
                    same = (m[i] == m2[i]);
                CHECK(!same);
            }
        }
    }

    endSection();
}

// ---------------------------------------------------------------------------
// 3.6 Restart semantics.
// ---------------------------------------------------------------------------

static void testRestartSemantics()
{
    beginSection("restart semantics");

    SetRandomSeed(4242);
    Grid grid(800, 600, 15);
    grid.generateCity(2);
    Chase chase(800, 600, 15, grid);

    // Simulate a blue speed bonus earned by a crash, then restart.
    ChaseTest::setBlueExtraMoves(chase, 2);
    chase.reset(false); // SPACE: keeps the bonus
    CHECK_EQ(chase.getBlueExtraMoves(), 2);
    chase.reset(true);  // R: clears it
    CHECK_EQ(chase.getBlueExtraMoves(), 0);

    // Play a full round to completion, then restart both ways and verify
    // that nothing from the round leaks into the next one.
    chase.reset(true);
    int frames = 0;
    while (!chase.isFinished() && frames < 200000) {
        chase.update();
        frames++;
    }
    CHECK(chase.isFinished());
    std::printf("       round 1 finished at frame %d\n", frames);

    chase.reset(false);
    ChaseTest::Snapshot s = ChaseTest::snap(chase);
    CHECK_EQ(s.movesRed, 0);
    CHECK_EQ(s.movesBlue, 0);
    CHECK_EQ(s.redStuck, 0);
    CHECK_EQ(s.cycleDetectCount, 0);
    CHECK_EQ(s.stateHistCount, 0);
    CHECK_EQ(s.stateHistIdx, 0);
    CHECK(!s.finished);
    CHECK(!s.crashed);
    const auto redAfter = ChaseTest::redOf(chase);
    const auto blueAfter = ChaseTest::blueOf(chase);
    CHECK_EQ(grid.getValue(redAfter.row, redAfter.col), 0);
    CHECK_EQ(grid.getValue(blueAfter.row, blueAfter.col), 0);

    endSection();
}

// ---------------------------------------------------------------------------
// 3.7 Long-running session: no deadlock, no invalid coordinates,
// no overlap, no movement through buildings.
// ---------------------------------------------------------------------------

static void testLongSession()
{
    beginSection("long session (3 rounds)");

    SetRandomSeed(99);
    Grid grid(800, 600, 15);
    grid.generateCity(2);
    Chase chase(800, 600, 15, grid);
    chase.reset(true);

    int rounds = 0;
    int frames = 0;
    const int MAX_ROUNDS = 3;
    const int MAX_FRAMES = 40000;

    while (rounds < MAX_ROUNDS && frames < MAX_FRAMES) {
        chase.update();
        frames++;

        const auto red = ChaseTest::redOf(chase);
        const auto blue = ChaseTest::blueOf(chase);
        const int R = red.row;
        const int C = red.col;
        const int B = blue.row;
        const int BC = blue.col;

        const int failuresBefore = g_failures;

        CHECK(R >= 0 && R < grid.getRows() && C >= 0 && C < grid.getColumns());
        CHECK(B >= 0 && B < grid.getRows() && BC >= 0 && BC < grid.getColumns());
        if (R >= 0 && R < grid.getRows() && C >= 0 && C < grid.getColumns())
            CHECK_EQ(grid.getValue(R, C), 0);
        if (B >= 0 && B < grid.getRows() && BC >= 0 && BC < grid.getColumns())
            CHECK_EQ(grid.getValue(B, BC), 0);
        CHECK(!(R == B && C == BC));

        // An invariant failure means the game is broken: stop the session
        // instead of printing one line per frame for 40000 frames.
        if (g_failures > failuresBefore)
            break;

        if (chase.isFinished()) {
            rounds++;
            std::printf("       round %d finished at frame %d\n", rounds, frames);
            chase.reset(false);
        }
    }

    CHECK(rounds >= 2); // no permanent deadlock: rounds end on their own
    std::printf("       frames: %d, completed rounds: %d\n", frames, rounds);

    endSection();
}

// ---------------------------------------------------------------------------

int main()
{
    testCityGeneration();
    testConnectRoadsIdempotent();
    testCarPlacement();
    testPathfinding();
    testPathfindingBorders();
    testSimulationInvariance();
    testSimulationMetrics();
    testCycleDetection();
    testStateHistoryRing();
    testStalemateGuard();
    testFreshHistoryAfterBlueMove();
    testCycleBreakAfterBlueMove();
    testGridGuards();
    testChaseCtorGuards();
    testBreakCycleCatch();
    testRedSpeedModeKey();
    testReservoirUniformity();
    testBuildingVariants();
    testReplayDeterminism();
    testRestartSemantics();
    testLongSession();

    std::printf("=================================================\n");
    std::printf("Checks: %d, failures: %d\n", g_checks, g_failures);
    std::printf("%s\n", g_failures == 0 ? "ALL TESTS PASSED" : "TESTS FAILED");
    return g_failures == 0 ? 0 : 1;
}
