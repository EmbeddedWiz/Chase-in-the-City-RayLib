#include "chase.hpp"
#include <algorithm>
#include <cstdlib>
#include <limits>
#include <stdexcept>
#include <utility>

// ---------------------------------------------------------------------------
// Movement directions and named tuning constants (no magic numbers
// left in the decision logic).
// ---------------------------------------------------------------------------
namespace {

const int kRowDelta[] = {-1, 1, 0, 0};
const int kColDelta[] = {0, 0, -1, 1};

int DirIndex(int rowDelta, int colDelta)
{
    for (int dir = 0; dir < 4; dir++)
        if (kRowDelta[dir] == rowDelta && kColDelta[dir] == colDelta)
            return dir;
    return -1;
}

// --- red (pursuer) lookahead scoring ---
const int kScoreCaught = 100000;
const int kScoreEvaderStuck = 80000;
const int kScoreRedStuck = -50000;
const int kScoreCycledBase = -30000;
const int kScoreCycledFinalDist = -50;
const int kScoreCycledMinDist = 20;
const int kScoreFinalDist = -300;
const int kScoreMinDist = -100;
const int kScoreDirChange = 200;
const int kScoreContinueBonus = 500;
const int kScoreReversePenalty = -300;
const int kLookaheadAcceptThreshold = -5000;

// --- blue (evader) lookahead scoring ---
const int kScoreCaughtFlee = -200000;
const int kScoreStuckFlee = -100000;
const int kScoreCycledFleeBase = -25000;
const int kScoreCycledFleeMinDist = 200;
const int kScoreCycledFleeFinalDist = 100;
const int kScoreFleeMinDist = 3000;
const int kScoreFleeFinalDist = 500;
const int kScoreFleeDirChange = 300;
const int kScoreFleeContinueBonus = 8000;
const int kScoreFleeTurnPenalty = -3000;
const int kEdgeThreshold = 4;
const int kEdgePenaltyPerCell = 1500;

// --- movement schedule and round limits ---
const int kMoveTimerInterval = 8;
const int kBaseRedMoves = 2;
const int kSpeedUpMoves1 = 150;
const int kSpeedUpMoves2 = 300;
const int kMaxBlueExtraMoves = 3;
const int kStuckTicksToCrash = 12;
// Stalemate guard (no irreversible oscillation). If the
// Manhattan distance to blue stops improving (no new best) for
// kStalemateLimitTicks consecutive ticks, the round ends as a crash
// with the usual blue speed bonus. The red lookahead simulation is
// structurally more optimistic than the real blue escape policy (in
// the simulation blue flees with plain bfsAway, in the real game it
// replans every move with the continue/turn bonuses), so a
// near-miss loop can pin the distance at 9..15 forever; the guard
// terminates such rounds.
const int kStalemateLimitTicks = 400;
const int kCloseCatchDistance = 4;
const int kCloseCatchTimerGrow = 2;
const int kCloseCatchTimerLimit = 30;

// --- car placement on reset ---
const int kSpawnMinRoadDist = 12;
const int kSpawnMaxRoadDist = 50;
const int kSpawnMidTarget = 30;
const int kSpawnInBandScore = 100000;
const int kSpawnAttempts = 200;
const int kSpawnFarAttempts = 16;

// --- cycle deviation move ---
const int kDeviationLastDirPenalty = 5000;
const int kDeviationRandomSpread = 100;
const int kCycleDetectLimit = 2;

// --- unreachable-goal fallback in bfsToward ---
const int kFallbackWeight = 100;
const int kFallbackRandomSpread = 30;
}

Chase::Chase(int width, int height, int cellSize, Grid &gridRef) : cellSize(cellSize), grid(gridRef)
{
    // Validate before any arithmetic: cellSize == 0 would divide by zero
    // below, and a non-positive field holds no cells at all.
    if (width <= 0 || height <= 0 || cellSize <= 0)
        throw std::invalid_argument("Chase: width, height and cellSize must be positive");

    rows = height / cellSize;
    cols = width / cellSize;
    // The chase indexes the grid with its own rows/cols; a differently
    // sized Grid would be an out-of-bounds access, so reject it up front
    // to keep Chase and Grid sizes consistent.
    if (grid.getRows() != rows || grid.getColumns() != cols)
        throw std::invalid_argument("Chase: the passed Grid does not match width/height/cellSize");
    redCar = {0, 0, RED, "Red", 0, 0};
    blueCar = {0, 0, BLUE, "Blue", 0, 0};
    movesRed = movesBlue = 0;
    finished = false;
    crashed = false;
    blueExtraMoves = 0;
    moveTimer = 0;
    closeCatchTimer = 0;
    redStuck = 0;
    cycleDetectCount = 0;
    bestDist = 0;
    stalemateTicks = 0;

    stateHistIdx = 0;
    stateHistCount = 0;
}

void Chase::pushState(const ChaseState &state)
{
    stateHistory[stateHistIdx] = state;
    stateHistIdx = (stateHistIdx + 1) % STATE_HIST_SIZE;
    if (stateHistCount < STATE_HIST_SIZE)
        stateHistCount++;
}

bool Chase::isStateInHistory(const ChaseState &state) const
{
    int count = std::min(stateHistCount, STATE_HIST_SIZE);
    for (int i = 0; i < count; i++) {
        if (stateHistory[i] == state)
            return true;
    }
    return false;
}

int Chase::manhattanDist(int row1, int col1, int row2, int col2) const
{
    return abs(row1 - row2) + abs(col1 - col2);
}

bool Chase::isCellFree(int row, int col, const ChaseState &block) const
{
    if (row < 0 || row >= rows || col < 0 || col >= cols)
        return false;
    if (grid.getValue(row, col) != 0)
        return false;
    if ((row == block.redRow && col == block.redCol) || (row == block.blueRow && col == block.blueCol))
        return false;
    return true;
}

Chase::Step Chase::bfsToward(int startRow, int startCol, int goalRow, int goalCol,
                             BfsWorkspace &ws, const ChaseState &block)
{
    if (manhattanDist(startRow, startCol, goalRow, goalCol) <= 1)
        return {0, 0};

    auto idx = [this](int r, int c) { return (int)((size_t)r * cols + c); };

    ws.ensure(rows * cols);
    std::fill(ws.dist.begin(), ws.dist.end(), -1);
    std::fill(ws.parentDir.begin(), ws.parentDir.end(), -1);
    ws.queue.clear();

    ws.dist[idx(startRow, startCol)] = 0;
    ws.queue.push_back(idx(startRow, startCol));
    size_t qHead = 0;

    bool found = false;
    while (qHead < ws.queue.size() && !found) {
        int cur = ws.queue[qHead++];
        int r = cur / cols;
        int c = cur % cols;
        for (int dir = 0; dir < 4; dir++) {
            int nr = r + kRowDelta[dir];
            int nc = c + kColDelta[dir];
            if (nr < 0 || nr >= rows || nc < 0 || nc >= cols)
                continue;
            int ni = idx(nr, nc);
            if (ws.dist[ni] != -1)
                continue;
            if (grid.getValue(nr, nc) != 0)
                continue;

            // The goal cell is the target car's own cell: a road cell by
            // construction (no car ever sits in a building). It is accepted
            // *before* the blocked-cell check on purpose: the
            // goal is "the cell the other car occupies", so blocking it
            // would make the goal unreachable. Application sites (update,
            // breakCycle) re-check isCellFree() before applying the step,
            // so the cars never actually share a cell; only the simulation
            // applies the step, modelling the catch.
            if (nr == goalRow && nc == goalCol) {
                ws.dist[ni] = ws.dist[cur] + 1;
                ws.parentDir[ni] = dir;
                found = true;
                break;
            }

            if (!isCellFree(nr, nc, block))
                continue;

            ws.dist[ni] = ws.dist[cur] + 1;
            ws.parentDir[ni] = dir;
            ws.queue.push_back(ni);
        }
    }

    if (found) {
        int cr = goalRow;
        int cc = goalCol;
        while (ws.parentDir[idx(cr, cc)] != -1 && ws.dist[idx(cr, cc)] > 1) {
            int dir = ws.parentDir[idx(cr, cc)];
            cr -= kRowDelta[dir];
            cc -= kColDelta[dir];
        }
        if (manhattanDist(startRow, startCol, cr, cc) == 1)
            return {cr - startRow, cc - startCol};
        for (int dir = 0; dir < 4; dir++) {
            int nr = startRow + kRowDelta[dir];
            int nc = startCol + kColDelta[dir];
            if (nr == cr && nc == cc)
                return {kRowDelta[dir], kColDelta[dir]};
        }
    }

    // Fallback: no connected path. Move to a neighbour that reduces the
    // Manhattan distance to the goal, with a small random tie-breaker.
    // A negative row delta (moving up) is a valid fallback result.
    int curDist = manhattanDist(startRow, startCol, goalRow, goalCol);
    int bestDir = -1;
    int bestScore = std::numeric_limits<int>::min();

    for (int dir = 0; dir < 4; dir++) {
        int nr = startRow + kRowDelta[dir];
        int nc = startCol + kColDelta[dir];
        if (!isCellFree(nr, nc, block))
            continue;
        int nd = manhattanDist(nr, nc, goalRow, goalCol);
        int score = (curDist - nd) * kFallbackWeight + GetRandomValue(0, kFallbackRandomSpread);
        if (score > bestScore) {
            bestScore = score;
            bestDir = dir;
        }
    }

    if (bestDir >= 0)
        return {kRowDelta[bestDir], kColDelta[bestDir]};

    // No legal neighbour at all: the caller sees INVALID_STEP;
    // unreachable goals surface as an explicit status, not a silent move.
    return {INVALID_STEP, 0};
}

Chase::Step Chase::bfsAway(int startRow, int startCol, const ChaseState &block, BfsWorkspace &ws)
{
    auto idx = [this](int r, int c) { return (int)((size_t)r * cols + c); };

    // BFS from the follower position to compute distances to all cells.
    ws.ensure(rows * cols);
    std::fill(ws.dist.begin(), ws.dist.end(), -1);
    ws.queue.clear();

    int fr = block.redRow;
    int fc = block.redCol;
    ws.dist[idx(fr, fc)] = 0;
    ws.queue.push_back(idx(fr, fc));
    size_t qHead = 0;
    while (qHead < ws.queue.size()) {
        int cur = ws.queue[qHead++];
        int r = cur / cols;
        int c = cur % cols;
        for (int dir = 0; dir < 4; dir++) {
            int nr = r + kRowDelta[dir];
            int nc = c + kColDelta[dir];
            if (nr < 0 || nr >= rows || nc < 0 || nc >= cols)
                continue;
            int ni = idx(nr, nc);
            if (ws.dist[ni] != -1)
                continue;
            if (grid.getValue(nr, nc) != 0)
                continue;
            ws.dist[ni] = ws.dist[cur] + 1;
            ws.queue.push_back(ni);
        }
    }

    int bestDir = -1;
    int bestCellDist = -1;
    int bestEdge = -1;

    for (int dir = 0; dir < 4; dir++) {
        int nr = startRow + kRowDelta[dir];
        int nc = startCol + kColDelta[dir];
        if (!isCellFree(nr, nc, block))
            continue;

        int distFromCell = ws.dist[idx(nr, nc)];
        if (distFromCell < 0)
            continue;

        int edgeDist = std::min({nr, rows - 1 - nr, nc, cols - 1 - nc});

        if (distFromCell > bestCellDist) {
            bestCellDist = distFromCell;
            bestEdge = edgeDist;
            bestDir = dir;
        }
        else if (distFromCell == bestCellDist && bestDir >= 0) {
            // On equal distance to the follower, pick the one further from the edge.
            if (edgeDist > bestEdge) {
                bestEdge = edgeDist;
                bestDir = dir;
            }
        }
    }

    if (bestDir >= 0)
        return {kRowDelta[bestDir], kColDelta[bestDir]};

    // All neighbours blocked: nothing to run into.
    return {INVALID_STEP, 0};
}

Chase::SimResult Chase::simulate(const SimulationState &state, int firstRowDelta, int firstColDelta,
                                 int ticks, int redMovesPerTick, int blueMovesPerTick, bool actorIsRed)
{
    int simRedRow = state.redRow;
    int simRedCol = state.redCol;
    int simBlueRow = state.blueRow;
    int simBlueCol = state.blueCol;
    // The real game's scoring bonuses depend on the last move direction,
    // so the simulation continues from the cars' actual directions
    // (read-only: speculative state never writes back).
    int simRedDR = redCar.lastMoveRow;
    int simRedDC = redCar.lastMoveCol;
    int simBlueDR = blueCar.lastMoveRow;
    int simBlueDC = blueCar.lastMoveCol;

    // The first step extends the candidate direction under evaluation.
    if (actorIsRed) {
        simRedRow += firstRowDelta;
        simRedCol += firstColDelta;
        simRedDR = firstRowDelta;
        simRedDC = firstColDelta;
    }
    else {
        simBlueRow += firstRowDelta;
        simBlueCol += firstColDelta;
        simBlueDR = firstRowDelta;
        simBlueDC = firstColDelta;
    }

    int startDist = manhattanDist(simRedRow, simRedCol, simBlueRow, simBlueCol);
    int minDist = startDist;
    int prevDist = startDist;
    bool caught = false;
    bool cycled = false;
    bool stuck = false;
    bool evaderStuck = false;
    int dirChanges = 0;
    int prevDir = -1;
    int stagnant = 0;
    int totalSteps = 1;

    // Reusable speculative-state buffer: reserved to the worst
    // case once, cleared per run — both lookaheads and every candidate
    // direction share it, so no allocation happens on the hot path.
    simWs.ensure(ticks * (redMovesPerTick + blueMovesPerTick + 1) * 2 + 2);
    simWs.states.clear();

    // Remembers the current (position, direction) pair and reports a
    // speculative cycle. The global history is read-only here: speculative
    // states never pollute it.
    auto rememberState = [&]() -> bool {
        // The speculative key carries the sim's own red schedule (the
        // caller passes the real tick schedule, so the key stays
        // comparable with the global history entries).
        ChaseState test = {simRedRow, simRedCol, simBlueRow, simBlueCol,
                           simRedDR, simRedDC, simBlueDR, simBlueDC,
                           redMovesPerTick};
        if (isStateInHistory(test))
            return true;
        for (const ChaseState &prev : simWs.states)
            if (prev == test)
                return true;
        simWs.states.push_back(test);
        return false;
    };

    for (int s = 0; s < ticks && !(caught || cycled || stuck || evaderStuck); s++) {
        ChaseState block = {simRedRow, simRedCol, simBlueRow, simBlueCol,
                            simRedDR, simRedDC, simBlueDR, simBlueDC,
                            redMovesPerTick};

        // Red pursues: redMovesPerTick moves (the real schedule, not a constant).
        for (int m = 0; m < redMovesPerTick && !(caught || cycled || stuck); m++) {
            Step redStep = bfsToward(simRedRow, simRedCol, simBlueRow, simBlueCol, redWs, block);
            if (redStep.rowDelta == INVALID_STEP) {
                if (actorIsRed)
                    stuck = true; // the evaluated actor itself cannot move
                break; // for blue: a stuck pursuer is an advantage — it shows up as a large finalDist
            }
            // bfsToward returns {0,0} only when the cars are already adjacent
            // (Manhattan distance <= 1); the real game ends with a catch in
            // exactly that situation, so the prediction mirrors it.
            if (redStep.rowDelta == 0 && redStep.colDelta == 0) {
                caught = true;
                break;
            }

            simRedRow += redStep.rowDelta;
            simRedCol += redStep.colDelta;
            simRedDR = redStep.rowDelta;
            simRedDC = redStep.colDelta;
            totalSteps++;

            if (actorIsRed) {
                int dir = DirIndex(redStep.rowDelta, redStep.colDelta);
                if (prevDir >= 0 && dir != prevDir)
                    dirChanges++;
                prevDir = dir;
            }

            int dist = manhattanDist(simRedRow, simRedCol, simBlueRow, simBlueCol);
            minDist = std::min(minDist, dist);
            if (dist <= 1) {
                caught = true;
                break;
            }
            if (rememberState()) {
                cycled = true;
                break;
            }

            // The real update() refreshes the blocked cells after every
            // successful red move; the simulation mirrors it.
            block = {simRedRow, simRedCol, simBlueRow, simBlueCol,
                     simRedDR, simRedDC, simBlueDR, simBlueDC,
                     redMovesPerTick};
        }
        if (caught || cycled || stuck || evaderStuck)
            break;

        // Blue escapes: blueMovesPerTick moves.
        for (int m = 0; m < blueMovesPerTick && !(caught || cycled || stuck || evaderStuck); m++) {
            ChaseState blockNow = {simRedRow, simRedCol, simBlueRow, simBlueCol,
                                   simRedDR, simRedDC, simBlueDR, simBlueDC,
                                   redMovesPerTick};
            Step blueStep = bfsAway(simBlueRow, simBlueCol, blockNow, blueWs);
            if (blueStep.rowDelta == INVALID_STEP) {
                if (actorIsRed)
                    evaderStuck = true; // boxed-in evader: the pursuer's advantage
                else
                    stuck = true;       // the evaluated evader itself cannot move
                break;
            }
            // bfsAway only returns one of the four directions or INVALID_STEP
            // — never {0,0} — so there is no extra {0,0} branch here.

            simBlueRow += blueStep.rowDelta;
            simBlueCol += blueStep.colDelta;
            simBlueDR = blueStep.rowDelta;
            simBlueDC = blueStep.colDelta;
            totalSteps++;

            if (!actorIsRed) {
                int dir = DirIndex(blueStep.rowDelta, blueStep.colDelta);
                if (prevDir >= 0 && dir != prevDir)
                    dirChanges++;
                prevDir = dir;
            }

            int dist = manhattanDist(simRedRow, simRedCol, simBlueRow, simBlueCol);
            minDist = std::min(minDist, dist);
            if (dist <= 1) {
                caught = true;
                break;
            }
            if (rememberState()) {
                cycled = true;
                break;
            }

            if (!actorIsRed) {
                // Stagnation is measured on the evader's own progress
                // (the blue-actor engine semantics are preserved).
                if (dist >= prevDist)
                    stagnant++;
                else
                    stagnant = 0;
                prevDist = dist;
                if (stagnant > 2)
                    break; // the evader stopped gaining: truncate this segment
            }
        }
        if (caught || cycled || stuck || evaderStuck)
            break;

        if (actorIsRed) {
            // Tick-level stagnation: the pursuer stopped closing in.
            int curDist = manhattanDist(simRedRow, simRedCol, simBlueRow, simBlueCol);
            if (curDist >= prevDist)
                stagnant++;
            else
                stagnant = 0;
            prevDist = curDist;
            if (stagnant > ticks)
                break;
        }
    }

    SimResult result;
    result.finalDist = manhattanDist(simRedRow, simRedCol, simBlueRow, simBlueCol);
    result.minDist = minDist;
    result.caught = caught;
    result.cycled = cycled;
    result.stuck = stuck;
    result.evaderStuck = evaderStuck;
    result.dirChanges = dirChanges;
    if (actorIsRed) {
        result.netRowDelta = simRedRow - state.redRow;
        result.netColDelta = simRedCol - state.redCol;
    }
    else {
        result.netRowDelta = simBlueRow - state.blueRow;
        result.netColDelta = simBlueCol - state.blueCol;
    }
    result.totalSteps = totalSteps;
    return result;
}

Chase::Step Chase::bfsLookahead(int startRow, int startCol, int goalRow, int goalCol,
                                int ticks, int redMovesPerTick, int blueMovesPerTick)
{
    int bestDir = -1;
    int bestScore = std::numeric_limits<int>::min();
    int backupDir = -1;
    int cycleDir = -1;
    int bestCycleScore = std::numeric_limits<int>::min();

    // The simulation runs on the supplied coordinates only.
    SimulationState simState = {startRow, startCol, goalRow, goalCol};
    ChaseState block = {startRow, startCol, goalRow, goalCol, 0, 0, 0, 0};

    for (int dir = 0; dir < 4; dir++) {
        int nextRow = startRow + kRowDelta[dir];
        int nextCol = startCol + kColDelta[dir];
        if (!isCellFree(nextRow, nextCol, block))
            continue;

        SimResult sim = simulate(simState, kRowDelta[dir], kColDelta[dir],
                                 ticks, redMovesPerTick, blueMovesPerTick, true);

        int score;
        if (sim.caught) {
            score = kScoreCaught;
        }
        else if (sim.evaderStuck) {
            // Blue has no legal escape move in the simulation: it is
            // boxed in and the pursuer will catch it. Strong positive
            // score, second only to a confirmed catch.
            score = kScoreEvaderStuck;
        }
        else if (sim.stuck) {
            score = kScoreRedStuck;
        }
        else if (sim.cycled) {
            score = kScoreCycledBase + sim.finalDist * kScoreCycledFinalDist
                  + sim.minDist * kScoreCycledMinDist;
            // Track the best *cyclic* candidate separately from the
            // global best so a cyclic direction cannot win by accident.
            if (cycleDir < 0 || score > bestCycleScore) {
                cycleDir = dir;
                bestCycleScore = score;
            }
        }
        else {
            score = sim.finalDist * kScoreFinalDist + sim.minDist * kScoreMinDist;
            score += (4 - sim.dirChanges) * kScoreDirChange;

            if (redCar.lastMoveRow == kRowDelta[dir] && redCar.lastMoveCol == kColDelta[dir])
                score += kScoreContinueBonus;

            if (redCar.lastMoveRow == -kRowDelta[dir] && redCar.lastMoveCol == -kColDelta[dir])
                score += kScoreReversePenalty;
        }

        if (backupDir < 0)
            backupDir = dir;

        if (score > bestScore) {
            bestScore = score;
            bestDir = dir;
        }

    }

    if (bestDir >= 0 && bestScore > kLookaheadAcceptThreshold)
        return {kRowDelta[bestDir], kColDelta[bestDir]};

    // Fall back to plain BFS if the lookahead gives nothing usable.
    // Explicit comparison against INVALID_STEP: a negative row delta
    // (up) is a valid movement and must not be rejected.
    {
        Step bfsStep = bfsToward(startRow, startCol, goalRow, goalCol, redWs, block);
        if (bfsStep.rowDelta != INVALID_STEP &&
            isCellFree(startRow + bfsStep.rowDelta, startCol + bfsStep.colDelta, block)) {
            return bfsStep;
        }
    }

    // If all options are cyclic - pick the best cyclic one.
    if (cycleDir >= 0)
        return {kRowDelta[cycleDir], kColDelta[cycleDir]};

    // If all cells are busy - try backup.
    if (backupDir >= 0)
        return {kRowDelta[backupDir], kColDelta[backupDir]};

    // Full stuck - crash.
    return {INVALID_STEP, 0};
}

Chase::Step Chase::fleeLookahead(int startRow, int startCol, int followerRow, int followerCol,
                                 int ticks, int redMovesPerTick, int blueMovesPerTick)
{
    int bestDir = -1;
    int bestScore = std::numeric_limits<int>::min();
    int backupDir = -1;
    int cycleDir = -1;
    int bestCycleScore = std::numeric_limits<int>::min();

    // The simulation runs on the supplied coordinates only (start = the
    // evader, follower = the pursuer).
    SimulationState simState = {followerRow, followerCol, startRow, startCol};
    ChaseState block = {followerRow, followerCol, startRow, startCol, 0, 0, 0, 0};

    for (int dir = 0; dir < 4; dir++) {
        int nextRow = startRow + kRowDelta[dir];
        int nextCol = startCol + kColDelta[dir];
        if (!isCellFree(nextRow, nextCol, block))
            continue;

        SimResult sim = simulate(simState, kRowDelta[dir], kColDelta[dir],
                                 ticks, redMovesPerTick, blueMovesPerTick, false);

        int score;
        if (sim.caught) {
            score = kScoreCaughtFlee;
        }
        else if (sim.stuck) {
            score = kScoreStuckFlee;
        }
        else if (sim.cycled) {
            score = kScoreCycledFleeBase + sim.minDist * kScoreCycledFleeMinDist
                  + sim.finalDist * kScoreCycledFleeFinalDist;
            // Track the best *cyclic* candidate separately from the
            // global best so a cyclic direction cannot win by accident.
            if (cycleDir < 0 || score > bestCycleScore) {
                cycleDir = dir;
                bestCycleScore = score;
            }
        }
        else {
            score = sim.minDist * kScoreFleeMinDist + sim.finalDist * kScoreFleeFinalDist;
            score += (4 - sim.dirChanges) * kScoreFleeDirChange;

            if (blueCar.lastMoveRow == kRowDelta[dir] && blueCar.lastMoveCol == kColDelta[dir])
                score += kScoreFleeContinueBonus;

            if (blueCar.lastMoveRow == -kRowDelta[dir] && blueCar.lastMoveCol == -kColDelta[dir])
                score += kScoreFleeTurnPenalty;
        }

        // Edge avoidance: blue avoids field edges, prefers the center.
        {
            int edgeDist = std::min({nextRow, rows - 1 - nextRow, nextCol, cols - 1 - nextCol});
            if (edgeDist < kEdgeThreshold)
                score -= (kEdgeThreshold - edgeDist) * kEdgePenaltyPerCell;
        }

        if (backupDir < 0)
            backupDir = dir;

        if (score > bestScore) {
            bestScore = score;
            bestDir = dir;
        }
    }

    if (bestDir >= 0 && bestScore > kLookaheadAcceptThreshold)
        return {kRowDelta[bestDir], kColDelta[bestDir]};

    // Fall back to plain bfsAway if the lookahead gives nothing usable.
    // Explicit comparison against INVALID_STEP: a negative row delta
    // (up) is a valid movement and must not be rejected.
    {
        Step bfsStep = bfsAway(startRow, startCol, block, blueWs);
        if (bfsStep.rowDelta != INVALID_STEP &&
            isCellFree(startRow + bfsStep.rowDelta, startCol + bfsStep.colDelta, block)) {
            return bfsStep;
        }
    }

    if (cycleDir >= 0)
        return {kRowDelta[cycleDir], kColDelta[cycleDir]};
    if (backupDir >= 0)
        return {kRowDelta[backupDir], kColDelta[backupDir]};
    return {INVALID_STEP, 0};
}

void Chase::reset(bool hard)
{
    // Collect free road cells (margin of 2: cars do not spawn at field borders).
    std::vector<std::pair<int, int>> freeCells;
    for (int r = 2; r < rows - 2; r++)
        for (int c = 2; c < cols - 2; c++)
            if (grid.getValue(r, c) == 0)
                freeCells.push_back({r, c});

    int redRow = -1;
    int redCol = -1;
    int blueRow = -1;
    int blueCol = -1;
    bool placed = false;
    bool voidNetwork = false;

    auto idx = [this](int r, int c) { return (int)((size_t)r * cols + c); };
    std::vector<int> dist(rows * cols);
    std::vector<int> q;
    auto fillDist = [&](int sr, int sc) {
        std::fill(dist.begin(), dist.end(), -1);
        q.clear();
        dist[idx(sr, sc)] = 0;
        q.push_back(idx(sr, sc));
        size_t head = 0;
        while (head < q.size()) {
            int cur = q[head++];
            int r = cur / cols;
            int c = cur % cols;
            for (int d = 0; d < 4; d++) {
                int nr = r + kRowDelta[d];
                int nc = c + kColDelta[d];
                if (nr < 0 || nr >= rows || nc < 0 || nc >= cols)
                    continue;
                int ni = idx(nr, nc);
                if (dist[ni] != -1)
                    continue;
                if (grid.getValue(nr, nc) != 0)
                    continue;
                dist[ni] = dist[cur] + 1;
                q.push_back(ni);
            }
        }
    };

    if (freeCells.size() >= 2) {
        // Main strategy: two distinct road cells with an actual road
        // (BFS) distance in [12, 50]. Road connectivity is guaranteed
        // by Grid::connectRoads(), so distance 12 is reachable.
        for (int attempt = 0; attempt < kSpawnAttempts && !placed; attempt++) {
            int redIdx = GetRandomValue(0, (int)freeCells.size() - 1);
            fillDist(freeCells[redIdx].first, freeCells[redIdx].second);
            std::vector<int> band;
            for (int i = 0; i < (int)freeCells.size(); i++) {
                int d = dist[idx(freeCells[i].first, freeCells[i].second)];
                if (d >= kSpawnMinRoadDist && d <= kSpawnMaxRoadDist)
                    band.push_back(i);
            }
            if (!band.empty()) {
                int blueIdx = band[GetRandomValue(0, (int)band.size() - 1)];
                redRow = freeCells[redIdx].first;
                redCol = freeCells[redIdx].second;
                blueRow = freeCells[blueIdx].first;
                blueCol = freeCells[blueIdx].second;
                placed = true;
            }
        }

        if (!placed) {
            // Explicit fallback: approximate the farthest pair on the road
            // network, preferring a distance close to the [12, 50] range.
            int bestScore = std::numeric_limits<int>::min();
            for (int attempt = 0; attempt < kSpawnFarAttempts; attempt++) {
                int redIdx = GetRandomValue(0, (int)freeCells.size() - 1);
                fillDist(freeCells[redIdx].first, freeCells[redIdx].second);
                int farIdx = -1;
                int farDist = 0;
                for (int i = 0; i < (int)freeCells.size(); i++) {
                    int d = dist[idx(freeCells[i].first, freeCells[i].second)];
                    if (d > farDist) {
                        farDist = d;
                        farIdx = i;
                    }
                }
                if (farIdx < 0)
                    continue;
                int score = (farDist >= kSpawnMinRoadDist && farDist <= kSpawnMaxRoadDist)
                    ? kSpawnInBandScore - abs(farDist - kSpawnMidTarget)
                    : -abs(farDist - kSpawnMidTarget);
                if (score > bestScore) {
                    bestScore = score;
                    redRow = freeCells[redIdx].first;
                    redCol = freeCells[redIdx].second;
                    blueRow = freeCells[farIdx].first;
                    blueCol = freeCells[farIdx].second;
                    placed = true;
                }
            }
        }
    }

    if (!placed) {
        // Final safety net: any two distinct free cells. Never rewrite a
        // building cell just to make the cars fit.
        if (freeCells.size() >= 2) {
            redRow = freeCells[0].first;
            redCol = freeCells[0].second;
            blueRow = freeCells[1].first;
            blueCol = freeCells[1].second;
        }
        else {
            // No usable road network at all: both cars have no distinct
            // road cell to stand on. The round must NOT stay active with
            // both cars on the same (0,0) cell.
            redRow = 0;
            redCol = 0;
            blueRow = 0;
            blueCol = 0;
            voidNetwork = true;
        }
    }

    redCar.row = redRow;
    redCar.col = redCol;
    redCar.lastMoveRow = 0;
    redCar.lastMoveCol = 0;
    blueCar.row = blueRow;
    blueCar.col = blueCol;
    blueCar.lastMoveRow = 0;
    blueCar.lastMoveCol = 0;
    movesRed = movesBlue = 0;
    finished = voidNetwork; // no road network: the game layer restarts
    crashed = false;
    moveTimer = 0;
    closeCatchTimer = 0;
    redStuck = 0;
    cycleDetectCount = 0;
    // The stalemate guard measures improvements against the spawn distance.
    bestDist = manhattanDist(redRow, redCol, blueRow, blueCol);
    stalemateTicks = 0;

    stateHistIdx = 0;
    stateHistCount = 0;

    if (hard) {
        blueExtraMoves = 0;
    }
}

int Chase::currentRedSpeedMode() const
{
    int mode = kBaseRedMoves;
    if (movesRed > kSpeedUpMoves1)
        mode = 3; // red speeds up during long chase
    if (movesRed > kSpeedUpMoves2)
        mode = 4; // even more speed up
    return mode;
}

void Chase::update()
{
    if (finished)
        return;
    moveTimer++;
    if (moveTimer < kMoveTimerInterval)
        return; // a bit faster
    moveTimer = 0;

    // The actual movement schedule for this tick. The same values feed the
    // lookahead simulations so the prediction mirrors the real game.
    int redMovesPerTick = currentRedSpeedMode();
    int blueMovesPerTick = 1 + blueExtraMoves;

    ChaseState block = {redCar.row, redCar.col, blueCar.row, blueCar.col,
                        redCar.lastMoveRow, redCar.lastMoveCol,
                        blueCar.lastMoveRow, blueCar.lastMoveCol,
                        currentRedSpeedMode()};
    bool movedRedThisTick = false;

    for (int i = 0; i < redMovesPerTick && !finished; i++) {
        Step step = bfsLookahead(redCar.row, redCar.col, blueCar.row, blueCar.col,
                                 LOOKAHEAD_TICKS, redMovesPerTick, blueMovesPerTick);

        if (step.rowDelta == INVALID_STEP) {
            // Red has nowhere to go: crash.
            crashed = true;
            finished = true;
            blueExtraMoves = std::min(blueExtraMoves + 1, kMaxBlueExtraMoves);
            return;
        }

        if (step.rowDelta == 0 && step.colDelta == 0)
            break;

        int nextRow = redCar.row + step.rowDelta;
        int nextCol = redCar.col + step.colDelta;

        if (isCellFree(nextRow, nextCol, block)) {
            redCar.row = nextRow;
            redCar.col = nextCol;
            redCar.lastMoveRow = step.rowDelta;
            redCar.lastMoveCol = step.colDelta;
            movesRed++; // count a real, successful move only
            movedRedThisTick = true;
        }

        if (manhattanDist(redCar.row, redCar.col, blueCar.row, blueCar.col) <= 1) {
            finished = true;
            return;
        }
        // Refresh the blocked cells after every successful red move,
        // exactly like the simulation does.
        block = {redCar.row, redCar.col, blueCar.row, blueCar.col,
                 redCar.lastMoveRow, redCar.lastMoveCol,
                 blueCar.lastMoveRow, blueCar.lastMoveCol,
                 currentRedSpeedMode()};
    }

    // Stuck detector: the counter grows only when no successful move
    // happened during the tick (a car that moved out and back is not stuck).
    if (movedRedThisTick) {
        redStuck = 0;
    }
    else {
        redStuck++;
        if (redStuck >= kStuckTicksToCrash && !finished) {
            crashed = true;
            finished = true;
            blueExtraMoves = std::min(blueExtraMoves + 1, kMaxBlueExtraMoves);
            redStuck = 0;
            return;
        }
    }

    for (int blueMove = 0; blueMove < blueMovesPerTick && !finished; blueMove++) {
        Step step = fleeLookahead(blueCar.row, blueCar.col, redCar.row, redCar.col,
                                  LOOKAHEAD_TICKS, redMovesPerTick, blueMovesPerTick);

        if (step.rowDelta == INVALID_STEP) {
            break; // blue is boxed in; the red car will close in
        }

        if (step.rowDelta == 0 && step.colDelta == 0)
            break;

        int nextRow = blueCar.row + step.rowDelta;
        int nextCol = blueCar.col + step.colDelta;

        block = {redCar.row, redCar.col, blueCar.row, blueCar.col,
                 redCar.lastMoveRow, redCar.lastMoveCol,
                 blueCar.lastMoveRow, blueCar.lastMoveCol,
                 currentRedSpeedMode()};
        if (isCellFree(nextRow, nextCol, block)) {
            blueCar.row = nextRow;
            blueCar.col = nextCol;
            blueCar.lastMoveRow = step.rowDelta;
            blueCar.lastMoveCol = step.colDelta;
            movesBlue++; // count a real, successful move only
        }

        if (manhattanDist(redCar.row, redCar.col, blueCar.row, blueCar.col) <= 1) {
            finished = true;
            return;
        }
    }

    // Rebuild the state key from the actual post-tick car fields: the
    // block captured inside the blue loop predates the last successful
    // blue move. The pushed state and breakCycle() must
    // both see the real game state, including blue's new position and its
    // new last-move direction.
    block = {redCar.row, redCar.col, blueCar.row, blueCar.col,
             redCar.lastMoveRow, redCar.lastMoveCol,
             blueCar.lastMoveRow, blueCar.lastMoveCol,
             currentRedSpeedMode()};
    pushState(block);

    // Detection and reaction are separate: detectCyclePeriod()
    // only inspects the history; breakCycle() performs the deviation move.
    int period = detectCyclePeriod();
    if (period >= 0) {
        cycleDetectCount++;
        if (cycleDetectCount >= kCycleDetectLimit) {
            stateHistIdx = 0;
            stateHistCount = 0;
            cycleDetectCount = 0;
            breakCycle(block);
        }
    }
    else {
        cycleDetectCount = 0;
    }

    // breakCycle() may have ended the round as a catch — no further
    // moves or guards this tick.
    if (finished)
        return;

    // Stalemate guard (no irreversible oscillation).
    updateStalemateGuard();
    if (finished)
        return;

    // Close-catch counter.
    int dist = manhattanDist(redCar.row, redCar.col, blueCar.row, blueCar.col);
    if (dist <= kCloseCatchDistance) {
        closeCatchTimer += kCloseCatchTimerGrow;
    }
    else {
        if (closeCatchTimer > 0)
            closeCatchTimer--;
    }
    if (closeCatchTimer >= kCloseCatchTimerLimit) {
        finished = true;
    }
}

int Chase::detectCyclePeriod() const
{
    int n = std::min(stateHistCount, STATE_HIST_SIZE);

    // Check repeating pattern of period 2-6 (detection only — the reaction
    // lives in breakCycle()).
    for (int period = 2; period <= 6; period++) {
        if (n < period * 2)
            continue;
        bool match = true;
        for (int i = 0; i < period && match; i++) {
            int index1 = (stateHistIdx - 1 - i + STATE_HIST_SIZE) % STATE_HIST_SIZE;
            int index2 = (stateHistIdx - 1 - i - period + STATE_HIST_SIZE) % STATE_HIST_SIZE;
            const ChaseState &state1 = stateHistory[index1];
            const ChaseState &state2 = stateHistory[index2];
            if (!(state1 == state2))
                match = false;
        }
        if (match)
            return period;
    }
    return -1;
}

void Chase::breakCycle(ChaseState &block)
{
    // Rebuild the key from the actual car fields: the caller's block may
    // predate a blue move, and the deviation move must check the real
    // blue position so the cars can never end up on the same cell.
    block = {redCar.row, redCar.col, blueCar.row, blueCar.col,
             redCar.lastMoveRow, redCar.lastMoveCol,
             blueCar.lastMoveRow, blueCar.lastMoveCol,
             currentRedSpeedMode()};

    // Deviation move: a random direction, penalized when it continues the
    // last move; blue takes one bfsAway step (the reaction half).
    int bestDir = -1;
    int bestScore = std::numeric_limits<int>::min();
    for (int dir = 0; dir < 4; dir++) {
        int nextRow = redCar.row + kRowDelta[dir];
        int nextCol = redCar.col + kColDelta[dir];
        if (!isCellFree(nextRow, nextCol, block))
            continue;
        int score = 0;
        if (kRowDelta[dir] == redCar.lastMoveRow && kColDelta[dir] == redCar.lastMoveCol)
            score -= kDeviationLastDirPenalty;
        score += GetRandomValue(0, kDeviationRandomSpread);
        if (score > bestScore) {
            bestScore = score;
            bestDir = dir;
        }
    }
    if (bestDir >= 0 && isCellFree(redCar.row + kRowDelta[bestDir], redCar.col + kColDelta[bestDir], block)) {
        redCar.row += kRowDelta[bestDir];
        redCar.col += kColDelta[bestDir];
        redCar.lastMoveRow = kRowDelta[bestDir];
        redCar.lastMoveCol = kColDelta[bestDir];
        movesRed++;

        // The deviation move may close the gap: end the round as a normal
        // catch BEFORE blue gets its move — the main loop's catch rule
        // (finished only, no crash).
        if (manhattanDist(redCar.row, redCar.col, blueCar.row, blueCar.col) <= 1) {
            finished = true;
            block = {redCar.row, redCar.col, blueCar.row, blueCar.col,
                     redCar.lastMoveRow, redCar.lastMoveCol,
                     blueCar.lastMoveRow, blueCar.lastMoveCol,
                     currentRedSpeedMode()};
            pushState(block);
            return;
        }
    }

    Step blueStep = bfsAway(blueCar.row, blueCar.col,
                            ChaseState{redCar.row, redCar.col, blueCar.row, blueCar.col,
                                       redCar.lastMoveRow, redCar.lastMoveCol,
                                       blueCar.lastMoveRow, blueCar.lastMoveCol,
                                       currentRedSpeedMode()},
                            blueWs);
    if (blueStep.rowDelta != INVALID_STEP) {
        int nextRow = blueCar.row + blueStep.rowDelta;
        int nextCol = blueCar.col + blueStep.colDelta;
        block = {redCar.row, redCar.col, blueCar.row, blueCar.col,
                 redCar.lastMoveRow, redCar.lastMoveCol,
                 blueCar.lastMoveRow, blueCar.lastMoveCol,
                 currentRedSpeedMode()};
        if (isCellFree(nextRow, nextCol, block)) {
            blueCar.row = nextRow;
            blueCar.col = nextCol;
            blueCar.lastMoveRow = blueStep.rowDelta;
            blueCar.lastMoveCol = blueStep.colDelta;
            movesBlue++;

            // A boxed-in escape can walk blue right back onto red's
            // doorstep: same catch rule as the main loop.
            if (manhattanDist(redCar.row, redCar.col, blueCar.row, blueCar.col) <= 1)
                finished = true;
        }
    }
    // Consistency: the pushed key is rebuilt from the actual post-move car
    // fields, including the freshly derived speed mode.
    block = {redCar.row, redCar.col, blueCar.row, blueCar.col,
             redCar.lastMoveRow, redCar.lastMoveCol,
             blueCar.lastMoveRow, blueCar.lastMoveCol,
             currentRedSpeedMode()};
    pushState(block);
}

void Chase::updateStalemateGuard()
{
    // End the round when the best distance stops improving for
    // kStalemateLimitTicks consecutive ticks: the cars are locked in an
    // irreversible oscillation (the red lookahead is structurally more
    // optimistic than the real blue escape policy, so a near-miss loop
    // can pin the distance forever). The round is scored as a red crash
    // so blue earns the usual speed bonus, like every other
    // non-catch ending.
    int dist = manhattanDist(redCar.row, redCar.col, blueCar.row, blueCar.col);
    if (dist < bestDist) {
        bestDist = dist;
        stalemateTicks = 0;
    }
    else if (++stalemateTicks >= kStalemateLimitTicks) {
        crashed = true;
        finished = true;
        blueExtraMoves = std::min(blueExtraMoves + 1, kMaxBlueExtraMoves);
    }
}

void Chase::draw()
{
    DrawRectangle(redCar.col * cellSize + 2, redCar.row * cellSize + 2, cellSize - 4, cellSize - 4, RED);
    DrawRectangle(blueCar.col * cellSize + 2, blueCar.row * cellSize + 2, cellSize - 4, cellSize - 4, BLUE);
}
