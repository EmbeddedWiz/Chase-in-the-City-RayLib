#pragma once

#include "grid.hpp"
#include <cstddef>
#include <raylib.h>
#include <vector>

// All movement data types below are nested and private: they are internal
// to the chase logic. The test harness reaches them through the
// CHASE_TESTING friend declaration; outside of that they never leave
// the class.
class Chase {
  public:
    Chase(int width, int height, int cellSize, Grid &gridRef);

    // hard = false: keeps the blue car speed bonus (SPACE after catch/crash)
    // hard = true:  full reset, including the speed bonus (R - new city)
    void reset(bool hard = false);
    void update();
    void draw();

    bool isFinished() const { return finished; }
    bool isCrashed() const { return crashed; }
    int getBlueExtraMoves() const { return blueExtraMoves; }

#ifdef CHASE_TESTING
    friend class ChaseTest;
#endif

  private:
    struct Car {
        int row;
        int col;
        Color color;
        const char *name;
        int lastMoveRow;
        int lastMoveCol;
    };

    // Cycle-detection key. It must capture everything deterministic that
    // feeds the next step: the positions of both cars AND their last move
    // directions (the scoring bonuses depend on the last direction). The
    // red car's speed MODE (2/3/4 moves per tick, set by the 150/300-move
    // thresholds) is part of the key too: the same positions and
    // directions under different red schedules do not guarantee the same
    // future. The raw movesRed counter is intentionally NOT part of the
    // key — it keeps growing; only the derived schedule matters.
    struct ChaseState {
        int redRow;
        int redCol;
        int blueRow;
        int blueCol;
        int redLastRowDelta;
        int redLastColDelta;
        int blueLastRowDelta;
        int blueLastColDelta;
        int redSpeedMode;

        bool operator==(const ChaseState &other) const
        {
            return redRow == other.redRow && redCol == other.redCol
                && blueRow == other.blueRow && blueCol == other.blueCol
                && redLastRowDelta == other.redLastRowDelta
                && redLastColDelta == other.redLastColDelta
                && blueLastRowDelta == other.blueLastRowDelta
                && blueLastColDelta == other.blueLastColDelta
                && redSpeedMode == other.redSpeedMode;
        }
    };

    // Hypothetical position of both cars, used by the lookahead simulations.
    // Simulations operate on this value type only; the real Car objects are
    // never modified by a speculative move.
    struct SimulationState {
        int redRow;
        int redCol;
        int blueRow;
        int blueCol;
    };

    // Result of a next-step computation.
    struct Step {
        int rowDelta;
        int colDelta;
    };

    // Explicit "no valid move" status instead of the magic value {-2, 0}.
    // rowDelta == INVALID_STEP means the step must not be applied;
    // {-1, 0} (up) remains a perfectly valid movement direction.
    static const int INVALID_STEP = -2;

    // Result of one lookahead simulation run.
    struct SimResult {
        int finalDist;
        int minDist;
        bool caught;
        bool cycled;
        bool stuck;        // the evaluated actor had no legal move
        bool evaderStuck;  // blue (the evader) had no legal move:
                           // the pursuer's advantage, a GOOD result for red
        int dirChanges;
        int netRowDelta;
        int netColDelta;
        int totalSteps;
    };

    int cellSize;
    int rows;
    int cols;
    Grid &grid;

    Car redCar;
    Car blueCar;

    int movesRed;
    int movesBlue;
    bool finished;
    bool crashed;
    int blueExtraMoves;
    int moveTimer;
    int closeCatchTimer;

    // Instance-level counters (previously local statics that leaked
    // across resets and shared instances).
    int redStuck;
    int cycleDetectCount;

    // Stalemate guard: best (lowest) Manhattan distance
    // red reached this round, and the streak of ticks without a new
    // best. When the streak reaches kStalemateLimitTicks the round
    // ends as a red crash (blue earns the usual speed bonus, like
    // every other non-catch ending), because the two cars are locked
    // in an irreversible oscillation the cycle detector cannot resolve.
    int bestDist;
    int stalemateTicks;

    // Number of full simulated cycles used by the lookahead (kept in sync
    // with the actual movement schedule passed per call).
    static const int LOOKAHEAD_TICKS = 4;

    // Reusable BFS buffers: flat arrays sized once to rows * cols and
    // refilled on every search (no per-call allocation).
    struct BfsWorkspace {
        std::vector<int> dist;
        std::vector<int> parentDir;
        std::vector<int> queue; // flat cell indices, head pointer kept locally
        int cellCount = 0;

        void ensure(int totalCells)
        {
            if (cellCount == totalCells)
                return;
            cellCount = totalCells;
            dist.assign(totalCells, -1);
            parentDir.assign(totalCells, -1);
            queue.reserve(totalCells);
        }
    };
    BfsWorkspace redWs;
    BfsWorkspace blueWs;

    // Reusable speculative-state list for the simulation. Both lookaheads
    // and every candidate direction share this one buffer: it is reserved
    // to the worst case (LOOKAHEAD_TICKS * (4+4+1) * 2) once and cleared
    // per run — no allocation on the hot path.
    struct SimWorkspace {
        std::vector<ChaseState> states;
        int reservedFor = 0;

        void ensure(int n)
        {
            if (reservedFor < n) {
                reservedFor = n;
                states.reserve(n);
            }
        }
    };
    SimWorkspace simWs;

    // One simulation engine for both actors: runs `ticks` full cycles of
    // the real movement schedule (redMovesPerTick / blueMovesPerTick) and
    // extends the first step (firstRowDelta, firstColDelta) chosen for
    // the actor. `actorIsRed` selects who takes the first step and whose
    // direction changes are tracked; the other car is the opponent.
    SimResult simulate(const SimulationState &state, int firstRowDelta, int firstColDelta,
                       int ticks, int redMovesPerTick, int blueMovesPerTick, bool actorIsRed);

    // Lookahead simulation, used to pick the red car step.
    Step bfsLookahead(int startRow, int startCol, int goalRow, int goalCol,
                      int ticks, int redMovesPerTick, int blueMovesPerTick);
    // Lookahead escape simulation, used to pick the blue car step.
    Step fleeLookahead(int startRow, int startCol, int followerRow, int followerCol,
                       int ticks, int redMovesPerTick, int blueMovesPerTick);

    // BFS helpers. Blocked cells are passed explicitly (real or simulated
    // car positions) so a search never depends on the mutable Car objects.
    Step bfsToward(int startRow, int startCol, int goalRow, int goalCol,
                   BfsWorkspace &ws, const ChaseState &block);
    Step bfsAway(int startRow, int startCol, const ChaseState &block, BfsWorkspace &ws);

    // Global state history used for cycle detection.
    static const int STATE_HIST_SIZE = 256;
    ChaseState stateHistory[STATE_HIST_SIZE];
    int stateHistIdx;
    int stateHistCount;

    bool isStateInHistory(const ChaseState &state) const;
    void pushState(const ChaseState &state);

    // Detection and reaction are separate:
    // detectCyclePeriod() only inspects the history and returns the
    // repeated period (2..6) or -1; breakCycle() performs the deviation
    // move that escapes a confirmed cycle.
    int detectCyclePeriod() const;
    void breakCycle(ChaseState &block);

    // The red car's movement schedule for the next tick (2 -> 3 after
    // kSpeedUpMoves1 successful moves, 4 after kSpeedUpMoves2). The part
    // of the state that depends on movesRed is this derived mode, not the
    // ever-growing counter itself.
    int currentRedSpeedMode() const;

    // Ends a round that never improves its best distance (see the
    // kStalemateLimitTicks comment in chase.cpp for the reasoning).
    void updateStalemateGuard();

    // Helpers.
    int manhattanDist(int row1, int col1, int row2, int col2) const;
    bool isCellFree(int row, int col, const ChaseState &block) const;
};
