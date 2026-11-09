# City Chase

City Chase is a small arcade simulation written in C++17 with [raylib](https://www.raylib.com).
The city is a procedurally generated grid of streets and buildings. The **red car is the
police (the pursuer)** — it drives toward the blue car; the **blue car is the thief
(the evader)** — it runs away. The chase ends when the police catches the thief, or
when the police crashes (has no legal move), which hands the thief a speed bonus
for the next rounds.

## Controls

- `R` — new city + full reset (clears the blue speed bonus)
- `SPACE` — restart the chase on the same city (keeps the blue speed bonus)
- After a crash or catch, a game-over panel is shown and the game soft-restarts
  automatically after 7 seconds (420 frames at 60 FPS)

## Project structure

```
src/
  main.cpp       — game loop, screens, HUD
  grid.cpp       — city grid: building library, generation, road repair
  grid.hpp
  chase.cpp      — chase logic, BFS, lookahead AI, cycle/stuck detection
  chase.hpp
tests/
  test_main.cpp  — headless logic tests (chase_tests target)
CMakeLists.txt   — canonical build (game + tests)
Makefile         — thin delegate to CMake (all/debug/release/test/clean)
```

## How the code works

### City generation (`grid.cpp`, `grid.hpp`)

The city is a 2D grid where `0` means street and `1..6` means a building of the
corresponding size class. `Grid` validates its constructor arguments and guards all
writes; `getValue()` returns `-1` for out-of-bounds coordinates, so a missing cell
can never be mistaken for a traversable road.

**Building library.** `initBuildings()` defines source patterns for six size classes
(2×2, 3×3, 4×4, 5×5, 6×6, 7×7). For every source pattern the three remaining 90°
rotations are generated and duplicates removed. The result is stored in
`buildings[sizeIndex][variant][row][col]` (`sizeIndex`: 0 = 2×2 … 5 = 7×7).
Every source pattern is a square matrix. The 7×7 pattern is zero-padded: its
top row has 7 cells, the remaining rows 6, and the missing trailing cells are
an empty margin — the padding keeps `canPlace()` from reading `row 6, col 6`
out of bounds.

**Placement strategy** (`generateCity(clearance)`). Buildings are placed from largest
to smallest in seven passes, each with a different road clearance (the empty margin
kept around a building):

| Phase | Size class | Clearance | Iteration cap |
|-------|-----------|-----------|---------------|
| 1 | 7×7 → 6×6 → 5×5 | `clearance` (2 in the game) | `rows × columns`, stops at the first failed attempt |
| 2 | 4×4 | `clearance` | `rows × columns / 2` |
| 3 | 4×4 | `1` | `rows × columns / 2` |
| 4 | 3×3 | `clearance` | `rows × columns` |
| 5 | 3×3 | `1` | `rows × columns` |
| 6 | 2×2 | `2` | `rows × columns / 4` |
| 7 | 2×2 | `1` | `rows × columns / 4` |

`tryPlaceGroup()` picks one uniformly random valid candidate via reservoir sampling
(`canPlace()` — the rectangle expanded by `clearance` must be fully inside the grid
and contain only street cells), so no candidate list is ever materialized (O(1)
extra memory). `place()` writes the building cells with their class value.

**Road repair** (`connectRoads()`). After placement, some street cells can be
isolated inside building clusters. `connectRoads()` repeatedly flood-fills from the
first road cell, finds an isolated road cell, locates the closest main-component
cell by Manhattan distance, and carves an L-shaped corridor between them through
buildings (horizontal-first or vertical-first chosen randomly). The loop has no
iteration cap: every carving step merges one isolated component into the main one,
so it always terminates. `isRoadNetworkConnected()` re-checks the result with an
independent BFS count and is used by the tests.

### Car placement (`Chase::reset`)

Cars spawn on two distinct road cells, at least 2 cells away from the field border.
The main strategy samples road cells until the actual **road (BFS) distance** between
the two cars lands in the `[12, 50]` band (up to 200 attempts); this is guaranteed to be
reachable because `connectRoads()` leaves a single connected road network. If that
fails, an explicit fallback approximates the farthest road pair (16 attempts), and a
final safety net picks the first two free cells. Placement never rewrites a building
cell. `reset(false)` keeps the blue speed bonus, `reset(true)` clears it.

### Chase AI (`chase.cpp`, `chase.hpp`)

Both cars move only on road cells and never share a cell. `Chase::update()` ticks at
60 FPS with a `moveTimer` divider of 8. Every tick the cars move several times; the
same schedule feeds the lookahead simulations, so the prediction mirrors the real
game:

| Condition | Red moves per tick | Blue moves per tick |
|-----------|--------------------|--------------------|
| default | 2 | `1 + blueExtraMoves` |
| `movesRed > 150` | 3 | `1 + blueExtraMoves` |
| `movesRed > 300` | 4 | `1 + blueExtraMoves` |

A move that was actually applied counts toward `movesRed` / `movesBlue`; a blocked or
failed step does not.

**Movement API.** A step is a `{rowDelta, colDelta}` pair computed by a helper; the
explicit constant `Chase::INVALID_STEP` (`rowDelta == -2`) is the only "no valid
move" sentinel. A negative row delta (moving up, `-1`) is a perfectly valid
direction everywhere — fallbacks compare against `INVALID_STEP`, never against
zero or positivity.

**BFS helpers.** Blocked cells (the two car positions, real or simulated) are passed
as an explicit `ChaseState`, so a search never depends on the mutable `Car` objects.
BFS runs on reusable flat buffers (`BfsWorkspace`: `dist`, `parentDir`, `queue`
vectors sized once to `rows * cols` and refilled per search — no per-call
allocation).

- `bfsToward()` — BFS from the pursuer to the evader. The goal cell is accepted even
  though it is blocked (the pursuer wants to end up adjacent to it). When no
  connected path exists, a greedy fallback scores each free neighbour by
  `(curDist - nd) * 100 + random(0,30)` and moves to the best one; if there is no
  free neighbour at all, `INVALID_STEP` is returned.
- `bfsAway()` — BFS from the pursuer position computes distances to every reachable
  cell; the evader picks the neighbour farthest from the pursuer, ties broken by
  distance to the field edge (prefers the center).

**Lookahead simulations.** Each candidate first step is extended by a speculative
simulation of `LOOKAHEAD_TICKS` (= 4) full cycles using the real movement schedule.
Simulations operate on a value type (`SimulationState`) only — the real car objects
are never touched, and speculative states never pollute the global history
(read-only check). `SimResult` reports `finalDist`, `minDist`, `caught`, `cycled`,
`stuck` (the evaluated actor had no legal move), `evaderStuck` (the evader had no
legal move — a trap, which is a *good* result for the pursuer), `dirChanges`, net
displacement and total steps.

`bfsLookahead()` (red) scores each direction:
- `caught` → `100000` (highest priority)
- `evaderStuck` → `80000`
- `stuck` → `-50000`
- `cycled` → `-30000 - finalDist*50 + minDist*20` — the best cyclic candidate is
  tracked separately (`bestCycleScore`) so a cyclic direction can only win when
  *all* directions are cyclic
- otherwise → `-finalDist*300 - minDist*100 + (4-dirChanges)*200`, `+500` for
  continuing the current heading, `-300` for reversing

If the best score is above `-5000`, that direction is chosen; otherwise plain BFS
(`bfsToward`) is the fallback; then the best cyclic direction; then the first free
backup; and only if nothing works, `INVALID_STEP` (crash).

`fleeLookahead()` (blue) scores each direction:
- `caught` → `-200000` (worst)
- `stuck` → `-100000`
- `cycled` → `-25000 + minDist*200 + finalDist*100` (tracked separately, same
  rule as above)
- otherwise → `minDist*3000 + finalDist*500 + (4-dirChanges)*300`, `+8000` for
  continuing the current heading, `-3000` for reversing

Edge avoidance subtracts `(4 - edgeDist) * 1500` when the candidate cell is within 4
rows/columns of the border, keeping the evader away from the edges. The fallback
chain is the same as for red, with `bfsAway` in place of `bfsToward`.

**Stuck detector.** `redStuck` grows only when no successful red move happened
during a tick (a car that moved out and back is not stuck). 12 consecutive
stuck ticks → crash, and the blue car gains +1 speed bonus (capped at 3).

**Cycle detection.** The last 256 global states are kept in a ring buffer
(`stateHistory`). The state key is 9 fields: the positions of both cars, their
last-move directions (the scoring bonuses depend on them) and the current
red speed mode (2/3/4 moves per tick, derived from the 150/300-move
thresholds) — under a different red schedule the same positions and
directions do not guarantee the same future, so the derived mode, not the
raw ever-growing `movesRed` counter, is part of the key. After each
tick, `update()` checks for repeating patterns of period 2–6
(`detectCyclePeriod()`). When a cycle is detected twice in a row, `breakCycle()`
clears the history, the red car makes a random deviation move (penalising the
direction that caused the cycle) and the blue car makes one `bfsAway` move.
After every successful deviation move the main-loop catch rule is re-checked
(`manhattanDist <= 1` → the round ends as a catch, `finished` only, no
crash) and no further move is made once the round has ended.

**Close-catch counter.** While the cars stay within Manhattan distance 4 the timer
grows by 2 per tick; at distance 5+ it decays by 1. At 30 the chase ends as a catch.

**Stalemate guard.** The red car's lookahead simulation is structurally more
optimistic than the real blue escape policy (in the simulation blue flees with
plain `bfsAway`; in the real game it replans every move with `fleeLookahead` and
its continue/turn bonuses). A near-miss loop can therefore pin the distance at
9–15 forever — a bounded drift the tick-level cycle detector (periods 2–6) cannot
resolve. The guard tracks the best (lowest) Manhattan distance reached this round
(`bestDist`); when it stops improving for `kStalemateLimitTicks` (400)
consecutive ticks, `updateStalemateGuard()` ends the round as a red crash and
blue earns the usual speed bonus, like every other non-catch ending.

*Map assumption.* The guard measures the **Manhattan** distance. The city
generator guarantees a connected open street field (`connectRoads()` merges
every isolated road component into the main one), so on the maps the game
actually produces the Manhattan distance tracks real road distance closely
enough: a pinned-in-corridor blue on an arbitrary hand-made maze could let
the guard end a still-active round early, but such maps are outside the
supported generator output. The guard is a backstop, not a mechanic.

**Speed bonus.** When the red car crashes, `blueExtraMoves` increases by 1 (maximum
3), so the blue car makes `1 + blueExtraMoves` moves per tick. `reset(true)`
(key `R`) clears the bonus; `reset(false)` (key `SPACE` and the auto-restart)
keeps it.

### Game loop (`main.cpp`)

Window is `800 × 600` at 60 FPS, cell size `15` pixels (field: 53 × 40 cells).
Each frame:

1. Poll keys: `R` regenerates the city and hard-resets the chase; `SPACE` soft-resets
   (keeps the speed bonus).
2. Auto-restart counter: once `chase.isFinished()`, the timer counts up to 420
   frames (7 seconds at 60 FPS), then a soft reset happens automatically.
3. `chase.update()` runs one tick of the chase.
4. `grid.draw()` renders streets (dark gray) and buildings (gray) with a faint grid
   overlay. `chase.draw()` draws the cars as filled squares.
5. HUD shows `Blue speed: +N` when a bonus is active.
6. When finished, a centered panel (`420 × 270`) shows `CRASH` (orange) / `CATCH!`
   (red), a title and subtitle, an auto-restart countdown with a progress bar, the
   restart instructions, and on a crash the bonus message for the next round.

## Build

The supported configuration is **MSVC (Visual Studio 2022) on Windows**.
CMake is the canonical build system; the repository's `Makefile` is a thin
delegate to it. No dependencies are required beyond a C++17 compiler,
CMake 3.16+ and a prebuilt raylib directory: `include/raylib.h` with the
static library `raylib.lib` next to it (and `glfw.lib` if present).

### With CMake

```bash
cmake -S . -B build
cmake --build build --config Release
```

The executable is produced in `build/Release/`. If raylib is not found by
`find_package(raylib)`, point
`-DRAYLIB_ROOT=<dir>` (or the `RAYLIB_ROOT` environment variable) at a prebuilt
directory containing `include/raylib.h` and the libraries.

### With Makefile

```bash
make            # Release build
make debug      # Debug build
make test       # build + run the headless logic tests (Release, via ctest)
make clean      # remove the build tree
```

Pass `RAYLIB_ROOT=<dir>` if auto-detection does not find raylib
(e.g. `make release RAYLIB_ROOT=<dir-to-prebuilt-raylib>`).

### Notes

- The prebuilt raylib libraries are built against the **release DLL CRT**; CMake
  pins `CMAKE_MSVC_RUNTIME_LIBRARY` to `MultiThreadedDLL` before `project()` so no
  linker mismatch (LNK4098) appears in Debug builds.
- Both the game and the tests are compiled with `/W4 /permissive-` and build
  without warnings in Debug and Release.

### Sanitized test build

The `chase_tests` target can be built with AddressSanitizer, off by default:

```
cmake -S . -B build-asan -DCHASE_SANITIZE=ON
cmake --build build-asan --config Debug --target chase_tests
```

MSVC adds `/fsanitize=address /Zi` + `/DEBUG` (ASan only: UBSan is not
available for MSVC, so the flag is deliberately not passed). At run time
the MSVC ASan runtime DLL (`clang_rt.asan_dynamic-x86_64.dll`, shipped
with the MSVC toolset) must be on `PATH` — add the toolset's
`bin\Hostx64\x64` directory and run the test executable directly, not
via `ctest`.

## Tests

Headless logic tests live in `tests/test_main.cpp` (target `chase_tests`, enabled by
the `CHASE_BUILD_TESTS` option, ON by default). They are built with `CHASE_TESTING`
so the harness reaches private `Chase` members via a test-only `friend`
declaration. Twenty-one sections: city generation (20 seeds, values, road
coverage, independent flood-fill connectivity check), `connectRoads`
idempotence, car placement (road distance band, no building overwrite),
`bfsToward`/`bfsAway` in all four directions (direct path, obstacle detour,
corridor, dead end, sealed goal, explicit "up is not rejected" case, field
borders), simulation invariance and metrics (a simulation run never mutates
the real cars, `SimResult` fields are correct), cycle detection, the
state-history ring eviction, the stalemate guard (`updateStalemateGuard`
drives a distance that never improves to a bounded crash), a fresh-history
key check (after every blue move the pushed `ChaseState` equals the actual
post-tick car fields), a cycle-detection + `breakCycle` integration check
(period detection on a real ticked tail, then the deviation move with the
cars never overlapping or leaving the road network), the `Grid` guards
(a zero-cell constructor and a transposed-shape `copyCellsFrom` are
rejected), the `Chase` constructor guards (zero or negative width, height
or cellSize are rejected with `std::invalid_argument` before any
arithmetic, valid positive arguments are accepted), the `breakCycle`
catch checks (after each deviation move the main-loop catch rule ends the
round as a catch), the red speed mode in the cycle key (same positions and
directions under different speed modes are distinct keys, and the 150/300
threshold transitions yield modes 2, 3 and 4), reservoir-sampling
uniformity, the building-rotation library, replay determinism (the
simulated movement schedule equals the real one, tick-for-tick on a fixed
seed), restart semantics, a multi-round long session with per-frame
invariant checks.

```bash
cmake -S . -B build
cmake --build build --config Release --target chase_tests
ctest --test-dir build -C Release --output-on-failure
```

or simply `make test`.

### Results

On MSVC 14.44 (Visual Studio 2022) the full suite passes in every build
configuration — Debug, Release and Debug + ASan: 21 sections, **33591
checks, 0 failures**, no compiler warnings, no ASan reports.

## Code style

- C++17, no dependencies beyond raylib, no multithreading
- Method names use `camelCase`; types use `PascalCase`; comments are in English
- Magic values are named constants: `INVALID_STEP` (no valid move),
  `LOOKAHEAD_TICKS` (simulation depth)

## Change history

See `CHANGELOG.md` for the technical change history of the project.

## License

Provided as-is.
