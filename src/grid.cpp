#include "grid.hpp"
#include <algorithm>
#include <cstdlib>
#include <limits>
#include <raylib.h>

static std::vector<std::vector<int>> Rotate90(const std::vector<std::vector<int>> &matrix)
{
    int height = (int)matrix.size();
    int width = (int)matrix[0].size();
    std::vector<std::vector<int>> rotated(width, std::vector<int>(height, 0));
    for (int i = 0; i < height; i++)
        for (int j = 0; j < width; j++)
            rotated[j][height - 1 - i] = matrix[i][j];
    return rotated;
}

static bool MatEqual(const std::vector<std::vector<int>> &a, const std::vector<std::vector<int>> &b)
{
    if (a.size() != b.size() || a[0].size() != b[0].size())
        return false;
    for (size_t i = 0; i < a.size(); i++)
        for (size_t j = 0; j < a[i].size(); j++)
            if (a[i][j] != b[i][j])
                return false;
    return true;
}

Grid::Grid(int width, int height, int cellSize, bool withLibrary) : cellSize(cellSize)
{
    if (width <= 0 || height <= 0 || cellSize <= 0)
        throw std::invalid_argument("Grid: width, height and cellSize must all be positive");
    rows = height / cellSize;
    columns = width / cellSize;
    if (rows == 0 || columns == 0)
        // Integer division can still yield a zero-sized field; a zero
        // grid would make every modulo / index calculation undefined.
        throw std::invalid_argument("Grid: the field must span at least one cell in both dimensions");

    cells.assign((size_t)rows * columns, 0);

    if (withLibrary)
        initBuildings();
}

void Grid::initBuildings()
{
    buildings.clear();

    // Each group: a set of source patterns.
    // When initializing, every pattern is rotated 4 times,
    // duplicates are removed.

    // --- 2x2 (3 source patterns) ---
    std::vector<std::vector<std::vector<int>>> base2 = {{{0, 1}, {0, 1}}, {{1, 1}, {1, 0}}, {{1, 0}, {1, 1}}};

    // --- 3x3 (3 source patterns) ---
    std::vector<std::vector<std::vector<int>>> base3 = {{{1, 1, 1}, {0, 1, 0}, {0, 1, 0}},

                                                        {{1, 0, 1}, {1, 1, 1}, {0, 0, 1}},

                                                        {{1, 1, 1}, {0, 0, 1}, {1, 0, 0}}};

    // --- 4x4 (2 source patterns) ---
    std::vector<std::vector<std::vector<int>>> base4 = {{{1, 1, 1, 1}, {0, 1, 0, 0}, {0, 1, 0, 0}, {0, 0, 0, 0}},

                                                        {{1, 0, 0, 0}, {1, 1, 1, 1}, {1, 0, 0, 0}, {1, 0, 0, 0}}};

    // --- 5x5 (2 source patterns) ---
    std::vector<std::vector<std::vector<int>>> base5 = {
        {{0, 0, 1, 1, 1}, {0, 0, 1, 0, 0}, {0, 0, 1, 0, 0}, {0, 0, 1, 0, 0}, {1, 1, 1, 0, 0}},

        {{1, 1, 1, 0, 0}, {0, 0, 1, 0, 0}, {0, 0, 1, 1, 1}, {0, 0, 1, 0, 0}, {1, 1, 1, 0, 0}}};

    // --- 6x6 (3 source patterns, full width) ---
    std::vector<std::vector<std::vector<int>>> base6 = {{{1, 1, 1, 1, 1, 0},
                                                         {1, 0, 0, 0, 0, 0},
                                                         {1, 0, 0, 0, 0, 0},
                                                         {1, 0, 0, 0, 0, 0},
                                                         {1, 0, 0, 0, 0, 0},
                                                         {1, 1, 1, 1, 1, 0}},
                                                        {{1, 1, 1, 1, 1, 1},
                                                         {0, 0, 0, 0, 0, 1},
                                                         {0, 0, 0, 0, 0, 1},
                                                         {0, 0, 0, 0, 0, 1},
                                                         {0, 0, 0, 0, 0, 1},
                                                         {0, 0, 0, 0, 0, 0}},
                                                        {{1, 0, 0, 0, 0, 1},
                                                         {1, 0, 0, 0, 0, 1},
                                                         {1, 1, 1, 1, 1, 1},
                                                         {0, 0, 0, 0, 0, 1},
                                                         {0, 0, 0, 0, 0, 1},
                                                         {1, 1, 1, 1, 1, 1}}};

    // --- 7x7 (3 source patterns, full width) ---
    // The first pattern was historically ragged (top row of 7, the rest of
    // 6): canPlace() derived the width from row 0 and read row 6, col 6
    // out of bounds (garbage). The missing trailing cells are 0 — an empty
    // margin — so padding is behaviour-preserving.
    std::vector<std::vector<std::vector<int>>> base7 = {{{1, 1, 1, 1, 1, 1, 1},
                                                         {0, 0, 0, 0, 0, 1, 0},
                                                         {0, 0, 0, 0, 0, 1, 0},
                                                         {0, 0, 0, 0, 0, 1, 0},
                                                         {0, 0, 0, 0, 0, 1, 0},
                                                         {0, 0, 0, 0, 0, 1, 0},
                                                         {0, 0, 0, 0, 0, 0, 0}},
                                                        {{1, 1, 1, 1, 1, 1},
                                                         {1, 0, 0, 0, 0, 0},
                                                         {1, 0, 0, 0, 0, 0},
                                                         {0, 0, 0, 0, 0, 0},
                                                         {0, 0, 0, 0, 0, 0},
                                                         {0, 0, 0, 0, 0, 0},
                                                         {0, 0, 0, 0, 0, 0}},
                                                         {
                                                             {0, 0, 0, 0, 0, 0, 1},
                                                             {0, 0, 0, 0, 0, 0, 1},
                                                             {0, 0, 0, 0, 0, 0, 1},
                                                             {1, 1, 1, 1, 1, 1, 1},
                                                             {0, 0, 0, 0, 0, 0, 1},
                                                             {0, 0, 0, 0, 0, 0, 1},
                                                             {0, 0, 0, 0, 0, 0, 1},
                                                         }};

    auto addGroup = [&](const std::vector<std::vector<std::vector<int>>> &bases) {
        std::vector<std::vector<std::vector<int>>> group;
        for (const auto &pattern : bases) {
            std::vector<std::vector<std::vector<int>>> rotations;
            rotations.push_back(pattern);
            rotations.push_back(Rotate90(rotations.back()));
            rotations.push_back(Rotate90(rotations.back()));
            rotations.push_back(Rotate90(rotations.back()));

            for (const auto &rot : rotations) {
                bool duplicate = false;
                for (const auto &existing : group)
                    if (MatEqual(existing, rot)) {
                        duplicate = true;
                        break;
                    }
                if (!duplicate)
                    group.push_back(rot);
            }
        }
        return group;
    };

    buildings.push_back(addGroup(base2)); // [0] 2x2
    buildings.push_back(addGroup(base3)); // [1] 3x3
    buildings.push_back(addGroup(base4)); // [2] 4x4
    buildings.push_back(addGroup(base5)); // [3] 5x5
    buildings.push_back(addGroup(base6)); // [4] 6x6
    buildings.push_back(addGroup(base7)); // [5] 7x7
}

void Grid::draw()
{
    auto idx = [this](int r, int c) { return (size_t)r * columns + c; };

    for (int row = 0; row < rows; row++) {
        for (int col = 0; col < columns; col++) {
            Color cellColor = (cells[idx(row, col)] == 0) ? Color{35, 35, 35, 255}     // street
                                                           : Color{120, 120, 120, 255}; // building

            DrawRectangle(col * cellSize, row * cellSize, cellSize, cellSize, cellColor);
        }
    }

    // Weak grid, almost invisible.
    Color gridColor = Color{255, 255, 255, 18};

    for (int row = 0; row <= rows; row++) {
        DrawLine(0, row * cellSize, columns * cellSize, row * cellSize, gridColor);
    }

    for (int col = 0; col <= columns; col++) {
        DrawLine(col * cellSize, 0, col * cellSize, rows * cellSize, gridColor);
    }
}

void Grid::clear()
{
    std::fill(cells.begin(), cells.end(), 0);
}

bool Grid::isWithinBounds(int row, int col) const { return row >= 0 && row < rows && col >= 0 && col < columns; }

void Grid::setValue(int row, int col, int value)
{
    if (isWithinBounds(row, col))
        cells[(size_t)row * columns + col] = value;
}

int Grid::getValue(int row, int col) const
{
    if (!isWithinBounds(row, col))
        return -1; // out-of-bounds must never look like an empty road cell
    return cells[(size_t)row * columns + col];
}

bool Grid::canPlace(int row, int col, const std::vector<std::vector<int>> &building, int clearance) const
{
    int h = (int)building.size(), w = (int)building[0].size();
    for (int i = -clearance; i < h + clearance; i++)
        for (int j = -clearance; j < w + clearance; j++) {
            int rr = row + i, cc = col + j;
            if (!isWithinBounds(rr, cc))
                return false;
            if (cells[(size_t)rr * columns + cc] != 0)
                return false;
        }
    return true;
}

void Grid::place(int row, int col, const std::vector<std::vector<int>> &building, int buildingType)
{
    for (int i = 0; i < (int)building.size(); i++)
        for (int j = 0; j < (int)building[i].size(); j++)
            if (building[i][j] > 0)
                cells[(size_t)(row + i) * columns + (col + j)] = buildingType;
}

bool Grid::tryPlaceGroup(int groupIndex, int clearance, int *outVariant, int *outRow, int *outCol)
{
    if (groupIndex < 0 || groupIndex >= (int)buildings.size())
        return false;
    const auto &group = buildings[groupIndex];
    if (group.empty())
        return false;

    // Reservoir sampling (R1): pick one uniformly random valid candidate
    // without building the whole candidate list. The acceptance probability
    // 1/seen makes the final selection uniform over all candidates, so the
    // distribution is the same uniform one as an explicit list + random
    // index, but memory stays O(1).
    int selectedVariant = -1;
    int selectedRow = -1;
    int selectedCol = -1;
    int seen = 0;
    for (int v = 0; v < (int)group.size(); v++) {
        const auto &b = group[v];
        int h = (int)b.size(), w = (int)b[0].size();
        for (int r = 0; r <= rows - h; r++) {
            for (int c = 0; c <= columns - w; c++) {
                if (!canPlace(r, c, b, clearance))
                    continue;
                seen++;
                if (GetRandomValue(1, seen) == 1) {
                    selectedVariant = v;
                    selectedRow = r;
                    selectedCol = c;
                }
            }
        }
    }

    if (selectedVariant < 0)
        return false;

    place(selectedRow, selectedCol, group[selectedVariant], groupIndex + 1);
    if (outVariant)
        *outVariant = selectedVariant;
    if (outRow)
        *outRow = selectedRow;
    if (outCol)
        *outCol = selectedCol;
    return true;
}

#ifdef CHASE_TESTING
Grid::Placement Grid::placeRandomGroup(int groupIndex, int clearance)
{
    int variant = -1, row = -1, col = -1;
    bool ok = tryPlaceGroup(groupIndex, clearance, &variant, &row, &col);
    return {ok, variant, row, col};
}
#endif

void Grid::generateCity(int clearance)
{
    clear();

    if (buildings.empty())
        return;

    // Strategy: from large to small, in several passes.
    // [5]=7x7, [4]=6x6, [3]=5x5, [2]=4x4, [1]=3x3, [0]=2x2.

    const int maxIters = rows * columns;

    // Phase 1: large buildings (7, 6, 5) - maximum fill.
    for (int iter = 0; iter < maxIters; iter++) {
        bool placed = false;

        // First 7x7, then 6x6, then 5x5.
        for (int g = 5; g >= 3 && !placed; g--) {
            placed = tryPlaceGroup(g, clearance);
        }

        if (!placed)
            break;
    }

    // Phase 2: medium buildings (4) - squeezing in.
    for (int iter = 0; iter < maxIters / 2; iter++) {
        bool placed = tryPlaceGroup(2, clearance);
        if (!placed)
            break;
    }

    // Phase 3: medium buildings (4) - tight fit with clearance=1 (road 1).
    for (int iter = 0; iter < maxIters / 2; iter++) {
        bool placed = tryPlaceGroup(2, 1);
        if (!placed)
            break;
    }

    // Phase 4: small buildings (3) - tight fit with clearance=2 (road 2).
    for (int iter = 0; iter < maxIters; iter++) {
        bool placed = tryPlaceGroup(1, clearance);
        if (!placed)
            break;
    }

    // Phase 5: small buildings (3) - tight fit with clearance=1 (road 1).
    for (int iter = 0; iter < maxIters; iter++) {
        bool placed = tryPlaceGroup(1, 1);
        if (!placed)
            break;
    }

    // Phase 6: extreme fill (2) - with clearance=2 (road 2).
    for (int iter = 0; iter < maxIters / 4; iter++) {
        bool placed = tryPlaceGroup(0, 2);
        if (!placed)
            break;
    }

    // Phase 7: extreme (2) - with clearance=1 (road 1).
    for (int iter = 0; iter < maxIters / 4; iter++) {
        bool placed = tryPlaceGroup(0, 1);
        if (!placed)
            break;
    }

    // Link disconnected road sections.
    connectRoads();
}

void Grid::fillRandom()
{
    for (int row = 0; row < rows; row++)
        for (int col = 0; col < columns; col++)
            cells[(size_t)row * columns + col] = (GetRandomValue(0, 5) == 0) ? 1 : 0;
}

void Grid::toggleCell(int row, int col)
{
    if (isWithinBounds(row, col))
        cells[(size_t)row * columns + col] = !cells[(size_t)row * columns + col];
}

void Grid::ensureFloodBuffers() const
{
    int total = rows * columns;
    if (floodCellCount == total)
        return;
    floodCellCount = total;
    floodVisited.assign((size_t)total, 0);
    floodQueue.clear();
    floodQueue.reserve((size_t)total);
}

int Grid::floodFillFrom(int startRow, int startCol) const
{
    const int kRowDelta[] = {-1, 1, 0, 0};
    const int kColDelta[] = {0, 0, -1, 1};
    auto idx = [this](int r, int c) { return (size_t)r * columns + c; };

    std::fill(floodVisited.begin(), floodVisited.end(), 0);
    floodQueue.clear();

    if (!isWithinBounds(startRow, startCol) || cells[idx(startRow, startCol)] != 0)
        return 0;

    floodVisited[idx(startRow, startCol)] = 1;
    floodQueue.push_back(idx(startRow, startCol));
    size_t head = 0;
    while (head < floodQueue.size()) {
        size_t cur = floodQueue[head++];
        int r = (int)(cur / columns);
        int c = (int)(cur % columns);
        for (int dir = 0; dir < 4; dir++) {
            int nr = r + kRowDelta[dir];
            int nc = c + kColDelta[dir];
            if (nr < 0 || nr >= rows || nc < 0 || nc >= columns)
                continue;
            if (floodVisited[idx(nr, nc)])
                continue;
            if (cells[idx(nr, nc)] != 0)
                continue;
            floodVisited[idx(nr, nc)] = 1;
            floodQueue.push_back(idx(nr, nc));
        }
    }
    return (int)floodQueue.size();
}

void Grid::connectRoads()
{
    auto idx = [this](int r, int c) { return (size_t)r * columns + c; };

    ensureFloodBuffers();

    // Find the first road cell.
    int startRow = -1, startCol = -1;
    for (int row = 0; row < rows && startRow < 0; row++)
        for (int col = 0; col < columns && startRow < 0; col++)
            if (cells[idx(row, col)] == 0) {
                startRow = row;
                startCol = col;
            }

    if (startRow < 0)
        return;

    // Repeatedly: find isolated road sections and connect them.
    // No iteration cap: every carving step merges one isolated component
    // into the main one, so the loop always terminates.
    while (true) {
        floodFillFrom(startRow, startCol);

        // Find one isolated road cell.
        int isolatedRow = -1, isolatedCol = -1;
        for (int row = 0; row < rows && isolatedRow < 0; row++)
            for (int col = 0; col < columns && isolatedRow < 0; col++)
                if (!floodVisited[idx(row, col)] && cells[idx(row, col)] == 0) {
                    isolatedRow = row;
                    isolatedCol = col;
                }

        if (isolatedRow < 0)
            break; // all road cells belong to the main component

        // Nearest main-component cell by Manhattan distance, in one scan.
        // No component list is materialized: the flood buffer already marks
        // the main component.
        int bestRow = -1, bestCol = -1, bestDist = std::numeric_limits<int>::max();
        for (int row = 0; row < rows; row++)
            for (int col = 0; col < columns; col++)
                if (floodVisited[idx(row, col)]) {
                    int d = abs(row - isolatedRow) + abs(col - isolatedCol);
                    if (d < bestDist) {
                        bestDist = d;
                        bestRow = row;
                        bestCol = col;
                    }
                }
        if (bestRow < 0)
            break;

        bool rowFirst = GetRandomValue(0, 1) == 0;
        int row = isolatedRow, col = isolatedCol;

        // Both ends of the corridor are grid cells, and the walk advances
        // strictly toward its end, so every carved cell stays in bounds.
        auto carveLine = [&](int from, int to, bool isRowMove) {
            int step = (to > from) ? 1 : -1;
            if (isRowMove) {
                while (col != to) {
                    col += step;
                    cells[idx(row, col)] = 0;
                }
            }
            else {
                while (row != to) {
                    row += step;
                    cells[idx(row, col)] = 0;
                }
            }
        };

        if (rowFirst) {
            carveLine(isolatedCol, bestCol, true);
            carveLine(isolatedRow, bestRow, false);
        }
        else {
            carveLine(isolatedRow, bestRow, false);
            carveLine(isolatedCol, bestCol, true);
        }
    }
}

bool Grid::isRoadNetworkConnected() const
{
    auto idx = [this](int r, int c) { return (size_t)r * columns + c; };

    int startRow = -1, startCol = -1;
    int roadCount = 0;
    for (int row = 0; row < rows; row++)
        for (int col = 0; col < columns; col++)
            if (cells[idx(row, col)] == 0) {
                roadCount++;
                if (startRow < 0) {
                    startRow = row;
                    startCol = col;
                }
            }
    if (roadCount <= 1)
        return true; // nothing (or a single cell) to connect

    // Reuse the same buffers; floodFillFrom only reads cell values, so a
    // const caller may safely use the (mutable) workspace.
    ensureFloodBuffers();
    int seen = floodFillFrom(startRow, startCol);
    return seen == roadCount;
}

void Grid::copyCellsFrom(const Grid &other)
{
    // Compare the actual dimensions, not just the buffer size: two grids
    // with different rows/columns can still hold the same number of cells
    // (e.g. 3x8 and 4x6), and a flat copy of a mismatched shape would
    // silently transpose the data.
    if (rows != other.rows || columns != other.columns)
        throw std::invalid_argument("Grid::copyCellsFrom: dimension mismatch");
    cells = other.cells;
}
