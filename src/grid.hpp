#pragma once

#include <cstdint>
#include <stdexcept>
#include <vector>

// City grid: 0 = road, 1..6 = building of the given size class.
//
// The public API is deliberately split into two groups:
//   - map mutation: operations that change the cell buffer
//   - map reading : operations that observe the cell buffer (all const)
class Grid {
  public:
    // --- map mutation ---
    // withLibrary = false builds only the cell buffer (no building patterns).
    Grid(int width, int height, int cellSize, bool withLibrary = true);

    // Rendering mutates GPU state, not the map; kept in the mutation group.
    void draw();
    void clear();
    void setValue(int row, int col, int value);
    void fillRandom();
    void toggleCell(int row, int col);

    // Generate a city (building placement + road repair) and keep the road
    // network connected.
    void generateCity(int clearance);
    void connectRoads();

    // Copy only the cell buffer from another grid of the same size
    // (used by the legacy Game-of-Life module instead of copying the
    // whole object with its building library).
    void copyCellsFrom(const Grid &other);

  public:
    // --- map reading (does not modify the grid) ---
    // Out-of-bounds coordinates return -1 (never 0, so a missing cell can
    // not be mistaken for a traversable road).
    int getValue(int row, int col) const;
    bool isWithinBounds(int row, int col) const;
    int getRows() const { return rows; }
    int getColumns() const { return columns; }

    // True when every road cell belongs to a single connected component.
    bool isRoadNetworkConnected() const;

#ifdef CHASE_TESTING
    // Test-only access to the building library (verify that the
    // rotation/dedup pass produces well-formed, duplicate-free variants).
    const std::vector<std::vector<std::vector<std::vector<int>>>> &buildingVariants() const { return buildings; }

    // Test-only: one reservoir-sampled placement attempt on the current
    // cell buffer (checks uniformity of the variant/position choice).
    // Lays the building into the buffer, just like the generator does,
    // and reports exactly which candidate was picked — no re-derivation
    // from the cell pattern needed.
    struct Placement {
        bool ok;
        int variant;
        int row;
        int col;
    };
    Placement placeRandomGroup(int groupIndex, int clearance);
#endif

  private:
    int rows;
    int columns;
    int cellSize;

    // Flat contiguous cell buffer: index = row * columns + col.
    std::vector<int> cells;

    // buildings[sizeIndex][variant][row][col]
    // sizeIndex: 0 -> 2x2, 1 -> 3x3, 2 -> 4x4, 3 -> 5x5, 4 -> 6x6, 5 -> 7x7
    std::vector<std::vector<std::vector<std::vector<int>>>> buildings;

    // Reusable flood-fill workspace (connectRoads / isRoadNetworkConnected):
    // sized once, refilled per search — no per-call allocation. Scratch
    // state, hence mutable: const readers (isRoadNetworkConnected) may reuse
    // it without modifying the observable map.
    mutable std::vector<uint8_t> floodVisited;
    mutable std::vector<size_t> floodQueue; // flat cell indices
    mutable int floodCellCount = 0;

    void initBuildings();
    bool canPlace(int row, int col, const std::vector<std::vector<int>> &building, int clearance) const;
    void place(int row, int col, const std::vector<std::vector<int>> &building, int buildingType);
    // The out parameters (nullable) report the candidate finally picked
    // by the reservoir, so a test can measure the choice without reading
    // it back off the cell buffer.
    bool tryPlaceGroup(int groupIndex, int clearance, int *outVariant = nullptr,
                       int *outRow = nullptr, int *outCol = nullptr);

    void ensureFloodBuffers() const;
    // Flood-fills road cells reachable from (startRow, startCol) into
    // floodVisited / floodQueue. Returns the number of visited cells
    // (0 when the start cell is not a road cell).
    int floodFillFrom(int startRow, int startCol) const;
};
