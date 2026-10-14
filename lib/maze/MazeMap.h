// ============================================================================
//  MazeMap.h  --  what the robot knows about Section A, in the robot's OWN frame.
//  Must never include Arduino.h (needed for env:native builds).
//
//  The start cell, the start heading and the bridge corner change on the day, so
//  nothing here is a compass direction or a fixed coordinate. The map is a
//  (2N-1) x (2N-1) grid with the start cell in the middle, whatever corner or
//  heading it really has, so the whole 4x4 fits wherever it lies. "North" is the
//  way the robot faced at the start and directions run counter-clockwise, like
//  heading. Cells outside the true 4x4 are ruled out by the visited extent: the
//  section is N wide, so no cell is more than N-1 from every visited one.
//  docs/architecture/modules/maze_map.md
// ============================================================================

#pragma once

#include <stdint.h>

#include "RobotSpec.h"

namespace maze
{

constexpr int8_t kN = static_cast<int8_t>(SECTION_A_SIZE_TILES);
constexpr int8_t kMapSize = static_cast<int8_t>(2 * kN - 1);
constexpr int8_t kStartX = kN - 1;
constexpr int8_t kStartY = kN - 1;

// Counter-clockwise from the start heading, one step = 90 deg, so a left turn is +1.
enum class Dir : uint8_t { North = 0, West = 1, South = 2, East = 3 };

constexpr int8_t kDx[4] = {0, -1, 0, 1};
constexpr int8_t kDy[4] = {1, 0, -1, 0};

constexpr uint8_t dirIndex(Dir d)
{
    return static_cast<uint8_t>(d);
}

// `d` turned by `quarterTurns` * 90 deg CCW (negative = clockwise).
constexpr Dir dirRotate(Dir d, int8_t quarterTurns)
{
    return static_cast<Dir>((static_cast<int8_t>(dirIndex(d)) + quarterTurns + 8) & 3);
}

constexpr Dir dirOpposite(Dir d)
{
    return dirRotate(d, 2);
}

struct Cell {
    uint8_t wall = 0;  // bit per Dir: a wall on that side
    uint8_t known = 0; // bit per Dir: that side has been observed
    bool visited = false;
    // The white line tile was seen here but the robot had no reason to follow it (it was entered
    // from the side, not from the marker). Remembered so the bridge can still be found.
    bool line = false;
};

struct MazeMap {
    Cell cell[kMapSize][kMapSize];
    int8_t minVx = kMapSize, maxVx = -1; // extent of the visited cells
    int8_t minVy = kMapSize, maxVy = -1;
    bool hasMarker = false;
    int8_t markerX = 0, markerY = 0;
    // Scratch for the explorer's search: lives here so the search needs no stack of its own.
    int8_t dist[kMapSize][kMapSize];
    uint8_t queue[kMapSize * kMapSize];
};

inline void mapReset(MazeMap &m)
{
    m = MazeMap{};
}

inline bool mapInArray(int8_t x, int8_t y)
{
    return x >= 0 && y >= 0 && x < kMapSize && y < kMapSize;
}

// Can the cell be part of the 4x4? Before anything is visited, any cell in the array can.
inline bool mapInBounds(const MazeMap &m, int8_t x, int8_t y)
{
    if (!mapInArray(x, y)) {
        return false;
    }
    if (m.maxVx < 0) {
        return true;
    }
    return x >= m.maxVx - (kN - 1) && x <= m.minVx + (kN - 1) && y >= m.maxVy - (kN - 1) &&
           y <= m.minVy + (kN - 1);
}

// Has the visited extent pinned down the section's edge along that axis?
inline bool mapBoundsFixedX(const MazeMap &m)
{
    return m.maxVx >= 0 && m.maxVx - m.minVx + 1 == kN;
}

inline bool mapBoundsFixedY(const MazeMap &m)
{
    return m.maxVy >= 0 && m.maxVy - m.minVy + 1 == kN;
}

// Record one side of a cell, and the matching side of its neighbour.
inline void mapSetEdge(MazeMap &m, int8_t x, int8_t y, Dir d, bool wall)
{
    if (!mapInArray(x, y)) {
        return;
    }
    const uint8_t bit = static_cast<uint8_t>(1u << dirIndex(d));
    Cell &c = m.cell[y][x];
    c.known = static_cast<uint8_t>(c.known | bit);
    c.wall = wall ? static_cast<uint8_t>(c.wall | bit) : static_cast<uint8_t>(c.wall & ~bit);

    const int8_t nx = static_cast<int8_t>(x + kDx[dirIndex(d)]);
    const int8_t ny = static_cast<int8_t>(y + kDy[dirIndex(d)]);
    if (!mapInArray(nx, ny)) {
        return;
    }
    const uint8_t nbit = static_cast<uint8_t>(1u << dirIndex(dirOpposite(d)));
    Cell &n = m.cell[ny][nx];
    n.known = static_cast<uint8_t>(n.known | nbit);
    n.wall = wall ? static_cast<uint8_t>(n.wall | nbit) : static_cast<uint8_t>(n.wall & ~nbit);
}

// Mark a cell visited and widen the visited extent.
inline void mapVisit(MazeMap &m, int8_t x, int8_t y)
{
    if (!mapInArray(x, y)) {
        return;
    }
    m.cell[y][x].visited = true;
    m.minVx = x < m.minVx ? x : m.minVx;
    m.maxVx = x > m.maxVx ? x : m.maxVx;
    m.minVy = y < m.minVy ? y : m.minVy;
    m.maxVy = y > m.maxVy ? y : m.maxVy;
}

// The three sides a robot at (x, y) facing `facing` can see: ahead, left, right. The side behind
// is the one it came through (or, at the start, whatever a turn-round shows).
inline void mapObserve(MazeMap &m, int8_t x, int8_t y, Dir facing, bool front, bool left,
                       bool right)
{
    mapVisit(m, x, y);
    mapSetEdge(m, x, y, facing, front);
    mapSetEdge(m, x, y, dirRotate(facing, 1), left);
    mapSetEdge(m, x, y, dirRotate(facing, -1), right);
}

// Is the way from (x, y) toward `d` known to be open?
inline bool mapOpen(const MazeMap &m, int8_t x, int8_t y, Dir d)
{
    const uint8_t bit = static_cast<uint8_t>(1u << dirIndex(d));
    const Cell &c = m.cell[y][x];
    return (c.known & bit) != 0 && (c.wall & bit) == 0;
}

inline void mapMarkLine(MazeMap &m, int8_t x, int8_t y)
{
    if (mapInArray(x, y)) {
        m.cell[y][x].line = true;
    }
}

inline void mapMarkMarker(MazeMap &m, int8_t x, int8_t y)
{
    m.hasMarker = true;
    m.markerX = x;
    m.markerY = y;
}

} // namespace maze
