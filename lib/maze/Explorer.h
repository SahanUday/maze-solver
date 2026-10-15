// ============================================================================
//  Explorer.h  --  where to go next. Must never include Arduino.h (needed for
//  env:native builds).
//
//  Two jobs, both over the MazeMap:
//    frontier   walk to the nearest cell not yet entered (to find the marker)
//    candidate  once the marker is seen, walk to a cell that could be the bridge-out tile
//  Both are a breadth-first search from the goals back to the robot, so a path is
//  followed through visited cells only (every wall on it is known). Nothing here
//  looks for a SHORT route to the bridge; it only has to get there.
//  The START cell is avoided where another way exists: re-entering it aborts the
//  attempt under the rules.
//  docs/architecture/modules/explorer.md
// ============================================================================

#pragma once

#include <stdint.h>

#include "MazeMap.h"

namespace maze
{

// Relative to the way the robot faces.
enum class Move : uint8_t { None, Forward, Left, Right, Back };

constexpr int8_t kUnreached = -1;

namespace detail
{

inline Move relativeMove(Dir facing, Dir target)
{
    switch ((dirIndex(target) + 4 - dirIndex(facing)) & 3) {
        case 0:
            return Move::Forward;
        case 1:
            return Move::Left;
        case 2:
            return Move::Back;
        default:
            return Move::Right;
    }
}

inline void clearDist(MazeMap &m)
{
    for (auto &row : m.dist) {
        for (auto &v : row) {
            v = kUnreached;
        }
    }
}

// Is (x, y) a cell worth walking to when exploring: unvisited, possible, and reachable through
// an open edge from a visited cell?
inline bool isFrontier(const MazeMap &m, int8_t x, int8_t y)
{
    if (!mapInBounds(m, x, y) || m.cell[y][x].visited) {
        return false;
    }
    for (uint8_t d = 0; d < 4; ++d) {
        const int8_t nx = static_cast<int8_t>(x + kDx[d]);
        const int8_t ny = static_cast<int8_t>(y + kDy[d]);
        if (mapInArray(nx, ny) && m.cell[ny][nx].visited &&
            mapOpen(m, nx, ny, dirOpposite(static_cast<Dir>(d)))) {
            return true;
        }
    }
    return false;
}

// Breadth-first from the goal marked by dist == 0 cells (already seeded and queued: `tail`
// entries), expanding only through VISITED cells over open edges. `avoidStart` keeps the start
// cell out of the search unless it is the robot's own. Returns once (rx, ry) has a distance.
inline bool search(MazeMap &m, uint8_t tail, int8_t rx, int8_t ry, bool avoidStart)
{
    uint8_t head = 0;
    while (head < tail) {
        const uint8_t idx = m.queue[head++];
        const int8_t cx = static_cast<int8_t>(idx % kMapSize);
        const int8_t cy = static_cast<int8_t>(idx / kMapSize);
        if (cx == rx && cy == ry) {
            return true;
        }
        for (uint8_t d = 0; d < 4; ++d) {
            const int8_t nx = static_cast<int8_t>(cx + kDx[d]);
            const int8_t ny = static_cast<int8_t>(cy + kDy[d]);
            if (!mapInArray(nx, ny) || m.dist[ny][nx] != kUnreached) {
                continue;
            }
            if (!m.cell[ny][nx].visited || !mapInBounds(m, nx, ny)) {
                continue; // only walk through cells whose walls we know
            }
            if (avoidStart && nx == kStartX && ny == kStartY && !(nx == rx && ny == ry)) {
                continue;
            }
            // The edge must be open seen from the neighbour's side: it is the robot's way.
            if (!mapOpen(m, nx, ny, dirOpposite(static_cast<Dir>(d)))) {
                continue;
            }
            m.dist[ny][nx] = static_cast<int8_t>(m.dist[cy][cx] + 1);
            m.queue[tail++] = static_cast<uint8_t>(ny * kMapSize + nx);
        }
    }
    return false;
}

// The first step from (x, y) down the distance field. A straight run is preferred (it needs no
// turn), then the side `hand` favours (+1 left, -1 right), then the other, then back.
inline bool firstStep(const MazeMap &m, int8_t x, int8_t y, Dir facing, int8_t hand, Move &out)
{
    const int8_t here = m.dist[y][x];
    if (here <= 0) {
        return false; // already at a goal, or unreached
    }
    const Dir order[4] = {facing, dirRotate(facing, hand >= 0 ? 1 : -1),
                          dirRotate(facing, hand >= 0 ? -1 : 1), dirOpposite(facing)};
    for (const Dir d : order) {
        const int8_t nx = static_cast<int8_t>(x + kDx[dirIndex(d)]);
        const int8_t ny = static_cast<int8_t>(y + kDy[dirIndex(d)]);
        if (mapInArray(nx, ny) && m.dist[ny][nx] == here - 1 && mapOpen(m, x, y, d)) {
            out = relativeMove(facing, d);
            return true;
        }
    }
    return false;
}

// Two seeders, each fills dist/queue with the goal cells and returns how many there are.
inline uint8_t seedFrontier(MazeMap &m)
{
    clearDist(m);
    uint8_t tail = 0;
    for (int8_t y = 0; y < kMapSize; ++y) {
        for (int8_t x = 0; x < kMapSize; ++x) {
            if (isFrontier(m, x, y)) {
                m.dist[y][x] = 0;
                m.queue[tail++] = static_cast<uint8_t>(y * kMapSize + x);
            }
        }
    }
    return tail;
}

inline uint8_t seedCell(MazeMap &m, int8_t tx, int8_t ty)
{
    clearDist(m);
    m.dist[ty][tx] = 0;
    m.queue[0] = static_cast<uint8_t>(ty * kMapSize + tx);
    return 1;
}

// Search from the `seeded` goals, keeping to cells other than the start first and falling back to
// passing through it. The search appends to the queue past the seeds and never overwrites them,
// so the second pass can rebuild its field from them.
inline bool plan(MazeMap &m, uint8_t seeded, int8_t x, int8_t y, Dir facing, int8_t hand, Move &out)
{
    if (seeded == 0) {
        return false;
    }
    if (search(m, seeded, x, y, true)) {
        return firstStep(m, x, y, facing, hand, out);
    }
    clearDist(m);
    for (uint8_t i = 0; i < seeded; ++i) {
        m.dist[m.queue[i] / kMapSize][m.queue[i] % kMapSize] = 0;
    }
    if (search(m, seeded, x, y, false)) {
        return firstStep(m, x, y, facing, hand, out);
    }
    return false;
}

} // namespace detail

// Next move toward the nearest unexplored cell. False when there is none (the section is
// explored) or the robot cannot get to one.
inline bool exploreMove(MazeMap &m, int8_t x, int8_t y, Dir facing, int8_t hand, Move &out)
{
    return detail::plan(m, detail::seedFrontier(m), x, y, facing, hand, out);
}

// Next move toward one specific cell (`tx`, `ty`), which may itself be unvisited.
inline bool goToMove(MazeMap &m, int8_t x, int8_t y, Dir facing, int8_t hand, int8_t tx, int8_t ty,
                     Move &out)
{
    if (!mapInArray(tx, ty) || (x == tx && y == ty)) {
        return false;
    }
    return detail::plan(m, detail::seedCell(m, tx, ty), x, y, facing, hand, out);
}

// The cell whose open side leads out of the 4x4: the exit, found from the walls alone. Only a side
// that was observed open counts, and its neighbour must be outside the cells the 4x4 can still
// cover, which is only known once the visited extent has pinned both axes down (after the section
// has been explored). False before that, or if no such side was seen.
inline bool exitCell(const MazeMap &m, int8_t &ex, int8_t &ey, Dir &ed)
{
    for (int8_t y = 0; y < kMapSize; ++y) {
        for (int8_t x = 0; x < kMapSize; ++x) {
            if (!m.cell[y][x].visited) {
                continue;
            }
            for (uint8_t d = 0; d < 4; ++d) {
                const uint8_t bit = static_cast<uint8_t>(1u << d);
                if ((m.cell[y][x].known & bit) == 0 || (m.cell[y][x].wall & bit) != 0) {
                    continue;
                }
                const int8_t nx = static_cast<int8_t>(x + kDx[d]);
                const int8_t ny = static_cast<int8_t>(y + kDy[d]);
                if (!mapInBounds(m, nx, ny)) {
                    ex = x;
                    ey = y;
                    ed = static_cast<Dir>(d);
                    return true;
                }
            }
        }
    }
    return false;
}

// After the marker: pick the neighbour of the marker cell that could be the bridge-out tile: one
// not entered yet, or one whose white line tile was seen but not followed. It must be reachable
// through an open edge. Once the visited extent has pinned an axis
// down, the bridge-out tile (a corner of the section) has to sit on that axis' edge. Prefers the
// neighbour straight on from `heading`, the way the robot was going when it saw the marker.
inline bool bridgeCandidate(const MazeMap &m, Dir heading, int8_t &cx, int8_t &cy)
{
    if (!m.hasMarker) {
        return false;
    }
    const Dir order[4] = {heading, dirRotate(heading, 1), dirRotate(heading, -1),
                          dirOpposite(heading)};
    for (const Dir d : order) {
        const int8_t nx = static_cast<int8_t>(m.markerX + kDx[dirIndex(d)]);
        const int8_t ny = static_cast<int8_t>(m.markerY + kDy[dirIndex(d)]);
        if (!mapInBounds(m, nx, ny) || (m.cell[ny][nx].visited && !m.cell[ny][nx].line) ||
            !mapOpen(m, m.markerX, m.markerY, d)) {
            continue;
        }
        if (mapBoundsFixedX(m) && nx != m.minVx && nx != m.maxVx) {
            continue;
        }
        if (mapBoundsFixedY(m) && ny != m.minVy && ny != m.maxVy) {
            continue;
        }
        cx = nx;
        cy = ny;
        return true;
    }
    return false;
}

// Standing on the marker cell facing `facing`: the move that steps into its neighbour (gx, gy), so
// the bridge tile is entered along the marker's axis, not from the side. None if it is not a
// neighbour.
inline Move stepIntoNeighbour(const MazeMap &m, Dir facing, int8_t gx, int8_t gy)
{
    for (uint8_t d = 0; d < 4; ++d) {
        if (m.markerX + kDx[d] == gx && m.markerY + kDy[d] == gy) {
            return detail::relativeMove(facing, static_cast<Dir>(d));
        }
    }
    return Move::None;
}

} // namespace maze
