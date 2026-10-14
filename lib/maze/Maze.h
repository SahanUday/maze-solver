// ============================================================================
//  Maze.h  --  exploration of an unknown grid from an unknown start tile. No
//  hardware, no Arduino.h - testable under env:native.
//  docs/architecture/modules/maze.md
//
//  The robot does not know which tile it starts on, so the map is kept in the
//  start's own frame: the start tile is the centre of a (2N-1) x (2N-1) grid and
//  North is the way the robot faced at the start. N tiles in any direction fit.
//
//  One cell visit, in order:
//      explorerObserve()   walls seen from here, relative to the heading
//      explorerPlan()      first step toward the nearest unvisited cell
//      turn to it, explorerFace(), then explorerObserve() again: the new front
//                          may be an edge nothing has looked at yet
//      drive one tile, explorerAdvance()
//  A forward drive must follow an observation whose front reading was valid.
// ============================================================================

#pragma once

#include <stdint.h>

#include "RobotSpec.h"

namespace maze
{

enum class Dir : uint8_t { North = 0, East = 1, South = 2, West = 3 };

constexpr Dir turnRight(Dir d)
{
    return static_cast<Dir>((static_cast<uint8_t>(d) + 1) & 3);
}
constexpr Dir turnLeft(Dir d)
{
    return static_cast<Dir>((static_cast<uint8_t>(d) + 3) & 3);
}
constexpr Dir opposite(Dir d)
{
    return static_cast<Dir>((static_cast<uint8_t>(d) + 2) & 3);
}

constexpr uint8_t kSide = 2 * SECTION_A_SIZE_TILES - 1;
constexpr uint8_t kCells = kSide * kSide;
constexpr uint8_t kStartCell = (kSide / 2) * kSide + kSide / 2;

// Per cell: bits 0-3 are a wall on the North/East/South/West edge, bit 4 is visited.
constexpr uint8_t kVisited = 0x10;
constexpr uint8_t wallBit(Dir d)
{
    return static_cast<uint8_t>(1u << static_cast<uint8_t>(d));
}

// Planner scratch packs the path length (6 bits) over the first step (2 bits). The longest path
// is kCells - 1, which has to fit.
static_assert(kCells - 1 < 63, "path length must fit 6 bits");
constexpr uint8_t kUnseen = 0xFF;

struct Explorer {
    uint8_t cell[kCells];
    uint8_t seen[kCells];  // planner scratch
    uint8_t queue[kCells]; // planner scratch
    uint8_t pos;
    Dir heading;
};

inline void explorerInit(Explorer &e)
{
    for (uint8_t i = 0; i < kCells; ++i) {
        e.cell[i] = 0;
    }
    e.pos = kStartCell;
    e.heading = Dir::North;
}

// True if `d` from `idx` stays inside the map; `out` is the cell there.
inline bool neighbour(uint8_t idx, Dir d, uint8_t &out)
{
    const uint8_t row = idx / kSide;
    const uint8_t col = idx % kSide;
    switch (d) {
        case Dir::North:
            if (row == 0) {
                return false;
            }
            out = static_cast<uint8_t>(idx - kSide);
            return true;
        case Dir::South:
            if (row == kSide - 1) {
                return false;
            }
            out = static_cast<uint8_t>(idx + kSide);
            return true;
        case Dir::East:
            if (col == kSide - 1) {
                return false;
            }
            out = static_cast<uint8_t>(idx + 1);
            return true;
        case Dir::West:
            if (col == 0) {
                return false;
            }
            out = static_cast<uint8_t>(idx - 1);
            return true;
    }
    return false;
}

// A wall is recorded on both cells that share the edge, and is never erased: a wrongly seen
// wall can leave a cell unreachable, which ends the search with an answer, where a flickering
// one could keep the planner changing its mind for the rest of the run.
inline void markWall(Explorer &e, uint8_t idx, Dir d)
{
    e.cell[idx] = static_cast<uint8_t>(e.cell[idx] | wallBit(d));
    uint8_t other = 0;
    if (neighbour(idx, d, other)) {
        e.cell[other] = static_cast<uint8_t>(e.cell[other] | wallBit(opposite(d)));
    }
}

// What the three wall sensors say from the current cell. A side that was not heard (no echo) is
// passed as false: nothing is recorded, and the edge is checked when the robot faces it.
inline void explorerObserve(Explorer &e, bool frontWall, bool leftWall, bool rightWall)
{
    if (frontWall) {
        markWall(e, e.pos, e.heading);
    }
    if (leftWall) {
        markWall(e, e.pos, turnLeft(e.heading));
    }
    if (rightWall) {
        markWall(e, e.pos, turnRight(e.heading));
    }
    e.cell[e.pos] = static_cast<uint8_t>(e.cell[e.pos] | kVisited);
}

namespace detail
{

// Breadth-first from the robot over edges not known to be walls. With `avoidStart` the start tile
// is not entered (unless the robot is on it).
inline bool plan(Explorer &e, Dir &step, bool avoidStart)
{
    for (uint8_t i = 0; i < kCells; ++i) {
        e.seen[i] = kUnseen;
    }
    const Dir order[4] = {e.heading, turnRight(e.heading), turnLeft(e.heading),
                          opposite(e.heading)};
    uint8_t head = 0;
    uint8_t tail = 0;
    e.seen[e.pos] = 0;
    e.queue[tail++] = e.pos;

    while (head < tail) {
        const uint8_t cur = e.queue[head++];
        const uint8_t length = static_cast<uint8_t>(e.seen[cur] >> 2);
        for (uint8_t k = 0; k < 4; ++k) {
            const Dir d = order[k];
            uint8_t next = 0;
            if ((e.cell[cur] & wallBit(d)) != 0 || !neighbour(cur, d, next) ||
                e.seen[next] != kUnseen || (avoidStart && next == kStartCell)) {
                continue;
            }
            const uint8_t first =
                cur == e.pos ? static_cast<uint8_t>(d) : static_cast<uint8_t>(e.seen[cur] & 3);
            e.seen[next] = static_cast<uint8_t>(((length + 1) << 2) | first);
            if ((e.cell[next] & kVisited) == 0) {
                step = static_cast<Dir>(first);
                return true;
            }
            e.queue[tail++] = next;
        }
    }
    return false;
}

} // namespace detail

// The first step of the shortest path to the nearest unvisited cell, over edges not known to be
// walls. Equal paths keep going straight, then prefer a right turn, then left, then back. The
// start tile is kept out of the path when there is another way: re-entering it before the finish
// aborts the attempt. False: nothing unvisited is reachable.
inline bool explorerPlan(Explorer &e, Dir &step)
{
    return detail::plan(e, step, true) || detail::plan(e, step, false);
}

// Forget every wall and visit but keep the position and heading, so the search starts again from
// here with fresh readings. For when explorerPlan() runs dry before the finish was seen: with a
// single attempt, a wall that was only ever a bad reading is the likeliest reason.
inline void explorerRescan(Explorer &e)
{
    for (uint8_t i = 0; i < kCells; ++i) {
        e.cell[i] = 0;
    }
}

inline void explorerFace(Explorer &e, Dir d)
{
    e.heading = d;
}

// One tile along the heading. False, and no move, if that edge is a known wall or the map edge.
inline bool explorerAdvance(Explorer &e)
{
    uint8_t next = 0;
    if ((e.cell[e.pos] & wallBit(e.heading)) != 0 || !neighbour(e.pos, e.heading, next)) {
        return false;
    }
    e.pos = next;
    return true;
}

// A wall edge sits ~120 mm from the centreline of a centred robot; with an open side the next
// thing heard is a tile further, 370 mm or more. Midway is one tile pitch.
// `heard` is the ultrasonic valid bit: silence is not a measurement.
constexpr bool wallDetected(bool heard, uint16_t distMm)
{
    return heard && distMm < TILE_PITCH_MM;
}

} // namespace maze
