// ============================================================================
//  Maze.h  --  exploration of an unknown grid from an unknown start tile. No
//  hardware, no Arduino.h - testable under env:native.
//
//  The map is kept in the start's own frame: the start tile is the centre of a
//  (2N-1) x (2N-1) grid and North is the way the robot faced at the start. N is the maze's
//  size in tiles: ExplorerT<N> holds any N x N maze whatever tile the robot starts on.
//  Explorer is the Section A map (4 x 4); Section B (9 x 9) is ExplorerT<SECTION_B_SIZE_TILES>.
//
//  One cell visit:
//      explorerObserve()   walls heard from here, relative to the heading
//      explorerObserveOpen()  the edges heard open
//      explorerPlan()      first step toward the nearest unvisited cell
//      explorerFace()      turn to it, then observe again: the new front
//                          may be an edge nothing has looked at
//      explorerAdvance()   drive one tile
//  A forward drive must follow an observation whose front reading was heard.
//  docs/architecture/modules/maze.md
// ============================================================================

#pragma once

#include <stdint.h>

#include "spec/Competition.h"

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

// Per cell: bits 0-3 a wall on the North/East/South/West edge, bits 4-7 the same edges looked at
// ("checked"). A wall is always checked; an unchecked edge is unknown.
constexpr uint8_t wallBit(Dir d)
{
    return static_cast<uint8_t>(1u << static_cast<uint8_t>(d));
}
constexpr uint8_t checkedBit(Dir d)
{
    return static_cast<uint8_t>(wallBit(d) << 4);
}
// What looking at edge `d` records in the cell it belongs to.
constexpr uint8_t lookedBits(Dir d, bool wall)
{
    return static_cast<uint8_t>(wall ? wallBit(d) | checkedBit(d) : checkedBit(d));
}

constexpr uint8_t kUnseen = 0xFF; // the planner has not reached this cell

namespace detail
{
// The type that indexes a map: a byte while the cells fit in one.
template <bool Small> struct IndexOf {
    using type = uint8_t;
};
template <> struct IndexOf<false> {
    using type = uint16_t;
};
} // namespace detail

template <uint8_t N> struct ExplorerT {
    static_assert(N >= 1 && N <= 128, "the (2N-1) x (2N-1) map must have a side that fits a byte");
    static constexpr uint8_t kSide = 2 * N - 1;
    static constexpr uint16_t kCells = static_cast<uint16_t>(kSide) * kSide;
    using Index = typename detail::IndexOf<(kCells <= 255)>::type;
    static constexpr Index kStart = static_cast<Index>((kSide / 2) * kSide + kSide / 2);

    uint8_t cell[kCells];
    uint8_t visited[(kCells + 7) / 8]; // one bit per cell: the robot has been in it
    uint8_t seen[kCells]; // planner scratch: the first step of the way here, or kUnseen
    Index queue[kCells];  // planner scratch
    Index pos;
    Dir heading;

    // True if `d` from `idx` stays inside the map; `out` is the cell there.
    static bool neighbour(Index idx, Dir d, Index &out)
    {
        const uint8_t row = static_cast<uint8_t>(idx / kSide);
        const uint8_t col = static_cast<uint8_t>(idx % kSide);
        switch (d) {
            case Dir::North:
                if (row == 0) {
                    return false;
                }
                out = static_cast<Index>(idx - kSide);
                return true;
            case Dir::South:
                if (row == kSide - 1) {
                    return false;
                }
                out = static_cast<Index>(idx + kSide);
                return true;
            case Dir::East:
                if (col == kSide - 1) {
                    return false;
                }
                out = static_cast<Index>(idx + 1);
                return true;
            case Dir::West:
                if (col == 0) {
                    return false;
                }
                out = static_cast<Index>(idx - 1);
                return true;
        }
        return false;
    }
};

// The Section A map.
using Explorer = ExplorerT<SECTION_A_SIZE_TILES>;
constexpr uint8_t kSide = Explorer::kSide;
constexpr uint8_t kCells = Explorer::kCells;
constexpr uint8_t kStartCell = Explorer::kStart;

namespace detail
{

template <uint8_t N> inline void clearMap(ExplorerT<N> &e)
{
    for (uint16_t i = 0; i < ExplorerT<N>::kCells; ++i) {
        e.cell[i] = 0;
    }
    for (uint16_t i = 0; i < sizeof e.visited; ++i) {
        e.visited[i] = 0;
    }
}

// Edge `d` of `idx` has been looked at, on both cells that share it.
template <uint8_t N>
inline void markEdge(ExplorerT<N> &e, typename ExplorerT<N>::Index idx, Dir d, bool wall)
{
    typename ExplorerT<N>::Index other = 0;
    e.cell[idx] = static_cast<uint8_t>(e.cell[idx] | lookedBits(d, wall));
    if (ExplorerT<N>::neighbour(idx, d, other)) {
        e.cell[other] = static_cast<uint8_t>(e.cell[other] | lookedBits(opposite(d), wall));
    }
}

// The edges of the current cell in front, left and right of the heading that are flagged.
template <uint8_t N>
inline void markSides(ExplorerT<N> &e, bool front, bool left, bool right, bool wall)
{
    if (front) {
        markEdge(e, e.pos, e.heading, wall);
    }
    if (left) {
        markEdge(e, e.pos, turnLeft(e.heading), wall);
    }
    if (right) {
        markEdge(e, e.pos, turnRight(e.heading), wall);
    }
}

} // namespace detail

template <uint8_t N> inline void explorerInit(ExplorerT<N> &e)
{
    detail::clearMap(e);
    e.pos = ExplorerT<N>::kStart;
    e.heading = Dir::North;
}

// Never erased: a wrong wall can strand the search, which ends with an answer; a flickering one
// would keep it changing its mind.
template <uint8_t N> inline void markWall(ExplorerT<N> &e, typename ExplorerT<N>::Index idx, Dir d)
{
    detail::markEdge(e, idx, d, true);
}

// An edge a sensor heard open. Never from a guess: a wrong "open" would make a path through a wall
// look known.
template <uint8_t N> inline void markOpen(ExplorerT<N> &e, typename ExplorerT<N>::Index idx, Dir d)
{
    detail::markEdge(e, idx, d, false);
}

template <uint8_t N>
inline bool explorerVisited(const ExplorerT<N> &e, typename ExplorerT<N>::Index idx)
{
    return ((e.visited[idx >> 3] >> (idx & 7)) & 1) != 0;
}

template <uint8_t N> inline void markVisited(ExplorerT<N> &e, typename ExplorerT<N>::Index idx)
{
    e.visited[idx >> 3] = static_cast<uint8_t>(e.visited[idx >> 3] | (1u << (idx & 7)));
}

// The cell ahead has been entered: its walls were all looked at from inside, so the edge between
// the two is settled whatever the front sensor says from here (a silent front may be a wall in
// its dead zone, or an exit with nothing beyond).
template <uint8_t N> inline bool explorerAheadVisited(const ExplorerT<N> &e)
{
    typename ExplorerT<N>::Index other = 0;
    return ExplorerT<N>::neighbour(e.pos, e.heading, other) && explorerVisited(e, other);
}

// Walls heard from the current cell. A side that was not heard is passed as false: nothing is
// recorded, and the edge is looked at when the robot faces it.
template <uint8_t N>
inline void explorerObserve(ExplorerT<N> &e, bool frontWall, bool leftWall, bool rightWall)
{
    detail::markSides(e, frontWall, leftWall, rightWall, true);
    markVisited(e, e.pos);
}

// Edges heard open from the current cell.
template <uint8_t N>
inline void explorerObserveOpen(ExplorerT<N> &e, bool frontOpen, bool leftOpen, bool rightOpen)
{
    detail::markSides(e, frontOpen, leftOpen, rightOpen, false);
}

namespace detail
{

// Breadth-first from the robot over edges not known to be walls; `avoidStart` closes the start
// tile (unless the robot is on it). A cell's `seen` byte is the first step of the way to it.
template <uint8_t N> inline bool plan(ExplorerT<N> &e, Dir &step, bool avoidStart)
{
    using E = ExplorerT<N>;
    using Index = typename E::Index;
    for (uint16_t i = 0; i < E::kCells; ++i) {
        e.seen[i] = kUnseen;
    }
    const Dir order[4] = {e.heading, turnRight(e.heading), turnLeft(e.heading),
                          opposite(e.heading)};
    Index head = 0;
    Index tail = 0;
    e.seen[e.pos] = 0;
    e.queue[tail++] = e.pos;

    while (head < tail) {
        const Index cur = e.queue[head++];
        for (uint8_t k = 0; k < 4; ++k) {
            const Dir d = order[k];
            Index next = 0;
            if ((e.cell[cur] & wallBit(d)) != 0 || !E::neighbour(cur, d, next) ||
                e.seen[next] != kUnseen || (avoidStart && next == E::kStart)) {
                continue;
            }
            const uint8_t first = cur == e.pos ? static_cast<uint8_t>(d) : e.seen[cur];
            e.seen[next] = first;
            if (!explorerVisited(e, next)) {
                step = static_cast<Dir>(first);
                return true;
            }
            e.queue[tail++] = next;
        }
    }
    return false;
}

} // namespace detail

// First step of the shortest path to the nearest unvisited cell. Equal paths prefer straight on,
// then right, left, back. The start tile is kept out of the path when there is another way
// (re-entering it before the finish aborts the attempt). False: nothing unvisited is reachable.
template <uint8_t N> inline bool explorerPlan(ExplorerT<N> &e, Dir &step)
{
    return detail::plan(e, step, true) || detail::plan(e, step, false);
}

// Forget every wall and visit, keep the pose. For when explorerPlan() runs dry before the finish
// was seen: a wall that was only a bad reading is the likeliest reason.
template <uint8_t N> inline void explorerRescan(ExplorerT<N> &e)
{
    detail::clearMap(e);
}

template <uint8_t N> inline void explorerFace(ExplorerT<N> &e, Dir d)
{
    e.heading = d;
}

// One tile along the heading. False, and no move, if that edge is a known wall or the map edge.
template <uint8_t N> inline bool explorerAdvance(ExplorerT<N> &e)
{
    typename ExplorerT<N>::Index next = 0;
    if ((e.cell[e.pos] & wallBit(e.heading)) != 0 ||
        !ExplorerT<N>::neighbour(e.pos, e.heading, next)) {
        return false;
    }
    e.pos = next;
    return true;
}

// Undo one explorerAdvance(): back one tile, the way it came. False off the map. Not the way to
// leave a cell (that is a turn and an advance); for a move that did not happen.
template <uint8_t N> inline bool explorerRetreat(ExplorerT<N> &e)
{
    typename ExplorerT<N>::Index back = 0;
    if (!ExplorerT<N>::neighbour(e.pos, opposite(e.heading), back)) {
        return false;
    }
    e.pos = back;
    return true;
}

} // namespace maze
