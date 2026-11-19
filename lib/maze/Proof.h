// ============================================================================
//  Proof.h  --  is the best known path to a tile the shortest it can be? Reads the map of Maze.h,
//  stores nothing. No hardware, no Arduino.h - testable under env:native.
//
//  Two floods from the start tile over edges that are not walls:
//      open    every edge counts: the shortest path there could be
//      closed  only checked edges count: the shortest path known
//  Equal: no unchecked edge can do better. Otherwise the unchecked edge nearest the start on the
//  open path is the next to look at.
//  docs/architecture/modules/maze.md
// ============================================================================

#pragma once

#include <stdint.h>

#include "Maze.h"

namespace maze
{

enum class ProofStatus : uint8_t {
    NoRoute, // no path, even with every unchecked edge open
    Proved,  // `length` is the shortest path
    NotYet,  // an unchecked edge could make it shorter: look at `lookDir` from `lookTile`
};

struct Proof {
    ProofStatus status = ProofStatus::NoRoute;
    uint16_t length = 0;   // steps: the shortest path (Proved), or the shortest it could still be
    uint16_t lookTile = 0; // NotYet: a cell index, reached from the start by checked edges only
    Dir lookDir = Dir::North;
};

namespace detail
{

constexpr uint16_t kNoPath = 0xFFFF;

// Steps from `from` to `goal` by breadth-first search, or kNoPath. `closed`: only checked edges
// pass. Leaves the direction each reached cell was entered by in `e.seen`.
template <uint8_t N>
inline uint16_t flood(ExplorerT<N> &e, typename ExplorerT<N>::Index from,
                      typename ExplorerT<N>::Index goal, bool closed)
{
    using E = ExplorerT<N>;
    using Index = typename E::Index;
    for (uint16_t i = 0; i < E::kCells; ++i) {
        e.seen[i] = kUnseen;
    }
    if (from == goal) {
        return 0;
    }
    Index head = 0;
    Index tail = 0;
    e.seen[from] = 0; // reached, entered by no direction
    e.queue[tail++] = from;
    for (uint16_t steps = 1; head < tail; ++steps) {
        const Index levelEnd = tail;
        while (head < levelEnd) {
            const Index cur = e.queue[head++];
            for (uint8_t d = 0; d < 4; ++d) {
                const Dir dir = static_cast<Dir>(d);
                Index next = 0;
                if ((e.cell[cur] & wallBit(dir)) != 0 ||
                    (closed && (e.cell[cur] & checkedBit(dir)) == 0) ||
                    !E::neighbour(cur, dir, next) || e.seen[next] != kUnseen) {
                    continue;
                }
                e.seen[next] = d;
                if (next == goal) {
                    return steps;
                }
                e.queue[tail++] = next;
            }
        }
    }
    return kNoPath;
}

} // namespace detail

// Uses the planner's scratch (`seen`, `queue`).
template <uint8_t N>
inline Proof explorerProof(ExplorerT<N> &e, typename ExplorerT<N>::Index from,
                           typename ExplorerT<N>::Index goal)
{
    using Index = typename ExplorerT<N>::Index;
    Proof p;
    const uint16_t known = detail::flood(e, from, goal, true);
    const uint16_t best = detail::flood(e, from, goal, false); // its path stays in e.seen
    if (best == detail::kNoPath) {
        return p;
    }
    p.length = best;
    if (best == known) {
        p.status = ProofStatus::Proved;
        return p;
    }
    // Walk the open path back from the goal: the last unchecked edge met is the nearest the start.
    p.status = ProofStatus::NotYet;
    for (Index cur = goal; cur != from;) {
        const Dir d = static_cast<Dir>(e.seen[cur]);
        Index prev = 0;
        ExplorerT<N>::neighbour(cur, opposite(d), prev); // always inside the map
        if ((e.cell[prev] & checkedBit(d)) == 0) {
            p.lookTile = prev;
            p.lookDir = d;
        }
        cur = prev;
    }
    return p;
}

} // namespace maze
