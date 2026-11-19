# 0014: Explore Section B in a start-relative 17x17 map

- **Status:** Superseded by [0019](0019-section-b-map-17x9-folded-to-9x9.md)
- **Date:** 2026-11-17

## Context

Section B is 9x9 and is entered over the bridge through a corner tile. The rules (p. 3) say the
bridge's corners and direction are disclosed on the day, and the arena photos show the entry at a
corner at the end of a bending line corridor. [0009](0009-explore-in-a-start-relative-map.md) keeps
the Section A map in the start's own frame because the start tile is not known; the explorer was
fixed to that 7x7 frame, held a path length in 6 bits, and indexed its cells with a byte.

A 9x9 map anchored at the entry corner would need the robot to know which outer wall the entry tile
stands against. The side sensors cannot always say: both side edges of the entry tile may be walls.
No operator switch is fitted to tell it.

## Decision

`ExplorerT<N>` is the map for an N x N maze, kept in the start's frame as in 0009: a (2N-1) square
with the start tile at its centre. `Explorer` stays the Section A map (N = 4); Section B is
`ExplorerT<9>`, 17x17, with the entry tile as its start. No input from the operator is needed.

- A map with more than 255 cells indexes them with 16 bits; smaller maps keep a byte.
- The planner's scratch byte per cell holds only the first step of the way to it. The path length it
  used to hold decided nothing, and with it the 6-bit limit goes.

## Consequences

- The Section B map is about 1.16 KB of RAM, the Section A map stays 149 B. Static RAM is 1991 B of 8192 B.
- The planner runs over at most 289 cells at a stop, not every tick.
- Cells outside the real maze stay unvisited and unreachable once the outer walls are heard, as in
  Section A.
- The map does not yet tell an edge known to be open from one never looked at. The shortest-path
  run will need that; it is added with it.

## Alternatives considered

- **9x9 anchored at the entry corner, one operator switch for the side.** 243 B of RAM, but it needs
  a switch that is not fitted and a person to set it from the officials' announcement.
- **Infer the side from the walls at the entry tile.** Ambiguous when both side edges are walls.
