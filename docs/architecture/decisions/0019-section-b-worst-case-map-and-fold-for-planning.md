# 0019: Explore Section B on the worst-case map, plan on the folded 9x9

- **Status:** Accepted
- **Date:** 2026-11-19

Extends [0014](0014-explore-section-b-in-a-start-relative-map.md), which stays in force.

## Context

What `docs/Maze_Solver.pdf` says about the entry to Section B:

- The bridge "enters a corner tile of Section B (the bridge-in tile)" (stated, p. 3).
- "The specific corners used, and the direction of the bridge, will be disclosed at the event; do not
  assume fixed compass directions" (stated, p. 3). The finish location is variable and announced too.
  That is told to the teams; the robot may be given no information once the maze is revealed, only
  switch settings (p. 5).
- The outer walls enclose each section "except at the designated bridge opening(s)", and the bridge tile
  is "the connection through the outer wall" (p. 3). Figure 1 shows the line board joining the corner
  tile through a wall opening. The figure is an example: layouts "are subject to change" (p. 4), and the
  post-entry line corridor "may include bends" (p. 3).

**Not stated:** which way the robot faces when it stands on the entry tile, and which side of it the
maze lies. A design that assumes "facing in, wall behind" would be a guess, and a wrong guess fails the
run.

## Decision

Design for the worst case: **assume nothing about the entry's orientation.**

- The Section B explore map stays the start-relative 17x17 of 0014 (`ExplorerT<9>`, about 1.16 KB). It
  holds a 9x9 maze whichever way the robot faces and whichever corner it entered by. Nothing here needs
  a rules answer.
- The planner of [0018](0018-route-by-estimated-time-and-prove-it.md) does **not** run on it: a search
  over (tile, heading) on 17x17 is about 1156 states, roughly 5 KB, against a 6 KB RAM budget (a
  calculation). Once the explored tiles fix where the real 9x9 lies in the 17x17 map, the map is
  **folded into the real 9x9** (324 states, about 1.3 KB) and the planner runs on that. Which evidence
  fixes the position (outer walls heard, or tiles seen to span 9) is settled in #75; the planner never
  runs on an unfolded map.
- Section A stays the 7x7 start-relative map of [0009](0009-explore-in-a-start-relative-map.md), as its
  start is not known.

## Consequences

- The explorer code is unchanged by this ADR. The fold is new code (#75).
- If the instructor confirms how the robot enters (question Q-D in #73), a 17x9 explore map would save
  about 0.6 KB. That is an optimisation, not a requirement, and is not planned.

## Alternatives considered

- **17x9 map (entry in a corner, facing in).** Smaller, but it assumes the facing, which the text does
  not state. Rejected as a guess.
- **9x9 anchored at the entry with a switch for the side.** Needs a switch and a rules answer (Q-C).
