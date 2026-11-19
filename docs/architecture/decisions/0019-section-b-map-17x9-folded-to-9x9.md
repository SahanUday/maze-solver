# 0019: Section B: a 17x9 map from the known entry, folded to 9x9

- **Status:** Accepted
- **Date:** 2026-11-19

Supersedes [0014](0014-explore-section-b-in-a-start-relative-map.md).

## Context

0014 chose a 17x17 start-relative map for Section B because the robot was thought not to know where
the entry tile sits. What `docs/Maze_Solver.pdf` says (p. 3-4):

- The bridge "enters a corner tile of Section B (the bridge-in tile)" (stated, p. 3).
- "The specific corners used, and the direction of the bridge, will be disclosed at the event; do not
  assume fixed compass directions" (stated, p. 3). The finish location is variable and announced too.
  That is told to the teams; the robot may be given no information once the maze is revealed, only
  switch settings (p. 5).
- Figure 1 shows the line board joining the corner tile through an opening in the outer wall, with the
  marker on that tile. **That the robot arrives facing into the maze with the opening behind it is read
  from the figure, not stated in the text.** The post-entry line corridor "may include bends" (p. 3), so
  the last direction of the line is not fixed either.

Taking the figure as the rule, the entry is on the back row, in a corner, facing in. Rows ahead are
bounded: at most 9 including the entry row. Only the side is unknown, so the width must cover 8 tiles to each side.
17x17 is twice what is needed, and a planner over `(tile, heading)` on 17x17 would need about 1156
states: roughly 5 KB of arrays against a 6 KB RAM budget (static RAM is 2000 B now).

## Decision

- The Section B map is **17 wide by 9 deep tiles**, the entry tile in the middle of the back row, the
  robot facing in (153 tiles). `ExplorerT` takes a width and a height, and the start row, instead of
  one `N`.
- The planner does not run on it. When the side is known (the explored tiles show which half of the 17
  columns is outside the maze) the map is **folded into the real 9x9** (324 states, about 1.3 KB) and
  the planner of 0018 runs on that. Which edge of evidence fixes the side is settled in #61, but the
  planner must not run on an unfolded map.
- If the corner can be given by a switch, the explorer may start on the 9x9 directly; the 17x9 map is the
  one that needs no input, as in 0009 and 0014.
- Section A stays the 7x7 start-relative map of 0009, because its start is not known.

## Consequences

- The code is not changed by this ADR: `ExplorerT<N>` (N x N) and the 17x17 test stay until the
  Section B mission (#61) and the rectangular map change (#75) land. The change touches `lib/maze/Maze.h`,
  which #55 also changes; the map code is changed once, after #55.
- The Section B explorer map costs about 0.6 KB less than 17x17 and the planner fits the RAM budget.
- The map still does not tell an open edge from an unseen one (as in 0014), which #55 adds.

## Alternatives considered

- **Keep 17x17.** Works, but does not leave room for a planner on `(tile, heading)` states.
- **9x9 anchored at the entry corner with one switch for the side.** Smallest, and now the fall-back
  if the switch is allowed and fitted; it was rejected in 0014 because no switch was fitted, and the
  17x9 map needs none.
- **Infer the side from the walls at the entry tile.** Ambiguous when both side edges are walls; used
  only as one piece of evidence for the fold.
