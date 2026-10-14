# maze

Exploration of an unknown grid from an unknown start tile. Pure logic in
`lib/maze/Maze.h`: no hardware, no `Arduino.h`, tested on the PC by
`test/test_maze/`. Decision:
[0009](../decisions/0009-explore-in-a-start-relative-map.md).

## Interface

- `Explorer` - the map (`cell[]`), the planner's scratch, `pos` and `heading`.
  Cell `kStartCell` is where the run began; North is the start heading.
- `explorerInit(e)` - empty map, at the start cell, facing North.
- `explorerObserve(e, front, left, right)` - walls seen from the current cell,
  relative to the heading; marks the cell visited. `false` means no wall *or* not
  heard.
- `explorerPlan(e, step)` - first step toward the nearest unvisited cell. `false`
  when none is reachable.
- `explorerFace(e, dir)`, `explorerAdvance(e)` - record a turn, and a move of one
  tile (refused through a known wall or off the map).
- `explorerRescan(e)` - forget every wall and visit, keep pose.
- `wallDetected(heard, distMm)` - `heard && distMm < TILE_PITCH_MM`.

## How it works

One cell visit: observe, plan, turn to the step and observe again (the new front
may be an edge nothing has looked at), then drive. A wall is stored on both cells
that share the edge and is never erased. Planning is a breadth-first search over
edges that are not walls, expanding straight, right, left, back, and stops at the
first unvisited cell it discovers, which is also the nearest. Path length and the
first step are packed in one byte per cell.

## Invariants

- The 7x7 map holds any 4x4 start: the robot can be at most 3 tiles from the start
  tile in any direction.
- A forward drive follows an observation whose front reading was heard. The
  planner treats an unobserved edge as open.
- No recursion and no heap. One `Explorer` is 147 B.

## Status

16 host tests pass under `env:native` and `env:native_san`. A simulator runs the
Section A maze of the team's photo from all 16 tiles and 4 headings: every start
reaches the white tile without driving into a wall, in at most 31 tile moves, and
still does with 40 % of the side readings missed (5 seeds each). Deliberately
ignoring walls in the planner makes six of the tests fail.

**The maze in the test was read from a photograph by eye and has not been checked
against the arena.** Nothing here has run on the robot: the wall threshold
(`TILE_PITCH_MM` against a centred robot) is arithmetic from the tile geometry, not
a measurement, and there is no motion, turn or finish-detection code yet.
