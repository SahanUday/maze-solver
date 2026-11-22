# maze

Exploration of an unknown grid from an unknown start tile. Pure logic in `lib/maze/Maze.h`: no
hardware, no `Arduino.h`, tested on the PC by `test/unit/test_maze/`. The proof is in `lib/maze/Proof.h`.
Decision:
[0009](../decisions/0009-explore-in-a-start-relative-map.md).

## Interface

- `ExplorerT<N>` - the map of an N x N maze (`cell[]`, `visited[]`), the planner's scratch, `pos` and
  `heading`.
  Its cell `kStart` is where the run began; North is the start heading. `Explorer` is the Section A
  map (N = 4, with `kSide`, `kCells`, `kStartCell`); Section B is `ExplorerT<SECTION_B_SIZE_TILES>`
  ([0014](../decisions/0014-explore-section-b-in-a-start-relative-map.md)).
- `explorerInit(e)` - empty map, at the start cell, facing North.
- `explorerObserve(e, front, left, right)` - walls heard from the current cell, relative to the
  heading; marks the cell visited. `false` means no wall *or* not heard.
- `explorerObserveOpen(e, front, left, right)` - the edges heard open from the current cell.
- `explorerVisited(e, idx)` - the robot has been in the cell. `markWall`, `markOpen` and `markVisited`
  are what the observations use.
- `explorerPlan(e, step)` - first step toward the nearest unvisited cell, keeping out of the start
  tile when another way exists. `false` when none is reachable.
- `explorerFace(e, dir)`, `explorerAdvance(e)` - record a turn, and a move of one tile (refused
  through a known wall or off the map).
- `explorerRetreat(e)` - undo one `explorerAdvance`, for a move that did not happen (refused only
  off the map).
- `explorerRescan(e)` - forget every wall, checked edge and visit, keep the pose.
- `explorerProof(e, from, goal)` (`Proof.h`) - is the best known path from `from` to `goal` the
  shortest it can be? `Proved` with its length; `NotYet` with the cell and direction of the first
  unchecked edge on the best possible path (and that path's length); or `NoRoute`. It uses the
  planner's scratch and stores nothing.
- `explorerRoute(e, from, goal, step)` (`Proof.h`) - the first step of the shortest path from `from`
  to `goal` over checked edges, keeping out of the start tile when another way exists. `false` when
  there is none or `from` is the goal. The explorer's drive-to-tile mode uses it.

## How it works

One cell visit: observe, plan, turn to the step and observe again (the new front may be an edge
nothing has looked at), then drive.

A cell byte holds the four walls (bits 0-3, N E S W) and the same four edges as *checked* (bits 4-7):
an edge is unknown until a sensor has looked at it, then open or a wall. A wall is always checked.
Both are stored on both cells that share the edge. A wall is never erased. "Open" is set only from a
sensor that heard the edge open, never from a guess.

Planning is a breadth-first search over edges that are not walls, expanding straight, right, left,
back, and stopping at the first unvisited cell it finds, which is also the nearest. It runs first
with the start tile closed and again with it open if that finds nothing. Each cell's scratch byte is
the first step of the way to it.

The proof floods twice over edges that are not walls: *open* lets every edge pass, so its length is
the shortest path there could be; *closed* lets only checked edges pass, so its length is the
shortest known. Equal: no unchecked edge can do better. Unequal: the open path has an unchecked
edge, and the one nearest the start is where to look next. It is only as good as the readings: a
wall recorded wrongly is never erased, so a path "proved" around it may not be the true shortest.

## Invariants

- The (2N-1) square map holds any N x N start: the robot is at most N-1 tiles from the start tile in
  any direction. Cells are indexed with a byte up to 255 of them and with 16 bits beyond.
- A forward drive follows an observation whose front reading was heard. The planner treats an
  unobserved edge as open.
- No recursion and no heap. One `Explorer` is 156 B; `ExplorerT<9>` is about 1.2 KB.

## Status

Host tests (`env:native`, `env:native_san`). A simulator runs the photo maze from all 16 tiles and
4 headings: every start reaches the white tile without driving into a wall, in at most 31 tile
moves, and still does with 40 % of the side readings missed (5 seeds each). Every checked edge in
the map is what the maze has there. The start tile is re-entered in 10 of the 60 runs that do not
start on the white tile, all starts whose only way on goes through it. Ignoring walls in the planner
fails six tests.

The maze in the test was read from the photograph of the arena and its eight wall edges checked
against it; it has not been measured in the arena. It ran on the robot as part of the Section A run
at the mid-evaluation (tag `v0.1-section-a`). There is no motion, turn or finish-detection code in
this module.

The proof is tested on random mazes with loops on the Section A and Section B maps: from a start tile
and a goal it looks at the edge the proof names until it says proved, and the length is the true
shortest of a search over the maze itself. Letting unchecked edges pass the closed flood, or naming
the farthest unchecked edge instead of the nearest, each fails it.

`explorerRoute` is tested through the explorer (`test_section_a`, [cell_explorer](cell_explorer.md)).

A test on the 17x17 map (`ExplorerT<9>`) plans a 96-tile serpentine to the far corner. The Section B
map has not run on the robot.

`explorerAheadVisited(e)` is true when the cell in the heading direction has been entered: its walls
were all looked at from inside, so the shared edge is settled whatever the front sensor says now.
