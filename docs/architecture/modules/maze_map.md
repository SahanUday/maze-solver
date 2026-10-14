# maze_map

What the robot knows about Section A, in the robot's **own frame**. Header-only in
`lib/maze/MazeMap.h` (namespace `maze`, no `Arduino.h`), tested by `test/test_maze_explorer/`.

## Interface

- `MazeMap`, `Cell` (`wall`, `known`, `visited`, `line` bits), `Dir` (North/West/South/East,
  counter-clockwise, so a left turn is +1), `dirRotate()`, `dirOpposite()`.
- `mapReset`, `mapVisit`, `mapSetEdge`, `mapObserve(m, x, y, facing, front, left, right)`,
  `mapOpen`, `mapMarkMarker`, `mapMarkLine`.
- `mapInBounds`, `mapBoundsFixedX/Y`.

## How it works

The start cell, its heading and the bridge corner change on the day, so no coordinate or compass
direction is fixed anywhere. The map is `(2N-1) x (2N-1)` with the start cell in the middle,
whatever corner or heading it really has, so the 4x4 fits wherever it lies; "North" is the way the
robot faced at the start. Edges are stored on both sides of a wall. A cell outside the true 4x4 is
ruled out by the visited extent alone: the section is N wide, so no cell is more than N-1 from every
visited one, and once the visited extent spans N along an axis that axis is pinned. A single wall
cannot be told from the boundary, so the bounds are only ever inferred this way.

## Invariants

- The map holds the BFS scratch (`dist`, `queue`) so the search needs no stack of its own; one
  `MazeMap` is about 260 B.
- `mapOpen()` is true only for a side that was **observed** open.

## Status

Covered by the explorer's tests, which run every start cell and every heading on three mazes. All of
it is the host's simulation of the walls; none has been read from a real corridor.
