# explorer

Where to go next. Header-only in `lib/maze/Explorer.h` (namespace `maze`, no `Arduino.h`), tested
by `test/test_maze_explorer/`.

## Interface

- `exploreMove(m, x, y, facing, hand, out)` - the next move toward the nearest unexplored cell.
- `goToMove(m, x, y, facing, hand, tx, ty, out)` - toward one specific cell.
- `exitCell(m, x, y, dir)` - the cell with a side observed open whose neighbour is outside the 4x4 (only true once
  the visited extent has pinned both axes): the exit, from the walls alone. The mission uses it when no marker was
  reported.
- (The mission no longer uses the next two: the exit is straight on from the marker cell.)
- `bridgeCandidate(m, heading, cx, cy)` - after the marker: the neighbour of the marker cell that could
  be the bridge-out tile.
- `stepIntoNeighbour(m, facing, gx, gy)` - from the marker cell, the move that enters it.
- `Move` - `None / Forward / Left / Right / Back`, relative to the way the robot faces.

## How it works

Both searches are a breadth-first from the goals back to the robot, walking only through **visited**
cells over observed-open edges, so every wall on a route is known. A straight run is preferred on a
tie (it needs no turn), then the side `hand` favours (a DIP switch). The START cell is avoided where
another way exists, because re-entering it aborts the attempt under the rules; a second pass allows it.

Nothing here looks for a *short* route to the bridge: for the mid evaluation it only has to get there.
The bridge candidate must be reachable through an open edge, may be a cell already seen to be a white
line tile (`Cell::line`), and, once an axis is pinned, must sit on that axis' edge, since the bridge-out
tile is a corner of the section.

## Invariants

- No heap and no recursion.
- The search is stateless: it is recomputed at each decision from the map.

## Status

23 host tests including sweeps over every start cell x every heading x both hands on a ring maze
(loops), a snake maze (a single route, dead ends) and an open room. Simulated walls only.
