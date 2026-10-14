# 0009: Find the finish by exploring a start-relative map

- **Status:** Accepted
- **Date:** 2026-10-14

## Context

The first task is Section A (4x4): from any tile, in a single attempt, reach the white rectangle
tile and stop on it (in the competition, the corner tile that is the bridge-out tile). Two facts
shape the design.

1. **The start tile is not known.** There is no localisation: the robot has walls, odometry and a
   gyro, and nothing that tells it which of the 16 tiles it is on.
2. **The maze has an island.** The four walls in the middle of the team's maze are not joined to the
   outer wall. A robot that follows one wall keeps going around whatever it first touches, so
   wall following reaches the finish only from starts that touch the outer wall first.

There is one attempt: nothing learnt on a run can be used on a second. Under the competition rules,
coming back to the start tile before the finish aborts the attempt.

## Decision

Keep the map in the start's own frame and explore until the floor sensor sees white.

- The start tile is the centre of a 7x7 grid (`2N - 1`, N = 4), and North is the way the robot
  faced. Any start in a 4x4 fits.
- At each tile the three wall sensors are read; the robot goes to the nearest tile it has not been
  in, by breadth-first search over edges not known to be walls, preferring straight on.
- Walls are only ever added. A wall that was a bad reading can strand the search, which then
  reports it is out of tiles; the caller may clear the map and start again from the same pose once
  (`explorerRescan`).
- The start tile is kept out of the path whenever another way exists, and crossed only when it is
  the sole way to the unvisited cells.
- The finish is recognised by the floor sensor, not by a coordinate. The white tile has a black
  rectangle and a line in it, so a full-white tile (the competition's start tile) must not be
  mistaken for it.

## Consequences

- No dependence on the start tile, its heading, or the maze layout.
- The path is not the shortest: the worst start in the photo maze needs 31 tile moves where an
  informed run would need at most 7. Section A is not timed beyond the run limit.
- Starts behind a chokepoint at the start tile still cross it (10 of the 60 starts in the photo
  maze, 30 without the avoidance).
- It needs reliable stationary readings: it reads all three sensors at each tile and turns in place
  to look at unseen edges, and the ultrasonics are the weakest sensor (angled walls read as
  silence, [ultrasonic](../modules/ultrasonic.md)). The planner tolerates missed side readings
  (tested at 40 %); a front reading must be heard before driving.
- The finish test needs a calibrated IR array (`LineSense.h`), which nothing triggers yet.

## Alternatives considered

- **Left- or right-hand wall following.** Smallest code, no map. Fails on the island, from the
  starts that reach it before the outer wall.
- **Hard-coded map plus localisation.** Shortest route, but needs the robot to find its tile from
  wall patterns, which several tiles share, and ties the code to one layout.
- **Depth-first with a stack (Tremaux).** Same coverage and no BFS, but walks corridors it has
  already seen; nearest-unvisited has the same RAM (about 150 B) and fewer moves.
