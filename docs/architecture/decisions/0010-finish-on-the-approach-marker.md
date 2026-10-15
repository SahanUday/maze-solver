# 0010: Finish Section A on the approach marker, not on "white"

- **Status:** Accepted
- **Date:** 2026-10-15
- **Supersedes:** the finish rule of [0009](0009-explore-in-a-start-relative-map.md) (the
  exploration it describes stands)

## Context

[0009](0009-explore-in-a-start-relative-map.md) took the finish to be a tile whose floor reads
white. The arena (photos and the measurements in PR #28) is different:

- The maze floor is **black**. The "white tile" is the approach marker: a white frame around a
  black box, on a black tile. Crossing it along its middle the array reads white ~36 mm, black
  ~118 mm, white ~37 mm.
- The array is **86 mm ahead of the axle**. At the point where the walls of a tile are decided,
  50 mm before its centre, the array is already over the black box, so "white for 100 ms" is never
  true there.
- The marker's tile is a dead end with walls on three sides; the fourth side is open to the floor
  outside the maze, so the front sensor hears nothing from it.

## Decision

The finish is the **pattern** white, black, white along the path (`FloorPattern.h`, measured
proportions), read while the robot drives a step. When the far frame has 12 mm under the array the
event fires, the step brakes at once, and the mission is `Done` when the robot stands still.

Because the marker's tile is open to nothing, a silent front is no longer always a wall in the
front sensor's dead zone. The mission now tells the two apart from what the side sensors, which
have no dead zone, measured: the sides it measured on the way in, a quarter turn to look at an
edge with a side sensor, and cells it has already been in.

## Consequences

- A robot placed ON the marker tile cannot finish there: it must leave and come back, and the
  explorer does not re-enter its own start tile. Start anywhere else.
- Without a calibration the pattern cannot be read (`wallsOnly`): the run explores everything and
  stops. A stop 20 mm or less short of a target, standing still, is an arrival (not a fault).
- The pattern is fed only while stepping: a pivot sweeps the array across the frame with the
  odometer standing still.
- A marker crossed more than ~30 mm off its centre line reads as an edge (mixed), not black, and is
  missed. Wall centring keeps the robot well inside that in the host model.

## Alternatives considered

- **Keep "white under the array":** never true at the decision point (above).
- **Stop at the marker's tile by the front wall:** there is no wall; the edge is open.
- **Drive out blindly when the front is silent:** a wall inside the dead zone is silent too, and
  28 mm is all that lies between the nose and it.
