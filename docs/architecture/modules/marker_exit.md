# marker_exit

From the approach marker out of the maze: tell the marker, find the way out of its tile, drive on over
the strip and the gap, and hand over to the line follower. Pure logic in
`include/mission/MarkerExit.h`, tested through the mission manager (`test/test_section_a/`) and
`test/test_floor_pattern/` (`BridgeHandover`). Decisions:
[0010](../decisions/0010-finish-on-the-approach-marker.md),
[0011](../decisions/0011-leave-the-maze-onto-the-line-without-stopping.md) and
[0015](../decisions/0015-split-the-section-a-mission.md).

## Interface

- `MarkerExit` - the state: the `ExitPhase` (`Stand`, `Turn`, `Approach`), the floor pattern, whether
  the marker was seen (`atMarker`) and where its box centre is, whether the way out was to a side
  (`sideExit`), the side readings, the straight and the hand-over.
- `MarkerExitParams` - the marker's `PatternParams` and the approach (`approachSpeedMmS`,
  `noLineMm`, `handoverMinMm`, `handoverNoGapMm`).
- `markerWatch(me, perception, pattern)` - one step tick fed to the pattern; true on the tick the
  marker is told. `markerPending(...)` / `markerTaken(...)` - the marker under the array as far as the
  path goes, for a robot that stopped a little short of it.
- `markerExitBeginStand(me)`, `markerExitStandStep(...)` - stand and read both sides; returns an
  `ExitChoice` (`Wait`, `Left`, `Right`, `NoWay`).
- `markerExitBeginApproach(...)`, `markerExitApproachStep(...)` - drive on; returns an
  `ApproachStatus` (`Running`, `HandOver`, `NoLine`).
- `BridgeHandover` - when the array has been off the gap for `minMm` and sees the line.

## How it works

The floor sensor tells the marker: white, black, white under the array along the path
([floor_pattern](floor_pattern.md)), at the odometer position of the array's sensing line
(`IR_ARRAY_FORWARD_OFFSET_MM` ahead of the axle). It is fed only while stepping: a pivot at a centre
sweeps the array across the frame with the odometer standing still. The event comes as the far frame
reaches `MARKER_FAR_FRAME_MIN_MM`, about 15 mm before the robot's centre is on the tile's. A stop
short of it still counts (`floorPatternMarkerPending`: near frame and box complete, far frame
begun).

On the marker's tile the way out is straight on, or to a side when a wall is ahead:

- **Stand:** stand `senseMs` reading both sides, a silent reading counted as the sensor's range,
  sampled every other tick. The way out is the side that answers from far or not at all, by the
  medians (`medianOf`): a tile's far wall is within 2.5 m, the room beyond the exit is not. No open
  side is `NoLine`.
- **Turn:** to that side (the explorer's turn, with its retries).
- **Approach:** straight on at the line speed along the map direction, over the strip and the gap
  (`straightStep`, `FloorPattern` for the gap). The hand-over is at the first line view past the gap
  or, if no gap read as air (it did not on the robot: 915-1010 against the floor's 1009-1020), at a
  mixed line view `BRIDGE_HANDOVER_NO_GAP_MM` (230 mm) past the box's centre. No line within
  `noLineMm` of the box's centre is `NoLine`.

## Invariants

- The pattern starts afresh at the approach: what the array has seen so far belongs to the marker.
- A robot placed on the marker's own tile cannot finish there.

## Status

Tested through the mission manager in the simulated arena: the straight exit, an exit to either
side, a gap that does not read as air, a line that never comes. Ran on the robot at the
mid-evaluation (tag `v0.1-section-a`) before the split; not run since.
