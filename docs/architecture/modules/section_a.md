# section_a

The Section A mission: from any tile, explore until the approach marker, drive on over the gap and
follow the line out of the maze. Pure logic in `include/mission/SectionA.h`, tested by
`test/test_section_a/` in a simulated arena. It composes [cell_explorer](cell_explorer.md) (sense,
turn, step), [marker_exit](marker_exit.md) (the marker, the way out, the gap) and the line follower
([line_follow](line_follow.md)), and owns the one motion controller they feed
([motion_control](motion_control.md)). Decisions:
[0009](../decisions/0009-explore-in-a-start-relative-map.md) and
[0015](../decisions/0015-split-the-section-a-mission.md).

## Interface

- `SectionA` - the state: the `SectionAStage` (`WaitStart`, `Explore`, `Exit`, `Line`, `Done`,
  `Fault`), `phase` as the status line numbers it, `fault`, the `CellExplorer`, the `MarkerExit`,
  the `LineFollow` and the motion controller's integrals.
- `sectionAReset(ms)`, `sectionAFail(ms, code)` - the caller fails a run for a precondition it
  found missing (`NoImu`, `TimeUp`).
- `SectionAParams`, `sectionAParams()` - the explorer's and the marker exit's parameters, the
  controller gains and the line follower's, every one at its `spec/` value. The caller may set
  `wallsOnly` and `followLine`.
- `sectionAStep(ms, params, perception)` - one tick; returns the `DriveCommand`.
- `sectionATakeHeadingFix(ms)` - degrees to subtract from the gyro heading
  (`headingSet(hd, hd.deg - fix)`), 0 when there is none. Call it every tick after
  `sectionAStep`.

## How it works

`WaitStart` holds the motors off until `startPressed` (the run loop sets it when the run begins:
[run_loop](run_loop.md)), then the explorer starts and takes the gyro heading as map North.

- **Explore:** the explorer's phases ([cell_explorer](cell_explorer.md)). While it steps, the mission
  feeds the floor pattern; when the marker is told it asks the explorer to stop on the tile
  (`cellExplorerStopHere`), and if the way out is straight on it goes straight through without a stop.
- **Exit:** the marker exit's phases ([marker_exit](marker_exit.md)): stand and read both sides when a
  wall is ahead, turn to the open one, drive on over the strip and the gap. The first line view past
  the gap hands over to the line follower.
- **Line:** `lineFollowStep`. Nothing ends it: the robot is powered off at the end of the line, and
  if not, a lost line is searched for and then `LineLost` cuts the motors.
- **Done:** only with `followLine` false (stop on the marker's tile, holding with the drivers
  enabled) or `wallsOnly` (everything explored; the marker is not looked for).
- **Fault:** motors off, with `fault` saying why. A stalled motor controller is `Stall`.

`phase` is what the status line prints: the explorer's `Sense`, `Turn` and `Step`, the marker exit's
standing and turning as `Sense` and `Turn`, then `Approach`, `Line`, `Done`, `Fault`.

## Invariants

- The mission never writes `RobotState`; it reads `Perception` and returns a `DriveCommand`.
- Only the mission calls `motionControlStep()`; the explorer and the marker exit fill a
  `MotionTarget`.
- `Approach` and `Line` come last in `SectionAPhase`, so the numbers logged for the older phases
  are unchanged.

## Status

Host tests against ray-cast walls, a plant with stiction, encoder and gyro noise, ultrasonic
dropouts and strays: all 60 starts of the photo maze (with the marker tile open to the outside) reach the marker within 40 mm of its
centre, with no wall contact and the map position right; also with 30 % dropped readings and 5 %
stray echoes, a gyro scale 2 % off with 9 deg/min of drift, a wall in the front sensor's dead zone
and an obstacle held in front for a moment.

Ran on the robot at the mid-evaluation (tag `v0.1-section-a`), called by `main.cpp` through
[run_loop](run_loop.md), before the mission was split into its parts; not run on the robot since. The
array's distance ahead of the axle (`ARRAY_AHEAD_MM`) and the nose position in the test world are
assumptions.
