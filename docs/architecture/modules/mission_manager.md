# mission_manager

The run, part by part. Pure logic in `include/mission/MissionManager.h` (with `Parts.h`, `RunStart.h`,
`MissionParams.h` and `Fault.h`), tested through `test/integration/test_section_a/` in a simulated arena and by
`test/integration/test_run_loop/`.
The parts are defined, and the reasons for the cuts given, in
[0016](../decisions/0016-cut-the-run-into-parts.md). Built parts: Section A's maze
([cell_explorer](cell_explorer.md)), the way out of the marker tile ([marker_exit](marker_exit.md))
and the line zone ([line_follow](line_follow.md)). The manager owns the one motion controller they feed
([motion_control](motion_control.md)).

## Interface

- `Part` - `AExplore`, `AProve`, `AExit`, `Line`, `BEntry`, `BExplore`, `BProve`, in course order.
  `partBuilt()`, `partStartsAlone()` and `partUsesMap()` say what each is; `PartRange {first, last}` is
  which to run (the whole course by default). The enum is in `spec/tuning/Run.h`.
- `RUN_PART_FIRST`, `RUN_PART_LAST` (`spec/tuning/Run.h`) - the parts this firmware runs: `main.cpp` passes
  them to `runLoopInit()`. The whole course is the setting for the competition. To test one part or a
  stretch, edit the two constants and flash; no switch selects the parts. Put the robot where the
  first part expects it (the start tile for `AExplore`, on the line for `Line`). A first part after
  the last is a compile error; a selection that cannot run is refused at the start (`BadConfig`).
- `MissionManager` - the state: `stage` (`WaitStart`, `Running`, `Done`, `Fault`), the running `part`,
  `phase` as the status line numbers it, `fault`, the start latch (`RunStart`), the `CellExplorer`, the
  `MarkerExit`, the `LineFollow` and the motion controller's integrals.
- `MissionParams`, `missionParams()` - the part range, the explorer's and the marker exit's
  parameters, the controller gains and the line follower's, every one at its `spec/` value. The caller
  may set `parts` and `wallsOnly`.
- `missionReset(ms)`, `missionFail(ms, code)` - the caller fails a run for a precondition it found
  missing (`NoImu`).
- `missionTick(ms, params, perception, nowMs, go)` - one tick of a run: the start, the run
  limit, then the parts. `missionStep(ms, params, perception)` is the same without the start and the
  limit, for a caller that sets `ms.start.started` itself. Both return the `DriveCommand`.
- `missionTakeHeadingFix(ms)` - degrees to subtract from the gyro heading
  (`headingSet(hd, hd.deg - fix)`), 0 when there is none. Call it every tick after the step.
- `RunStart`, `runStartTick()` - the start latch: the run begins the first time `go` is true and stays
  begun. When to say `go` is the operator panel's ([panel](panel.md)); the mission knows nothing of
  buttons.

## How it works

`WaitStart` holds the motors off until the run has started. Then the range is resolved and the first
part begins:

- `first` must be built and able to start alone, or the run is refused (`BadConfig`).
- `last` is the last built part up to the one asked for. With `wallsOnly` (no line calibration) it is
  `AExplore`.

Each tick the running part is stepped and reports one of: `Running`, `Done` (the next part begins on the
next tick), `HandOver` (it ended while moving on, so the next part acts in the same tick), `DoneHold`
(ended, wheels held with no control step this tick) or `Fault`. When the last part of the range has
ended the run is `Done` and holds the wheels with the drivers enabled. The run limit (`RUN_LIMIT_MS`, 8
minutes from the start) fails a run still in progress (`TimeUp`). A stalled controller is `Stall`.

- **AExplore:** the explorer's phases ([cell_explorer](cell_explorer.md)). While it steps, the manager
  feeds the floor pattern; when the marker is told it asks the explorer to stop on the tile, and if the
  way out is straight on and the exit part is next it hands over at once, without a stop. With no
  part after it, the explorer brakes and the run ends on the marker's tile.
- **AExit:** [marker_exit](marker_exit.md): stand and read both sides when a wall is ahead, turn to the
  open one, drive on over the strip and the gap. The first line view past the gap hands over.
- **Line:** `lineFollowStep`. A lost line is searched for and then `LineLost` cuts the motors; the
  part ends when the line is over and the robot has stopped (only with `LINE_END_LOST_MM` above zero).

`phase` is what the status line prints: the explorer's `Sense`, `Turn` and `Step`, the marker exit's
standing and turning as `Sense` and `Turn`, then `Approach`, `Line`, `Done`, `Fault`. The phase line
also prints the part number.

## Invariants

- The manager never writes `RobotState`; it reads `Perception` and returns a `DriveCommand`.
- Only the manager calls `motionControlStep()`; the parts fill a `MotionTarget`.
- Where the part has a map the robot holds still square to the map direction, otherwise on the heading
  it has.
- The marker is looked for only in `AExplore`.
- `Approach` and `Line` come last in `MissionPhase`, so the numbers logged for the older phases are
  unchanged.

## Status

Host tests against ray-cast walls, a plant with stiction, encoder and gyro noise, ultrasonic
dropouts and strays: all 60 starts of the photo maze reach the marker within 40 mm of its centre,
with no wall contact and the map position right; also with 30 % dropped readings and 5 % stray
echoes, a gyro scale 2 % off with 9 deg/min of drift, a wall in the front sensor's dead zone and an
obstacle held in front for a moment. Section A alone, the line alone (with the robot placed on it) and
selections that cannot run are tested in one table.

Ran on the robot at the mid-evaluation (tag `v0.1-section-a`), before the mission was split and the
manager added; not run on the robot since. The array's distance ahead of the axle (`ARRAY_AHEAD_MM`)
and the nose position in the test world are assumptions.
