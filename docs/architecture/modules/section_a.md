# section_a

The Section A mission: from any tile, explore until the approach marker and stop on its tile. Pure logic in
`include/mission/SectionA.h`, tested by `test/test_section_a/` in a simulated arena. It joins the planner
([maze](maze.md)), the cell step ([step_cell](step_cell.md)), the turn
([motion_primitives](motion_primitives.md)) and [perception](perception.md); the decision is
[0009](../decisions/0009-explore-in-a-start-relative-map.md).

## Interface

- `SectionA` - the state: phase, fault, the `Explorer` map, the `StepCell`, the `Turn` and the
  motion controller's integrals. Static; about 400 B.
- `sectionAReset(ms)`, `sectionAFail(ms, code)` - the caller fails a run for a precondition it
  found missing (`NoImu`).
- `sectionAParams()` - every parameter at its `spec/` value (`spec/Mission.h` and the others) (`SECTION_A_*`, the plan, the
  controller gains). The caller may set `wallsOnly`, `finishAtStart`.
- `sectionAStep(ms, params, perception)` - one tick; returns the `DriveCommand`.
- `sectionATakeHeadingFix(ms)` - degrees to subtract from the gyro heading
  (`headingSet(hd, hd.deg - fix)`), 0 when there is none. Call it every tick after
  `sectionAStep`.

## How it works

`WaitStart` holds the motors off until `startPressed` (the run loop sets it when the run begins:
[run_loop](run_loop.md)), then takes the gyro heading as map North.

- **Sense:** stand still for `senseMs` and vote on the three walls. The explorer records what was
  heard (a wall only if heard and present) and plans.
- **Turn:** face the planned tile, then Sense again: the new front may be an edge nothing has
  looked at. A stuck turn is retried `turnRetries` times, then `StuckTurn`.
- **Step:** drive one tile. The step reads the walls of the tile ahead 50 mm before its centre;
  there the explorer moves into it, records them and plans. A tile straight on is rolled through
  (`stepCellContinue`), anything else stops on the centre and the planner turns.
- **Approach:** the marker has been seen. The hand-over is at the first line view past the gap, or,
  if no gap read as air (it did not on the robot: 915-1010 against the floor's 1009-1020), at a
  mixed line view `BRIDGE_HANDOVER_NO_GAP_MM` (230 mm) past the box's centre. Straight on at the line speed over the strip and the gap
  (`straightStep` along the grid heading, `FloorPattern` for the gap). If a wall is ahead on the
  marker's tile it stops, stands `senseMs` reading both sides, turns to the open side that answers
  from far or not at all (`medianOf`, a silent reading counted as the sensor's range), then goes.
- **Line:** the first line view past the gap hands over to `lineFollowStep`
  ([line_follow](line_follow.md)). Nothing ends it: the robot is powered off at the end of the line,
  and if not, a lost line is searched for and then `LineLost` cuts the motors.
- **Done:** only with `followLine` false (stop on the marker's tile, holding with the drivers
  enabled) or `wallsOnly` (everything explored).
- **Fault:** motors off, with `fault` saying why.

### Rules that keep the map true

- A front that stays unheard after `frontTries` looks is a wall in the sensor's dead zone (below
  ~114 mm from the axle): it is recorded as one, not driven at. Unless the way is settled another
  way, which the marker tile (open to nothing) needs: an edge a side sensor measured open on the
  way in (`openHeard`), an edge the right-hand sensor found open after a quarter turn to look at it
  (`peekedOpen`: a side sensor has no dead zone), or a cell already entered
  (`explorerAheadVisited`). A silent front through a sampling window, once the sensor has worked,
  is also open.
- A verdict from a step that was cut short is not taken. After such a stop the robot stands and
  looks: with no wall ahead the step is resumed to its planned centre (`resumes` times), with
  one the explorer is put where the odometer says (`stepCellCells`, `explorerRetreat`).
- No route and no marker: the map is forgotten and explored again up to `rescans` times (a wall
  that was only a bad reading is the likeliest cause), then `NoRoute`.
- More than `maxBlocked` cut-short steps in a run: `TooManyStops`.

### The finish

Decision: [0010](../decisions/0010-finish-on-the-approach-marker.md), then
[0011](../decisions/0011-leave-the-maze-onto-the-line-without-stopping.md) for what follows.

The floor sensor tells the marker: white, black, white under the array along the path
([floor_pattern](floor_pattern.md)), at the odometer position of the array's sensing line
(`IR_ARRAY_FORWARD_OFFSET_MM` ahead of the axle). It is fed only in the Step phase. The event comes
as the far frame reaches `MARKER_FAR_FRAME_MIN_MM`, about 15 mm before the robot's centre is on the
tile's. A stop short of it still counts (`floorPatternMarkerPending`: near frame and box complete,
far frame begun). `wallsOnly` skips it: the run explores every reachable tile and stops. A robot
placed on the marker's own tile cannot finish there. Decision:
[0010](../decisions/0010-finish-on-the-approach-marker.md).

## Invariants

- The mission never writes `RobotState`; it reads `Perception` and returns a `DriveCommand`.
- Map North is the gyro heading at start, and map directions run clockwise where the gyro runs
  counter-clockwise: `gridHeading` is `h0 - 90 * dir`.
- The explorer's cell and the robot's agree at every stop on a centre: the tests check it.
- `Approach` and `Line` come last in `SectionAPhase`, so the numbers logged for the older phases
  are unchanged.

## Status

Host tests against ray-cast walls, a plant with stiction, encoder and gyro noise, ultrasonic
dropouts and strays: all 60 starts of the photo maze (with the marker tile open to the outside) reach the marker within 40 mm of its
centre, with no wall contact and the map position right; also with 30 % dropped readings and 5 %
stray echoes, a gyro scale 2 % off with 9 deg/min of drift, a wall in the front sensor's dead zone
and an obstacle held in front for a moment.

Not run on the robot. `main.cpp` calls it through [run_loop](run_loop.md). The IR array is not
calibrated, so the finish test has not seen a real floor; the array's distance ahead of the axle
(`ARRAY_AHEAD_MM`) and the nose position in the test world are assumptions.
