# section_a

The Section A mission: from any tile, explore until the white tile and stop on it. Pure logic in
`include/SectionA.h`, tested by `test/test_section_a/` in a simulated arena. It joins the planner
([maze](maze.md)), the cell step ([step_cell](step_cell.md)), the turn
([motion_primitives](motion_primitives.md)) and [perception](perception.md); the decision is
[0009](../decisions/0009-explore-in-a-start-relative-map.md).

## Interface

- `SectionA` - the state: phase, fault, the `Explorer` map, the `StepCell`, the `Turn` and the
  motion controller's integrals. Static; about 400 B.
- `sectionAReset(ms)`, `sectionAFail(ms, code)` - the caller fails a run for a precondition it
  found missing (`NoImu`).
- `sectionAParams()` - every parameter at its `RobotSpec.h` value (`SECTION_A_*`, the plan, the
  controller gains). The caller may set `wallsOnly`, `finishAtStart`.
- `sectionAStep(ms, params, perception)` - one tick; returns the `DriveCommand`.
- `sectionATakeHeadingFix(ms)` - degrees to subtract from the gyro heading
  (`headingSet(hd, hd.deg - fix)`), 0 when there is none. Call it every tick after
  `sectionAStep`.

## How it works

`WaitStart` holds the motors off until `startPressed`, then takes the gyro heading as map North.

- **Sense:** stand still for `senseMs` and vote on the three walls. The explorer records what was
  heard (a wall only if heard and present) and plans.
- **Turn:** face the planned tile, then Sense again: the new front may be an edge nothing has
  looked at. A stuck turn is retried `turnRetries` times, then `StuckTurn`.
- **Step:** drive one tile. The step reads the walls of the tile ahead 50 mm before its centre;
  there the explorer moves into it, records them and plans. A tile straight on is rolled through
  (`stepCellContinue`), anything else stops on the centre and the planner turns.
- **Done:** the white tile is under the array: it stops on its centre and holds with the drivers
  enabled. **This is where what follows Section A takes over** (the line follower, not built yet).
- **Fault:** motors off, with `fault` saying why.

### Rules that keep the map true

- A front that stays unheard after `frontTries` looks is a wall in the sensor's dead zone (below
  ~114 mm from the axle): it is recorded as one, not driven at.
- A verdict from a step that was cut short is not taken. After such a stop the robot stands and
  looks: with no wall ahead the step is resumed to its planned centre (`resumes` times), with
  one the explorer is put where the odometer says (`stepCellCells`, `explorerRetreat`).
- No route and no white tile: the map is forgotten and explored again up to `rescans` times (a wall
  that was only a bad reading is the likeliest cause), then `NoRoute`.
- More than `maxBlocked` cut-short steps in a run: `TooManyStops`.

### The finish

The floor sensor tells the white tile: white under the array (`LineView` valid and not all black)
for `finishWhiteMs`, read while the robot is in a tile that is not the start tile. It is decided
at the step's verdict, 50 mm before the centre, with the array already well over the new tile.
`wallsOnly` skips it: the run explores every reachable tile and stops. `finishAtStart` makes a run
that begins on the white tile finish at once; it is off because the competition's start tile is
white and must be left.

## Invariants

- The mission never writes `RobotState`; it reads `Perception` and returns a `DriveCommand`.
- Map North is the gyro heading at start, and map directions run clockwise where the gyro runs
  counter-clockwise: `gridHeading` is `h0 - 90 * dir`.
- The explorer's cell and the robot's agree at every stop on a centre: the tests check it.

## Status

Host tests against ray-cast walls, a plant with stiction, encoder and gyro noise, ultrasonic
dropouts and strays: all 60 starts of the photo maze reach the white tile within 40 mm of its
centre, with no wall contact and the map position right; also with 30 % dropped readings and 5 %
stray echoes, a gyro scale 2 % off with 9 deg/min of drift, a wall in the front sensor's dead zone
and an obstacle held in front for a moment.

Not run on the robot, and not wired into `main.cpp`: nothing calls `sectionAStep` yet. The IR array
is not calibrated, so the finish test has not seen a real floor; the array's distance ahead of the
axle (`ARRAY_AHEAD_MM`) and the nose position in the test world are assumptions.
