# step_cell

Drive from one cell centre to the next, reading its walls on the way. Pure logic in
`include/StepCell.h`, tested by `test/test_step_cell/` in a simulated corridor.

## Interface

- `StepCellParams` - `PlanParams`, `WallParams`, and pitch, cruise speed, `sampleLeadMm`,
  `sampleWindowMm`, `frontStopMm`, `frontAgreeMm`, `anchorFilter`, `echoMs`, `blindMs`, `snapMm`,
  `snapGain`, `snapMinDeg`, `snapMaxDeg`, which default to `RobotSpec.h`.
- `stepCellBegin(sc, params, fb, headingRef)` - start a move to the next cell centre.
- `stepCellStep(sc, params, fb, view, dtMs, out)` - one tick; returns a `MoveStatus`.
- `stepCellContinue(sc, params)` - the next cell is straight on: roll through this centre.
- `sc.wallsReady`, `sc.walls` (`CellWalls`: front, left, right, `unreliable`).
- `sc.blocked` - the move ended short of its cell. `Done` with `blocked` is not an arrival.
  `stepCellCells()` gives the cells covered (from the odometer; after a stop on a wall's stop
  point, the cell it is in) and `stepCellRemainingMm()` the distance to the planned centre;
  `stepCellResume()` begins again toward that centre, for a stop that no wall explains.
- `stepCellTakeHeadingFix(sc)` - degrees to subtract from the gyro heading
  (`headingSet(hd, hd.deg - fix)`), and 0 when there is none.

## How it works

`sampleLeadMm` before the target centre, the step has voted on the cell's three walls over the
preceding `sampleWindowMm` and raised `wallsReady`. `stepCellContinue()` extends the target one
pitch and the robot rolls through; **no answer stops it on the centre**, so a caller that never
answers is merely slow. Straight cells run without stopping and only turns stop.

A vote is present / open / no-echo per sensor. A wall is present when seen at least as often as
open; a sensor that heard no echo all window counts as present and sets `unreliable` (a few
missed echoes among heard ones do not).

### The wall ahead

A front echo is placed on the odometer (the position it was taken at plus its excess over
`frontStopMm`) and compared with the echo before it; two that agree within `frontAgreeMm` fix the
wall's stop point, so one stray echo cannot. That position is kept: when the sensor goes quiet (the
dead zone below ~114 mm from the axle, a dropout) the wall does not vanish. It moves the stop in
two roles:

- **Drift correction.** The cell's vote confirmed the wall: the centre is `frontStopMm` short of
  it, which cancels odometer drift. Believed only under half a pitch, and smoothed by
  `anchorFilter` (the front alternated 110 and 119 mm at a dead end).
- **Safety stop.** The wall's stop point is within the braking distance at the current speed plus
  one echo of travel (`echoMs`): the stop is pulled in by any amount, never out. Half a pitch or
  more sets `blocked`. At rest the trigger is `frontStopMm` itself.

`straightRetarget` never puts the stop inside the braking distance, so a late correction brakes
hard instead of reversing.

**Start.** A move that has no agreeing pair of echoes and no echo well clear of the stop point
(more than `sampleLeadMm` beyond it) is held at rest, waiting. If none comes within `blindMs` the
wall is taken to be in the dead zone: `blocked`, without moving. `blindMs = 0` turns the wait off,
for a caller that chooses to go blind on the odometer.

### The heading frame

Along one wall mode (both walls, or one) the lateral error changes at the rate of the robot's
heading against the corridor, whatever the controller does. Over `snapMm` of travel a fitted line
gives that heading to about a degree, which puts the corridor in the gyro frame; its distance from
the grid heading is the frame's error (drift, scale, a turn that stopped short). `snapGain` of it
is handed back through `stepCellTakeHeadingFix()` and taken out of the steer's bias, which was
making up for it. Without this the bias is dropped at every turn and the error grows with the run.
A measured error over `snapMaxDeg` is not taken as the gyro's.

## Invariants

- `wallsReady` rises once per cell; begin and continue clear the cell's votes, walls and `blocked`.
- With no wall in view the move holds the gyro heading. The `WallSteer` keeps its bias across
  `stepCellBegin()` and drops it when the heading moves 20 degrees; a caller that snaps the
  heading to the grid calls `wallSteerReset()`.
- `frontStopMm == 0` turns off all of the front logic.
- The stop is never closer than the braking distance.

## Status

Host tests, 100% line coverage; `RobotSpec.h` asserts the sampling lead covers the braking
distance.

The host robot model (the real perception, step and motion chain, in random 6x6 mazes under a
left-hand rule, 800 stops over 10 mazes) completes with no position error, no wall contact and the
heading within 4 degrees, also with battery sag, a rough floor, gyro scale 1% and drift 3 deg/min,
a track 3% off, a weak wheel, 15% dropouts and 5% stray side echoes. Odometer errors of 3% still
rub a wall in some runs: nothing corrects distance along a long open run except a wall ahead.

Run on the robot through the mission (2026-10-14), before #30's wall integral and everything
since: steps through cells, stops at dead ends and reads the walls correctly, within the 8 mm step
tolerance. The echo gate, safety stop, blind start and heading fix have only run on the host, and
`FRONT_AGREE_MM`, `FRONT_BLIND_MS`, `WALL_SNAP_MM` and `WALL_SNAP_GAIN` come from the measured
jitter and the host model, not from logs of stray echoes or a long run.
