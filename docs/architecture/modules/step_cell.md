# step_cell

Drive from one cell centre to the next, reading its walls on the way. Pure logic in
`include/StepCell.h`, tested by `test/test_step_cell/` in a simulated corridor.

## Interface

- `StepCellParams` - `PlanParams`, `WallParams`, and pitch, cruise speed, `sampleLeadMm`,
  `sampleWindowMm`, `frontStopMm`, `frontAgreeMm`, `anchorFilter`, which default to `RobotSpec.h`.
- `stepCellBegin(sc, params, fb, headingRef)` - start a move to the next cell centre.
- `stepCellStep(sc, params, fb, view, dtMs, out)` - one tick; returns a `MoveStatus`.
- `stepCellContinue(sc, params)` - the next cell is straight on: roll through this centre.
- `sc.wallsReady`, `sc.walls` (`CellWalls`: front, left, right, `unreliable`).
- `sc.blocked` - the move ended short of its cell. `Done` with `blocked` is not an arrival: the
  map must not advance.

## How it works

`sampleLeadMm` before the target centre, the step has voted on the cell's three walls over the
preceding `sampleWindowMm` and raised `wallsReady`. `stepCellContinue()` extends the target one
pitch and the robot rolls through; **no answer stops it on the centre**, so a caller that never
answers is merely slow. Straight cells run without stopping and only turns stop.

A vote is present / open / no-echo per sensor. A wall is present when seen at least as often as
open; a sensor with no echo counts as present and sets `unreliable`.

A wall ahead moves the stop in two roles:

- **Drift correction.** The cell's vote confirmed the wall: the centre is `frontStopMm` short of
  it, which cancels odometer drift. Believed only under half a pitch.
- **Safety stop.** The reading is at or inside `frontStopMm`: the robot is already past where it
  should brake, so the stop is pulled in by any amount (never out). Half a pitch or more sets
  `blocked`.

Each echo is placed on the odometer and compared with the one before; it moves the stop only if
they agree within `frontAgreeMm`, so one stray echo cannot. A real wall is believed one echo later
(9 mm at 150 mm/s). An echo is paired with the position **at the time it changed**, not the
current one: pairing it with the current position made the target chase the robot and hunt (found
by the mission simulation). The accepted position is smoothed by `anchorFilter`, since the front
alternated 110 and 119 mm at a dead end and retargeting to each restarted the profile.

## Invariants

- `wallsReady` rises once per cell; begin and continue clear the cell's votes, walls and `blocked`.
- With no wall in view the move holds the gyro heading. The `WallSteer` keeps its bias across
  `stepCellBegin()` and drops it when the heading moves 20 degrees; a caller that snaps the
  heading to the grid calls `wallSteerReset()`.
- `frontStopMm == 0` disables anchoring and the safety stop.
- The stop is never closer than the braking distance (`straightRetarget`).

## Status

Host tests, 100% line coverage; `RobotSpec.h` asserts the sampling lead covers the braking
distance. The simulated sensors are ideal apart from alternating jitter and single stray echoes.

Run on the robot through the mission (2026-10-14), before #30's wall integral and the echo
gate: steps through cells, stops at dead ends and reads the walls correctly, within the 8 mm
step tolerance. The gate, the safety stop and `blocked` are not run on the robot, and
`FRONT_AGREE_MM` comes from the measured jitter, not from logged stray echoes.
