# step_cell

Drive from one cell centre to the next, reading its walls on the way. Pure logic in
`include/StepCell.h`, tested by `test/test_step_cell/` in a simulated corridor.

## Interface

- `StepCellParams` - `PlanParams`, `WallParams`, pitch, cruise speed, `sampleLeadMm`,
  `sampleWindowMm`, `frontStopMm`, `frontAgreeMm`, `anchorFilter`. All but the two plan structs
  default to `RobotSpec.h` (`TILE_PITCH_MM`, `SEARCH_SPEED_MM_S`, `WALL_SAMPLE_LEAD_MM`,
  `WALL_SAMPLE_WINDOW_MM`, `FRONT_STOP_MM`, `FRONT_AGREE_MM`, `FRONT_ANCHOR_FILTER`).
- `stepCellBegin(sc, params, fb, headingRef)` - start a move to the centre of the next cell.
- `stepCellStep(sc, params, fb, view, dtMs, out)` - one tick; returns a `MoveStatus`.
- `stepCellContinue(sc, params)` - the next cell is straight on: carry on through this centre
  without stopping.
- `sc.wallsReady` / `sc.walls` (`CellWalls`: front, left, right, `unreliable`); `sc.walls` is
  cleared by `stepCellBegin()` and `stepCellContinue()`.
- `sc.blocked` - the move ended short of the cell because a wall stood inside the stop distance
  (see below). `Done` with `blocked` is not an arrival: the map must not advance.

## How it works

Shortly before the target centre (`sampleLeadMm` away) the step has voted on that cell's three
walls over the preceding `sampleWindowMm` and raised `wallsReady` once. The caller decides what
comes next. Answering with `stepCellContinue()` extends the target one pitch and the robot rolls
through the centre; **no answer means it stops on the centre by itself**, so a caller that never
answers is merely slow. This is what lets straight cells run without stopping and only turns stop.

Votes are present / open / no-echo per sensor; a wall is present when seen at least as often as
open, and a sensor that never heard an echo counts as present and sets `unreliable`.

A wall ahead moves the stop, in two roles:

- **Drift correction.** The cell's own vote confirmed the wall: the centre is `frontStopMm` short
  of it, which cancels the odometer drift built up over the run. Only small corrections are
  believed (under half a pitch); a big one means the reading is something else.
- **Safety stop.** The reading is already at or inside `frontStopMm`: the robot is past where it
  should brake, whatever the odometer says, so the stop is pulled in by any amount. A pull of
  half a pitch or more sets `blocked`. Stopping is always the safe answer, so this role has no
  size limit; it only ever shortens the move.

Each echo is placed on the odometer (the position it was taken at plus the reading's excess over
`frontStopMm`) and compared with the echo before it. It moves the stop only if the two agree
within `frontAgreeMm`, so one stray echo, near or far, cannot. The cost is one extra echo (9 mm of
travel at 150 mm/s) before a real wall is believed. The echo is paired with the odometer position
**at the time it changed**, not the current one: a sensor refreshes every few ticks, and combining
an old reading with the current position made the target chase the robot and hunt back and forth
(found by the mission simulation, not by the unit tests).

The accepted position is then **smoothed** (`anchorFilter`, once per accepted echo, on the position
and not on the reading, so it adds no lag while the robot approaches). On the robot the front
reading alternated 110 and 119 mm at a dead end; retargeting to each one restarted the profile
every few readings and the move never settled inside the stop tolerance.

The stop never comes closer than the reference's braking distance (`straightRetarget`), so a
late correction brakes hard instead of reversing.

## Invariants

- `wallsReady` rises once per cell and is cleared by `stepCellContinue()`.
- Steering is by trim on the gyro-held heading, so with no wall in view it just holds heading. The
  `WallSteer` keeps what it learned across `stepCellBegin()` and drops it by itself when the heading
  moves 20 degrees (a turn); a caller that snaps the heading to the grid calls `wallSteerReset()`.
- `frontStopMm == 0` disables anchoring, safety stop included.

## Status

Host tests, 100% line coverage. `RobotSpec.h` asserts the sampling lead is at least the stopping
distance (`v^2 / 2a`, 22 mm against the 50 mm lead).

The simulation has ideal ultrasonics apart from the alternating-jitter test and single stray
echoes. Run on the robot through the mission (2026-10-14), before the wall integral of #30 and the
echo-agreement gate: from several start cells, steps through cells, stops at dead ends and reads
the walls correctly; the stop is within the 8 mm step tolerance. The gate, the safety stop and
`blocked` have not been run on the robot, and `FRONT_AGREE_MM` is from the measured jitter, not a
log of stray echoes.
