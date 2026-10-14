# step_cell

Drive from one cell centre to the next, reading its walls on the way. Pure logic in
`include/StepCell.h`, tested by `test/test_step_cell/` in a simulated corridor.

## Interface

- `StepCellParams` - `PlanParams`, `WallParams`, pitch, cruise speed, `sampleLeadMm`,
  `sampleWindowMm`, `frontStopMm`. The values come from `RobotSpec.h` (`TILE_PITCH_MM`,
  `SEARCH_SPEED_MM_S`, `WALL_SAMPLE_LEAD_MM`, `WALL_SAMPLE_WINDOW_MM`, `FRONT_STOP_MM`).
- `stepCellBegin(sc, params, fb, headingRef)` - start a move to the centre of the next cell.
- `stepCellStep(sc, params, fb, view, dtMs, out)` - one tick; returns a `MoveStatus`.
- `stepCellContinue(sc, params)` - the next cell is straight on: carry on through this centre
  without stopping.
- `sc.wallsReady` / `sc.walls` (`CellWalls`: front, left, right, `unreliable`).

## How it works

Shortly before the target centre (`sampleLeadMm` away) the step has voted on that cell's three
walls over the preceding `sampleWindowMm` and raised `wallsReady` once. The caller decides what
comes next. Answering with `stepCellContinue()` extends the target one pitch and the robot rolls
through the centre; **no answer means it stops on the centre by itself**, so a caller that never
answers is merely slow. This is what lets straight cells run without stopping and only turns stop.

Votes are present / open / no-echo per sensor; a wall is present when seen at least as often as
open, and a sensor that never heard an echo counts as present and sets `unreliable`.

A wall ahead re-anchors the stop: the centre is `frontStopMm` short of it. This cancels the
odometer drift built up over the run. It is trusted when this cell's own vote confirmed the wall,
or when the reading is already at or inside the stop distance (a wall never voted on still cannot
be driven into), and only small corrections are believed (under half a pitch). The reading is
paired with the odometer position **at the time it changed**, not the current one: a sensor
refreshes every few ticks, and combining an old reading with the current position made the target
chase the robot and hunt back and forth (found by the mission simulation, not by the unit tests).

The anchored stop position is then **smoothed** (`STEP_ANCHOR_FILTER`, applied once per new reading, to the
position and not to the reading, so it adds no lag while the robot approaches). On the robot the front reading
alternated 110 and 119 mm at a dead end; retargeting to each one restarted the profile every few readings, the
move never settled inside the stop tolerance, and it timed out.

## Invariants

- `wallsReady` rises once per cell and is cleared by `stepCellContinue()`.
- Steering is by trim on the gyro-held heading, so with no wall in view it just holds heading.
- `frontStopMm == 0` disables anchoring.

## Status

16 host tests, 100% line coverage. The sampling lead must be at least the stopping distance (`v^2 / 2a`) or
a decision to stop brakes late; nothing checks that yet (at 150 mm/s and 500 mm/s^2 it is 22 mm against the 50 mm
lead). Run on the robot through the mission (2026-10-14): from several start cells, steps through
cells, stops at dead ends and reads the walls correctly; the stop is within the 8 mm step tolerance. The
simulation has ideal ultrasonics apart from the alternating-jitter test.
