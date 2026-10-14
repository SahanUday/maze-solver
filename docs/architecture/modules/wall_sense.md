# wall_sense

Ultrasonic readings to "is there a wall" and a centring trim. Pure logic in
`include/WallSense.h`, tested by `test/test_wall_sense/`. The readings come from
[ultrasonic](ultrasonic.md) through `RobotState`.

## Interface

- `WallParams` - presence thresholds, the single-wall centre distance, `kp`, `kd`,
  `steerMaxDeg`; built by `specWall()`.
- `wallView(state, params)` - one tick's `WallView`: per sensor `valid`, `mm`, and
  `front/left/right` (valid and under the threshold).
- `wallCenterError(view, params, errMm)` - lateral error, positive when the robot sits right of
  centre; false with no side wall in view.
- `wallSteerTrim(steer, view, params, dtMs)` - heading offset in degrees (CCW positive) that
  steers back to the centre line.

## How it works

Readings carry their `US_OFFSET_*`, so left and right are measured from the same reference
line and `(left - right) / 2` is the offset from the corridor's centre. With one wall the error
is that wall's distance against `WALL_CENTER_DIST_MM`. The error is low-passed
(`WALL_ERR_FILTER`) and the filter **restarts** when the set of walls changes (both, left only,
right only), so a wall appearing or ending does not give a spike. The trim is `kp * error +
kd * rate`, clamped to `steerMaxDeg`.

A sensor with no echo is neither wall nor open: its distance is stale and is never used. The
mission's vote ([step_cell](step_cell.md)) reports it as a wall, the safe answer.

## Invariants

- Each sensor refreshes only every `US_SENSOR_COUNT * US_SLOT_MS` (60 ms), so a reading is up to
  that old.
- With no side wall in view the trim is 0 and the gyro alone holds the heading.

## Status

15 host tests, 100% line coverage. All the thresholds and gains in `RobotSpec.h` are still 0.
The corridor width used for the single-wall target is not derived from `USABLE_CORRIDOR_MM`
(235) because that disagrees with the arena sheet (250 - 10 = 240); it is its own constant to be
measured on a real tile.
