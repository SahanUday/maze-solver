# wall_sense

Ultrasonic readings to "is there a wall" and a centring trim. Pure logic in
`include/WallSense.h`, tested by `test/test_wall_sense/`. The readings come from
[ultrasonic](ultrasonic.md) through `RobotState`.

## Interface

- `WallParams` - presence thresholds, the single-wall centre distance, `kp`, `kd`,
  `steerMaxDeg`. The values come from `RobotSpec.h` (`WALL_PRESENT_SIDE_MM`, `WALL_PRESENT_FRONT_MM`,
  `WALL_CENTER_DIST_MM`, `KP_WALL`, `KD_WALL`, `STEER_MAX_DEG`).
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

A sensor with no echo is neither wall nor open: its distance is stale and is never used. A caller
that has to decide treats it as a wall, the safe answer.

## Invariants

- Each sensor refreshes only every `US_SENSOR_COUNT * US_SLOT_MS` (60 ms), so a reading is up to
  that old.
- With no side wall in view the trim is 0 and the gyro alone holds the heading.

## Status

15 host tests, 100% line coverage. Measured on the robot (2026-10-14): a wall reads 117 +- 30 mm from the
centreline and an opening 396 mm or more, so `WALL_PRESENT_SIDE_MM` is 250; the corridor measured 231-235 mm
(left plus right of a centred robot), so `WALL_CENTER_DIST_MM` is 117, its own constant and not derived from
`USABLE_CORRIDOR_MM`. `KP_WALL` is 0.1: with the heading integral at its turn-sized value, 0.2 and 0.4 swung
the heading by 17 to 19 degrees, and 0.1 held the robot within about +-10 mm of the centre. The front reading is
good to about 3 mm up to 300 mm and falls up to 6% short beyond that (the beam is wider than the corridor).
