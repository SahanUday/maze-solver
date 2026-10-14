# wall_sense

Ultrasonic readings to "is there a wall" and a centring trim. Pure logic in `include/WallSense.h`,
tested by `test/test_wall_sense/`. The readings come from [ultrasonic](ultrasonic.md) through
`RobotState`.

## Interface

- `WallParams` - presence thresholds, the single-wall centre distance, `kp`, `kd`, `steerMaxDeg`.
  From `RobotSpec.h`: `WALL_PRESENT_SIDE_MM`, `WALL_PRESENT_FRONT_MM`, `WALL_CENTER_DIST_MM`,
  `KP_WALL`, `KD_WALL`, `STEER_MAX_DEG`.
- `wallView(state, params)` - one tick's `WallView`: per sensor `valid`, `mm`, and
  `front/left/right` (valid and under the threshold).
- `wallCenterError(view, params, errMm)` - lateral error, positive when the robot sits right of
  centre; false with no side wall in view.
- `wallSteerTrim(steer, view, params, dtMs)` - heading offset in degrees (CCW positive) toward the
  centre line.

## How it works

With both walls the error is `(left - right) / 2`; with one it is that wall's distance against
`WALL_CENTER_DIST_MM`. The error is low-passed (`WALL_ERR_FILTER`), and the filter restarts when the
set of walls changes (both, left only, right only). The trim is `kp * error + kd * rate`, clamped
to `steerMaxDeg`.

A sensor with no echo is neither wall nor open: its distance is stale and never used. A caller that
has to decide treats it as a wall.

## Invariants

- Each sensor refreshes every `US_SENSOR_COUNT * US_SLOT_MS` (60 ms), so a reading is up to that old.
- With no side wall in view the trim is 0 and the gyro alone holds the heading.

## Status

Measured on the robot (2026-10-14): thresholds and centre distance in `RobotSpec.h`; `KP_WALL` held
the robot within about +-10 mm of the centre.
