# wall_sense

Ultrasonic readings to "is there a wall" and a centring trim. Pure logic in `include/sense/WallSense.h`,
tested by `test/test_wall_sense/`. The readings come from [ultrasonic](ultrasonic.md) through
`RobotState`.

## Interface

- `WallParams` - presence thresholds, the single-wall centre distance, the pair tolerance, `kp`,
  `ki`, the bias limits, `steerMaxDeg`. The defaults are the `RobotSpec.h` values
  (`WALL_PRESENT_*`, `WALL_CENTER_DIST_MM`, `WALL_PAIR_TOL_MM`, `KP_WALL`, `KI_WALL`,
  `WALL_BIAS_*`, `STEER_MAX_DEG`).
- `wallView(state, params)` - one tick's `WallView`: per sensor `valid`, `mm`, and
  `front/left/right` (valid and under the threshold).
- `wallCenterError(view, params, errMm)` - lateral error, positive when the robot sits right of
  centre; false with no side wall in view, or with two that disagree with the corridor width.
- `wallSteerTrim(steer, view, params, headingDeg, dtMs)` - heading offset in degrees (CCW positive)
  toward the centre line. `headingDeg` is the gyro heading.
- `wallSteerReset(steer)` - forget the filter and the learned bias.
- `wallSteerShiftFrame(steer, deltaDeg)` - the caller moved the heading frame (`headingSet`): keep
  the bias anchored to it.

## How it works

With both walls the error is `(left - right) / 2`; with one it is that wall's distance against
`WALL_CENTER_DIST_MM`. The error is low-passed (`WALL_ERR_FILTER`), and the filter restarts when the
set of walls changes (`WallMode`: both, left only, right only).

The trim is `kp * error + bias`. A proportional term alone leaves a steady offset of
`heading error / kp` (3 deg is 30 mm, past the 26.5 mm each side a centred robot has), because the
heading reference the gyro holds is wrong by that much. The integral (`ki`) learns that error as
`biasDeg` and keeps applying it through gaps in the walls. It is:

- clamped to `WALL_BIAS_MAX_DEG`, and not advanced while the trim is pinned at `steerMaxDeg`;
- dropped by itself when the heading moves `WALL_BIAS_RESET_DEG` from where it was learned (a
  turn), so no caller has to remember to reset it;
- held for `WALL_BIAS_HOLD_MS` with no wall to confirm it, then faded at `WALL_BIAS_FADE_DEG_S`, so
  a bias learned from a bad reading does not steer the robot for ever.

There is no derivative term: a sensor refreshes every 60 ms but is read every tick, so a
derivative only spikes at each refresh.

Two checks keep a wrong reading from steering the robot: a frozen sensor is dropped in
[perception](perception.md), and with both walls in view `|left + right - 2 * centre|` above
`WALL_PAIR_TOL_MM` means one is wrong and there is no telling which, so there is no centring and the
gyro holds the heading.

A sensor with no echo is neither wall nor open: its distance is stale and never used. A caller that
has to decide treats it as a wall.

## Invariants

- Each sensor refreshes every `US_SENSOR_COUNT * US_SLOT_MS` (60 ms), so a reading is up to that old.
- With no side wall in view the trim is the learned bias, fading to 0.
- The output is within `steerMaxDeg`.

## Status

Measured on the robot (2026-10-14): thresholds and centre distance in `RobotSpec.h`; `KP_WALL` held
the robot within about +-10 mm of the centre. `KI_WALL`, the bias limits, the pair tolerance and the
frozen time come from a host model of the corridor and the sensors, not from the robot.
