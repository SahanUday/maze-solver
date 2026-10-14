# odometry

Encoder counts to distance travelled and speed. Pure logic in `include/Odometry.h`:
no hardware, no `Arduino.h`, tested on the PC by `test/test_odometry/`. The counts
come from [encoders](encoders.md) through `RobotState`.

## Interface

- `Odometry` - `distMm` (cumulative, signed, mean of both wheels) and `speedMmS`
  (low-pass filtered), plus the last counts.
- `odometryReset(odo)`, `odometryUpdate(odo, countL, countR, mmPerCount, dtMs)` -
  fold in one tick. The first update only records the counts, so a non-zero count
  at power-up is not a jump.

## How it works

Distance advances by the mean of the two wheel steps times `mmPerCount`, so a pivot
in place (equal and opposite steps) adds nothing. A move records the value it started
at and works in differences, which keeps the scale (a `float`, 24-bit mantissa) from
mattering over a run. Speed is that step over `dtMs`, filtered with
`ODOM_SPEED_FILTER` because counts arrive in whole steps and one tick's raw speed is
quantised.

## Invariants

- `mmPerCount == 0` (not yet calibrated) leaves distance and speed at 0 rather than
  dividing by anything.
- `dtMs == 0` updates distance and leaves speed alone.
- One `Odometry` is 20 B.

## Status

8 host tests, 100% line coverage. `MM_PER_ENCODER_COUNT` is still `0` in `RobotSpec.h`
(it comes from the calibration branch), so on the robot every distance is 0 until it
lands. Nothing here has run on a real encoder; the fixtures are synthetic counts.
