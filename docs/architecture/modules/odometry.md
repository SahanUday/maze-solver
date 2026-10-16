# odometry

Encoder counts to distance and speed. Pure logic in `include/sense/Odometry.h`, tested by
`test/test_odometry/`. The counts come from [encoders](encoders.md).

## Interface

- `Odometry` - `distMm` (cumulative, signed, mean of both wheels), `speedMmS` (low-pass
  filtered), and the last counts.
- `odometryReset(odo)`, `odometryUpdate(odo, countL, countR, mmPerCount, dtMs)` - fold in
  one tick. The first update only records the counts.

## How it works

Distance advances by the mean of the two wheel steps times `mmPerCount`, so a pivot in
place adds nothing. `distMm` is the integer sum of counts times `mmPerCount`, so it does not
drift over a run; `mmPerCount` must stay constant. Speed is the last step over `dtMs`, filtered
with `ODOM_SPEED_FILTER`.

## Invariants

- `mmPerCount == 0` leaves distance and speed at 0.
- `dtMs == 0` updates distance and leaves speed alone.

## Status

Host tests use synthetic counts. On the robot (floor, 2026-10-14) the odometer matched a tape
measure to under 1%.
