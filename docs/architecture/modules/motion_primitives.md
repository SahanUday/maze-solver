# motion_primitives

Two basic moves: a straight run along a held heading, and an in-place turn. Pure logic in
`include/MotionPrimitives.h`, tested by `test/test_motion_primitives/` against the simulated
plant in `sim.h`.

## Interface

A primitive is begun, then stepped once per tick; the step writes the `MotionTarget` and returns
a `MoveStatus` (`Running`, `Done`, `Stuck`). It never touches the motors.

- `Straight` - `straightBegin(s, fb, distMm, vMax, vEnd, headingRef)`,
  `straightExtend(s, addMm, vEnd)`, `straightRetarget(s, targetMm, vEnd)`,
  `straightStep(s, plan, fb, trimDeg, dtMs, out)`. `trimDeg` is added to the held heading.
- `Turn` - `turnBegin(t, fb, targetHeadingDeg, deltaDeg)`, `turnStep(...)`. A 180 is ambiguous
  from the headings alone, so both are given.
- `PlanParams` - accelerations, rates, tolerances, settle time, stuck detection. From
  `RobotSpec.h`: `ACCEL_MM_S2`, `TURN_RATE_MAX_DPS`, `TURN_ACCEL_DPS2`, `STEP_TOLERANCE_MM`,
  `TURN_TOLERANCE_DEG`, `TURN_SETTLE_MS`, `PROGRESS_MIN_MM`, `PROGRESS_MIN_DEG`, `PATIENCE_MS`.

## How it works

Both profile a reference (`refMm`, `refAngleDeg`) with [motion_profile](motion_profile.md) and
hand [motion_control](motion_control.md) the reference, not the measurement.

- **Straight:** begins at the measured speed. `Done` when the profile has stopped and the robot
  is within `stepToleranceMm` (`vEnd == 0`), or when the reference passes the target in the
  direction of travel (`vEnd > 0`). Distances are signed, so a move can run in reverse.
- **Turn:** once the profile has stopped within a degree of its sweep, aims at the absolute grid
  heading. `Done` after staying within `turnToleranceDeg` for `turnSettleMs`. Holds the odometer
  position while pivoting.
- **Stuck:** there is no time limit per move. A move is `Stuck` when its error sets no new low
  (by `progressMin`) for `patienceMs`. An error that climbs more than 4 steps above its low
  (pushed back, overshoot) restarts the low, so recovering from a shove is not stuck. Within
  one `progressMin` of the tolerance the approach is slow by nature and is never stuck.

## Invariants

- A move's length and speed cannot make it `Stuck`; a slow robot that keeps closing the gap runs
  on. The run limit (`RUN_LIMIT_MS`) is the only overall bound.
- `patienceMs == 0` turns the detector off.
- Only a turn sets `MotionTarget::pivot`.
- The first step after a begin sets `MotionTarget::newMove`; `straightExtend` and
  `straightRetarget` do not.
- A turn goes the way `deltaDeg` says, within +-180.

## Status

Host tests, closed loop against a plant with first-order wheel lag and no slip, deadband or
sensor noise.
