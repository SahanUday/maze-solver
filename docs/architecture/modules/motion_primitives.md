# motion_primitives

Two basic moves: a straight run along a held heading, and an in-place turn. Pure logic in
`include/MotionPrimitives.h`, tested by `test/test_motion_primitives/` against the simulated
plant in `sim.h`.

## Interface

A primitive is begun, then stepped once per tick; the step writes the `MotionTarget` and returns
a `MoveStatus` (`Running`, `Done`, `Timeout`). It never touches the motors.

- `Straight` - `straightBegin(s, fb, distMm, vMax, vEnd, headingRef)`,
  `straightExtend(s, addMm, vEnd)`, `straightRetarget(s, targetMm, vEnd)`,
  `straightStep(s, plan, fb, trimDeg, dtMs, out)`. `trimDeg` is added to the held heading.
- `Turn` - `turnBegin(t, fb, targetHeadingDeg, deltaDeg)`, `turnStep(...)`. A 180 is ambiguous
  from the headings alone, so both are given.
- `PlanParams` - accelerations, rates, tolerances, settle time, move timeout. From `RobotSpec.h`:
  `ACCEL_MM_S2`, `TURN_RATE_MAX_DPS`, `TURN_ACCEL_DPS2`, `STEP_TOLERANCE_MM`,
  `TURN_TOLERANCE_DEG`, `TURN_SETTLE_MS`, `MOVE_TIMEOUT_MS`.

## How it works

Both profile a reference (`refMm`, `refAngleDeg`) with [motion_profile](motion_profile.md) and
hand [motion_control](motion_control.md) the reference, not the measurement.

- **Straight:** begins at the measured speed. `Done` when the profile has stopped and the robot
  is within `stepToleranceMm` (`vEnd == 0`), or when the reference passes the target
  (`vEnd > 0`).
- **Turn:** once the profile has stopped within a degree of its sweep, aims at the absolute grid
  heading. `Done` after staying within `turnToleranceDeg` for `turnSettleMs`. Holds the odometer
  position while pivoting.

## Invariants

- `moveTimeoutMs` is per leg, 0 = no limit. `straightExtend()` restarts it.
- Only a turn sets `MotionTarget::pivot`.
- A turn goes the way `deltaDeg` says.

## Status

Host tests, closed loop against a plant with first-order wheel lag and no slip, deadband or
sensor noise.
