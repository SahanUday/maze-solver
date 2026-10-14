# motion_primitives

The two basic moves: a straight run along a held heading, and an in-place turn. Pure logic
in `include/MotionPrimitives.h`, tested by `test/test_motion_primitives/` against the
simulated plant in `sim.h`. Cell stepping and line following are built on them in later modules.

## Interface

A primitive is a small per-tick state machine: `...Begin()`, then `...Step()` once a tick;
it writes the `MotionTarget` and returns a `MoveStatus` (`Running`, `Done`, `Timeout`). It
never touches the motors.

- `Straight` - `straightBegin(s, fb, distMm, vMax, vEnd, headingRef)`,
  `straightExtend(s, addMm, vEnd)` (carry on through the centre), `straightRetarget(s,
  targetMm, vEnd)`, `straightStep(s, plan, fb, trimDeg, dtMs, out)`. `trimDeg` is added to
  the held heading: wall centring steers through it.
- `Turn` - `turnBegin(t, fb, targetHeadingDeg, deltaDeg)`, `turnStep(...)`. Both are given
  because a 180 is ambiguous from the headings alone.
- `PlanParams` - accelerations, rates, tolerances, settle time, move timeout. The values come from
  `RobotSpec.h` (`ACCEL_MM_S2`, `TURN_RATE_MAX_DPS`, `TURN_ACCEL_DPS2`, `STEP_TOLERANCE_MM`,
  `TURN_TOLERANCE_DEG`, `TURN_SETTLE_MS`, `MOVE_TIMEOUT_MS`).

## How it works

Both profile a **reference** (`refMm` / `refAngleDeg`) with [motion_profile](motion_profile.md)
and hand `MotionControl` the reference, not the measurement. A `Straight` begun while the
robot is rolling starts at the measured speed, so there is no jerk. It is `Done` when the
profile has stopped and the robot is within `stepToleranceMm` of the target (stop moves), or
when the reference passes the target (pass-through moves, `vEnd > 0`).

A `Turn` aims at the **absolute grid heading**, not at "start + 90": the sweep is only as
exact as the start heading was, so once the profile has stopped within a degree the target
switches to the grid heading and gyro error does not accumulate from turn to turn. It is `Done`
after staying within `turnToleranceDeg` for `turnSettleMs`. It holds the odometer position
while pivoting.

## Invariants

- `moveTimeoutMs == 0` means no limit; otherwise the primitive returns `Timeout`. The clock is per
  leg: `straightExtend()` restarts it, so a chain of cells is not one long move.
- Only a turn sets `MotionTarget::pivot`, which tells [motion_control](motion_control.md) to use the
  turn-sized heading integral.
- A turn goes the way `deltaDeg` says (a 180 is not "whichever is shorter").

## Status

18 host tests, 100% line coverage, closed loop against the simulated plant. The simulator has
first-order wheel lag but no slip, no deadband and no sensor noise.
