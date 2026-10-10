# safe_run

A run for [run_layer](run_layer.md), for Section A (4x4, no ramp, bridge or line): drive between
the walls at low duty, turn at a front wall, and end on anything unexpected. Pure logic in
`include/SafeRun.h`, limits in `include/SafeRunSpec.h`: no hardware, no `Arduino.h`, tested on the
PC by `test/test_safe_run/` (through the layer, as it runs). Start, countdown, self-test, e-stop,
the cap and the ramp are the layer's; this module is the behaviour.

It drives open loop on duty, because the motion PID gains are still `<<TBD CALIBRATION>>`. It is
not the maze solver.

## Phases

The run's phases, after the layer's self-test: `Settle` -> `Cruise` <-> `Settle` -> `Turn`.

- **Settle** (`SAFE_SETTLE_MS`): both wheels at zero between phases, so a wheel never reverses at
  speed.
- **Cruise**: both wheels at `SAFE_CRUISE_DUTY`, steered by the heading hold (below), or by
  left-minus-right distance (`SAFE_STEER_COUNTS_PER_MM`, capped at `SAFE_STEER_MAX`). Steering
  only applies when both sides are heard and closer than `SAFE_SIDE_OPEN_MM`; otherwise it drives
  straight.
- **Turn**: pivot in place until the gyro has turned the target angle less
  `SAFE_TURN_EARLY_DEG`. The test is on the absolute angle, so a wrong `GYRO_YAW_INVERT` does not
  matter; a gyro that does not move times out.

Its limits to the layer: `SAFE_MAX_DUTY`, `SAFE_RAMP_PER_TICK`, `SAFE_RUN_LIMIT_MS`, a self-test
at `SAFE_CRUISE_DUTY`, and `SAFE_FRONT_STOP_MM`. It will not start without a front echo, since it
would have no way to stop at a wall.

## Heading hold

With a working gyro, a straight stretch steers on heading, not on wall distance alone.

- **Inner loop**: a PI on `target - heading`, in duty (`SAFE_HEADING_KP`, `SAFE_HEADING_KI`,
  capped by `SAFE_HEADING_I_MAX` and `SAFE_STEER_TOTAL_MAX`). On the floor the right wheel turns
  fewer counts than the left at the same duty ([motors](motors.md)); the integral term absorbs
  that, and stays across corners because the mismatch does.
- **Outer loop**: when both side walls are close, left-minus-right distance tilts the target away
  from the nearer wall (`SAFE_WALL_TILT_*`), and a slow integral (`SAFE_WALL_LEARN_*`) learns the
  corridor's direction, so a robot placed a few degrees off straightens out. Nothing is
  tilted or learned when a side is open or silent.
- **Target**: the heading the first stretch was placed with. After a turn it moves by the ideal
  angle (90 or 180), not the measured one, so an overshoot is corrected rather than followed.
- Without a gyro, straight stretches steer on wall distance alone (`SAFE_STEER_MAX`) and the run
  stops at the first front wall.

Every move into Settle (self-test end, front wall, turn end) is a hard stop to zero, not a ramp:
the wheels coast on from a pivot.

A pivot that is not `SAFE_TURN_PROGRESS_DEG` round after `SAFE_TURN_PROGRESS_MS` is jammed
against a wall, and ends in a `TurnTimeout` fault instead of spinning the wheels. The robot
pivots about its axle with only ~3 cm to each side wall, so it must stop close to the front wall
(`SAFE_FRONT_STOP_MM`) for the tail to clear.

## Turn choice

Front distance at or below `SAFE_FRONT_STOP_MM`: turn left 90 degrees if the left
is open, else right 90 if the right is open, else 180 if both are walls. A side
that is silent is unknown, not open, so with a silent side and no open one the
run stops (`NoClearSide`). Without a working gyro the run stops at the first front
wall (`FrontWall`).

## Stops and faults

The layer's own reasons (`EStop`, `TimeLimit`, `SelfTestReversed`, `SelfTestNoMotion`) are in
[run_layer](run_layer.md). This run ends with:

| Reason | Kind |
|---|---|
| `FrontWall`, `NoClearSide`, `TooManyTurns` | `Stopped` |
| `Stall`, `FrontBlind`, `TurnTimeout` | `Fault` |

`Stall`: an encoder moved less than `SAFE_STALL_MIN_COUNTS` in a `SAFE_STALL_WINDOW_MS` window
while Cruise or Turn was driving. `FrontBlind`: no front echo for `SAFE_FRONT_BLIND_MS` while
cruising.

## Status

Tested on the PC (including a simulated weak right wheel) and run on the robot's first chassis.
On that chassis a 90 degree pivot in the corridor jams the tail against a side wall: a run that
stopped 87 mm from the front wall jammed, and one at 59 mm turned. `SAFE_FRONT_STOP_MM` is tuned
to that outline and needs redoing on the new chassis.

Duties come from bench pulses: left starts at ~160 and right at ~250 on the floor, and right
reverse needs ~300, so `SAFE_TURN_DUTY` is 320. The ultrasonic offsets are uncalibrated, so
`SAFE_FRONT_STOP_MM` and `SAFE_SIDE_OPEN_MM` are face-to-wall distances that may be a few mm off.
There is no watchdog and no battery check, so a hung loop leaves the motors at their last command.
