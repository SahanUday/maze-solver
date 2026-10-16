# motion_control

Turns a `MotionTarget` and a `MotionFeedback` into left and right duty. Pure logic in
`include/motion/MotionControl.h` and `MotionTypes.h`, tested by `test/test_motion_control/`.
`DriveCommand.h` is the struct a caller hands to the motors.

## Interface

- `motionControlStep(mc, params, target, feedback, dtMs)` - signed duty counts
  `MotionOutput {left, right}`. Call once per tick.
- `motionControlClearIntegrals(mc)` - forget both integrals, keep the stall latch. Called by
  `motionControlStep` on `target.newMove`.
- `motionControlReset(mc)` - clear everything, including the stall latch.
- `MotionControlParams` - gains, feedforward, minimum PWM, stall settings. From `RobotSpec.h`.

## How it works

`left = linear - angular`, `right = (linear + angular) * ffScaleR`. A CCW turn speeds the right
wheel.

- **Linear:** PID on the odometer error + speed feedforward (`ffPwmPerMmS`); D on the speed error.
- **Friction feedforward:** `ffPwmFriction` is added along the speed reference while its magnitude
  is above 1 mm/s, and not at a standstill. The wheels need 160 / 250 duty to start but keep
  rolling on ~110, so without it a move ends short, stands still, and the distance integral takes
  seconds to break the wheels loose. `RobotSpec.h` `FEEDFORWARD_FRICTION_DUTY` is a host-model
  value, not measured.
- **Angular:** PID on the gyro heading error + yaw-rate feedforward (`ffPwmPerDps`); D on the
  yaw-rate error. The gyro closes this loop, so a turn does not depend on the track width.
- **Heading integral:** `kiTurn` while the target is `pivot`, else `kiHold` (negative = `kiTurn`).
  Each integral is clamped to `MOTION_INTEGRAL_MAX_DUTY` of duty.
- **Pivot integral:** an integral that points against the error is dropped while pivoting. It only
  breaks the wheels loose; once the error has flipped the wheels are moving and it would carry the
  turn past the target and hunt. A straight keeps its heading integral through an overshoot, as it
  holds a steady disturbance.
- **New move:** `target.newMove` (the first tick of a begun move) clears both integrals before
  the step. A chain of cells (`straightExtend`) does not set it.
- **Right scale:** `ffScaleR` multiplies the whole right command.
- **Minimum PWM:** non-zero commands are rescaled onto `[minPwm, pwmTop]`. `RobotSpec.h` sets
  `CONTROL_MIN_PWM_*` to 0.

## Stall latch

`stalled` latches, and the output is zero until `motionControlReset()`, when the larger wheel
duty is at least `stallDuty` while speed and yaw rate stay under their minimums for
`stallTimeMs`. Either of `stallDuty` and `stallTimeMs` at 0 turns it off.

## Invariants

- Output is limited to `pwmTop`.
- An integral term contributes at most `MOTION_INTEGRAL_MAX_DUTY`.
- With all gains 0 the output is 0.

## Status

Host tests cover the feedforward, each PID term, the mixing, the clamps, the integral gains and
the stall latch. On the robot (floor, 2026-10-14): 240 to 500 mm straights arrive within 2 mm
with the heading held within about 1 degree; 90 and 180 degree turns, both ways, arrive within
2 degrees. The stall latch has not been exercised by a real stall.
