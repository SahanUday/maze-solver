# motion_control

Turns a `MotionTarget` (where the robot should be, how fast, and which way it should face) and a
`MotionFeedback` (where it is) into a left and right duty. Pure logic in `include/MotionControl.h`,
with the data types in `MotionTypes.h`, tested by `test/test_motion_control/`. It touches no
hardware: `DriveCommand.h` is the struct a caller hands to the motors.

## Interface

- `motionControlStep(mc, params, target, feedback, dtMs)` returns a `MotionOutput {left, right}` in
  signed duty counts. Call it once per tick. State lives in `MotionControl`.
- `motionControlClearIntegrals(mc)` forgets the two integrals (not the stall latch). Call it when a
  move starts.
- `motionControlReset(mc)` clears everything, including the stall latch.
- `MotionControlParams` carries the gains, feedforward, minimum PWM, and stall settings. They come
  from `include/RobotSpec.h`; the host tests use values of their own.

## How it works

Two channels, mixed: `left = linear - angular`, `right = (linear + angular) * ffScaleR`. A counter-clockwise
turn therefore speeds the right wheel.

- **Linear:** a PID on the odometer error plus a speed feedforward (`ffPwmPerMmS`). The D term acts on the
  speed error.
- **Angular:** a PID on the heading error plus a yaw-rate feedforward (`ffPwmPerDps`). The heading comes from
  the **gyro**, not the encoders, so a turn does not depend on the measured track width. The D term acts on
  the yaw-rate error.

Two details came from running it on the robot:

- **Two heading-integral gains.** While the target says `pivot` (a turn in place) the integral gain is
  `kiTurn`, and otherwise `kiHold`. The wheels stick below a duty of about 160, so at the end of a turn the
  integral is what gets them moving, and it needs to be large (60). The same gain on a straight rings
  and shows up as a zig-zag, so straights use a much smaller one (8). A negative `kiHold` means "use
  `kiTurn`". Both integrals are clamped to `MOTION_INTEGRAL_MAX_DUTY` of duty, and they must be cleared when
  a move starts (`motionControlClearIntegrals`), otherwise the wind-up from a turn carries into the next
  straight.
- **Right-wheel scale.** On the floor the right wheel needs about 20% more duty than the left for the same
  speed (the two matched on a stand), so `ffScaleR` multiplies the whole right command. The integral then has
  nothing constant left to supply.

`toDuty()` adds `minPwm` as an offset to every non-zero command, so a lift of 160 or 250 makes the output
chatter around a target. `RobotSpec.h` keeps the start-from-rest values (`MOTOR_MIN_PWM_*`) as measurements
and gives the controller its own `CONTROL_MIN_PWM_*`, currently 0.

## Stall latch

When the larger wheel duty is at least `stallDuty` while neither speed nor yaw rate exceeds the minimums for
`stallTimeMs`, `MotionControl::stalled` latches and the output is zero until `motionControlReset()`.
Either of `stallDuty` and `stallTimeMs` at 0 turns it off.

## Invariants

- Output is limited to the timer's `pwmTop`; a caller may clamp lower.
- An integral term can contribute at most `MOTION_INTEGRAL_MAX_DUTY`, so a long stall cannot wind it up.
- With all gains 0 the output is 0.

## Status

Host tests cover the feedforward, each PID term, the mixing, the clamps, the integral gains and the stall
latch. On the robot (tools/probe motion tests, 2026-10-14, wall steering off): 240 to 500 mm straights arrive
within 2 mm of the target with the heading held within about 1 degree; 90 and 180 degree turns, both ways,
arrive within 2 degrees. The stall latch has not been exercised by a real stall.
