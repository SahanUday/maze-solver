# motion_profile

Trapezoid speed profile for one axis, distance (mm, mm/s) or angle (deg, deg/s). Pure logic in
`include/motion/MotionProfile.h`, tested by `test/test_motion_profile/`.

## Interface

- `profileStep(v, remaining, vMax, accel, vEnd, dtS)` - the next signed speed: accelerate at
  `accel` up to `vMax`, brake as late as possible to arrive at `vEnd` (0 = stop, `vMax` = pass
  through). The sign of `remaining` is the direction.

## How it works

The speed target is the lower of `vMax` and `sqrt(vEnd^2 + 2 accel remaining)`, approached by
`accel * dt` per tick. Once one braking step is enough and the speed is within one step of
zero, the stop is finished at 0.

It is stepped on the reference trajectory ([motion_primitives](motion_primitives.md)), not the
measured robot.

## Invariants

- `accel <= 0` produces no motion.
- A negative `vMax` or `vEnd` is floored to 0; `vEnd` above `vMax` is clamped to it.
- A move asked for closer than it can brake overshoots, then reverses.

## Status

Host tests only.
