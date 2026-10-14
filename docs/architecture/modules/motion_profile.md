# motion_profile

Trapezoid speed profile for one axis. Pure logic in `include/MotionProfile.h`, tested
by `test/test_motion_profile/`. The same function profiles distance (mm, mm/s) and
angle (deg, deg/s).

## Interface

- `profileStep(v, remaining, vMax, accel, vEnd, dtS)` - the next signed speed:
  accelerate at `accel` up to `vMax`, and start braking as late as possible while
  still arriving at `vEnd` (0 = stop, `vMax` = pass through without slowing).

## How it works

The fastest speed from which `accel` can still reach `vEnd` within `remaining` is
`sqrt(vEnd^2 + 2 accel remaining)`; the profile aims at the lower of that and `vMax`,
moving by `accel * dt` per tick. The sign of `remaining` is the direction. It is
stepped on the **reference** trajectory (see [motion_primitives](motion_primitives.md)),
not the measured robot, so a slipping wheel cannot stall it.

The braking envelope shrinks with the root of the distance left, so it would creep
toward the target for ever. Once one tick of braking is enough, and the speed itself is
within one step of zero, the stop is finished. (The speed condition matters: without it a
deliberate decelerate-to-zero from cruise would stop instantly.)

## Invariants

- `accel <= 0` produces no motion; a caller that wants to know uses the move timeout.
- A negative `vMax` or `vEnd` is floored to 0; `vEnd` above `vMax` is clamped to it.
- A move asked for closer than it can brake overshoots and the position loop backs it
  off; it never asks the profile for a reversal.

## Status

12 host tests (stopping without overshoot, short moves never reaching the cap,
pass-through, reverse, the stop-from-speed ramp), 100% line coverage. Two bugs were found
and fixed by closed-loop tests rather than by reading: the endless creep, and the
snap-to-zero firing from cruise.
