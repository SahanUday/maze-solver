# heading

Gyro bias estimation and yaw integration: the arithmetic that turns the raw
counts from [imu](imu.md) into `RobotState::headingDeg`. Pure logic in
`include/motion/Heading.h`: no hardware, no `Arduino.h`, tested on the PC by
`test/test_heading/`. Decision:
[0008](../decisions/0008-mpu6050-heading-and-tilt.md), which places it above the
HAL for the reason [0006](../decisions/0006-line-calibration-above-the-hal.md)
placed line calibration there.

## Interface

- `GyroBias` - a running `sum` and `samples`, a running `mean` and `m2` for the
  spread, the finished estimate in `counts`, `spread` and `ready`.
- `gyroBiasReset(bias)`, `gyroBiasAccumulate(bias, yawRateCounts)` - reset, then
  call once per sample with the robot held still.
- `gyroBiasFinish(bias)` - sets `counts` to the mean, `spread` to the standard
  deviation of the samples, and `ready`; returns `false` if fewer than
  `GYRO_BIAS_MIN_SAMPLES` got through, or if the samples scatter more than
  `GYRO_BIAS_MAX_SPREAD_COUNTS`.
- `Heading` - the integrated angle, `deg`, in [-180, 180).
- `headingUpdate(heading, bias, yawRateCounts, dtMs)` - integrate one tick. Does
  nothing while `!bias.ready`.
- `gyroYawRateDps(bias, yawRateCounts)` - the corrected rate on its own, for
  telemetry and for a turn controller that wants rate rather than angle.
- `headingSet(heading, deg)` / `headingWrapDeg(deg)` - override the integral,
  and fold an angle back into range.

## How it works

A gyro measures a rate, so the angle is its integral - and the integral of a
constant offset grows without bound. `gyroBiasAccumulate()` sums raw counts
while the robot is still and `gyroBiasFinish()` divides once, in float, so the
estimate keeps the sub-count precision that an integer mean would throw away.
`headingUpdate()` then subtracts that bias, divides by `GYRO_COUNTS_PER_DPS`,
applies `GYRO_YAW_INVERT` (an `if constexpr`, so the unused branch is not
compiled), multiplies by `dtMs`, and wraps.

Three deliberate refusals: `headingUpdate()` integrates nothing at all while the
bias is not `ready`, because integrating an unknown offset is the exact failure
this module exists to prevent; `gyroBiasAccumulate()` saturates at
`GYRO_BIAS_SAMPLES` so an over-running caller cannot overflow the sum; and
`gyroBiasFinish()` refuses a capture whose samples scatter more than
`GYRO_BIAS_MAX_SPREAD_COUNTS`.

That last one is the *stillness gate*. A robot that is being moved during the
capture does not read zero, so its mean is a wrong zero that is subtracted for
the rest of the run, and the heading drifts at `(captured - true) / GYRO_COUNTS_PER_DPS`
for as long as it runs. The spread is a Welford running mean and sum of squared
deviations, so it stays accurate with counts up to the full +-32767 range.
`src/main.cpp` repeats a rejected capture up to `GYRO_BIAS_MAX_ATTEMPTS` times
(about 1.4 s each) and, if none is still enough, reports the failure and leaves
the heading at 0, which is the same fallback as a capture with too few samples.

The gate sees scatter, not a steady offset: a rotation at a perfectly constant
rate reads as a still gyro with a large zero, and passes. A person turning the
robot by hand scatters far too much for that to matter in practice, but nothing
in the readings alone can tell the two apart.

## Invariants

- `headingUpdate()` is a no-op until `gyroBiasFinish()` has returned `true`.
- A bias holds only for the chip, its temperature and the mounting it was taken
  on. It is captured at run time and never written into `spec/Imu.h`.
- `deg` stays in [-180, 180), so it never grows without bound; +180 wraps to
  -180.
- The sum cannot overflow `int32_t`: `GYRO_BIAS_SAMPLES` (1000) times the full
  +-32767 range is well inside it.
- One `GyroBias` is 28 B and one `Heading` is 4 B.

## Status

22 host tests and the whole suite passes under `env:native`, `env:native_san`
(ASan + UBSan) and `env:native_cov`. On the Mega the header compiles under
`-Werror -Wstack-usage=128`; adding the spread took 12 B of SRAM and about 900 B
of flash.

The fixtures are **synthetic**. No MPU-6050 has been read, so the tests cover the
arithmetic and its guards - that a still robot holds its heading whatever the
bias, that 90 deg/s for one second integrates to 90 deg, that an uncorrected
bias is what would drift, that nothing integrates before the bias is ready, that
the angle wraps - and not any measured sensor behaviour.

Measured on the compact chassis, 2026-10-13 and 14. Every still capture and still
stretch of a log scattered 1.9 to 2.9 counts, and a still capture was accepted on
the first attempt. Captured while the
robot was turned by hand at about 24 deg/s, the zero came out at 1599 counts
instead of about 15, and the heading ended wrong by exactly `(captured - true)`
times the elapsed time. With the gate, the same handling gave five rejections
(spread 725, 532, 627, 819 and 28 counts) and then an accepted capture of 15.09
counts once the robot was set down, after which the heading drifted 0.02 deg/min.
The threshold sits between about 3 (still) and about 330 (turning); a robot being
set down scattered 6.5 on the attempt that was accepted, so it is not generous
at the low end.

Nothing calls `gyroYawRateDps()`, `headingSet()` or `headingWrapDeg()` from the
firmware yet; `main.cpp` calls only the bias functions and `headingUpdate()`.

Measured at rest on the compact chassis (2026-10-14, motors off): the zero is about 15
counts (0.23 deg/s) and the heading wandered +2.4 deg in 10 min with the chip warm and
+4.8 deg just after power-on, from a bias that moved 0.5 to 0.8 counts (0.008 to
0.012 deg/s). A fixed warm-up wait would not have helped, and capturing the zero again at
START made the 8-minute error smaller in 2 of 4 cases and larger in the other 2, so
neither is built. The gyro scale reproduces closed hand laps to 1.001 to 1.002
(+-0.002). Tables:
[docs/calibration/gyro-and-accelerometer-compact-chassis.md](../../calibration/gyro-and-accelerometer-compact-chassis.md).

Still to check: how much heading is actually lost over `RUN_LIMIT_MS` with the motors
running (these were all bench runs), and the per-tick cost of the float arithmetic on the
Mega. Snapping to the maze's 90 deg grid against wall readings - the mechanism meant to
bound the residual drift - is **not built**; it has no trigger yet.
