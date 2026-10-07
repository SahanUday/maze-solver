# heading

Gyro bias estimation and yaw integration: the arithmetic that turns the raw
counts from [imu](imu.md) into `RobotState::headingDeg`. Pure logic in
`include/Heading.h`: no hardware, no `Arduino.h`, tested on the PC by
`test/test_heading/`. Decision:
[0007](../decisions/0007-mpu6050-heading-and-tilt.md), which places it above the
HAL for the reason [0006](../decisions/0006-line-calibration-above-the-hal.md)
placed line calibration there.

## Interface

- `GyroBias` - a running `sum` and `samples`, the finished estimate in `counts`,
  and `ready`.
- `gyroBiasReset(bias)`, `gyroBiasAccumulate(bias, yawRateCounts)` - reset, then
  call once per sample with the robot held still.
- `gyroBiasFinish(bias)` - sets `counts` to the mean and `ready`; returns
  `false` if fewer than `GYRO_BIAS_MIN_SAMPLES` got through.
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

Two deliberate refusals: `headingUpdate()` integrates nothing at all while the
bias is not `ready`, because integrating an unknown offset is the exact failure
this module exists to prevent; and `gyroBiasAccumulate()` saturates at
`GYRO_BIAS_SAMPLES` so an over-running caller cannot overflow the sum.

## Invariants

- `headingUpdate()` is a no-op until `gyroBiasFinish()` has returned `true`.
- A bias holds only for the chip, its temperature and the mounting it was taken
  on. It is captured at run time and never written into `RobotSpec.h`.
- `deg` stays in [-180, 180), so it never grows without bound; +180 wraps to
  -180.
- The sum cannot overflow `int32_t`: `GYRO_BIAS_SAMPLES` (1000) times the full
  +-32767 range is well inside it.
- One `GyroBias` is 12 B and one `Heading` is 4 B.

## Status

16 host tests, `include/Heading.h` at 100% line coverage in the gcovr report, and
the whole suite passes under `env:native`, `env:native_san` (ASan + UBSan) and
`env:native_cov`. On the Mega the header compiles under `-Werror
-Wstack-usage=128`.

The fixtures are **synthetic**. No MPU-6050 has been read, so the tests cover the
arithmetic and its guards - that a still robot holds its heading whatever the
bias, that 90 deg/s for one second integrates to 90 deg, that an uncorrected
bias is what would drift, that nothing integrates before the bias is ready, that
the angle wraps - and not any measured sensor behaviour.

Nothing calls `gyroYawRateDps()`, `headingSet()` or `headingWrapDeg()` from the
firmware yet; `main.cpp` calls only the bias functions and `headingUpdate()`.

Still to check: the real bias magnitude and its warm-up drift, `GYRO_YAW_INVERT`,
how much heading is actually lost over `RUN_LIMIT_MS`, and the per-tick cost of
the float arithmetic on the Mega. The two mechanisms meant to bound the residual
drift - re-zeroing the bias when the robot is known stationary, and snapping to
the maze's 90 deg grid against wall readings - are **not built**; neither has a
trigger yet.
