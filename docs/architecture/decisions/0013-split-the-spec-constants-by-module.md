# 0013: Split the spec constants by module

- **Status:** Accepted
- **Date:** 2026-10-17

## Context

`include/RobotSpec.h` held 154 constants in 441 lines: competition limits, wheel
geometry, IMU calibration, the ultrasonic offsets, every PID gain and the Section A
mission tuning. Almost every header included all of it, so nothing showed which module
used which constant, and every PR that tuned one value edited the same file. The
calibration branch merges `main` in, so those PRs collided there too. A few constants
sat in the wrong section (`RAMP_INCLINE_COUNTS` is derived from the IMU's 1 g figure
but was filed under line following).

## Decision

Replace `RobotSpec.h` with one header per module in `include/spec/`, included by path
(`#include "spec/Motion.h"`):

| Header | Holds |
|---|---|
| `Competition.h` | arena geometry, time limits, battery and SRAM budget |
| `Platform.h` | `CPU_HZ`, control-loop period, debug baud |
| `Drivetrain.h` | motor PWM, polarity, wheel and encoder geometry, minimum duty |
| `Imu.h` | MPU-6050 settings, gyro bias capture and polarity, accel axis and level, incline, `RAMP_INCLINE_COUNTS` |
| `LineSensor.h` | IR array, ADC settings, line calibration limits, mounting |
| `FloorPattern.h` | approach marker and gap, the air threshold |
| `Ultrasonic.h` | HC-SR04 timing, range, per-sensor offsets |
| `Motion.h` | distance and turn PID, speed limits, feedforward, tolerances, stuck and stall detection |
| `WallFollow.h` | wall thresholds, centring gains, heading trim, front-echo filtering |
| `LineFollow.h` | line-following gains, speed, search, slowdown |
| `Mission.h` | Section A, bridge hand-over, run start, status line |

Constant names, values and comments are unchanged. A file includes only the spec
headers whose constants it uses, and a spec header includes another only for a
`static_assert` or a derived value (`Drivetrain` and `Platform`; `WallFollow` and
`Motion`). There is no umbrella header.

## Consequences

- An include list says which tunables a module depends on.
- A tuning PR touches the one file for its module.
- A new constant needs a home: pick the module that owns its meaning, not the one
  that first uses it.
- Earlier ADRs name `RobotSpec.h`; they are left as written.

## Alternatives considered

- **Keep one file.** No churn, but the problems above stay.
- **Put each module's constants next to its logic header** (`motion/MotionSpec.h`).
  Self-contained, but tuning the robot would mean opening five folders.
- **Split by kind (hardware facts / calibration / gains) instead of by module.** A
  module's gains would then be spread over three files.
