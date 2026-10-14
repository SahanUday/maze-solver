# perception

`RobotState` in, one derived `Perception` out per tick. Pure logic in `include/Perception.h` (with
`Tilt.h`, `WallSense.h` and `LineView.h`), tested by `test/test_perception/`, `test/test_tilt/`,
`test/test_wall_sense/` and `test/test_line_view/`.

## Interface

- `Perceiver` - the state `perceive()` carries between ticks: odometry, last heading, filtered yaw
  rate, level reading, line calibration, mask.
- `perceiverReset(pc)`, `perceiverSetCalibration(pc, cal)` - the calibration comes from the EEPROM
  store at boot; this also works out which channels it accepts.
- `perceiverSetLevel(pc, counts)` - the forward-axis reading on level ground, from `InclineLevel`.
- `perceive(pc, state, params, out)` - fills `Perception`: `motion` (distance, speed, heading, yaw
  rate), `inclineCounts`, `walls` ([wall_sense](wall_sense.md)), `line` ([line_view](line_view.md)),
  `startPressed`.

## Incline

`inclineCounts(imu, levelCounts)` (`Tilt.h`) is the nose-up slope in accelerometer counts: the
forward axis (`IMU_ACCEL_AXIS_FORWARD`, sign from `IMU_ACCEL_NOSE_UP_NEGATIVE`) minus the level
reading. Gravity on that axis is `IMU_ACCEL_ONE_G_COUNTS * sin(incline)`, so a consumer compares
counts with a threshold and no trigonometry is needed
([0008](../decisions/0008-mpu6050-heading-and-tilt.md)).

- `InclineLevel` averages the forward-axis reading at boot on the start surface. A capture with
  fewer than `INCLINE_LEVEL_MIN_SAMPLES`, or spread wider than `INCLINE_LEVEL_MAX_RANGE_COUNTS`, is
  refused and `IMU_ACCEL_FORWARD_LEVEL_COUNTS` (the bench value) stays.
- Only valid at constant speed or at rest. The robot's own acceleration reads as a slope
  (500 mm/s^2 is about 875 counts, 2.9 deg), so a consumer needs a threshold well above that or a
  gate on motion.

## How it works

`RobotState` is raw (counts, ADC counts, mm) and is never written here. Yaw rate is the wrapped
difference of successive headings, filtered. The IR counts go through `LineSense.h` (normalise,
mask with hysteresis) and then `lineView()`.

## Invariants

- No hardware and no `millis()`: time comes from `CONTROL_LOOP_PERIOD_MS`.
- `inclineCounts` saturates at +-32767 and uses no floating point.

## Status

Distance, speed and yaw rate have been checked on the robot (the odometer matched a tape measure to
under 1%). The incline and the line view have not: no bridge threshold is set yet, and the IR array
is not calibrated. The level capture has not been run with the motors on.
