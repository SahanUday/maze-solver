# perception

`RobotState` in, one derived `Perception` out per tick. Pure logic in `include/Perception.h` (with
`Tilt.h`, `WallSense.h` and `LineView.h`), tested by `test/test_perception/`, `test/test_tilt/`,
`test/test_wall_sense/` and `test/test_line_view/`.

## Interface

- `Perceiver` - the state `perceive()` carries between ticks: odometry, last heading, filtered yaw
  rate, line calibration, mask.
- `perceiverReset(pc)`, `perceiverSetCalibration(pc, cal)` - the calibration comes from the EEPROM
  store at boot; this also works out which channels it accepts.
- `perceive(pc, state, params, out)` - fills `Perception`: `motion` (distance, speed, heading, yaw
  rate), `tiltDeg`, `walls` ([wall_sense](wall_sense.md)), `line` ([line_view](line_view.md)),
  `startPressed`.
- `tiltDeg(imu)` - the angle between gravity and the board's Z axis.

## How it works

`RobotState` is raw (counts, ADC counts, mm) and is never written here. Yaw rate is the wrapped
difference of successive headings, filtered. The IR counts go through `LineSense.h` (normalise,
mask with hysteresis) and then `lineView()`.

## Invariants

- No hardware and no `millis()`: time comes from `CONTROL_LOOP_PERIOD_MS`.
- Tilt reads high while the robot accelerates hard, so it is only compared with a threshold well
  above a few degrees.

## Status

Distance, speed and yaw rate have been checked on the robot (the odometer matched a tape measure to
under 1%). The line view has not: the IR array is not calibrated.
