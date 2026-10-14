# perception

`RobotState` in, what the control layer acts on out: one derived `Perception` per tick. Pure
logic in `include/Perception.h` (with `Tilt.h`, `WallSense.h` and `LineView.h`), tested by
`test/test_perception/`, `test/test_tilt/`, `test/test_wall_sense/` and `test/test_line_view/`.

## Interface

- `Perceiver` - the state `perceive()` carries between ticks (odometry, last heading, filtered yaw
  rate, line calibration, mask).
- `perceiverReset(pc)`, `perceiverSetCalibration(pc, cal)` - the calibration comes from the
  EEPROM store at boot; this also works out which channels it accepts.
- `perceive(pc, state, params, out)` - fills `Perception`: `motion` (distance, speed, heading, yaw
  rate), `tiltDeg`, `walls` ([wall_sense](wall_sense.md)), `line` ([line_view](line_view.md)),
  `startPressed`.
- `tiltDeg(imu)` - the angle between gravity and the board's Z axis, so which way X and Y point
  does not matter.

## How it works

`RobotState` is raw (counts, ADC counts, mm); the control layer wants a yaw **rate**, which
walls exist and where the line is. `RobotState` itself is never written here - the drivers own it.
Yaw rate is the wrapped difference of successive headings, filtered. The IR counts go through the
existing `LineSense.h` (normalise, mask with hysteresis) and then `lineView()`.

## Invariants

- No hardware and no `millis()`: time comes from `CONTROL_LOOP_PERIOD_MS`.
- Tilt reads high while the robot accelerates hard (a few degrees), so it is only ever compared
  with a threshold well above that.

## Status

12 host tests for `perceive()` (distance, speed, yaw rate across the wrap, the wall view, tilt, the
calibration channel mask, reset) and 4 for the tilt arithmetic. The distance, speed and yaw rate have been
checked against the robot (the odometer matched a tape measure to under 1%, and 90 degree turns by the
gyro looked like quarter turns); the line view has not, because the IR array is not calibrated yet.
