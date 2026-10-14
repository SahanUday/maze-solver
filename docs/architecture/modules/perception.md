# perception

`RobotState` in, what the control layer acts on out: one derived `Perception` per tick. Pure
logic in `include/Perception.h` (and `include/Tilt.h`), tested by `test/test_tilt/` and through
every suite that runs the whole stack.

## Interface

- `Perceiver` - the state `perceive()` carries between ticks (odometry, last heading, filtered yaw
  rate, line calibration, mask).
- `perceiverReset(pc)`, `perceiverSetCalibration(pc, cal)` - the calibration comes from the
  EEPROM store at boot; this also works out which channels it accepts.
- `perceive(pc, state, params, out)` - fills `Perception`: `motion` (distance, speed, heading, yaw
  rate), `tiltDeg`, `walls` ([wall_sense](wall_sense.md)), `line` ([line_follow](line_follow.md)),
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

4 host tests for the tilt arithmetic; `perceive()` is covered by the full-arena simulation. The
simulation feeds it encoder counts, a heading and IR counts the way the drivers would, but nothing
has come from a real sensor.
