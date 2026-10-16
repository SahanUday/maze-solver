# perception

`RobotState` in, one derived `Perception` out per tick. Pure logic in `include/sense/Perception.h` (with
`Tilt.h`, `WallSense.h` and `LineView.h`), tested by `test/test_perception/`, `test/test_tilt/`,
`test/test_wall_sense/` and `test/test_line_view/`.

## Interface

- `Perceiver` - the state `perceive()` carries between ticks: odometry, level reading, line
  calibration, mask.
- `perceiverReset(pc)`, `perceiverSetCalibration(pc, cal)` - the calibration comes from the EEPROM
  store at boot; this also works out which channels it accepts (`lineCalibrationUsable`).
- `perceiverSetLevel(pc, counts)` - the forward-axis reading on level ground, from `InclineLevel`.
- `PerceiveParams` - `mmPerCount`, `dtMs`, `frozenMs`, `frontFrozenMm`, `wall`, `irPitchMm`. Every default is the
  value in `RobotSpec.h`, so a default-constructed one is the spec.
- `perceive(pc, state, params, out)` - fills `Perception`: `motion` (distance, speed, heading, yaw
  rate), `inclineCounts`, `walls` ([wall_sense](wall_sense.md)), `sideBlindMs`, `line`, `floor` ([floor_pattern](floor_pattern.md): black, white, air or mixed)
  ([line_view](line_view.md)), `startPressed`.

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

## Sensor health

- **Frozen side sensor:** a valid side reading that does not change at all for `frozenMs`
  (`WALL_FROZEN_MS`, 400 ms) is a stuck sensor, since a real one scatters by a few mm. Its valid bit
  is dropped, so it is neither wall nor open and cannot steer the robot.
- **Frozen front sensor:** the front is judged by travel, not time (a still robot against a wall
  reads the same value for ever): an unchanging valid reading over `FRONT_FROZEN_MM` (60 mm) of
  odometer is frozen, since closing on a wall changes it by the distance covered.
- **Stray side echoes:** each side reading is the median of its last three distinct echoes, so
  one stray echo is dropped; a real change comes through one echo (60 ms) later. The front is not
  filtered here: the step cell refuses its stray echoes (two must agree), and a median would lag a
  ramp by an echo and hide the strays from that check.
- **`sideBlindMs`:** how long neither side sensor has given a usable reading (no echo, or frozen).
  An open side reads far and is valid, so this is a sensor failure, not a gap in the walls; the
  mission decides what to do about it.

## How it works

`RobotState` is raw (counts, ADC counts, mm) and is never written here. Heading and yaw rate come
from the gyro driver through `RobotState` (`headingDeg`, `yawRateDps`), so a heading snap to the grid
is not read as a rotation. The IR counts go through `LineSense.h` (normalise, mask with hysteresis)
and then `lineView()`.

## Invariants

- No hardware and no `millis()`: time comes from `CONTROL_LOOP_PERIOD_MS`.
- `inclineCounts` saturates at +-32767 and uses no floating point.
- With no usable IR channel the `LineView` is not `valid` and every state in it is false.

## Status

Distance, speed and yaw rate have been checked on the robot (the odometer matched a tape measure to
under 1%). The incline and the line view have not: no bridge threshold is set yet, and the IR array
is not calibrated. The level capture has not been run with the motors on.
