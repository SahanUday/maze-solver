# imu

MPU-6050 6-axis IMU (GY-521 breakout) on the I2C bus, read once per tick for the
yaw rate that becomes `RobotState::headingDeg` and the accelerometer counts that
will detect the bridge incline. Code: `src/hal/imu.{h,cpp}`. Decision:
[0008](../decisions/0008-mpu6050-heading-and-tilt.md). The counts are turned into
an angle by [heading](heading.md).

This is the one HAL module that uses the Arduino framework rather than registers,
which [0001](../decisions/0001-hybrid-hardware-abstraction.md) allows for the IMU.
It is also the one driver whose device can simply not be there, so both entry
points report failure instead of returning a reading.

## Interface

- `imuInit()` - frees the bus if the chip is holding it, starts `Wire`, clears the
  AVR's internal I2C pull-ups, checks `WHO_AM_I` and writes the four configuration
  registers; runs that sequence a second time if the first fails. `false` if the
  sensor does not answer or answers wrongly both times. Blocks
  `IMU_STARTUP_DELAY_MS` (30 ms), twice that if it has to retry. Call once.
- `imuRead(out)` - one burst read of the 14 data registers into `ImuRaw`. `false`
  on a failed transfer, and `out` is then left untouched rather than
  half-written. Blocks ~0.4 ms. Not callable from an ISR.

`ImuRaw` lives in `RobotState.h` (plain data, no `Arduino.h`) so the boundary
struct can hold it directly and `readSensors()` needs no copy: `accel[3]`,
`temp`, `gyro[3]`, all raw `int16_t`, a straight mirror of registers
`0x3B`-`0x48`. The driver converts nothing and interprets nothing - not the
scales, not the axes, not the sign.

The board is the 8-pin GY-521: `VCC` (5 V, it has its own regulator), `GND`,
`SCL` -> D21, `SDA` -> D20, and `XDA`/`XCL`/`ADO`/`INT` unconnected. `ADO`
floating selects address `0x68`; `XDA`/`XCL` are for the MPU mastering a
downstream sensor, which we do not do.

## How it works

`imuInit()` runs `releaseStuckBus()` (below), calls `Wire.begin()`, then clears PD0/PD1 in
`PORTD`. That second step matters: the MPU-6050's logic pins are not 5 V tolerant and the breakout's
own 4.7k pull-ups go to its 3.3 V rail, but `Wire.begin()` switches the AVR's
internal pull-ups on and pulls the bus toward 5 V. It then sets the bus to
`IMU_I2C_CLOCK_HZ` (400 kHz), requires `WHO_AM_I` = `0x68`, writes `PWR_MGMT_1`
= `0x01` (the chip boots **asleep** and reads zeros until that clears SLEEP; the
same byte selects the gyro's PLL over the internal oscillator), waits 30 ms for
gyro start-up and PLL lock, and finally writes `CONFIG` = `0x04` (DLPF 20 Hz
gyro / 21 Hz accel, 1 kHz internal rate), `GYRO_CONFIG` = `0x08` (+-500 deg/s),
`ACCEL_CONFIG` = `0x00` (+-2 g) and `SMPLRT_DIV` = `0`.

A reset that lands in the middle of a transfer does not reset the chip, which can be
left waiting for the clocks that finish its byte; the first transfer after the restart
then times out. `releaseStuckBus()` is for that. With both pins as plain inputs it
clocks SCL nine times (up to eighteen, if the chip is still holding SDA after one
byte) at 10 us per clock, then sends a STOP. It is harmless on an idle bus. If the
sequence still fails, `imuInit()` calls `Wire.end()` and runs it once more.

A transfer that gets no answer within `IMU_I2C_TIMEOUT_US` gives up and resets the TWI
hardware, so a bus held low (an unplugged or unpowered gyro) fails the read instead of hanging the
control loop with the motors driving.

`imuRead()` writes the start address, ends the transmission with `false` for a
repeated start, and requests all 14 bytes in **one** transaction. The chip holds
its data registers still for the duration of a single read, so several smaller
reads can straddle two samples and return a combination that never physically
occurred. Each value arrives high byte first. The seven values are assembled in
a local array and copied into `out` only once all of them are in.

The chip updates at 1 kHz while the loop reads at 100 Hz, so every read gets a
fresh sample and there is no beat between the two clocks - matching them would
duplicate and skip samples, and an integrator turns both into permanent heading
error.

## Invariants

- SDA stays pin 20 and SCL stays pin 21: `static_assert`s against
  `RobotConfig.h` fail the build otherwise. They exist because the pull-up clear
  writes `PORTD` directly, which is also what `check-pin-map.py`'s `registers`
  rule looks for.
- `IMU_GYRO_AXIS_YAW` must index `ImuRaw::gyro`, and the 14-byte burst must fit
  `Wire`'s `BUFFER_LENGTH` (32). Both are `static_assert`ed.
- `CPU_HZ` must equal `F_CPU`, since `_delay_ms()` is computed from the macro.
- The module shares `DDRD` and `PORTD` with [encoders](encoders.md), which
  read-modify-writes PD2/PD3 for its pull-ups (`encoders.cpp:76-77`); this module
  touches only PD0/PD1, in `releaseStuckBus()` and the pull-up clear. Both run
  at init, in sequence, on disjoint bits, and no ISR writes `DDRD` or `PORTD`
  (the encoder ISR only reads `PIND`) - safe, but safe by that argument rather
  than by construction.
- `Wire.h` owns the TWI peripheral. `check-pin-map.py` detects `twi` ownership
  from `TWCR`/`TWSR`, which are inside the library and not in `src/`, so a
  future register-level TWI driver would **not** be flagged as colliding.
- Blocking and busy-waiting: not for use inside an ISR.
- The board must be mounted flat and rigid, parallel to the floor, for Z to be
  yaw and for gravity to sit on accel Z.

## Status

Compiles under `-Wall -Wextra -Werror -fno-rtti -Wstack-usage=128`. Both
`static_assert`s were checked by deliberately breaking them (SDA moved to pin
22, `IMU_GYRO_AXIS_YAW` set to 3) and both failed the build. Adding this module
and `Wire.h` took the firmware from 248 to 506 bytes of SRAM (3.1% of
`SRAM_BUDGET_BYTES`) and 3714 to 8954 bytes of flash; `check-ram-budget.sh` and
`check-flash-budget.sh` pass.

**Measured on the robot (2026-10-13).** The sensor answers at `0x68`, `imuInit()`
succeeds, and `imuRead()` returned a burst on every tick (accel Z about 1 g with
the board flat). The start-up recovery was tested by bit-banging a transfer and
leaving it unfinished at 27 positions (start condition, partial address byte,
partial register byte, partial read data), releasing the pins as a reset does,
restarting, and counting `imuInit()` results alongside an uninterrupted control:
0 failures in 743 starts. With 40 real hardware resets while `imuRead()` ran in a
tight loop, 0 failures. Without the release and retry the same two tests failed
124 of 859 starts (all at four read-data positions) and 12 of 40 resets.

Orientation, measured by hand on the compact chassis (2026-10-13): a counter-clockwise
turn of about 90 deg takes `headingDeg` from 0 to +89.4 with `GYRO_YAW_INVERT` false, and
gyro Z carries it (`IMU_GYRO_AXIS_YAW` = 2). Lifting the nose about 26 deg moves accel Y by
-7415 counts, so Y is the forward axis and it reads negative nose-up
(`IMU_ACCEL_AXIS_FORWARD` = 1, `IMU_ACCEL_NOSE_UP_NEGATIVE` = true). The turn was not
measured against a reference angle, so this fixes the sign, not the gyro scale.

Accelerometer, level (2026-10-14): 30 s still in each of two orientations 180 deg apart.
|g| is 17152 counts in both (1.047 x 16384), so this unit's sensitivity reads about 4.7% high
(the datasheet tolerance was not checked here). The part of X and Y that stays with the robot
is X +57 and Y +245 counts, about 0.8 deg, and the surface contributed up to +-210 counts of
Y; `IMU_ACCEL_ONE_G_COUNTS` and `IMU_ACCEL_FORWARD_LEVEL_COUNTS` record them. A mount tilt of
that size couples about 1.5% of a pitch or roll rate into the yaw axis. Tables:
[docs/calibration/gyro-and-accelerometer-compact-chassis.md](../../calibration/gyro-and-accelerometer-compact-chassis.md).

**Everything else about the device is unverified**: that the configuration takes,
that the counts mean what the scales say. The ~0.4 ms read time is derived from
the bit count at 400 kHz, not measured.

Still to check, in roughly this order: that 400 kHz is reliable over the real harness length (fall back to 100 kHz if
not); the actual read duration; vibration amplitude at the real mount with the motors
running; the accelerometer's offset against temperature and with all six faces
(sensor offset against mount tilt); and whether clearing the internal pull-ups is
enough or the bus needs level shifting.
