# 0008: Heading and tilt from the MPU-6050

- **Status:** Accepted
- **Date:** 2026-10-07

## Context

`RobotState::headingDeg` has been a declared field since the struct was written
("fused/integrated yaw, 0 = start heading") with nothing filling it. The sensor
is a GY-521 breakout carrying an InvenSense MPU-6050: a 3-axis MEMS gyroscope
and a 3-axis MEMS accelerometer behind one I2C slave at address `0x68`. Header
order on the board is `VCC, GND, SCL, SDA, XDA, XCL, ADO, INT`.

Nothing is wired yet and no measurement in this ADR comes from the real sensor;
the numbers below are datasheet figures and arithmetic, marked where they are
derived. `RobotConfig.h` already reserves D20/D21 for this device and
[0002](0002-drive-and-sensing-hardware-allocation.md) spent INT0/INT1 on them,
so the bus assignment is not in question here. What is open: what the driver
measures, in what units, and who turns raw counts into angles.

### What the robot needs from the gyroscope

Turn accuracy is the reason this sensor exists on the robot. A pivot turn
scrubs both wheels sideways across the floor, so encoder-derived rotation
depends on `TRACK_WIDTH_MM`, `WHEEL_DIAMETER_MM` and an assumption of no slip
that a pivot turn breaks by construction. Heading error does not average out -
it accumulates across turns, and Section B is 9x9 tiles
(`SECTION_B_SIZE_TILES`) of 250 mm pitch. A gyro measures chassis rotation
without reference to the wheels.

### What the robot needs from the accelerometer

The route crosses a raised bridge (`BRIDGE_LINE_WIDTH_MM` marks its line), so
the firmware has to know when it is on an incline and when it has levelled off.
Inferring that from encoder distance means trusting a dead-reckoned position to
tell the robot about its own physical attitude; the accelerometer measures the
attitude directly. The same reading also gives a collision signal - hitting a
wall is a sharp acceleration spike, which is a cheaper and faster "back up and
retry" trigger than noticing a stalled encoder.

### Why yaw cannot be corrected by the accelerometer

The accelerometer measures the gravity vector, which is why pitch and roll are
self-correcting. Rotation about the gravity vector does not change it at all,
so yaw has no absolute reference in this chip. Yaw comes from integrating one
rate signal, and the integral of a constant offset grows without bound.

Gyro bias dominates that growth. Taking an uncalibrated bias of 2 deg/s against
`RUN_LIMIT_MS` (480 s) gives 960 deg of accumulated error - the heading is
worthless. At a residual 0.05 deg/s the same arithmetic gives 24 deg over a
full run, which is still more than a 90 deg decision can absorb. So bias
removal is not an optimization here, and bias removal alone is not sufficient
either.

### Why the accelerometer cannot be trusted while accelerating

An accelerometer measures proper acceleration, and cannot distinguish the
robot's own acceleration from a change in tilt: accelerating forward pushes the
proof mass backward exactly as pitching nose-up does. Accel-derived pitch is
therefore only valid at constant speed or at rest. This is a property of the
sensor, not a calibration problem.

## Decision

- **One burst read of all 14 bytes from `0x3B`** per tick: accel X/Y/Z,
  temperature, gyro X/Y/Z. A single I2C transaction, never several - the chip
  snapshots its data registers for the duration of one read, so separate reads
  can straddle two samples and return a combination that never physically
  occurred.
- **`RobotState` mirrors the whole burst as raw counts, no angles:**
  `accelRaw[3]`, `imuTempRaw`, `gyroRaw[3]`. All seven values arrive on the
  wire whether or not they are kept, and copying a subset is more driver logic
  than copying the block, so the driver is a straight mirror of registers
  `0x3B`-`0x48`. 14 bytes, 0.17% of `SRAM_BUDGET_BYTES`.
- **Incline is detected from the raw forward-axis accel count, with no
  trigonometry.** Gravity is 16384 counts at +-2 g, so the forward axis tracks
  `sin(incline)`: about 1430 counts at 5 deg and 2850 at 10 deg, far above the
  noise left by the 21 Hz filter. A threshold on counts answers "on the bridge"
  without an `atan2f` per tick, in the same way
  [0006](0006-line-calibration-above-the-hal.md) thresholds raw ADC counts
  rather than converting them. The threshold constant is `<<TBD CALIBRATION>>`
  until the real bridge is measured.
- **Which accel axis is "forward", and which sign is nose-up, is settled on the
  bench**, not asserted here - the same treatment `MOTOR_L_INVERT` and
  `ENC_L_INVERT` get in `RobotSpec.h`.
- **Gyro full scale +-500 deg/s** (`GYRO_CONFIG` = `0x08`, 65.5 LSB per deg/s).
  A 90 deg pivot in 0.5 s is 180 deg/s, so this leaves headroom without
  spending the resolution that +-2000 deg/s would.
- **Accel full scale +-2 g** (`ACCEL_CONFIG` = `0x00`, 16384 LSB/g). Written
  explicitly even though it is the reset default, because the driver now
  depends on it. Tilt resolution is the priority: gravity is 1 g and robot
  accelerations are well under it. Impacts clip past 2 g, which is acceptable
  because collision detection wants the threshold crossing, not the peak value.
- **DLPF at 20 Hz** (`CONFIG` = `0x04`, which also sets the internal rate to
  1 kHz) and `SMPLRT_DIV` = `0`. One filter setting serves both sensors
  (20 Hz gyro, 21 Hz accel). The driver polls the newest sample at 100 Hz
  rather than matching the chip's rate to the tick: two independent clocks at
  the same nominal rate drift in and out of phase, duplicating and skipping
  samples, and an integrator turns duplicates and gaps into permanent heading
  error. 20 Hz is far below the 50 Hz Nyquist limit of a 100 Hz sampler, so
  motor vibration cannot alias into a slow signal that no later filtering could
  remove, and a maze turn is a ~2 Hz signal, so the bandwidth costs nothing.
- **Init sequence:** read `WHO_AM_I` (`0x75`) and require `0x68` before
  anything else, then `PWR_MGMT_1` (`0x6B`) = `0x01`. The chip comes out of
  reset asleep and returns zeros until that bit is cleared; `0x01` also selects
  the gyro's own PLL over the internal oscillator, as the register map
  recommends. Then `CONFIG`, `GYRO_CONFIG`, `ACCEL_CONFIG`, `SMPLRT_DIV`, and a
  wait of at least 30 ms for gyro start-up.
- **Interpretation lives above the HAL**, following
  [0006](0006-line-calibration-above-the-hal.md). The driver converts nothing.
  A pure `include/Heading.h` holds the gyro bias estimate and the yaw
  integrator; pitch from `accelRaw` belongs with it or beside it, as
  host-testable functions alongside `Quadrature.h` and `LineSense.h`.
  `readSensors()` calls the driver, then the integrator, then writes
  `headingDeg` - so the existing field keeps its documented meaning and the
  layering of `RobotState` holds.
- **Gyro bias is captured at run time, never written into `RobotSpec.h`.**
  Average ~1000 samples with the robot held still at boot, blocking, before the
  control loop starts. Re-zeroing whenever the robot is known stationary is the
  intended mechanism for the residual drift; what triggers it is not built yet.
- **Any pitch consumer must gate on the robot being at constant speed or
  stopped**, for the reason in Context. A tilt reading taken while accelerating
  is not a tilt reading.
- **Heading is snapped to the maze, not trusted outright.** The arena is all
  90 deg turns on a 250 mm grid, so the control layer rounds heading to the
  nearest 90 deg against wall readings once a turn completes. This is recorded
  as the plan for the drift, not as code that exists.
- **`Wire.h`, no third-party library.** [0001](0001-hybrid-hardware-abstraction.md)
  allows the IMU a vetted library because of *fusion*-correctness risk. This
  driver does no fusion - it is five register writes and one burst read - so
  the risk the exception was granted for does not arise, and `lib_deps` stays
  empty. `Wire.setClock(400000)` for the 400 kHz the MPU-6050 supports. Note
  `BUFFER_LENGTH` in the AVR `Wire` implementation is 32 bytes, so a 14-byte
  burst fits with room to spare.
- **The AVR's internal I2C pull-ups are cleared** right after `Wire.begin()`,
  by writing `PORTD` directly. The MPU-6050's logic pins are not 5 V tolerant
  and the breakout's own 4.7k pull-ups go to its 3.3 V rail; `Wire.begin()`
  otherwise pulls the bus toward 5 V. `digitalWrite()` is banned inside
  `src/hal/` by `check-banned-patterns.sh`, so the register write is the only
  available form as well as the correct one.
- **`ADO` and `INT` are left unconnected.** `ADO` floating selects `0x68`. The
  four external-interrupt vectors are encoders and INT0/INT1 are the bus
  itself ([0002](0002-drive-and-sensing-hardware-allocation.md)), so a
  data-ready interrupt would need a PCINT pin (D14/D15 are free) for no gain at
  a 4%-of-tick read.
- **The board is mounted flat and rigid**, parallel to the floor, X/Y aligned
  to the chassis, so yaw is rotation about Z and gravity sits on accel Z. A
  flexible mount reads chassis vibration as rotation.

## Consequences

**Good**

- Turn accuracy stops depending on `TRACK_WIDTH_MM`, `WHEEL_DIAMETER_MM` and a
  no-slip assumption, all of which a pivot turn violates.
- The bridge incline becomes directly observable instead of inferred from
  dead-reckoned distance, and collisions get a signal that does not wait for a
  stalled encoder.
- Comparing the gyro rate against the encoders' predicted
  `(v_right - v_left) / track` gives a slip/jam check for free, once both exist.
- Heading is a fast, smooth 100 Hz signal, which suits a PID far better than
  ultrasonic side readings that are slow and unreliable near gaps and corners.
- Keeping the whole block costs nothing to read and leaves no axis to
  re-litigate later: `imuTempRaw` is the only handle on the warm-up bias drift
  this ADR cannot quantify, and `gyroRaw[0]`/`[1]` are what a complementary
  filter needs to make pitch survive the robot's own acceleration.
- Bridge detection needs no floating-point trigonometry, only a comparison.
- The integrator and the tilt math are tested on the host against recorded
  counts, with no robot attached, like `LineSense.h`.
- No new dependency to pin, audit or fit in flash.

**Bad**

- The read is ~0.4 ms against ~0.15 ms for gyro Z alone - about 4% of a 10 ms
  tick, blocking, and `Wire.h` busy-waits (derived from the bit count, not
  measured). It must not be called from an ISR.
- At `DLPF_CFG = 4` the accelerometer path carries ~11.6 ms of group delay,
  more than one tick. Irrelevant for a slow tilt reading, but it puts a floor
  under how fast a collision can be detected. A faster DLPF would trade the
  vibration rejection that makes the gyro usable, so the gyro wins the setting.
- Accel-derived pitch is unusable during acceleration and braking, so the
  bridge logic needs a motion gate it does not have yet. This is the sharpest
  constraint in this ADR.
- `imu.cpp` writes `PORTD`, which `encoders.cpp:77` also does
  (`PORTD |= (1 << PD2) | (1 << PD3)` for its pull-ups). Both are
  read-modify-write and both run once at init, in sequence, on disjoint bits,
  and no ISR writes `PORTD` - so this is safe, but it is only safe by that
  argument and not by construction.
- `check-pin-map.py` detects `twi` ownership from `TWCR`/`TWSR`/etc., which live
  inside `Wire.h` rather than in `src/`. So nothing in CI would notice a future
  register-level TWI driver colliding with this one. A
  `// pin-check: shared twi - owned by Wire.h` comment documents it but does
  not make the check fire.
- `RobotConfig.h`'s "Owned by the Wire library - do not reference directly" on
  `PIN_I2C_SDA`/`PIN_I2C_SCL` stops being true: the `registers` rule in
  `check-pin-map.py` requires a `static_assert` naming a `PIN_*` on the port a
  driver touches, so `imu.cpp` must reference both. That comment needs
  correcting in the PR that adds the driver.
- Yaw drift is reduced, not solved. Without the stationary re-zero and the
  90 deg snap - neither of which exists yet - the arithmetic above says a full
  run ends tens of degrees out.
- Impacts above 2 g clip, so the accelerometer ranks collision severity only up
  to that point.
- Every number here is from the datasheet or arithmetic. Bias magnitude,
  residual drift after calibration, warm-up behaviour, vibration amplitude at
  the real mount and actual I2C timing are all unmeasured, and the module doc
  will list them as such.

## Alternatives considered

**Gyro Z alone, accelerometer unread.** The first draft of this ADR. Rejected:
it leaves the bridge incline to be inferred from dead-reckoned distance, gives
up the collision signal, and saves only ~0.25 ms of a 10 ms tick, since the
accel bytes sit in the same contiguous burst.

**Two transactions - 6 accel bytes, then 2 gyro-Z bytes.** Rejected: it costs
about the same as one 14-byte burst and breaks the register snapshot, so the
accelerometer and gyro samples could come from different instants.

**Accel at +-4 g to stop impacts clipping.** Rejected: it halves tilt
resolution, which is the primary use, to improve severity ranking for a
collision response that only needs a threshold crossing.

**The DMP (on-chip fusion, via `electroniccats/MPU6050`).** Rejected for now.
It would give gravity-referenced pitch that survives the robot's own
acceleration, which is a real answer to this ADR's sharpest constraint - but it
cannot fix yaw drift, because the chip has no yaw reference either, and it costs
a ~3 KB firmware blob plus FIFO packet handling to debug, inside an 8 KB SRAM
budget (`SRAM_BUDGET_BYTES`) and `-Wstack-usage=128`. Revisit with a new ADR if
a motion-gated accel pitch proves too coarse for the bridge.

**A complementary filter over accel pitch and gyro pitch rate.** The cheap
middle road between a motion gate and the DMP, and the reason the gyro X/Y
counts are kept rather than discarded. Not built yet: there is no bridge logic
to tune it against, and a filter written before its consumer is a filter tuned
against nothing. The inputs are in `RobotState` when it is wanted, so adding it
needs no change to the driver or to this decision.

**Storing only the axes with a consumer today (`accelRaw[3]`, `gyroRawZ`).** The
first draft of this ADR. Rejected: it saves 4 bytes of a 8192-byte budget, adds
selective-copy logic to a driver whose whole job is not to interpret, and would
have to be reopened the moment pitch needed fusing - which is the one case where
accel pitch is known to fail.

**Gyro bias as a constant in `RobotSpec.h`.** Rejected for the reason
[0006](0006-line-calibration-above-the-hal.md) rejected baked-in line
thresholds: it varies per chip and with temperature, the chip self-heats, and
the competition allows no reprogramming between trials.

**Heading from the encoders alone, no gyro.** This is the status quo and it is
what the sensor is here to replace; see Context.

**Position by integrating the accelerometer.** Rejected outright. Double
integration compounds noise and bias into metres of error within seconds.
Distance stays with the encoders.
