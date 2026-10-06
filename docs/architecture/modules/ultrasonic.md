# `ultrasonic` — 3× HC-SR04 wall ranging

Non-blocking HAL driver for the front/left/right wall sensors. Reasoning:
[`../decisions/0007-non-blocking-ultrasonic-ranging.md`](../decisions/0007-non-blocking-ultrasonic-ranging.md).

## Files

| File | Layer | Role |
|---|---|---|
| `src/hal/ultrasonic.{h,cpp}` | 1 (HAL) | round-robin scheduling, trigger, echo timing |
| `include/UltrasonicMath.h` | 2 | ticks → mm; Arduino-free, header-only |
| `test/test_ultrasonic/` | — | the math under `env:native` |

## Hardware interface

| Sensor | TRIG | Port | ECHO | Port |
|---|---|---|---|---|
| Front | 30 | `PC7` | **A10** | `PK2` |
| Left | 32 | `PC5` | **A11** | `PK3` |
| Right | 34 | `PC3` | **A12** | `PK4` |

Echo is on **PORT K, not PORT C**. Port C has no interrupt capability on the
ATmega2560 — no external interrupt and no pin-change interrupt — so echo timing
there would have to be polled. Port K carries `PCINT16`–`PCINT23`.

`ECHO` outputs 5 V and the Mega is a 5 V part, so no divider is needed despite
what most tutorials say.

## How a measurement happens

```
startPing()   TCNT5 = 0; PCMSK2 = this sensor; 10us pulse on TRIG
     |
     |  module emits 8 cycles at 40 kHz, raises ECHO
     v
ISR (rising)  g_startTicks = TCNT5
     |
     v
ISR (falling) g_endTicks = TCNT5; PCMSK2 = 0
     |
     v
update()      ultrasonicReadingFromTicks(end - start) -> mm, or invalid
```

Timer 5 runs free in normal mode at prescaler 8 — **0.5 µs per tick**, wrapping
at 32.7 ms. The ISR copies `TCNT5` and nothing else: no arithmetic, no division.

## Scheduling

One sensor per **20 ms slot**, which is exactly two ticks, so slots land on tick
boundaries with no drift.

```
t (ms)   0         20        40        60        80
front  TRIG                                    TRIG
left             TRIG                                   
right                      TRIG                         
```

Each sensor re-triggers every 60 ms (**16.7 Hz**), satisfying the datasheet's
recycle requirement. At 300 mm/s the robot moves ~18 mm between readings on a
given sensor, so **wall distances are staler than everything else in
`RobotState`** — anything consuming them should know that.

### The driver stops listening before the module does

The module holds `ECHO` high for ~38 ms when nothing returns, which is longer
than the slot. Rather than lengthen the slot, the driver gives up at
`US_RANGE_CAP_MM` (2500 mm ≈ 14.6 ms) and moves on.

Nothing is lost: 2500 mm already exceeds the 2250 mm longest sightline a 9×9
arena can present. The datasheet is not violated either — its 60 ms guidance is
about trigger-to-trigger spacing, which is unchanged.

### `PCMSK2` masking is load-bearing

All three echo lines share the `PCINT2` vector. An abandoned sensor drops its
echo pin at t≈38 ms, part-way through a *later* sensor's slot. Masking `PCMSK2`
to only the active sensor means that stray edge never reaches the ISR, rather
than having to be detected and filtered.

`PCMSK2` also admits nothing on `PK0`/`PK1`, so `PIN_POT` and `PIN_VBAT_SENSE`
keep working as analog inputs.

## Port sharing

- **PORT C** also carries the IR array's emitter enable on `PC1`, so every
  trigger write is read-modify-write.
- **Timer 5** is claimed by this module, which zeroes it on every ping. Timer 0
  is `millis()`, Timer 2 is 8-bit, and `motors` owns Timer 1 and Timer 4. Timer 3
  is the only 16-bit timer still free; note its compare outputs are pins 5/2/3
  and pins 2/3 are the left encoder.

## Validity

`RobotState::usValid` carries one bit per sensor (0 front, 1 left, 2 right).
Distance is meaningful **only** when the matching bit is set, and the driver
zeroes the distance whenever it clears the bit — there is no stale data.

A timeout means *"I heard nothing"*. On a wall angled past ~15° the sound
reflects away, so silence and open space are indistinguishable. Reporting a
timeout as maximum range would assert "nothing there" about a wall directly
ahead, which is why the flag exists instead.

## Conversion

`mm = ticks × 343 / 4000`, widened to `uint32_t` for the multiply. Integer
division truncates, so distances read ~1 mm short consistently — far inside the
sensor's own accuracy, and the tests assert the exact truncated values so the
behaviour cannot drift unnoticed.

Temperature is **not** compensated. The speed of sound runs about
`331.3 + 0.606 × T` m/s, so a 30 °C hall reads ~1.8% long against the 20 °C
constant — roughly 4 mm at 235 mm. A systematic bias, not noise, so it will not
average out.

## Calibration

Each sensor has a signed offset (`US_OFFSET_FRONT_MM` / `_LEFT_` / `_RIGHT_`),
applied inside `ultrasonicReadingFromTicks()` so `RobotState` holds corrected
distances. All
three are `0` until measured.

**To measure:** park the robot with a flat wall at a known distance from that
sensor's reference point, read `dist`, and take `offset = actual - reported`.
Do it at two or three distances — a constant error is an offset (what this
corrects); an error growing with distance is a speed-of-sound or temperature
issue affecting all three sensors, not per-sensor gain.

The dead zone and the echo deadline are judged on the **raw** reading, before
the offset, because both are properties of the sensor rather than of the
chassis. A negative offset that pushes a near reading past zero is reported
invalid, not wrapped.

## Known limitations

- **Angled walls read as silence.** Past roughly 15° off perpendicular the sound
  reflects away. This happens whenever the robot is skewed in a corridor, and is
  the most likely source of surprising behaviour.
- **The beam is a ~15° cone**, about 60 mm wide at 235 mm. You measure the
  nearest thing in a patch, not a point.
- **Dead zone below `US_MIN_RANGE_MM`.** A centred robot sees walls at ~117 mm,
  but one hugging a wall could get inside it.
- **16.7 Hz per sensor**, a sixth of the loop rate.
- **Unmeasured constants.** `US_MIN_RANGE_MM` / `US_MAX_RANGE_MM` hold datasheet
  figures, not bench measurements, and the real no-echo timeout of these
  specific units is unverified — clones vary from 38 ms to 200 ms.
- **Cross-sensor interference at 20 ms spacing is unverified.** A stale
  reflection would have travelled ~6.9 m by then, well past the sensor's range,
  but a walled maze is a reflective box. Lengthening the slot costs rate, not
  correctness.
