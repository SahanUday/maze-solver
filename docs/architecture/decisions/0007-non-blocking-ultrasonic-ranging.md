# 0007: Non-blocking ultrasonic ranging — PORT K echo, Timer 5 capture

- **Status:** Accepted
- **Date:** 2026-10-04

## Context

Three HC-SR04 modules sense walls front, left and right. Each reports distance
as a pulse width on its `ECHO` pin: trigger high for ≥10µs, the module emits 8
cycles at 40kHz, and holds `ECHO` high for the round-trip time. Sound travels
~0.343mm/µs at 20°C, so distance is `µs × 0.1715`.

Two datasheet numbers dominate the design:

| | Value | Consequence |
|---|---|---|
| No-echo timeout | ~38ms | `ECHO` stays high this long when nothing returns |
| Recommended recycle | ≥60ms per sensor | residual echo corrupts the next ping |

The control loop ticks every 10ms (`CONTROL_LOOP_PERIOD_MS`). A wall at 235mm
answers in 1.4ms, but **silence costs ~38ms — nearly four ticks.** The common
`pulseIn()` approach blocks for exactly that, stopping PID, odometry and maze
logic at the moment the robot is somewhere open and moving. ADR 0001 already
identified this as the real problem in the sensing budget, independent of
framework choice.

### No interrupt is available on the originally proposed echo pins

`RobotConfig.h` proposed echo on pins 31/33/35 — all PORT C. On the ATmega2560,
**PORT C has no interrupt capability at all**: no external interrupt, and no
pin-change interrupt either (PCINT covers ports B, E0, J and K only). The six
external-interrupt pins are already fully allocated to the encoders and I2C, as
`RobotConfig.h` notes.

So the usual design — interrupt on the echo falling edge, read a timer — is
unavailable on those pins. The alternatives were: poll from a fast timer ISR
(~8.6mm resolution at 50µs polling, ~6% CPU), move the echo lines to a port that
can interrupt, or use hardware input capture (`ICP4`/`ICP5` on pins 49/48 — only
two pins for three sensors).

### Arena geometry bounds the useful range

Section B is 9×9 tiles at 250mm pitch, so the longest sightline that can
physically exist is 2250mm (13.1ms round trip). The sensor's 4m capability is
far beyond anything the maze can present.

## Decision

### Echo moves to PORT K

`A10`/`A11`/`A12` = `PK2`/`PK3`/`PK4` for front/left/right. These were
unassigned, and PORT K carries `PCINT16`–`PCINT23` on the `PCINT2` vector.
`PCMSK2` masks in only the pin of the sensor currently ranging, so `PIN_POT`
(`A8`/`PK0`) and `PIN_VBAT_SENSE` (`A9`/`PK1`) are untouched and continue to
work as analog inputs.

Triggers stay on pins 30/32/34 (PORT C), written **read-modify-write** — PORT C
also carries the IR array's emitter enable on `PC1`.

### Timer 5, prescaler 8

0.5µs per tick, 32.7ms span. Resolution is 0.086mm — far finer than the sensor's
own accuracy, and cheap: the ISR latches `TCNT5` and returns.

The timer must be 16-bit and owned outright, because `startPing()` zeroes it on
every ping. Timer 0 is `millis()`, Timer 2 is 8-bit, and `motors` holds **Timer 1
(OC1A/OC1B, right) and Timer 4 (OC4A/OC4B, left)** — so the choice is Timer 3 or
Timer 5. Timer 5, because Timer 3's compare outputs are PE3/PE4/PE5 = pins 5/2/3,
and pins 2/3 are the left encoder: nothing here drives those outputs, but a later
driver enabling one would silently stomp the encoder. Timer 5's outputs
(pins 44/45/46) are unallocated. This module needs no timer *pin* at all — only
`TCNT5` as a time base — so the choice costs no wiring.

### Round-robin, one sensor per 20ms slot

```
t (ms)   0         20        40        60        80
front  TRIG                                    TRIG      <- 60 ms gap
left             TRIG                                    <- 60 ms gap
right                      TRIG                          <- 60 ms gap
```

20ms is exactly two ticks, so slots land on tick boundaries with no drift. Each
sensor re-triggers every 60ms (16.7Hz), satisfying the datasheet recycle
requirement. At 300mm/s the robot travels ~18mm between readings on a given
sensor — wall distances are therefore inherently staler than encoder counts.

### We stop listening before the module does

The module's ~38ms timeout is longer than the 20ms slot, so a failed ping would
still be driving `ECHO` high when the next sensor's slot begins. Rather than
lengthen the slot, **the driver caps its own wait** at `US_RANGE_CAP_MM`
(2500mm ≈ 14.6ms), abandons the reading, and moves on.

This costs nothing real: 2500mm already exceeds the 2250mm longest sightline the
arena can present. It does not violate the datasheet either — the 60ms guidance
is about trigger-to-trigger spacing, which is unchanged.

Cross-talk is unaffected. A module emits for ~200µs and then only listens, so at
t=20ms the front sensor is silent; the two never chirp together.

**The `PCMSK2` mask is load-bearing here.** An abandoned sensor drops its echo
pin at t≈38ms, part-way through a later slot. With all three lines on one
vector, that stray edge would otherwise land in the ISR and corrupt an unrelated
reading. Masking to the active sensor makes the problem not exist, rather than
needing to be filtered out.

### Timeouts are reported as invalid, never as a distance

`RobotState` gains a `usValid` bitmask beside the three distances. A timeout
means *"I heard nothing"*, which on a wall angled past ~15° is the opposite of
*"nothing is there"*. The driver never writes a plausible distance it did not
measure — the same reasoning that kept a lost line from reporting as centred in
ADR 0002.

### Per-sensor calibration

Each sensor carries a signed `int16_t` offset (`US_OFFSET_*_MM`,
`<<TBD CALIBRATION>>`, all `0` until measured), applied in `ultrasonicReadingFromTicks()` before
the result reaches `RobotState`. So `RobotState` holds **corrected** distances
and every consumer gets the same number.

Calibrating at read time was rejected: each consumer would have to remember, and
the ones that forget do not crash — they steer slightly wrong, consistently, in
a way that looks like a tuning problem. Two consumers could also apply it twice,
or use different references, leaving `RobotState` meaning different things
depending on who reads it.

The offset is **signed** because a sensor may sit ahead of or behind its
reference point, and each sensor may use a different reference.

Order of operations matters:

1. ticks → raw mm (face to target)
2. validity against **raw** — dead zone and deadline are properties of the
   *sensor*, so an offset must not move them
3. apply the offset
4. reject if the result underflows past zero

Only an **offset** is corrected, not a gain. All three sensors share one clock,
one Timer 5 and the same air, so there is no per-unit scale error to speak of;
what differs is where each module sits relative to the point navigation cares
about. If a bench measurement ever shows error growing proportionally with
distance rather than staying constant, the cause is the speed-of-sound constant
or temperature — which affects all three equally — not per-sensor gain.

Compiled constants rather than EEPROM: mounting geometry does not drift with
vibration or lighting, unlike the IR array's threshold.

### Integer conversion

`mm = ticks × SPEED_OF_SOUND_M_S / 4000`, widened to `uint32_t` for the
multiply. Derivation, with ticks of 0.5µs:

```
mm = (ticks × 0.5e-6 s) × (343 m/s) × 1000 mm/m ÷ 2   =   ticks × 343 / 4000
```

Lives in `include/UltrasonicMath.h` — Arduino-free and header-only, so it
unit-tests under `env:native` like `Scheduler.h`.

Temperature is **not** compensated. The speed of sound runs about
`331.3 + 0.606 × T` m/s, so a 30°C hall gives ~1.8% more than the 20°C constant
— roughly 4mm at 235mm. That is a systematic bias rather than noise, so it will
not average out, but it is well inside the sensor's own accuracy and not worth
carrying a temperature sensor for.

## Consequences

**Good**

- The tick is never blocked. A failed ping costs one slot, not four ticks.
- 0.086mm timing resolution, with an ISR that does no arithmetic. The `TCNT5`
  read is software-latched: the fixed entry delay is the same on both edges and
  cancels, so only jitter from other ISRs (~0.17mm per µs) adds error.
- One ISR vector for all three sensors, and no external-interrupt pins consumed
  — there were none left.
- The analog pins already in use (`A8`, `A9`) are unaffected.

**Bad**

- **Requires re-wiring** the echo lines relative to the pins `RobotConfig.h`
  originally proposed. Decided before the harness was built, so the cost is
  nil now and would not have been later.
- **16.7Hz per sensor**, a sixth of the loop rate. Wall distances are staler
  than everything else in `RobotState`, and any consumer needs to know that.
- Three more constants whose real values are unmeasured (`US_MIN_RANGE_MM`,
  `US_MAX_RANGE_MM`, and the actual no-echo timeout of these specific units,
  which clones vary wildly on).
- Readings remain silently wrong when a wall is angled past ~15°: the sensor
  reports a timeout, and a timeout is indistinguishable from open space.

## Alternatives considered

**Poll the echo pins on PORT C from a fast timer ISR.** Needs no re-wiring, but
50µs polling gives ~8.6mm resolution for ~6% CPU, and finer polling costs
proportionally more. Rejected because the re-wiring was free at this point;
revisit via a superseding ADR if the pins later become fixed.

**Hardware input capture on `ICP4`/`ICP5` (pins 49/48).** Exact, hardware-latched
timing with no ISR latency at all. Rejected: only two capture pins are broken
out on the Mega and there are three sensors, so it would need either a fourth
mechanism for the third sensor or external multiplexing.

**`pulseIn()`.** Rejected per ADR 0001 and R1 — blocks up to 38ms.

**Shorter slots for a faster update rate.** 3× the rate by ignoring the 60ms
recycle guidance. Rejected: a stale echo read as a close wall would make the
robot swerve from a phantom obstacle, and a wrong distance is worse than a slow
one.

**Fire all three sensors simultaneously.** Triples the rate. Rejected: the
modules share a 40kHz band and would hear each other, which is why
`RobotConfig.h` already specifies firing one at a time.
