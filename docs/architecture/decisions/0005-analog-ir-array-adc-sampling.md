# 0005: Analog IR array — ADC sweep, raw counts

- **Status:** Accepted
- **Date:** 2026-10-06
- **Supersedes:** [0004](0004-digital-ir-array-sensing.md)

## Context

[0004](0004-digital-ir-array-sensing.md) recorded the floor sensor as a digital
(comparator) array: one bit per channel, a fixed threshold, no calibration
possible. A quiet idle pin cannot tell a comparator from an analog node, and
every reading that decision rests on was a digital read. Reading the same pins
with the ADC shows the outputs are **analog**.

### Bench measurements

One board and one array on a bench Mega, wired as in `RobotConfig.h`. Heights
were set by hand with spacers (approximate), the surface was black *paper*, and
room light was on. ADC counts, 0-1023, emitters on, D1..D8:

| Condition | Counts | Digital `PINF` read |
|---|---|---|
| Open air | 1020 on all 8 | all HIGH |
| Emitters off (any surface) | 1023 on all 8 | all HIGH |
| White paper, ~3 mm | ~350-650 | D3/D4 partly HIGH, D6-D8 HIGH, the rest LOW |
| Black paper, ~3 mm | 956-1011 | all HIGH |
| White paper, 22.77 mm | 958-978 | all HIGH |
| Black paper, 22.77 mm | 1019 | all HIGH |
| Half white / half black, 22.77 mm | ramp 988 → 1016, D1 → D8 | all HIGH |

- Noise is ±1 count on a still surface. The ADC reads the same from a /128 to a
  /8 clock, and with or without a dummy first conversion, so source impedance is
  not a problem.
- Enabling the Mega's internal pull-up on a channel cuts its distance from the
  rail by a constant factor (0.78 on every channel and at every signal level).
  That puts the module's own pull-up at about 0.28x the Mega's, roughly
  5-14 kΩ: the channels are phototransistor collectors, not comparator outputs.
- A digital read of a mid-scale voltage is unreliable: the Mega guarantees LOW
  only below about 1.5 V and HIGH above about 3 V (counts ~307 and ~614), and
  white at 3 mm sits between them. At 22.77 mm both surfaces read HIGH, so a
  digital driver sees no line at all.
- The emitter enable (`IR`, pin 36): driven LOW the emitters are off, driven
  HIGH they are on, and a floating pin or the internal pull-up also lights them
  (the module pulls it up). Idle readings over open air are the same either way,
  so they cannot show it. After a switch the signal is 90% settled in ~0.2 ms and
  within one count in ~0.4 ms turning on, ~0.9 ms turning off.
- A sweep of the 8 channels takes 242 µs at /32 (13 ADC clocks per conversion;
  the digital driver's three-sample vote took 289 µs) and about 1.8 ms at the
  Arduino default /128 with a dummy conversion.
- Eight `digitalRead()` calls skew over 28.8 µs, about 29 µm at 1 m/s against a
  9.5 mm sensor pitch, so atomicity across the array does not decide
  register-vs-Arduino; the ADC sweep itself spans ~0.24 ms.
- All 8 channels respond in order (sensor #k, counted from the `D1` header end,
  lands on channel Dk), including A4-A7, which share pins with JTAG. The bench
  board's JTAG fuse is off (L=FF H=D8 E=FD).

## Decision

- `line_sensors` reads A0-A7 with the ADC, directly through its registers
  ([0001](0001-hybrid-hardware-abstraction.md)): AVcc reference, ADC clock
  CPU_HZ/32 (`IR_ADC_PRESCALER`), one polled conversion per channel, 10-bit
  counts into `RobotState::irRaw[8]`. Lower means more reflected light.
- The driver does not threshold, calibrate or interpret the counts. Position
  math and per-channel calibration wait for data from the real arena material
  and the final mount height.
- Every conversion writes `ADMUX` and clears `MUX5` itself. The Arduino core's
  `analogRead()` leaves `MUX5` set after reading A8-A15, so a boot-time read of
  `PIN_POT` would otherwise turn every later sweep into ADC8-15 without any error.
- The emitter enable is driven HIGH, and `lineSensorsSetEmitters()` waits
  `IR_EMITTER_SETTLE_US` when it changes the state. The Mega's pull-ups stay off
  (they would shift every reading ~20%) and the digital input buffers are off
  (`DIDR0`).
- The wiring assumptions (channels A0-A7 in order, emitter on pin 36, a prescaler
  giving an ADC clock of at most 1 MHz, the highest value checked) are
  `static_assert`-ed against `RobotConfig.h`.
- The debounce vote and `IR_BLACK_IS_HIGH` are gone: the signal is stable to ±1
  count, and polarity is a property of the counts, not a constant.

## Consequences

**Good**

- The magnitude is available, so per-channel calibration in software is
  possible, and there is measurable contrast at the 22.77 mm mount (about 40-60
  counts between white and black paper) where a digital read sees nothing.
- A sweep is cheaper than the digital driver it replaces (242 µs against 289 µs),
  with no vote and no `_delay_us` spacing.
- ADR 0001's original expectation of an ADC-read line array was right.

**Bad**

- The ADC registers are this module's. `main.cpp` can still read `PIN_POT` (A8)
  once at boot with `analogRead()`: the pin-map owners rule only sees register
  names, so it neither flags nor needs a declaration. A second register-level ADC
  driver would have to declare `// pin-check: shared adc - <reason>` in both
  modules.
- The channels are sampled one after another over ~0.24 ms, and
  `lineSensorsRead()` blocks that long.
- A0-A7 cannot be read as GPIO any more (`PINF` reads 0).
- At the 22.77 mm mount the white/black contrast is only 4-6% of full scale and
  falls steeply with height, so the mount height remains the largest lever.
  `IR_RIDE_HEIGHT_MM` stays `<<TBD HARDWARE>>`.
- The measurements come from one board and one surface material; the module doc
  lists what is still untested.

## Alternatives considered

**Keep the single-port `PINF` read.** Rejected: it discards the magnitude, reads
white at 3 mm in the undefined voltage band, and sees nothing at 22.77 mm.

**Free-running or auto-triggered ADC with an interrupt.** Not built: a polled
sweep is 2.4% of the tick. Revisit if profiling shows the blocking time matters.

**Emitters off/on differential for ambient rejection.** Not built: it costs the
settling time twice (about 1.3 ms per scan), and a phone torch and a wall lamp
moved readings by at most 1-2 counts. Revisit if IR-rich ambient proves to
matter at the venue.

**Lower the array.** The physical lever with the most effect (contrast was about
half of full scale at 3 mm). The choice belongs with the mount, not the driver.

**ADC clock at the Arduino default /128.** No accuracy benefit was measured
against /32 on this signal, and a sweep takes about 1.8 ms.
