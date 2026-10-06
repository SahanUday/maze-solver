# `line_sensors` — 8-channel analog IR array

HAL driver for the floor-sensing IR array. Code: `src/hal/line_sensors.{h,cpp}`.
Decision and bench measurements:
[`../decisions/0005-analog-ir-array-adc-sampling.md`](../decisions/0005-analog-ir-array-adc-sampling.md)
(supersedes [0004](../decisions/0004-digital-ir-array-sensing.md)).

## What it is for

The walled maze sections are navigated by ultrasonic ranging plus encoder/IMU
odometry. The IR array exists for the **bridge** between Section A and Section
B, where a 30mm black line (`BRIDGE_LINE_WIDTH_MM`) is the only lateral
reference and dead reckoning alone would drift off the edge.

## Hardware interface

Board: 8-channel line-follow module labelled **QYF-750** (no public datasheet
was found; the name comes from the original driver PR). Silkscreen pinout
`GND, IR, D1…D8, VCC`. The silkscreen says `D`, but each output is an analog
voltage.

| Array pin | Mega | Register |
|---|---|---|
| D1 … D8 | A0 … A7 | `ADC0` … `ADC7` = `PF0` … `PF7` |
| IR | 36 | `PC1` |
| VCC | 5 V | — |
| GND | GND | — |

Each output is a phototransistor collector with an on-board pull-up (inferred
from a bench probe as 5-14 kΩ, probably 10 kΩ), so the voltage **falls as reflected light rises**: ~1020 counts with
nothing in range, lower over white. The Mega's own pull-ups stay off; they would
shift every reading by about 20%.

`IR` is the emitter enable: driven HIGH the emitters are on, LOW they are off.
The module also pulls it up, so a floating pin lights them too; the driver
drives it anyway so the state is firm. The emitters draw roughly 140 mA from
`VCC` (an estimate, not measured).

## Interface

- `begin()` - takes over PORTF and the ADC, turns the emitters on and waits for
  them to settle. Call once.
- `read(counts)` - one 10-bit conversion per channel into `uint16_t[8]`, index 0 =
  A0 = D1. Called once per tick from `readSensors()` into `RobotState::irRaw`.
- `setEmitters(on)` - switches the emitter bank and waits `IR_EMITTER_SETTLE_US`
  if the state changed, so the next `read()` is valid.

## How it works

`begin()` clears `DDRF`/`PORTF`, sets `DIDR0 = 0xFF` (the pins are analog-only,
so `PINF` reads 0 from now on), selects the AVcc reference, enables the ADC at
F_CPU/`IR_ADC_PRESCALER` (500 kHz) and does one throwaway conversion. `read()`
clears `MUX5`, writes `ADMUX`, starts a conversion, polls `ADSC` and reads `ADC`,
channel by channel. There is no ISR.

The wiring assumptions are `static_assert`-ed against `RobotConfig.h`: the
channels are A0-A7 in order, the emitter enable is pin 36, and the prescaler is a
power of two giving an ADC clock of at most 1 MHz, the highest value checked.

## Invariants

- A0-A7 stay on `ADC0`-`ADC7` in order. Moving a channel means changing the
  driver, and the build fails until the assertion is updated with it.
- This module owns the ADC registers. A boot-time `analogRead(PIN_POT)` in
  `main.cpp` is fine: every conversion here re-selects the reference, the channel
  group (`MUX5`) and the channel, because the Arduino core leaves `MUX5` set after
  reading A8-A15. A second register-level ADC driver must declare
  `// pin-check: shared adc - <reason>` in both modules.
- `read()` is valid only after the emitters have settled, which `begin()` and
  `setEmitters()` guarantee. `setEmitters()` works only after `begin()` has made
  pin 36 an output.
- Counts are ratiometric to AVcc: power the module from the same 5 V rail as the
  Mega. A separate supply shifts every reading.

## Status

Compiles under `-Werror`, and the pin-map, banned-pattern and ISR checks pass.
The `static_assert`s were checked by breaking each assumption in turn. Verified
on a bench board (details and numbers in the ADR):

- registers as intended: `ADCSRA = 0x95`, `DIDR0 = 0xFF`, PC1 an output driven
  HIGH, pull-ups off;
- emitters off reads 1023 on all channels, emitters on reads real counts;
- `read()` takes 242 µs; `setEmitters()` costs 1 µs when nothing changes and ~1 ms
  when it does;
- all 8 channels respond in order, with white ~350-650 and black paper 956-1011 at
  ~3 mm, and white 958-978 against black 1019 at 22.77 mm.

A host build of the driver against a fake ADC shows `read()` still converts ADC0-7
when `MUX5` was left set the way `analogRead(A8)` leaves it. That is a simulation of
the register semantics; it is not yet confirmed on the board.

Still to check: the real arena line material (black tape can reflect far more than
black paper), sensitivity to height and tilt (the bridge incline changes both),
IR-rich ambient light, emitter current draw, true optical crosstalk, and a second
board.

## Known limitations

- **`read()` blocks** ~240 µs of the 10 ms tick.
- **The channels are not sampled in one instant**: the sweep spans ~0.24 ms, about
  0.24 mm of travel at 1 m/s.
- **Nothing interprets the counts yet.** There is no calibration and no position
  math; both wait for the real arena material and the final mount height.
- **Ride height is unresolved** (`IR_RIDE_HEIGHT_MM`, `<<TBD HARDWARE>>`). The
  white/black contrast was about half of full scale at 3 mm and 4-6% at 22.77 mm,
  and it falls steeply with height.
- **No ambient rejection.** An emitters-off/on comparison is possible (settling
  makes it cost ~1.3 ms per scan) and is not built.
- **A0-A7 are not usable as GPIO** while `DIDR0` is set.
