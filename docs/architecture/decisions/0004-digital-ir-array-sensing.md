# 0004: Digital IR array sensing — single-port read, no calibration

- **Status:** Accepted
- **Date:** 2026-10-04

## Context

The floor sensor is an **8-channel IR array with digital (comparator) outputs**
— one bit per channel, against a reference fixed by on-board resistor dividers.
There are no trimpots: the threshold is not adjustable at all. This is not the
analog (QTR-8A style) array that
[`0001-hybrid-hardware-abstraction.md`](0001-hybrid-hardware-abstraction.md)
assumed when it listed `line_sensors` as "registers only" to avoid
`analogRead()`'s ~104µs-per-channel cost. There is no ADC in the signal path, so
that rationale no longer applies to this peripheral.

ADR 0001's **mandate** still holds, for a different and simpler reason. On the
ATmega2560, `A0`–`A7` are `PF0`–`PF7` — one contiguous port. A single `PINF`
read captures all 8 channels **in the same clock cycle**. Eight `digitalRead()`
calls cannot: they smear the samples across several microseconds, long enough at
speed for the resulting pattern to be internally inconsistent. Atomicity, not
throughput, is what forces the register read here.

### Bench measurements

A throwaway diagnostic (not in this repo) was uploaded with the array wired to
`A0`–`A7`:

| Observation | Conclusion |
|---|---|
| Covering D1/D2/D3 drove `A0`/`A1`/`A2` low | channel order is 1:1, not mirrored |
| Only ever one bit low at a time | no crosstalk between adjacent channels |
| Idle held `0xFF` for tens of seconds, internal pull-ups off | lines are actively held high |
| Enabling internal pull-ups changed nothing | internal pull-ups unnecessary |
| Sustained covers gave 8–18 identical samples | settled state is clean; chatter is a boundary effect |

**The emitters were unpowered for all of the above.** The module's `IR` pin had
been left unconnected, so the phototransistors saw only ambient light — which in
a bright room saturates them. That is why every surface read `0xFF`, why white
and black cards were indistinguishable, and why no surface looked different from
any other.

`IR` turned out to be a **high-impedance logic enable, active HIGH**, feeding an
on-board transistor. The decisive observation: a ~30kΩ internal pull-up — at
most 0.17mA — lights the emitters at full brightness, which eight LEDs sharing
that current could never do. LED current comes from `VCC` through the board's
own 220Ω network, so an ordinary GPIO drives the enable directly.

With the emitters on, **polarity measured cleanly**: a white card at working
height reads **LOW**, open air reads **HIGH**. Reflection drives a channel low,
so black reads high. The earlier "finger over sensor reads LOW" result was a red
herring — with dark emitters the finger was shading ambient IR, not reflecting.

### Ambient light

An analog array rejects ambient by reading every channel with emitters off, then
on, and subtracting. With one bit per channel there is no magnitude to subtract,
so the full trick is unavailable.

A **one-bit approximation** exists, because the emitter enable is
GPIO-controllable: the channels whose bit *changes* between an emitters-off and
an emitters-on read are the ones genuinely seeing reflected light. Coarser, same
target. Not built in this PR.

## Decision

### Interface and wiring

- `PIN_IR[8] = {A0..A7}` confirmed as `PF0..PF7`, physical left to right,
  promoted `[PROPOSED]` → `[FIXED]`. **The array must stay on one contiguous
  port** — scattering channels forfeits the atomic read permanently.
- The driver reads `PINF` directly; `PIN_IR[]` documents the wiring. The two
  must change together.
- Internal pull-ups stay **off** (`DDRF = 0x00`, `PORTF = 0x00`).
- `PIN_IR_EMITTER` = pin 36 = `PC1`, output, set HIGH in `begin()`. **Without
  this the emitters stay dark and every channel reads the same regardless of
  what is under the array** — the failure behind every inconclusive measurement
  above. `setEmitters()` is exposed for the ~140mA saving during walled-maze
  phases, and for the ambient subtraction above.

### Polarity

Bench-measured and recorded as `IR_BLACK_IS_HIGH = true` in `RobotSpec.h`, but
**not applied by the driver**: `read()` returns the levels as the pins present
them, so `irRaw` is literally what the hardware says. Whatever interprets those
bits later owns the conversion.

### Debounce

Sample the port `IR_DEBOUNCE_SAMPLES` (3) times at `IR_DEBOUNCE_SPACING_US`
(100µs) and take a per-channel majority vote; the count is `static_assert`-ed
odd so it cannot tie. ~200µs of a 10ms tick (2%), affordable only because there
is no ADC. It adds no latency, unlike filtering across ticks.

### Mask → position

**Deferred.** The driver stores raw channel bits in `RobotState::irRaw` and
nothing interprets them. Turning a bit pattern into a position — and deciding
what empty, saturated or discontiguous patterns mean — waits until the array is
shown to work at its mounted height. Designing it now would be designing against
an unproven sensor.

### Calibration

**There is none, and none is possible.** No trimpots, and a digital array gives
no magnitude to calibrate against in software. This cuts both ways: nothing can
drift, and nothing can be corrected at the venue either.

What remains adjustable is entirely physical:

1. **Ride height** — the dominant variable, and a specification rather than a
   preference (`IR_RIDE_HEIGHT_MM`, `<<TBD HARDWARE>>`). Reflectance falls off
   sharply with distance, so height is what positions the real signal relative
   to a threshold that cannot move. Currently 22.77mm, which is expected to be
   well outside the usable 3–8mm range and is **not yet tested**.
2. **A shroud** against ambient IR.
3. **Early testing on arena-like material**, since nothing can be tuned later.

## Consequences

**Good**

- All 8 channels in one atomic instruction, versus 832µs for a naive analog
  sweep. The real-time argument driving ADR 0001 evaporates for this peripheral.
- No ADC, no ISR, no `ADMUX`/`ADCSRB` bank juggling, and no interaction with the
  upper-bank ADC users (`PIN_POT = A8`, `PIN_VBAT_SENSE = A9`).
- `RobotState` shrinks: `uint16_t irRaw[8]` (16 bytes) → `uint8_t irRaw` (1).
- Oversampling for debounce is nearly free, which it would not have been with an
  ADC in the path.

**Bad**

- **No threshold adjustment of any kind.** Whether the array can see the line at
  all comes down to mechanical setup against a fixed reference. This is the
  design's largest single risk: there is no in-the-field remedy, and it will not
  be known for certain until the array is tested at its mounted height on
  arena-like material.
- **Coarser resolution.** One bit per channel, so whatever consumes `irRaw`
  later sees a stepped signal; expect to need more derivative damping than an
  analog array would.
- **Weaker ambient rejection** — only the one-bit comparison above, not true
  subtraction.
- The emitter enable is load-bearing: forget to assert it and the array fails
  *silently and uniformly* rather than loudly. `begin()` owns it.

## Alternatives considered

**Swap to an analog array (QTR-8A).** Restores continuous position, software
calibration in EEPROM, and true ambient subtraction. Rejected for now because
the digital array is the hardware in hand and the bridge is its only consumer;
revisit via a superseding ADR if the mounting height, the shroud and the one-bit
subtraction together prove insufficient.

**Debounce across ticks instead of within one.** Zero blocking time, but 3 ticks
(30ms) of latency on every edge. Rejected: 200µs of a 10ms tick is cheaper, and
latency costs more than CPU in a steering loop.

**Read channels with eight `digitalRead()` calls.** Rejected per ADR 0001, and
independently because it is not atomic — see Context.
