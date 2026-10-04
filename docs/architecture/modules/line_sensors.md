# `line_sensors` — 8-channel digital IR array

HAL driver for the floor-sensing IR array. Decision record and bench
measurements:
[`../decisions/0002-digital-ir-array-sensing.md`](../decisions/0002-digital-ir-array-sensing.md).

## What it is for

The walled maze sections are navigated by ultrasonic ranging plus encoder/IMU
odometry. The IR array exists for the **bridge** between Section A and Section
B, where a 30mm black line (`BRIDGE_LINE_WIDTH_MM`) is the only lateral
reference and dead reckoning alone would drift off the edge.

## Hardware interface

Board: **QYF-750**, 8-channel line-follow module. Silkscreen pinout
`GND, IR, D1…D8, VCC`.

| Array pin | Mega | Register |
|---|---|---|
| D1 … D8 | A0 … A7 | `PF0` … `PF7` |
| IR | 36 | `PC1` |
| VCC | 5 V | — |
| GND | GND | — |

`begin()` sets `DDRF = 0x00` / `PORTF = 0x00` (inputs, internal pull-ups off —
the array holds its own lines high), then asserts the emitter enable.

**A0–A7 must stay on one port.** `read()` takes a single `PINF` read, so all 8
channels are sampled in the *same clock cycle*. This is about atomicity, not
speed: eight `digitalRead()` calls spread the samples over several microseconds,
and at speed that produces a pattern the floor never had.

`PIN_IR[8]` in `RobotConfig.h` documents the wiring; the driver reads the port
directly. The two must change together.

### The `IR` pin is not optional

**If pin 36 is not driven HIGH the emitters stay dark and every channel reads
the same regardless of what is under the array.** The failure is silent and
uniform — the board looks electrically healthy and simply reports `0xFF`
everywhere. `begin()` owns this; don't remove it.

`IR` is a high-impedance logic enable, active HIGH, feeding an on-board
transistor — not the LED supply rail. A ~30 kΩ internal pull-up (≤0.17 mA)
lights the emitters fully, which eight LEDs sharing that current could never do.
LED current comes from `VCC` through the board's own 220 Ω network, so pin 36
drives the enable directly: no MOSFET, no series resistor.

`setEmitters(false)` saves roughly 140 mA during phases that don't need the
floor.

## Debounce

`read()` samples `PINF` 3 times at 100 µs spacing and takes a per-channel
majority vote. The count is `static_assert`-ed odd so the vote cannot tie.

Costs ~200 µs of a 10 ms tick (~2%), and **blocks** for it. Affordable only
because there is no ADC in the path. Filtering across ticks instead would cost
no CPU but add 30 ms of latency on every edge — the wrong trade in a steering
loop.

Chatter is a threshold-*boundary* effect: the bench trace showed 8–18 identical
consecutive samples once a channel was fully covered. The vote exists for the
moment a line edge crosses a sensor, not for the settled state. It only catches
chatter faster than its 200 µs window; a channel ambiguous for milliseconds
votes consistently for the wrong answer and passes straight through.

## Polarity

Measured with the emitters on: a white card at working height reads **LOW**,
open air reads **HIGH**. Reflection drives a channel low, so black — which
absorbs — reads high. Recorded as `IR_BLACK_IS_HIGH = true` in `RobotSpec.h`.

The driver does **not** apply it. `read()` returns levels exactly as the pins
present them, so `irRaw` is what the hardware says and nothing more. Whatever
interprets those bits later owns the conversion.

An earlier reading of "finger over sensor → LOW" pointed the opposite way and
was a red herring: the emitters were unpowered, so the finger was shading
ambient IR rather than reflecting anything back. **Any polarity measurement
taken with dark emitters is meaningless.**

## Position math

**Not built.** `irRaw` holds raw channel bits and nothing interprets them.
Turning a bit pattern into a lateral position waits until the array is shown to
work at its mounted height.

## Integration

`readSensors()` in `src/main.cpp` calls `read()` once per tick and writes
`state.irRaw`. Nothing above the HAL touches a register.

## Known limitations

- **No calibration mechanism at all.** No trimpots; the comparator reference is
  fixed by on-board resistor dividers, and a digital array offers no magnitude
  to calibrate in software either. Nothing can drift, but nothing can be
  corrected at the venue either.
- **Ride height is the only adjustment**, and it is unresolved. The array is
  currently mounted at 22.77 mm. Reflective sensors of this type work at roughly
  3–8 mm, and returned signal falls off as about 1/d⁴ — so this height is
  expected to be well outside the usable range. Untested at the mounted height;
  `IR_RIDE_HEIGHT_MM` stays `<<TBD HARDWARE>>` until it is.
- **Weaker ambient-light rejection** than an analog array: one bit per channel
  leaves no magnitude to subtract. A one-bit approximation is available and not
  built — read `PINF` with emitters off, read again with them on, and the
  channels that *changed* are the ones seeing reflected light.
- **`read()` blocks** ~200 µs.
