# 0001: Hybrid Arduino-framework / bare-metal-register hardware abstraction

- **Status:** Accepted
- **Date:** 2026-09-24

## Context

The competition provides an Arduino Mega 2560 as the required microcontroller
**board**. Nothing in the rules requires the Arduino **software framework** —
that's a separate choice: PlatformIO's `atmelavr` platform can build firmware
against `framework = arduino` (giving `pinMode`/`digitalWrite`/`Serial`/`Wire.h`/
community libraries) or with no framework at all (raw `avr-gcc` against
`<avr/io.h>`, full register control, no library ecosystem).

The robot's control loop is fixed-period at 10ms (100Hz — see `RobotConfig.h`,
`CONTROL_LOOP_PERIOD_MS`). Each tick needs to: read 8 IR channels, ping 3
ultrasonic sensors, read the MPU-6050 over I2C, read 2 encoders, run PID, and
update motor PWM — all within that 10ms budget, with margin left for
maze-solving compute.

Two options were on the table:
1. **Pure Arduino framework** — fastest to develop against, but its I/O
   primitives are measurably slow: `digitalWrite()`/`digitalRead()` cost
   roughly 20-60x a direct register write/read (a few µs vs. ~1 cycle),
   and a default-prescaler `analogRead()` blocks for ~100-116µs per channel —
   an 8-channel sequential sweep of the IR array alone would cost ~0.8ms,
   ~8% of the entire tick budget, before touching ultrasonic or I2C.
   ([zbotic.in](https://zbotic.in/arduino-port-manipulation-direct-register-control-for-speed/),
   [crash-bang.com](https://www.crash-bang.com/arduino-digital-io/))
2. **Pure bare-metal registers** — full control and the smallest possible
   per-call cost everywhere, but no library ecosystem. In particular, MPU-6050
   heading fusion (DMP upload, or a hand-rolled complementary/Kalman filter)
   would need to be written and debugged from scratch, under the same deadline
   pressure as everything else. A subtly wrong fusion filter fails silently
   (slow heading drift), which is a much worse failure mode than a driver bug
   that crashes loudly.

Neither option is free of risk: (1) risks blowing the real-time budget as more
sensors come online; (2) risks the mid-evaluation deadline if register-level
I2C/ADC/timer work takes longer than expected for a student team of mixed AVR
experience.

Separately: **the actual hard problem in the sensing budget isn't the
framework, it's blocking I/O.** A blocking ultrasonic echo wait
(`pulseIn()`-style, or its register equivalent done wrong) can take tens of
milliseconds at range — several whole ticks — regardless of which framework
issues the wait. That has to be solved with a non-blocking state machine /
interrupt-driven timing capture either way; framework choice doesn't remove
that problem, it only changes how cheap the *rest* of the tick is once that's
handled correctly.

## Decision

Use a hybrid: `framework = arduino` at the project level, but require direct
AVR register access in every timing- or performance-critical HAL module.
Concretely:

| Module | Abstraction level | Why |
|---|---|---|
| `motors` (PWM + direction) | Registers only | Shared-timer duty cycle, no room for `analogWrite()`'s assumptions |
| `encoders` | Registers only | ISR latency feeds odometry accuracy directly |
| `ultrasonic` | Registers only (input capture or PCINT + timestamp) | Must be non-blocking; `pulseIn()` blocks for the full echo window |
| `line_sensors` (8-ch IR) | Registers only (free-running/auto-trigger ADC) | Sequential `analogRead()` costs ~8% of the tick budget every tick |
| `imu` (MPU-6050) | Arduino + vetted library (`Wire.h`) | Fusion-correctness risk outweighs the cycle cost; not in the 100Hz hot path |
| Serial debug, heartbeat LED, buzzer | Arduino calls | Low-frequency, non-critical |
| Button / DIP switches / speed pot | Arduino calls | Read once at boot, never in the hot path |
| Battery monitor | Arduino calls, low sample rate | Fine as blocking `analogRead` if sampled every N ticks, not every tick |
| SD logging | Arduino + library | Bench-only tooling, unplugged for the actual competition run |

## Consequences

- HAL modules (`motors`, `encoders`, `ultrasonic`, `line_sensors`) are written
  and reviewed against register-level correctness (pin/timer/interrupt
  assignment, ADC muxing) rather than library behavior.
- Non-HAL code keeps the convenience and safety of the Arduino framework and
  its libraries, so `Serial`, `Wire.h`, and an MPU-6050 library are available
  without writing/debugging a TWI driver and fusion filter from scratch.
- The `lib/maze` / `env:native` split (see `ARCHITECTURE.md`, once written) is
  unaffected either way — that boundary only depends on `lib/maze` never
  including `Arduino.h`, regardless of what `src/` HAL code does.
- Each HAL module's file gets its own `modules/<name>.md` once it's built,
  which is the natural place to record *why* it's structured the way it is —
  useful both for onboarding and for the individual viva.

## Alternatives considered

- **Pure Arduino framework** — rejected: doesn't reliably fit the sensor mix
  in a 10ms tick once every sensor is online (see Context).
- **Pure bare-metal, no framework** — rejected for this project's timeline:
  the MPU-6050 fusion risk and the team's mixed AVR-register experience make
  a from-scratch driver stack a real threat to the mid-evaluation deadline,
  for a benefit (saved cycles) that only matters in the modules already
  covered by the hybrid's "registers only" column.
- **RTOS (e.g. FreeRTOS on AVR)** — rejected: 8KB SRAM is already tight with
  the Section B (9×9) wall map and flood-fill distance map; a preemptive
  kernel's task-stack overhead isn't justified for a single-loop robot at
  this scale.
- **Second co-processor MCU** (one chip for motor/encoder control, one for
  maze logic) — not available: the competition provides exactly one
  microcontroller, and additional hardware needs prior instructor approval.
