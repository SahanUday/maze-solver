# 0003: 4x quadrature decoding for the wheel encoders

- **Status:** Accepted
- **Date:** 2026-10-03

## Context

The JGA25-370 encoder outputs 2-channel quadrature, 11 pulses per *motor* shaft
revolution before the gearbox. The team's earlier convention, and the motor/encoder
design notes' default, counts rising edges of channel A only (1x) and takes
direction from the sign of the commanded PWM.

The control loop runs at 100 Hz, so a speed PID sees `counts per 10 ms tick`.
With an *assumed* 1:50 gearbox and 65 mm wheel (neither measured yet) at
0.3 m/s:

| Decode | Counts per wheel rev | Counts per 10 ms tick |
|---|---|---|
| 1x (A rising) | 550 | about 8 |
| 4x (all edges) | 2200 | about 32 |

At about 8 counts a tick one count is a ~12% speed step, too coarse for PID.
Assumed-direction counting also cannot see a wheel pushed backwards or slipping.

## Decision

Decode all four edges of both channels (4x), with true direction. Each ISR reads
the motor's two channels in one port read and applies `quadratureDelta(prev, curr)`
(`include/Quadrature.h`): +/-1 for a legal Gray-code step, 0 for no change.

A two-state jump (a missed edge or noise) counts 0 rather than guessing a
direction.

## Consequences

- Needs a usable channel B on both motors. If a unit cannot provide one,
  supersede this decision with A-only counting.
- Encoder counts per wheel revolution = `11 x gear ratio x 4` (11 is the
  supplier's per-motor-shaft figure; confirm by measurement). The
  `ENCODER_COUNTS_PER_REV` and `MM_PER_ENCODER_COUNT` constants must assume 4x
  when they are filled in. Switching decode mode later would silently rescale
  them, so this is fixed here.
- ISR load: four vectors, about 70 instructions each as compiled (counted from
  the disassembly, not timed on hardware). At the assumed 0.3 m/s that is a few
  thousand interrupts a second per wheel; the real cost depends on the measured
  speed and should be re-checked once the gear ratio is known.
- Encoder direction needs a per-side invert flag (`ENC_*_INVERT`) because the
  motors are mounted mirrored.
- Internal pull-ups are enabled on all four pins, so open-collector and
  push-pull encoder outputs both work.

## Alternatives considered

- **1x, channel A only** (the earlier default): cheapest, but too coarse for the 100 Hz
  PID and blind to reverse motion.
- **2x: A on CHANGE, B polled in the ISR:** frees two interrupt pins, halves the
  resolution gain, and still reads B at a different instant from A's edge.
- **PCINT2 bank, one ISR for all four channels:** see decision 0002.
