# Architecture

Current-state reference for the maze-solver firmware: what's actually decided and
built, not a plan for future work. This file is updated in place as the system
changes. For the reasoning behind a specific choice, see `decisions/`. For a
specific module's design, see `modules/` — created only once that module exists.

## Hardware abstraction: hybrid, not all-or-nothing

The competition rules require the provided Arduino Mega **board**, not any
particular software framework on it. We use the Arduino framework
(`framework = arduino`) for the toolchain, `setup()`/`loop()` glue, and
non-critical I/O — it gives us a working `Serial`, `Wire.h`, and vetted
libraries (e.g. MPU-6050 fusion) without reinventing them under deadline
pressure.

Timing-critical / performance-critical HAL modules (motor PWM, encoders,
ultrasonic timing, IR array ADC muxing) bypass the Arduino I/O primitives
entirely and access AVR registers directly (`DDRx`/`PORTx`/`TCCRn`/`ADMUX`/
`EICRA`/etc.) instead. `digitalWrite`/`digitalRead`/`analogRead` each cost
roughly 20-60x a direct register access, and a naive sequential `analogRead()`
sweep of the 8-channel IR array alone would burn a meaningful slice of a 10ms
control-loop tick.

Full reasoning, benchmarks, and the alternatives considered:
[`decisions/0001-hybrid-hardware-abstraction.md`](decisions/0001-hybrid-hardware-abstraction.md).

## Documentation structure

This directory is deliberately split by how often each piece changes:

- **`ARCHITECTURE.md`** (this file) — current-state facts, edited in place.
- **`decisions/`** — one immutable ADR per specific choice; superseded, never
  rewritten, so the reasoning trail survives a change of mind.
- **`modules/`** — one living doc per real module, added alongside that
  module's first working code, never ahead of it.

This project does not maintain a separate long-range roadmap document as a
source of truth — see the closed
[build-roadmap issue](https://github.com/SahanUday/maze-solver/issues/5) for
why: a detailed upfront plan (Big Design Up Front) drifts out of sync with
reality faster than it gets updated. We document what's real, when it becomes
real (Just Enough Design Up Front).

---

*More sections land here as they're established by the PRs that build them —
this file intentionally does not yet describe work still in flight.*
