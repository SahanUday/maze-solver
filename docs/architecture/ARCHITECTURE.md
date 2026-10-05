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
source of truth. A detailed upfront plan (Big Design Up Front) drifts out of sync with
reality faster than it gets updated. We document what's real, when it becomes
real (Just Enough Design Up Front).

## Module layering

Three layers, enforced structurally — nothing in layer 2 or 3 may touch a
register or an Arduino I/O call directly:

1. **HAL** — one file pair per peripheral (`motors`, `encoders`, `ultrasonic`,
   `line_sensors`, ...). Only layer allowed to touch hardware, per the hybrid
   policy above.
2. **Control/algorithm layer** — works only through the HAL's function API
   and plain data. `lib/maze` lives here and must never include `Arduino.h`
   (see "Dual build target" below).
3. **Sequencing layer** — `src/main.cpp` and, eventually, the top-level
   run-phase state machine. Wires the other two layers together once per
   tick.

## Fixed-period control loop

`src/main.cpp`'s `loop()` runs `readSensors()` then `runAlgorithm()` once
every `CONTROL_LOOP_PERIOD_MS` (10ms / 100Hz — `include/RobotSpec.h`). The
tick-due/drift-resync logic itself lives in `include/Scheduler.h`
(`tickDue()`), not inline in `main.cpp` — it's plain arithmetic with no
hardware dependency, so keeping it Arduino-free is what lets it run under
`env:native`'s test suite instead of only being checked by reading it.
`test/test_scheduler/` covers the jitter, catch-up-resync, and
`millis()`-wraparound cases directly.

## HAL modules

Register-level drivers live in `src/hal/` (not `lib/`). Code in `src/` is
built with `-Wextra`, `-Werror` and `-Wstack-usage=128` on top of the default
`-Wall`, so a warning fails the build. `lib/maze` opts into
`-Wall -Wextra -Werror` through its `library.json`; the flags are not global
because they would also hit the Arduino core. The banned-pattern scan covers
`src/`, `include/` and `lib/`, and CI's cppcheck covers `src/` and `lib/`.

The HAL access policy of ADR 0001 is enforced by the banned-pattern scan:
`src/hal/` may not call `pinMode`, `digitalWrite`, `digitalRead`, `analogRead`,
`attachInterrupt` or the other Arduino GPIO/ADC/interrupt functions.

Logic that needs no hardware (`Quadrature.h`, `MotorDrive.h`) stays in
`include/` so `env:native` can test it.

Each HAL `.cpp` hand-maps registers to specific pins and `static_assert`s the
`RobotConfig.h` pin constants it depends on, so moving a pin without updating
the driver fails the build. The pin, timer and interrupt allocation is in
[`decisions/0002`](decisions/0002-drive-and-sensing-hardware-allocation.md).

State shared between an ISR and the main loop is a `volatile` variable private
to its driver file, read through an accessor under `ATOMIC_BLOCK` (an AVR
load or store wider than a byte can be torn by an interrupt). This is checked,
not left to review: see Continuous integration.

## Driver/algorithm boundary — RobotState

`include/RobotState.h` is the only channel between hardware and logic.
Drivers (called from `readSensors()`) are the only code that writes it, once
per tick. Algorithm code (`runAlgorithm()`, and everything in `lib/maze`)
only reads it — never a pin, `analogRead`, `digitalWrite`, or `millis()`
directly.

This is what lets `lib/maze` compile and run its unit tests on a laptop
(`env:native`) with no robot attached: its input is a plain struct, not a
pin.

## Config split: pins vs. spec

`include/RobotConfig.h` holds pin assignments only, and needs `Arduino.h` for
the `A0`-`A15` pin-name macros — fine, since it's only ever consumed by
`env:mega`-only code. `include/RobotSpec.h` holds everything else (geometry,
timing, motion, control, PID gains) and is Arduino-free, so `lib/maze` can
include it directly for constants like `TILE_PITCH_MM`/`SECTION_A_SIZE_TILES`
without pulling in a framework the host build doesn't have.

Placeholder values not yet measured (`<<TBD HARDWARE>>`) or tuned
(`<<TBD CALIBRATION>>`) are named now with their real unit, left at `0`, and
tracked via `scripts/list-tbds.sh` rather than a comment someone has to
happen to read.

## Dual build target

`platformio.ini` defines four environments, all extending `[common]`
(`-std=gnu++17`, `-Wall`, `-Wextra`, `-Werror`) so none drifts out of parity
with the others:

- `env:mega` — the real robot. The platform and its framework and toolchain
  packages are pinned to exact versions.
- `env:native` — host-PC build for `lib/maze` and the pure-logic headers in
  `include/`, which must never include `Arduino.h`.
- `env:native_san` — the same tests under AddressSanitizer and UBSan, aborting
  on the first violation.
- `env:native_cov` — the same tests with gcov instrumentation, read by
  `scripts/check-coverage.sh` (90% line floor over `include/` and `lib/`).

## Continuous integration

`.github/workflows/ci.yml` runs on every pull request and on every push to
`main`:

1. `lint` and `static-checks`, in parallel. `lint` is pre-commit's generic
   hooks (whitespace, line endings, clang-format). `static-checks` is this
   project's own rules (banned patterns and the HAL access policy, pin map, ISR
   atomicity, docs drift), the unit tests of those check scripts
   (`scripts/tests/`), and on a PR the check that an Accepted ADR is not edited.
   It also decides whether the change touches code at all.
2. `native` and `avr-build`, in parallel, only if both of the above pass and the
   change touches code: the three native environments above; the `env:mega`
   build, the pin table check against the Arduino core, the SRAM and flash
   budgets (`scripts/check-ram-budget.sh`, `scripts/check-flash-budget.sh`) and
   cppcheck.
3. `CI gate` — the one required status check. It always reports, so a
   docs-only change passes without running the heavy jobs.

The repository checks, all in `scripts/`:

- `check-pin-map.py` — duplicate pins in `RobotConfig.h`, reserved pins, port
  and `INTn`/`OCnX` comments that disagree with the pin, a HAL module that uses
  a `PIN_*` without a `static_assert`, and a timer, interrupt, USART, ADC, TWI
  or SPI driven by more than one module (Timer0 belongs to the Arduino core).
  Deliberate sharing is declared in every module involved with
  `// pin-check: shared <resource> - <reason>`.
- `check-isr-atomicity.py` — a `volatile` wider than one byte that an ISR
  touches may only be accessed inside an ISR, an `ATOMIC_BLOCK`, or a `static`
  helper called only from those. A reviewed exception is marked
  `// isr-safe: <reason>`.
- `check-docs-drift.py` — every `src/hal/<name>.cpp` has
  `modules/<name>.md`, ADR numbers are unique, and an Accepted ADR changes only
  by its Status moving to Superseded.

`.github/workflows/nightly.yml` runs what is too slow for every push: the
build at `-O1`/`-O2`/`-O3`, the host tests repeated 25 times under the
sanitizers, and cppcheck with every check enabled.

---

*More sections land here as they're established by the PRs that build them —
this file intentionally does not yet describe work still in flight.*
