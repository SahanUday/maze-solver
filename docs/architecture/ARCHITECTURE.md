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
   `line_sensors`, `imu`, ...). Only layer allowed to touch hardware, per the
   hybrid policy above. `imu` is the one module that takes the framework
   exception rather than the register rule: it drives the MPU-6050 through
   `Wire.h`, and is also the only driver whose device can be absent, so both of
   its entry points return success/failure instead of a reading
   ([`decisions/0008`](decisions/0008-mpu6050-heading-and-tilt.md)).
2. **Control/algorithm layer** — works only through the HAL's function API
   and plain data. `lib/maze` and the pure-logic headers in `include/`
   (`core/`, `sense/`, `motion/`, `mission/`) live here and must never include
   `Arduino.h` (see "Dual build target" below).
3. **Sequencing layer** — `src/main.cpp` and `RunLoop`
   (`include/mission/RunLoop.h`), the top-level run loop. Wires the other two
   layers together once per tick.

## Fixed-period control loop

`src/main.cpp`'s `loop()` runs `readSensors()` then `runAlgorithm()` once
every `CONTROL_LOOP_PERIOD_MS` (10ms / 100Hz — `include/spec/fixed/Hardware.h`). The
tick-due/drift-resync logic itself lives in `include/core/Scheduler.h`
(`tickDue()`), not inline in `main.cpp` — it's plain arithmetic with no
hardware dependency, so keeping it Arduino-free is what lets it run under
`env:native`'s test suite instead of only being checked by reading it.
`test/test_scheduler/` covers the jitter, catch-up-resync, and
`millis()`-wraparound cases directly.

## Application glue

`src/app/` holds what `main.cpp` would otherwise carry: `heading_estimator` (the IMU, the gyro
bias captured at boot and the yaw integration), `telemetry` (the status line and the phase-change
line over serial) and `operator_io` (the start button and the two LEDs). It is non-critical glue
in the sense of ADR 0001, so it uses the Arduino framework (`Serial`, `pinMode`, `digitalRead`,
`digitalWrite`) rather than registers, and nothing in it is timing-critical. `main.cpp` keeps the
driver start-up order, `readSensors()`, `runAlgorithm()` and the tick.

## HAL modules

Register-level drivers live in `src/hal/` (not `lib/`). Code in `src/` is built
with `-Wextra`, `-Werror` and `-fno-exceptions` on top of the default `-Wall`,
so a warning fails the build; `env:mega` adds `-fno-rtti` and
`-Wstack-usage=128`. `lib/maze` opts into the same through its `library.json`; the flags are
not global because they would also hit the Arduino core. `lib/maze` is
header-only today, so nothing is compiled with those flags yet — a header is
built under the flags of whatever includes it. They take effect when the
library gains its first `.cpp`. The HAL access policy of ADR 0001 (registers only, no
`digitalWrite`/`analogRead`/`attachInterrupt` and the like) is checked
automatically; see the [testing guide](../testing.md).

Logic that needs no hardware (`Quadrature.h`, `MotorDrive.h`, `LineSense.h`,
`Heading.h` and the motion headers listed under "Control layer" below) stays in
`include/` so `env:native` can test it. A header there
with function bodies in it must appear in the coverage report or declare
`// coverage-exempt: <reason>`; see the [testing guide](../testing.md).

`include/` is grouped by layer and headers are included by path
(`#include "motion/Heading.h"`; [ADR 0012](decisions/0012-group-headers-by-layer.md)):
`core/` (`RobotState`, `Scheduler`, `MathUtil`), `sense/`, `motion/`, `mission/`, and `spec/` for
the constants ([Spec layout](#spec-layout)).

Each HAL `.cpp` hand-maps registers to specific pins and `static_assert`s the
`spec/pins/PinMap.h` pin constants it depends on, so moving a pin without updating
the driver fails the build. The allocation is summarised under
[Hardware allocation](#hardware-allocation) below.

State shared between an ISR and the main loop is a `volatile` variable private
to its driver file, read through an accessor under `ATOMIC_BLOCK` (an AVR
load or store wider than a byte can be torn by an interrupt). This is checked
automatically, not left to review; see the [testing guide](../testing.md).

## Hardware allocation

Current state. Reasoning lives in
[`decisions/0002`](decisions/0002-drive-and-sensing-hardware-allocation.md) for
drive and sensing and
[`decisions/0007`](decisions/0007-non-blocking-ultrasonic-ranging.md) for the
ultrasonic echo path; per-module detail is in [`modules/`](modules/).

### Ports

| Pins | Port bits | Owner |
|---|---|---|
| A0–A7 | `PF0`–`PF7` | IR array channels, read as one `PINF` |
| 36 | `PC1` | IR array emitter enable |
| 30 / 32 / 34 | `PC7` / `PC5` / `PC3` | ultrasonic triggers |
| A10 / A11 / A12 | `PK2` / `PK3` / `PK4` | ultrasonic echoes |
| A8 / A9 | `PK0` / `PK1` | pot, battery sense (analog) |
| 2 / 3 / 18 / 19 | `PE4` / `PE5` / `PD3` / `PD2` | encoders |
| 20 / 21 | — | I2C |

`PORTC` is shared between the ultrasonic triggers and the IR emitter enable, so
every write to it is read-modify-write; a plain assignment from one driver would
switch off the other's pin. `PORTF` must stay whole: the IR driver reads all
eight channels in a single `PINF`, and splitting them across ports loses that.

### Timers

| Timer | Owner |
|---|---|
| 0 | `millis()` — Arduino core |
| 1 | motors, right (`OC1A`/`OC1B`) |
| 2 | unused; 8-bit, so unusable as a 16-bit time base |
| 3 | free, but its outputs are pins 5/2/3 and 2/3 are the left encoder |
| 4 | motors, left (`OC4A`/`OC4B`) |
| 5 | ultrasonic time base — `TCNT5` only, no timer pin |

A 16-bit timer's temp register is per timer, so the motor timers are unaffected
by the ultrasonic driver. Any main-context access to `TCNT5`/`OCR5x`/`ICR5`
needs `ATOMIC_BLOCK`.

### Interrupts

All six external-interrupt pins are taken: `INT2`–`INT5` by the encoders,
`INT0`/`INT1` by I2C. Serial1 is unavailable as a result.

`PORTC` has no pin-change or external interrupt at all, which is why the echo
lines sit on `PORTK`. The three echoes share the `PCINT2` vector as
`PCINT18`–`PCINT20`, and `PCMSK2` admits only the sensor currently ranging —
which is what keeps `PK0`/`PK1` usable as analog inputs.

A future pot or battery driver on A8/A9 must not write `DIDR2` wholesale:
setting bits 2–4 (`ADC10`–`ADC12`) disables the digital input buffers on the
echo pins, and `PINK` then reads 0 for them.

## Driver/algorithm boundary — RobotState

`include/core/RobotState.h` is the only channel between hardware and logic.
Drivers (called from `readSensors()`) are the only code that writes it, once
per tick. Algorithm code (`runAlgorithm()`, and everything in `lib/maze`)
only reads it — never a pin, `analogRead`, `digitalWrite`, or `millis()`
directly.

This is what lets `lib/maze` compile and run its unit tests on a laptop
(`env:native`) with no robot attached: its input is a plain struct, not a
pin.

## Spec layout

`include/spec/` holds every constant, grouped by what you would do if a value were wrong
([ADR 0020](decisions/0020-group-the-spec-constants-by-why-they-change.md)):

| Folder | Holds | Changes when |
|---|---|---|
| `pins/` | `PinMap.h`: every pin number | the robot is re-wired |
| `measurements/` | `Arena.h` (tiles, walls, markers A and B, bridge floor), `Robot.h` (wheels, sensor mounting and offsets, polarity, IMU orientation) | something is re-measured |
| `tuning/` | one header per logic module: `Motion`, `WallFollow`, `LineFollow`, `LineSensor`, `FloorPattern`, `Imu`, `CellExplorer`, `MarkerExit`, `Run` | the robot is run and adjusted |
| `fixed/` | `Rules.h` (competition limits), `Hardware.h` (clock, PWM, chip and module settings) | never: rules and datasheets |

- A file includes only the headers whose constants it uses. Includes point one way: `tuning/` to
  `measurements/` to `fixed/`; `pins/` and `fixed/` include nothing of the spec.
- Every constant is in exactly one file. Marker A, marker B, the bridge-out line and the bridge-in
  line have separate constants even where the rules say they match: the arena is hand-built.
- The headers are Arduino-free (`PinMap.h` writes `A0`-`A15` as 54-69), so the host builds and
  `lib/maze` can include them.
- `constexpr`, never `#define`; the unit is in the name (`_MM`, `_MS`, `_US`, `_DEG`, `_MV`, `_PWM`).
- Each section heading carries a provenance tag: `[RULES]` (competition rules, nominal), `[DESIGN]`
  (datasheet or design choice), `[MEASURED]` (measured on the robot or arena), `[TUNED]` (tuned on the
  robot), `[HOST MODEL]` (found in the host simulation, not yet checked on the robot). Pin headings
  use `[FIXED]` / `[PROPOSED]` instead (explained in `PinMap.h`).
- A value not yet measured (`<<TBD HARDWARE>>`) or tuned (`<<TBD CALIBRATION>>`) is named now with
  its real unit and a stand-in value, and listed by `scripts/list-tbds.sh` rather than in a comment
  someone has to happen to read.

## Control layer

Motion control, perception and the cell step are pure headers in `include/`, no `Arduino.h`, tested on
the host (the motion headers against a simulated plant, `test/support/sim.h`). `RunLoop` calls them
every tick from `main.cpp`, and the Section A run, line follower included, ran on the robot at the
mid-evaluation (tag `v0.1-section-a`), with a stored IR calibration that `main.cpp` loads at boot.
Per-module detail is in [`modules/`](modules/); the measured values are in `include/spec/measurements/`.

| Piece | File | Role |
|---|---|---|
| Odometry | `include/sense/Odometry.h` | encoder counts to distance and speed |
| Profile | `include/motion/MotionProfile.h` | trapezoid speed profile, stepped on a reference trajectory |
| Controller | `include/motion/MotionControl.h`, `include/motion/MotionTypes.h` | linear PID on the odometer + angular PID on the gyro, mixed into left and right duty |
| Moves | `include/motion/MotionPrimitives.h` | straight run along a held heading; in-place turn |
| Output | `include/motion/DriveCommand.h` | the `{left, right, enable}` handed to the motors |
| Perception | `include/sense/Perception.h` | once per tick, `RobotState` in and one derived `Perception` out: distance, speed, yaw rate, incline, the wall view and the line view |
| Cell step | `include/mission/StepCell.h` | one move to the next cell centre that votes on the cell's three walls on the way, can carry on through the centre without stopping, anchors its stop on a wall ahead, and hands back the corridor's measured heading as a gyro correction |
| Cell explorer | `include/mission/CellExplorer.h` | explores a maze cell by cell (sense, turn, step) through the cell step and `lib/maze`, generic in the maze size; the caller can ask it to stop on a tile or to drive to a chosen one over checked edges |
| Marker exit | `include/mission/MarkerExit.h` | tells the approach marker, finds the way out of its tile and drives over the gap to the line |
| Mission manager | `include/mission/MissionManager.h`, `MissionParams.h`, `Parts.h`, `RunStart.h` | the run, part by part ([0016](decisions/0016-cut-the-run-into-parts.md)): starts it, runs the parts from a first to a last (the whole course, one part, or a stretch), enforces the run limit and owns the one motion controller |
| Run loop | `include/mission/RunLoop.h` | one tick of a run: perception, then the mission manager; `src/main.cpp` applies it to the motors |
| Wall sensing | `include/sense/WallSense.h` | ultrasonic readings to "wall or open" per side, and a heading trim that centres the robot |
| Incline | `include/sense/Tilt.h` | nose-up slope in accelerometer counts from the forward axis, and a level capture that nothing calls yet (the bench level is used) |
| Line view | `include/sense/LineView.h` | one sweep of the IR array to where the line is |
| Line follower | `include/motion/LineFollow.h` | steers on the line view's centroid as a heading offset; searches for a lost line toward the side it was last seen; ignores an all-white view on the ramp |
| Floor pattern | `include/sense/FloorPattern.h` | the IR array's floor class (black, white, air, mixed) along the path: the approach marker and the gap before the ramp |

## Dual build target

`platformio.ini` defines four environments, all extending `[common]`
(`-std=gnu++17`, `-Wall`, `-Wextra`, `-Werror`, `-fno-exceptions`) so none
drifts out of parity
with the others:

- `env:mega` — the real robot. The platform and its framework and toolchain
  packages are pinned to exact versions.
- `env:native` — host-PC build for `lib/maze` and the pure-logic headers in
  `include/`, which must never include `Arduino.h`.
- `env:native_san`, `env:native_cov` — the same host tests built with
  sanitizers, and with coverage instrumentation. What they are for is in the
  [testing guide](../testing.md).

## Continuous integration

`.github/workflows/ci.yml` runs on every pull request and every push to `main`:
`lint` and `static-checks` first, then `native` and `avr-build` in parallel,
then `CI gate`. The gate is the one required status check; it always reports, so
a docs-only change passes without running the heavy jobs. Beside it, and not part
of it, an advisory `report` job comments the firmware size change and the open
`<<TBD>>` placeholders on each PR. `pr-title.yml` (the PR title check) and
`nightly.yml` (slow extras) are separate workflows because their triggers
differ. What each check does, how to run it locally and what to do when it
fails is in the [testing guide](../testing.md).

---

*More sections land here as they're established by the PRs that build them —
this file intentionally does not yet describe work still in flight.*
