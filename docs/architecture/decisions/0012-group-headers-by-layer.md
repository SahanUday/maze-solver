# 0012: Group the pure-logic headers by layer

- **Status:** Accepted
- **Date:** 2026-10-17

## Context

`include/` held 25 headers in one folder: pin and tuning constants, sensing, motion
control and the mission state machine side by side. Nothing in the tree showed which
layer a header belonged to, though the include graph already had one: the mission
includes motion and sensing, motion includes the heading and profile, and sensing
reads `RobotState`.

## Decision

Group the headers in `include/` by layer, and include them by path
(`#include "motion/Heading.h"`):

| Folder | Holds |
|---|---|
| `config/` | pin assignments (`RobotConfig.h`) |
| `core/` | the hardware-to-logic channel and the tick scheduler (`RobotState.h`, `Scheduler.h`) |
| `sense/` | encoders, ultrasonic, IR array, wall, incline and floor-pattern logic, `Perception.h` |
| `motion/` | heading, profile, controller, moves, motor duty split, line follower |
| `mission/` | the cell step, the Section A state machine, the run loop |

`RobotSpec.h` stays at the top of `include/` for now. Each header is included by its
folder-qualified path, so no extra `-I` flag is needed. Filenames and contents are
unchanged.

## Consequences

- A header's layer is visible from its path, and an include that points the wrong way
  (`motion/` including `mission/`) stands out in review.
- Every include of a moved header changed; tools that name a header by path
  (`.pre-commit-config.yaml`, `scripts/check-pin-map.py`) changed with it.
- Earlier ADRs name headers by their old flat path (`include/Quadrature.h`); they
  are left as written.

## Alternatives considered

- **Leave it flat.** Cheapest, but the layering stays implicit.
- **One PlatformIO library per layer under `lib/`.** Gives enforced boundaries, but
  each library needs a `library.json` and its own build flags (see the `lib/maze`
  notes in `platformio.ini`), which is more machinery than 25 headers justify.
- **Add `-Iinclude/motion` etc. and keep bare includes.** Shorter lines, but the
  layer disappears from the call site and header names can collide.
