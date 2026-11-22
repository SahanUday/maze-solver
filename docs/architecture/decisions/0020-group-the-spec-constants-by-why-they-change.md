# 0020: Group the spec constants by why they change

- **Status:** Accepted
- **Date:** 2026-11-22
- **Supersedes:** the file layout of [0013](0013-split-the-spec-constants-by-module.md) (one header per
  module stands, inside the folders below) and the `config/` row of
  [0012](0012-group-headers-by-layer.md)

## Context

- [0013](0013-split-the-spec-constants-by-module.md) put one module's constants in one header, but
  they differ in kind: measured on the arena, measured on the robot, tuned on the robot, or fixed by the
  rules and datasheets. Re-measuring the arena meant editing several headers.
- The rules PDF gives nominal sizes and the arena is hand-built. The bridge line is 30 mm in the PDF and
  24 mm measured; the marker is 90 / 30 mm in the PDF and 36-42 / 116-119 / 37-41 mm measured.
  `Competition.h` filed measured values under `[FROM SPEC]`.
- `test/support/world.h` kept private copies of the arena (pitch, wall thickness, marker, line width, the
  gap), so a spec change did not change what the host tests simulate.
- One marker and one line width served the whole course. Marker B and the bridge-in line are separate
  hand-made pieces that nobody has measured.
- `include/config/` held one file, `RobotConfig.h`, kept apart only because it includes `Arduino.h`.

## Decision

`include/spec/` is grouped by what you would do if a value were wrong:

| Folder | Holds | Changes when |
|---|---|---|
| `pins/` | `PinMap.h` (was `config/RobotConfig.h`) | the robot is re-wired |
| `measurements/` | `Arena.h` (tiles, walls, markers, bridge floor), `Robot.h` (wheels, sensor mounting and offsets, polarity, IMU orientation) | something is re-measured: the arena on a visit, the robot on a chassis change |
| `tuning/` | gains, speeds, thresholds, timeouts, retries, per logic module | the robot is run and adjusted |
| `fixed/` | `Rules.h` (competition limits), `Hardware.h` (clock, PWM, chip and module settings) | never: rules and datasheets |

- Includes point one way: `tuning/` may include `measurements/` and `fixed/`; `measurements/` may include
  `fixed/`; `fixed/` and `pins/` include nothing of the spec.
- Every constant lives in exactly one file. `world.h` reads `measurements/Arena.h` and keeps no arena
  numbers of its own.
- Marker A, marker B, the bridge-out line and the bridge-in line each have their own constants, even where
  the rules say they match. Unmeasured ones are `<<TBD HARDWARE>>` or `<<TBD CALIBRATION>>`, used or not;
  the change that first uses one sets its value.
- `PinMap.h` writes `A0`-`A15` as 54-69, so it needs no `Arduino.h` and lives with the other spec headers.
- Provenance tags are one set: `[RULES]`, `[DESIGN]`, `[MEASURED]`, `[TUNED]`, `[HOST MODEL]`.
- Names and values are unchanged, except: the marker windows are now `MARKER_A_*`;
  `BRIDGE_LINE_WIDTH_MM` (unused) became the two per-location line widths; `USABLE_CORRIDOR_MM`
  (unused; its 235 mm implies a 15 mm wall where `world.h` assumed 17 mm) is removed;
  `WALL_THICKNESS_MM` is new.

## Consequences

- Re-measuring the arena or the robot is one file; tuning is one folder; the wiring is one file.
- A module's constants can span folders (the line follower: tuning, the array mounting, the line width).
  An include list shows which kinds a module depends on.
- Values that are unused today (marker B, the bridge-in line, the wall thickness) are visible in
  `scripts/list-tbds.sh` until someone measures them.
- Earlier ADRs name the old paths; they are left as written.

## Alternatives considered

- **Keep one header per module ([0013](0013-split-the-spec-constants-by-module.md)).** A module's
  constants stay together, but a measurement session still touches many files.
- **Three folders: `pid/`, `measurements/`, `pins/`.** Most tuning is not PID (speeds, timeouts, retries,
  hand-over distances), and the rules and datasheet values would have no home.
- **One `measurements/` file.** The arena and the robot are re-measured on different visits.
- **Derive marker B and the bridge-in line from A.** The arena is hand-built, so equal sizes cannot be
  assumed.
