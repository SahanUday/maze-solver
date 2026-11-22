# 0020: Group the spec constants by why they change

- **Status:** Accepted
- **Date:** 2026-11-22
- **Supersedes:** the file layout of [0013](0013-split-the-spec-constants-by-module.md) (one header per
  module stands, inside the folders below) and the `config/` row of
  [0012](0012-group-headers-by-layer.md)

## Context

- A module's constants differ in kind (measured on the arena, measured on the robot, tuned, fixed by
  the rules), so re-measuring the arena meant editing several headers.
- The rules PDF gives nominal sizes and the arena is hand-built: the bridge line is 30 mm in the PDF
  and 24 mm measured, the marker 90 / 30 mm and 36-42 / 116-119 / 37-41 mm. `Competition.h` filed
  measured values as `[FROM SPEC]`.
- `test/support/world.h` kept private copies of the arena, so a spec change did not change what the
  host tests simulate.
- One marker and one line width served the whole course, but marker B and the bridge-in line are
  separate hand-made pieces that nobody has measured.
- `include/config/` held one file, kept apart only because it includes `Arduino.h`.

## Decision

`include/spec/` is grouped by what you would do if a value were wrong:

| Folder | Holds | Changes when |
|---|---|---|
| `pins/` | `PinMap.h` (was `config/RobotConfig.h`) | the robot is re-wired |
| `measurements/` | `Arena.h` (tiles, walls, markers, bridge floor), `Robot.h` (wheels, sensors, polarity, IMU orientation) | something is re-measured |
| `tuning/` | gains, speeds, thresholds, timeouts, retries, one header per logic module | the robot is run and adjusted |
| `fixed/` | `Rules.h` (competition limits), `Hardware.h` (clock, PWM, chip and module settings) | rarely: the rules or a datasheet change |

- Includes point one way: `tuning/` to `measurements/` to `fixed/`; `pins/` and `fixed/` include nothing
  of the spec.
- Every constant is in exactly one file. `world.h` reads `measurements/Arena.h`.
- Marker A, marker B, the bridge-out line and the bridge-in line have separate constants even where the
  rules say they match. Unmeasured ones are `<<TBD …>>`, used or not; the change that first uses one
  sets its value.
- `PinMap.h` writes `A0`-`A15` as 54-69, so it needs no `Arduino.h`.
- Provenance tags: `[RULES]`, `[DESIGN]`, `[MEASURED]`, `[TUNED]`, `[HOST MODEL]`.
- Names and values are unchanged, except: the marker windows are `MARKER_A_*`;
  `BRIDGE_LINE_WIDTH_MM` (unused) became the two line widths; `USABLE_CORRIDOR_MM` (unused; its 235 mm
  implies a 15 mm wall where `world.h` assumed 17 mm) is removed; `WALL_THICKNESS_MM` is new.

## Consequences

- Re-measuring is one file, tuning one folder, wiring one file.
- A module's constants can span folders; its include list shows which kinds it depends on.
- Unused placeholders show in `scripts/list-tbds.sh` until measured.
- Earlier ADRs name the old paths and are left as written.

## Alternatives considered

- **One header per module ([0013](0013-split-the-spec-constants-by-module.md)).** A measurement
  session still touches many files.
- **Three folders: `pid/`, `measurements/`, `pins/`.** Most tuning is not PID, and the rules and
  datasheet values have no home.
- **Derive marker B and the bridge-in line from A.** The arena is hand-built.
