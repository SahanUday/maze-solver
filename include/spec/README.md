# spec/

Every constant the firmware uses, in one folder per reason a value changes
([ADR 0020](../../docs/architecture/decisions/0020-group-the-spec-constants-by-why-they-change.md)).
The host tests read them from here too; numbers only the simulator needs are in `test/model/`.

| Folder | Holds | Edit it when |
|---|---|---|
| `pins/` | `PinMap.h`: every pin number | the robot is re-wired |
| `measurements/` | `Arena.h` (tiles, walls, marker A and B, bridge floor), `Robot.h` (wheels, sensor mounting and offsets, polarity, IMU orientation) | you re-measure the arena or the robot |
| `tuning/` | one header per logic module: gains, speeds, thresholds, timeouts, retries | you run the robot and adjust |
| `fixed/` | `Rules.h` (competition limits), `Hardware.h` (clock, PWM, chip and module settings) | rarely: the rules or a datasheet say so |

## Use

- Include only the headers you use: `#include "spec/tuning/Motion.h"`.
- Includes point one way: `tuning/` to `measurements/` to `fixed/`. `pins/` and `fixed/` include nothing of the spec.
- No `Arduino.h` in here: the host tests include these headers.
- Include a value, never copy it. (Unit-test fixtures with literal data are the exception.)

## Update

1. Find the folder with the question *"what would I do if this value were wrong?"*: re-wire, measure it, run the robot and adjust, or look it up.
2. Change the value and its comment: what was done, when, and the numbers.
3. Fix the section tag; delete `<<TBD …>>` once the value is real.
4. Run `pio test -e native` and `pio run -e mega`. A `static_assert` here fails the build if a value no longer fits its neighbours.

A new constant is `constexpr`, has its unit in the name (`_MM`, `_MS`, `_US`, `_DEG`, `_MV`, `_PWM`) and lives in exactly one file.

The arena is hand-built, so marker A, marker B, the bridge-out line and the bridge-in line each have their own constants, even where the rules say they match.

## Tags

One per section heading (two where a section mixes).

| Tag | Meaning |
|---|---|
| `[RULES]` | from the competition rules (nominal) |
| `[DESIGN]` | datasheet or design choice |
| `[MEASURED]` | measured on the robot or arena |
| `[TUNED]` | tuned or chosen while testing on the robot |
| `[HOST MODEL]` | found in the host simulation, not yet checked on the robot |
| `<<TBD HARDWARE>>` | not yet measured; holds a stand-in value |
| `<<TBD CALIBRATION>>` | not yet tuned; holds a stand-in value |

Open placeholders: `scripts/list-tbds.sh`. Pin headings use `[FIXED]` / `[PROPOSED]` instead (see `PinMap.h`).
