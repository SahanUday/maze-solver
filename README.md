# maze-solver
Arduino Mega robot that autonomously explores and shortest-path solves a two-section maze connected by a line-followed bridge.

## Development setup

```
pip install -r requirements-dev.txt
pre-commit install
```

Firmware builds/uploads are handled by the PlatformIO VS Code extension - no separate PlatformIO install needed for that.

## Choosing which parts the robot runs

The run is cut into parts (`AExplore`, `AProve`, `AExit`, `Line`, `BEntry`, `BExplore`, `BProve`;
[ADR 0016](docs/architecture/decisions/0016-cut-the-run-into-parts.md)). The firmware runs the stretch
set by `RUN_PART_FIRST` and `RUN_PART_LAST` in [include/spec/tuning/Run.h](include/spec/tuning/Run.h). The
competition setting is the whole course (`AExplore` to `BProve`). To test one part on the robot, edit the
two constants, flash, and place the robot where the first part expects it:

| FIRST | LAST | What runs | Put the robot |
|---|---|---|---|
| `AExplore` | `BProve` | The whole course (the competition setting). Today it runs `AExplore`, `AExit`, `Line`, because the Section B parts aren't built and are skipped. | On the Section A start tile |
| `AExplore` | `Line` | Section A, out to the line, then the line | On the Section A start tile |
| `AExplore` | `AExit` | Section A, out to the line, then stop | On the Section A start tile |
| `AExplore` | `AExplore` | Explore until the marker, then stop | On the Section A start tile |
| `Line` | `Line` | Follow the line (no end rule yet, #60) | On the line, past the gap |
| `Line` | `BProve` | The same today. Later it will be the line plus Section B | On the line, past the gap |

Refused selections:

- `AProve`, `AExit` or `BProve` as the first part, because they carry on a map and can't start alone.
- `BEntry` or `BExplore` as the first part, because they aren't built yet.
- A last part before the first, which fails to compile.
- Without a line calibration, the run ends after `AExplore` whatever the last part is, and `Line` alone is refused.

## Bench firmware: check and measure the robot

`tools/bench/bench.cpp` is a separate firmware for the bench: it checks every sensor and motor and
takes the measurements the constants in `include/spec/` need. It runs the mission's own drivers,
perception, motion controller and IR calibration, so what it reads is what the mission sees.

```
pio run -e bench -t upload        # then the serial monitor at 250000 baud, and type h
pio run -e mega -t upload         # put the mission back afterwards
```

Keep the robot still for the first two seconds (the gyro is zeroed). **The motors never move on their
own:** only `o`, `s` and `t` start them, and any key stops them. Hard limits hold whatever the
controller does: duty 600 of 799, 3 s open loop, 20 s and 2100 mm per move, a stop at 130 mm from a
wall ahead, and a stop when a wheel pushes but does not turn.

| Command | What it does |
|---|---|
| `w` | every sensor once, and the slowest tick since the last `w` |
| `v [ms]` | stream that line every `ms` (default 100, down to 10) until any key |
| `z` | zero the odometer and encoder counts (push the robot along a tape, then `w`) |
| `b` | zero the gyro again (hold still) |
| `m [ms] [label]` | hold still and measure: mean, sd, min, max of each IR channel, ultrasonic, the incline and the gyro, plus the heading drift |
| `k`, `f`, `save`, `cal` | IR calibration: start a capture, slide every channel over white and black, finish (in use at once), write it to EEPROM for the mission, show it |
| `o <L> <R> <ms>` | open loop: a fixed signed duty on each wheel; prints the counts and speed per wheel |
| `s <mm> [mm/s]` | closed-loop straight with the mission's controller; prints the time and the error |
| `t <deg>` | closed-loop turn in place, CCW positive |
| `p`, `h`, `x` | settings, help, stop |

Lines starting with `#` are for people; the rest are CSV whose first field says what they are
(`status`, `stat`, `open`, `move`, `result`, `cal`), so a saved log can be split with `grep`. Lift
the wheels off the floor for the first `o` and `s`.

## Testing and checks

Commits are checked automatically by pre-commit, and every pull request runs the
full set in CI. What each check is, how to run it yourself, and what to do when
one fails: [docs/testing.md](docs/testing.md).
