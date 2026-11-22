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

## Testing and checks

Commits are checked automatically by pre-commit, and every pull request runs the
full set in CI. What each check is, how to run it yourself, and what to do when
one fails: [docs/testing.md](docs/testing.md).
