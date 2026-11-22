# Tests

Host tests: they run on your PC, with no robot. The robot itself is checked by hand (see
[docs/testing.md](../docs/testing.md), which has the full guide).

## Layout

| Folder | What it holds |
|---|---|
| `unit/` | One header or module per suite, in milliseconds. |
| `integration/` | The mission run in the host world (a simulated arena), and tests of the world itself. |
| `support/` | Shared helpers, not suites: `world/` (the host world), `MazeGen.h`, `Seed.h`, `Fingerprint.h`, `sim.h`. |
| `model/` | Numbers only the simulator needs (IR levels, plant, noise, ramp). The firmware's own numbers stay in `include/spec/`. |

One suite is one folder, `test_<name>/test_<name>.cpp`, written with Unity.

## Run

```
pio test -e native                          # everything
pio test -e native -f "unit/*"              # one tier
pio test -e native -f "unit/test_maze"      # one suite
pio test -e native_san                      # same tests under ASan + UBSan
pio test -e native_cov && scripts/check-coverage.sh
```

Run all three environments before pushing. The scripts' own tests: `python -m unittest discover -s scripts/tests`.

## In CI

| When | What runs |
|---|---|
| Every PR, and every merge to `main` | `native`, `native_san` and `native_cov` (90% line coverage of `include/` and `lib/`), plus lint, the AVR build and the project's checks. `CI gate` must pass to merge. |
| Docs-only PRs | The heavy jobs are skipped. |
| Nightly | Extras: the suites repeated under the sanitizers, firmware builds at other optimisation levels, cppcheck. |

## Add or change a test

- Put it in the suite of the code it covers. Same check with different inputs: one table and a loop, not one test each.
- A new test must fail without the change it guards. Revert the fix, see it fail, put it back.
- Name tests as sentences: `test_a_wall_is_never_erased`.
- A failing seeded run prints its seed: `... [seed 7]`. Add `void tearDown(void) { seeds::report(); }` to a suite that seeds.
- A new suite needs no CI change.

## Use the host world

```cpp
World w;
world::photoMaze(w);              // Section A; or world::bridgeIn(w, {9, corner, opening, seed})
w.seed(42);                       // optional: one number replays the noise
w.start(2, 1, 0);                 // tile (2, 1), facing east
const Perception pc = w.sense();  // what the firmware would read this tick
w.drive(missionStep(ms, params, pc));
```

Examples: `integration/test_section_a/` and `integration/test_host_world/`.

## Update the world or its numbers

- **A number from the robot:** change it in `model/` (or `include/spec/` if the firmware uses it too), tag it `[MEASURED]` with the date, and drop its `<<TBD HARDWARE>>`. `scripts/list-tbds.sh` lists what is still a stand-in.
- **The world's code:** build with `-DWORLD_FINGERPRINT` before and after and compare the line each integration suite prints:
  `PLATFORMIO_BUILD_FLAGS=-DWORLD_FINGERPRINT pio test -e native -f "integration/*" -vv | grep FINGERPRINT`.
  The same line means no tick changed. A change meant to alter readings changes it on purpose: make it a commit of its own.
- Never read `include/spec/tuning/` as truth in the world: those are the firmware's choices, not facts about the robot.

## Still to come

| Issue | What |
|---|---|
| #60, #61 | Bridge and Section B tests, on `bridgeIn()` and its eight entries. |
| #82 | A sweep tier: many seeded mazes and faults, run nightly. |
| #83 | A fast raw log on the robot and replay of real recordings. |
| #54 | Robot measurements that replace the `<<TBD HARDWARE>>` numbers. |
