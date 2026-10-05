# Testing and checks guide

How this project catches mistakes before they reach the robot. No prior
knowledge assumed. For *why the code is structured the way it is*, see
[`architecture/ARCHITECTURE.md`](architecture/ARCHITECTURE.md).

## The idea in one minute

Nobody can plug a robot into a build server, so we can't simply "run the robot
and see". Instead the project stacks several **automatic checks**. Each one
catches a different kind of mistake, from cheap and instant (a stray space) to
slow and thorough (running the maze logic with a bug detector attached).
Everything runs by itself when you commit and when you open a pull request (PR);
a PR cannot be merged until all of it passes.

What this does **not** do: run the firmware on a real board. That is checked by
hand (see [Hardware](#what-is-not-tested-automatically)).

## All the checks at a glance

| # | Check | Catches | Where it runs |
|---|---|---|---|
| 1 | Formatting and hygiene | messy whitespace, wrong line endings, unformatted C++, huge files | commit, CI |
| 2 | Project rules | forbidden code (heap, STL), pin clashes, interrupt data races, docs out of step | commit, CI |
| 3 | Strict compiler build | any compiler warning, functions with too much stack | CI |
| 4 | Memory budgets | firmware growing past the RAM / flash limits | CI |
| 5 | cppcheck | likely bugs the compiler does not flag | CI |
| 6 | Unit tests (`env:native`) | logic that gives wrong answers | CI, your PC |
| 7 | Unit tests + bug detector (`env:native_san`) | memory errors, undefined behaviour | CI, your PC |
| 8 | Unit tests + coverage (`env:native_cov`) | tested code quietly shrinking | CI, your PC |
| 9 | Tests of the checks (`scripts/tests/`) | a check that has stopped working | CI, your PC |
| 10 | PR title | titles that don't follow `type(scope): Subject` | CI |
| 11 | Nightly extras | problems at other optimisation levels, flaky tests | once a day |

Plus two **reports** that never fail a PR: the firmware size change and the
list of open `<<TBD>>` placeholders, posted as a comment on every PR.

## Where checks run

1. **On your machine, when you `git commit`.** The pre-commit hooks (checks 1
   and 2) run on the files you changed. If one fails the commit is stopped, so
   you fix it before it ever leaves your computer. Set up once:
   `pip install -r requirements-dev.txt` then `pre-commit install`.
2. **On GitHub, for every PR and every merge to `main`.** The workflow
   `.github/workflows/ci.yml` runs checks 1-9 and the two reports; the PR title
   check (10) is its own small workflow, `pr-title.yml`.
3. **Nightly.** `.github/workflows/nightly.yml` runs the slow extras. It cannot
   block anyone; a failure is a lead to look into.

The PR page shows these checks. Only **`CI gate`** is required for merging: it
turns green when all the jobs below it pass. A docs-only PR skips the heavy
jobs and still gets a green gate.

```
Repo: lint ────────────┐
Repo: static checks ───┼─> Software: native tests ─┐
                       └─> Firmware: AVR build ────┴─> CI gate
```

## Each check explained

### 1. Formatting and hygiene (`Repo: lint`)
**What:** standard tidy-ups from pre-commit: no trailing spaces, files end with
a newline, Unix line endings, no leftover merge-conflict markers, no file over
1 MB, and C/C++ formatted by `clang-format` (settings in `.clang-format`).
**Why:** diffs stay small and reviewers read logic, not whitespace.
**Run:** `pre-commit run --all-files`. clang-format fixes files in place; stage
the result and commit again.

### 2. Project rules (`Repo: static checks`)
Four small programs in `scripts/` that read the source and enforce rules a
compiler can't. They run on the **whole project**, not just changed files.

- **`check-banned-patterns.sh`**: no `malloc`/`free`/`new`/`delete` (this chip
  has 8 KB of RAM and no safe heap), no STL containers (`vector`, `string`...),
  no `virtual` functions, no printing floats with `%f`. It also enforces the
  hardware rule of [ADR 0001](architecture/decisions/0001-hybrid-hardware-abstraction.md):
  files in `src/hal/` must use AVR registers directly, not Arduino's
  `digitalWrite`, `analogRead`, `attachInterrupt` and friends.
- **`check-pin-map.py`**: catches pin conflicts between people working in
  parallel. Fails on two names for the same pin, a pin that is reserved (USB
  serial, I2C) or doesn't exist, a comment like `// PE4, INT4` that disagrees
  with the pin number, a driver that uses a `PIN_*` without a `static_assert`
  for it, a driver in `src/hal/` that touches a port register (`DDRx`/`PORTx`/
  `PINx`) or names a pin bit (`PC1`) that no `static_assert` on a `PIN_*` covers,
  and two drivers using the same timer, interrupt, USART, ADC, I2C or SPI block
  (Timer0 belongs to the Arduino core). The pin rules make sure a driver's
  hardcoded pin is tied to `RobotConfig.h` by an assert on that port and bit; what
  the assert compares (`PIN_X == 36`) is still for the author and reviewer to get
  right.
  *Deliberate sharing:* add `// pin-check: shared timer4 - <reason>` in every
  module involved.
- **`check-isr-atomicity.py`**: on this 8-bit chip, reading a 16- or 32-bit
  variable takes several instructions. If an interrupt changes it halfway, the
  main loop reads a half-old, half-new number (a "torn read"). The rule: a
  `volatile` variable wider than one byte that an interrupt touches may only be
  accessed inside the interrupt, inside `ATOMIC_BLOCK(...)`, or in a `static`
  helper called only from those. *Reviewed exception:* `// isr-safe: <reason>`.
- **`check-docs-drift.py`**: keeps docs honest. Every `src/hal/<name>.cpp` needs
  `docs/architecture/modules/<name>.md`; ADR numbers are unique and match their
  heading; an `Accepted` ADR can't be edited (write a new one that supersedes
  it); and every check script and `env:` is described in this guide.

**Run:** `pre-commit run <hook-id> --all-files` (ids: `banned-patterns`,
`pin-map`, `isr-atomicity`, `docs-drift`) or just run the script, for example
`scripts/check-pin-map.py`.

### 3. Strict compiler build (`Firmware: AVR build`)
**What:** the robot firmware (`env:mega`) is compiled with `-Wall -Wextra
-Werror` (every warning is an error), `-fno-exceptions -fno-rtti` (so a `throw`
or a `dynamic_cast` is a compile error, not just a convention; the host envs get
`-fno-exceptions` only, since `-fno-rtti` is not valid for the one C file in
Unity's test harness) and
`-Wstack-usage=128` (a function using more than 128 bytes of stack is an error:
8 KB of RAM is shared with the stack, and a stack overflow corrupts memory
silently). `lib/maze` is set up for the same strictness in
`lib/maze/library.json`, which starts applying once the library has a `.cpp` of
its own — while it is header-only, `Maze.h` is compiled under the flags of
whatever includes it.
**Run:** `pio run -e mega`.

### 4. Memory budgets
**What:** after the build, `check-ram-budget.sh` and `check-flash-budget.sh`
read the finished firmware (`.pio/build/mega/firmware.elf`). Static RAM warns at
60% of 8 KB and fails at 75% (the rest is for the stack); flash warns at 60%
and fails at 85%.
**Run:** `scripts/check-ram-budget.sh .pio/build/mega/firmware.elf` (same for
`check-flash-budget.sh`) after `pio run -e mega`.

### 5. cppcheck
**What:** a separate static analyser. Warnings, performance and portability
findings fail the build; style findings are advisory. Covers `src/` and `lib/`.
**Run:** `cppcheck --enable=warning,performance,portability --std=c++17
--suppress=missingIncludeSystem -Iinclude src/ lib/` (needs cppcheck installed).

### 6-8. Unit tests on your PC (`Software: native tests`)
This is the part most people mean by "tests". A **unit test** calls one small
piece of logic with known inputs and checks the answer. They run on your
computer, not the robot, so they are fast and need no hardware. Three
**environments** (build recipes in `platformio.ini`) run the same tests in
different ways:

| Env | Same tests, but... | Why |
|---|---|---|
| `env:native` | built normally | the everyday test run |
| `env:native_san` | with AddressSanitizer + UBSan | the program watches itself and stops at the first memory error or undefined behaviour (reading past an array, integer overflow). These bugs often don't crash, so ordinary tests miss them. |
| `env:native_cov` | with coverage counting | records which lines ran, then `scripts/check-coverage.sh` fails if less than 90% of `include/` and `lib/` was exercised |

**Run** (in a PlatformIO terminal):
```
pio test -e native
pio test -e native_san
pio test -e native_cov && scripts/check-coverage.sh   # needs: pip install -r requirements-ci.txt
```
Add `-f test_scheduler` to run one suite. Coverage only counts files that some
test compiles, so a brand-new untested header isn't counted: write its test.

**What can be unit tested.** Only code that does not touch hardware and does not
include `Arduino.h`: the headers in `include/` (`Scheduler.h`, `Quadrature.h`,
`MotorDrive.h`) and the library `lib/maze`. Code in `src/` (the HAL drivers and
`main.cpp`) is built only for the robot. That is why drivers keep their
arithmetic in a separate header: the decision logic is testable even though the
register code isn't.

**Where tests live.** One folder per suite under `test/`, with the same name in
the file: `test/test_scheduler/test_scheduler.cpp`. Current suites:
`test_scheduler`, `test_quadrature`, `test_motor_drive`, and `test_native`
(a placeholder that proves the host build works). A test file looks like this
(Unity is the test framework):
```cpp
#include <unity.h>
#include "Scheduler.h"

static void test_two_plus_two()          // one small check per function
{
    TEST_ASSERT_EQUAL(4, 2 + 2);
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_two_plus_two);          // list every test here
    return UNITY_END();
}
```
**To add a test:** create `test/test_<name>/test_<name>.cpp` like the above, run
`pio test -e native -f test_<name>`, then run all three envs before pushing.

### 9. Tests of the checks (`scripts/tests/`)
**What:** a check that quietly stops working protects nothing, so the check
scripts have their own tests. Each feeds a script a tiny fake project (a
duplicated pin, a missing `ATOMIC_BLOCK`) and confirms it is reported, and that
a clean project passes. Files: `test_pin_map.py`, `test_isr_atomicity.py`,
`test_docs_drift.py`, `test_banned_patterns.py`, `test_pr_title.py`,
`test_size_report.py`, `test_tbd_report.py`, with shared helpers in
`support.py`. They use Python's
built-in `unittest`; nothing to install.
**Run:** `python -m unittest discover -s scripts/tests`.
**Changing a script?** Add or update its test in the same PR.

### 10. PR title (`PR title / Conventional title`)
**What:** PRs are squash-merged, so the PR title becomes the commit message on
`main`. It must look like `type(scope): Subject`, for example
`feat(hal): Add IR array driver`. Types: `feat fix docs refactor perf test
build ci chore revert`; the subject starts with a capital letter, has no
trailing full stop, and the whole title is at most 72 characters. It re-runs
when you edit the title. **Run:** `scripts/check-pr-title.py "your title"`.

### 11. Nightly extras
- The firmware built at `-O1`, `-O2` and `-O3` (the robot uses `-Os`), to
  find code that only works at one optimisation level. It only builds; it can't
  run the result.
- The unit tests run 25 times under the bug detector, to expose hidden state
  that makes a test pass or fail by luck.
- cppcheck with every check turned on (advisory).

## The two PR reports (they never fail a PR)
- **Firmware size**: flash and static RAM now versus `main`, and which
  functions or variables grew most, from `scripts/size-report.py`.
- **Open placeholders**: every `<<TBD HARDWARE>>` (to measure on the robot) and
  `<<TBD CALIBRATION>>` (to tune) still in the code, from
  `scripts/tbd-report.py` (built on `scripts/list-tbds.sh`), so they are not
  forgotten. List them yourself with `scripts/list-tbds.sh`.

Both appear in one comment that the bot edits on each push.

## What is not tested automatically
**The firmware itself running on the robot.** CI compiles it and checks its
size, but no board or simulator executes it. So an interrupt-service routine, a
PWM setting or a pin mapping can pass every check above and still misbehave
on the real hardware. For hardware-facing code:
- keep the *decision* logic in a testable header and the register code thin;
- use the static checks above (pins, interrupt data, banned patterns);
- verify on the robot by hand, and write what you did and what is still
  unchecked in the module's `## Status` section (see
  `architecture/modules/encoders.md`).

Not yet covered because the code does not exist yet: tests of the maze solver.
They will be added with it and run in the jobs above without CI changes.

## When a check fails

| It says | Do this |
|---|---|
| clang-format / whitespace | run `pre-commit run --all-files`, stage the changes, commit again |
| banned pattern | replace it (fixed-size array instead of `vector`; registers instead of `digitalWrite` in `src/hal/`) |
| pin map: duplicate pin / owner clash | pick a free pin or coordinate with the other contributor; if sharing is intended, add the `pin-check: shared` comment in each module |
| ISR atomicity | wrap the access in `ATOMIC_BLOCK(ATOMIC_RESTORESTATE)` or read it through the driver's accessor |
| docs drift | add the missing module doc, renumber your ADR to the next free number, or document the new script/env here |
| compiler error with `-Werror` | fix the warning; don't silence it |
| stack usage over 128 bytes | shrink local arrays, or make them `static` if that is safe |
| RAM / flash budget | find the big symbol in the size report; keep tables in `PROGMEM` |
| unit test fails | read the line it points at; the test names the case |
| sanitizer error | the message names the file and line of the invalid access |
| coverage below the floor | add a test for the new logic |

## Words you may meet
- **ISR**: interrupt service routine, code the chip runs when a hardware event
  (an encoder edge) interrupts the main program.
- **HAL**: hardware abstraction layer, the `src/hal/` drivers that talk to pins
  and timers.
- **Static analysis**: reading code for problems without running it.
- **Sanitizer**: extra instrumentation compiled in that detects invalid memory
  use while the program runs.
- **Coverage**: the share of code lines that tests actually executed.
- **pre-commit**: a tool that runs checks automatically on `git commit`.
- **CI**: continuous integration, the automatic checks GitHub runs on every PR.
- **ADR**: architecture decision record, a short note explaining one design
  choice (`docs/architecture/decisions/`).
