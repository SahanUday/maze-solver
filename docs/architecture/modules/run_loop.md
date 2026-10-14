# run_loop

One control tick of a Section A run: sensors in, drive command out. Pure logic in
`include/RunLoop.h`, tested by `test/test_run_loop/`; `src/main.cpp` is the thin shell that reads the
hardware and applies the result.

## Interface

- `RunLoop` - the perceiver, the latest `Perception`, the mission ([section_a](section_a.md)) and
  its parameters, the start-button latch.
- `runLoopInit(rl, imuPresent, lineCal)` - a run not yet started. `lineCal` is the IR array's
  calibration or null. Without a usable channel the run is `wallsOnly`; without a gyro it is
  failed (`NoImu`).
- `runLoopTick(rl, state, nowMs, buttonDown, headingFixDeg)` - one tick; returns the
  `DriveCommand`. `headingFixDeg` is what to subtract from the gyro heading.

## How it works

1. The raw start button is debounced (`RUN_START_DEBOUNCE_TICKS` in a row) and latched: one press
   begins the run, and releasing it does nothing.
2. `RobotState` goes through `perceive()` (with the button as the latch) and the mission steps.
3. The run limit (`RUN_LIMIT_MS`, 8 minutes from the press) fails a run still in progress
   (`TimeUp`). A run that is `Done` or `Fault` is left alone.

`main.cpp` applies the command: `motorsEnable()` when `enable` changes, `motorSet()` for both
wheels, the heading fix through `headingSet()`, and a line on the serial port when the phase
changes. The error LED follows `Fault`.

## Invariants

- `RobotState` is copied, never written.
- A command with `enable` false leaves the drivers coasting: no press, no gyro, a fault.
- The motors are touched only through the command this returns.

## Status

Host tests, including a whole run from the button to the white tile through the raw sensors in the
simulated arena. Not run on the robot. No IR calibration is stored yet, so `main.cpp` passes none
and the run is walls only: it explores every reachable tile and stops (`Done`).
