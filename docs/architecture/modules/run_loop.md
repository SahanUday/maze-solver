# run_loop

One control tick of a run: sensors in, drive command out. Pure logic in
`include/mission/RunLoop.h`, tested by `test/integration/test_run_loop/`; `src/main.cpp` is the thin shell that reads the
hardware and applies the result.

## Interface

- `RunLoop` - the perceiver, the latest `Perception`, the mission manager
  ([mission_manager](mission_manager.md)) and its parameters.
- `runLoopInit(rl, imuPresent, lineCal, parts)` - a run not yet started. `lineCal` is the IR array's
  calibration or null. Without a usable channel the run is `wallsOnly`; without a gyro it is failed
  (`NoImu`). `parts` is which parts of the course to run (all of them by default).
- `runLoopTick(rl, state, nowMs, buttonDown, headingFixDeg)` - one tick; returns the
  `DriveCommand`. `headingFixDeg` is what to subtract from the gyro heading. `buttonDown` is
  ignored while no button is fitted.

## How it works

1. `RobotState` goes through `perceive()`.
2. The mission manager takes the tick: it starts the run (by itself `RUN_START_DELAY_MS` after the first
   tick, as no button is fitted: the power switch is the start, `setup()` has zeroed the gyro with the
   robot held still by then, and the delay lets the hands come off; with a button, a debounced press),
   enforces the run limit and steps the parts.
3. The mission's heading correction is handed back for the caller to apply.

`main.cpp` applies the command: `motorsEnable()` when `enable` changes, `motorSet()` for both
wheels, and the heading fix through the heading estimator (`src/app/heading_estimator.*`). The
serial lines come from `src/app/telemetry.*` and the error LED, which follows `Fault`, from
`src/app/operator_io.*`.

## Invariants

- `RobotState` is copied, never written.
- A command with `enable` false leaves the drivers coasting: before the start, no gyro, a fault.
- The motors are touched only through the command this returns.

## Status

Host tests, including a whole run from power-on to the marker and onto the line through the raw sensors in the
simulated arena. Ran on the robot at the mid-evaluation (tag `v0.1-section-a`). `main.cpp` loads the
stored IR calibration at boot ([line_sense](line_sense.md)) and passes it; without a usable one the
run is walls only: it explores every reachable tile and stops (`Done`).

## Status line

While the run is going, `src/app/telemetry.cpp` prints one CSV line every `RUN_LOG_EVERY_MS` (200 ms; 0 =
off) after a `# status ...` header: time, phase, fault, moves, turns, flags (1 marker seen, 2 gap
seen, 4 turning to the exit), odometer, speed, heading, the three ultrasonics (0 = no echo), the IR
floor class (0 black, 1 white, 2 air, 3 mixed), the line error, the eight raw IR counts and the
drive duty. Integers only. A phase change also prints `run: phase N fault F moves M turns T`.
