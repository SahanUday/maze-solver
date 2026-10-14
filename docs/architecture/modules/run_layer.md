# run_layer

What every test run goes through, so each run only has to say what it wants. Pure logic in
`include/RunLayer.h`, shared constants in `include/RunLayerSpec.h`: no hardware, no `Arduino.h`,
tested on the PC by `test/test_run_layer/` against a stub run. The entry point of `env:calibrate`
(`src/calibration/main.cpp`) reads the sensors, calls `runLayerStep()` once per tick, applies the
result to `motors`, and records the tick with `src/calibration/flight_log.*`. Not part of the
competition firmware (`env:mega`). Runs on top of it: [safe_run](safe_run.md) and
[sweep_run](sweep_run.md); the entry point picks one over serial (`s` or `w`) until the
self-test starts.

## Interface

`runLayerInit(layer, buttonFitted)`, then `runLayerStep(layer, run, state)` once per tick, which
returns `RunOutput` (`enable`, signed `left` / `right` duty).

A run is any type with four functions, found by argument type (no virtuals):

| Function | Called | Returns |
|---|---|---|
| `runLimits(run)` | every tick | `RunLimits`: `maxDuty`, `rampPerTick`, `timeLimitMs`, `selfTestDuty`, `frontStopMm` |
| `runReady(run, state)` | when the countdown ends | `None`, or the fault reason that keeps the run from starting |
| `runBegin(run, state)` | when the self-test passes | the first `RunWant` |
| `runStep(run, state)` | every tick after that | `RunWant`: `left` / `right` duty, `hardStop`, `phase`, `end` |

`RunWant::phase` is for the log and the status LED. A run that ends sets `end` to a reason.

## Phases

`WaitStart` -> `Countdown` -> `SelfTest` -> the run's own phases -> `Stopped` (expected) or `Fault`.
Both end states are final until reset and disable the motors with no ramp.

- **WaitStart**: motors off. With `RUN_START_BUTTON_FITTED` false there is no button: the
  countdown begins at once and lasts `RUN_AUTOSTART_MS`, and the button input is ignored, so there
  is no button e-stop either. With a button, a rising edge on the debounced button starts the run;
  a button held through boot does not.
- **Countdown** (`RUN_COUNTDOWN_MS`): motors off, hands clear. Ends in `Fault` with the reason
  from `runReady()`.
- **SelfTest** (`RUN_SELFTEST_MS`): both wheels forward at `selfTestDuty`. Each encoder must count
  up by `RUN_SELFTEST_MIN_COUNTS`; counting down is `SelfTestReversed`, not moving is
  `SelfTestNoMotion`. This catches a motor/encoder sign mismatch, not a robot that drives backwards
  with both signs wrong. A heard wall closer than `frontStopMm` ends it as `FrontWall`.
- **Running**: the run's phases. The layer ends the run on the button (`EStop`), on `timeLimitMs`
  (`TimeLimit`), or on the run's own `end` reason.

## Status LED

`include/StatusLed.h` drives the Mega's on-board LED (pin 13) so a run can be read without a
serial monitor. Waiting and countdown: slow blink. Driving (self-test and every run phase): solid.
Ended: N flashes then a pause, slow for `Stopped`, fast for `Fault`.

| Flashes | Stopped | Fault |
|---|---|---|
| 1 | button pressed | encoder counted down in the self-test |
| 2 | run time limit | a wheel did not move in the self-test |
| 3 | wall ahead, no gyro | stall |
| 4 | wall ahead, a side sensor silent | no front echo |
| 5 | too many turns in a row | turn did not finish |
| 6 | run complete | - |

## Invariants

- Every wheel command is capped at `maxDuty`, and never above `MOTOR_PWM_TOP`, then moved toward
  by at most `rampPerTick` per tick. A `hardStop` zeroes both wheels first.
- A reason is a fault when it is `SelfTestReversed` or later in `RunReason`; the rest are stops.
  The log keeps the reason in a nibble, so `RunReason` has room for 16.
- The countdown never drives.

## Status

Tested on the PC. The self-test and the button flow have run on the robot through
[safe_run](safe_run.md); [sweep_run](sweep_run.md) is the second run on the four-function
interface and has not run on the robot yet. CI does not build `env:calibrate`.
