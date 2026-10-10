# sweep_run

A run for [run_layer](run_layer.md) that measures wheel speed against duty: one wheel at a time,
forward then reverse, climbing a duty ladder and coming back. The data for the dead zone and the
duty-to-speed slope. Pure logic in `include/SweepRun.h`, schedule in `include/SweepRunSpec.h`: no
hardware, no `Arduino.h`, tested on the PC by `test/test_sweep_run/` through the layer against a
motor model. Part of `env:calibrate` only.

## Interface

`sweepRunInit(run)`, then step it with `runLayerStep(layer, run, state)`. After each tick,
`sweepRunTakeRow(run, row)` hands over the rung that finished on it, once.

A row is `SweepRow`: `right`, `reverse`, `up`, `duty` (magnitude), `counts` (signed, over the
measuring window) and `dtMs` (the window's length).

## How it works

Four combos in order: left forward, left reverse, right forward, right reverse. Each combo is a
settle (`SWEEP_SETTLE_MS`, both wheels at zero) and then the ladder: `SWEEP_DUTY_START` to
`SWEEP_DUTY_MAX` in `SWEEP_DUTY_STEP`, then back down from the top. Each rung is held
`SWEEP_STEP_MS`; the speed comes from the last `SWEEP_MEASURE_MS`, so the wheel has settled. The
other wheel is held at zero duty.

Wheel speed is `counts` over `dtMs`, converted with `ENCODER_COUNTS_PER_REV` and
`WHEEL_DIAMETER_MM`. The robot reports raw counts so a corrected constant does not need a re-run.
It uses the tick timestamps, so `dtMs` has 1 ms resolution (about 0.5 percent on the window).

The run needs no walls: it has no front-stop, and it starts without a front echo. The layer's
self-test runs both wheels forward at `SWEEP_SELFTEST_DUTY` first, so a wrong motor or encoder
sign is a fault, not a curve.

## Output

`src/calibration/main.cpp` streams it over serial when the sweep is selected (send `w` before the
self-test starts; the default is the safe run):

```
# sweep v1
# built <date> <time>
# step_ms=400 measure_ms=200 duty=40..760/20
# counts_per_rev=898 wheel_mm=65 pwm_hz=20000
# pack_v= surface=
motor,direction,sweep,duty,counts,dt_ms
L,forward,up,40,0,200
...
```

Lines starting with `#` are notes. The pack voltage and the surface are for the person to
write in: there is no battery sense, and duty to speed moves with the pack. Every other line that
is not a row is the monitor's chatter. A sweep does not touch the EEPROM flight log.

## Reading a capture

Save the serial output of a sweep to a file and run
`scripts/fit-motor-sweep.py <file>`. It ignores everything that is not a row or a `#` note, works
out rpm and mm/s from the raw counts with the constants in `RobotSpec.h`, and prints, per wheel
and direction, the duty it starts and stops turning at, the rpm-per-duty slope over duty 300 to
700, the top speed, and the right/left speed ratio at the same duty. It exits 1 if rungs are
missing. `--write-csv` writes the layout `scripts/plot-motor-curve.py` reads. Run on
`docs/calibration/motor-speed-curve.csv` it gives the figures in [motors](motors.md).

## Invariants

- 4 x 2 x `SWEEP_STEPS_PER_RAMP` rows (296 at the defaults), in the order above.
- Only the swept wheel gets a non-zero duty, and never above `SWEEP_DUTY_MAX`.
- The wheels are at zero for the whole of every settle.
- A finished sweep ends `Stopped` with `Complete`, well inside `SWEEP_TIME_LIMIT_MS`.

## Status

Tested on the PC against a dead-zone-then-linear motor model; mutations that break the stop
between combos or the reverse sign are caught. Not run on the robot. A sweep takes about two
minutes and must be run with the wheels off the ground: the self-test and the ladder drive the
robot, and the wheel that is not being swept is braked, not free.

The firmware that produced `docs/calibration/motor-speed-curve.csv` is not in the repository, so
the first real sweep is the check that this schedule reproduces that file.
