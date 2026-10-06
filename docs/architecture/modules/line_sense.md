# line_sense

Per-channel calibration, normalization and black/white mask for the IR array
counts. Pure logic in `include/LineSense.h`: no hardware, tested on the PC by
`test/test_line_sense/`. Decision:
[0006](../decisions/0006-line-calibration-above-the-hal.md). The counts come from
[line_sensors](line_sensors.md).

## Interface

- `LineCalibration` - per channel `white` (lowest count seen), `black` (highest)
  and `scale`.
- `lineCalibrationReset(cal)`, `lineCalibrationAccumulate(cal, counts)` - reset,
  then call for every sweep while the robot passes over white and black.
- `lineCalibrationFinish(cal)` - sets the scales; returns the usable channels,
  bit ch = 1.
- `lineNormalize(cal, counts, norm)` - 0 (white) .. 255 (black), clamped; an
  unusable channel reads 0.
- `lineMaskUpdate(norm, previous)` - bit ch = 1 for black, with hysteresis.

## How it works

Counts fall as reflection rises, so a channel's lowest count is its white and its
highest its black. `lineCalibrationFinish()` stores `(255 << 16) / range` per
channel, the only division, so `lineNormalize()` is a multiply and a shift. The
mask sets a channel above `LINE_MASK_ON_PCT` of its range, clears it below
`LINE_MASK_OFF_PCT`, and keeps its previous state in between.

## Invariants

- A channel needs a range of at least `LINE_CAL_MIN_SPAN_COUNTS`; a run that saw
  only one colour leaves its channels unusable.
- A calibration holds only for the mount height, floor and lighting it was taken
  in: run it again after changing any of them.
- One `LineCalibration` is 64 B.

## Status

The host suite uses counts measured on the robot as fixtures, and each of nine
deliberate breakages of the header is caught by it. On the Mega the header
compiles under `-Werror -Wstack-usage=128`; `lineNormalize()` takes about 25 µs
per sweep, `lineMaskUpdate()` up to 21 µs and `lineCalibrationFinish()` 337 µs
once. On the robot, a run over white and then black gave a usable range of 56-92
counts on all 8 channels, and the live values read 251-255 with mask `11111111` on
black; the white and half-on-black checks were done by hand and behaved as
expected.

Nothing calls it yet: there are no `RobotState` fields and no calibration trigger;
both come with the first consumer.

Still to check: the response to a 30 mm line (which sets `LINE_MASK_ON_PCT` and
`LINE_MASK_OFF_PCT`), tilt on the bridge incline, and a change of mount height.
