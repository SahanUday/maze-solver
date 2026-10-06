# 0006: Line calibration and black/white mask above the HAL

- **Status:** Accepted
- **Date:** 2026-10-06

## Context

`RobotState::irRaw` carries the raw 10-bit counts from `line_sensors`
([0005](0005-analog-ir-array-adc-sampling.md)). Later code wants a black or white
answer per channel. With the array mounted on the robot (about 15 mm, not
measured exactly) white and black differ by tens of counts per channel against
noise of ±1: 29-46 counts in the first runs and 56-92 after the mount was
adjusted, 3-9% of full scale. The channels differ from each other by up to 17
counts in range and 8 in position, and the white level drifted 4 counts in a few
minutes. The range also depends on the mount height (several hundred counts at
3 mm), the floor material and the lighting, and the competition allows no
reprogramming between trials.

## Decision

- `RobotState` keeps the raw counts and the HAL stays free of thresholds.
  Calibration, normalization and the mask are pure functions in
  `include/LineSense.h`, tested on the host and called by the control layer.
- Calibration is captured at run time, not written into `RobotSpec.h`: the lowest
  and highest count per channel over a run in which the sensors pass over white and
  black. What triggers a run (a physical control) is not built yet.
- Counts normalize to 0 (white) .. 255 (black) per channel. The one division
  happens when the run ends and sets a scale; every sweep after that multiplies.
- The mask turns a channel black at 60% of its range and white at 40%, and keeps
  its state in between, so noise near a threshold does not make it flicker.
- A channel whose range is under `LINE_CAL_MIN_SPAN_COUNTS` (10, ten times the
  noise) is unusable: it normalizes to 0 and the run reports it.

## Consequences

**Good**

- The raw counts stay available for interpolation and diagnostics, and the mask
  gives the simple black/white view for marker patterns and similar logic.
- A change of mount height, floor or lighting needs a new run, not new code.
- The logic is tested on the PC against counts measured on the robot.

**Bad**

- Normalizing costs about 25 µs per sweep and the mask up to 21 µs on the Mega,
  0.5% of a 10 ms tick, and one calibration takes 64 B of RAM.
- At this mounting each channel has only 29-92 levels between white and black.
- Calibrating on a large black area probably overstates the response of a 30 mm
  line, so `LINE_MASK_ON_PCT` and `LINE_MASK_OFF_PCT` stay `<<TBD CALIBRATION>>`
  until they are measured on the real line.
- Black is still indistinguishable from no surface, which calibration cannot fix.
- Nothing calls it yet: no `RobotState` fields and no trigger.

## Alternatives considered

**Black/white bits in the HAL or `RobotState` instead of counts.** Rejected: it
discards the magnitude that interpolation and diagnostics need, and ties
thresholds to the driver and the mount height. The digital driver of
[0004](0004-digital-ir-array-sensing.md) lost information the same way.

**Fixed thresholds in `RobotSpec.h`.** Rejected: no reprogramming between trials,
and the range moves with height, material and lighting.

**Continuous min/max tracking with no calibration run.** Not built: on the bridge
the robot may never see black, so the range would collapse. Revisit with real runs.

**Per-sweep normalization across channels.** Not built: when every channel sees the
same surface it gives no absolute black or white.
