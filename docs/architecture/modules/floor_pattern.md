# floor_pattern

What the IR array sees as the robot drives out of the maze: the approach marker and the gap before the
ramp. Pure logic in `include/FloorPattern.h`, tested by `test/test_floor_pattern/`. It sits on the
line view ([line_follow](line_follow.md) for `lineView()`) and takes the odometer position of the array's
sensing line.

## Interface

- `classifyFloor(view, raw, airCounts, airChannels)` - one sweep to `Black`, `White`, `Air` or `Mixed`.
  `Air` means nothing is under the array: at least `airChannels` channels read `airCounts` or more.
- `floorPatternStep(f, seen, distMm, params, centreMm)` - feed one sweep per tick; returns `PatternEvent::None`,
  `Marker` (with `centreMm`, the middle of the marker's black box) or `Gap`.
- `PatternParams` - the frame and centre windows, the minimum gap, and the glitch length. Values in
  `RobotSpec.h`: `MARKER_FRAME_MIN/MAX_MM`, `MARKER_CENTRE_MIN/MAX_MM`, `GAP_MIN_MM`, `FLOOR_GLITCH_MM`,
  `IR_AIR_COUNTS`, `IR_AIR_CHANNELS`.

## How it works

The marker is a white frame around a black box, so crossing it gives **white, black, white** (measured:
about 36, 118 and 37 mm). The detector keeps the runs of each class along the path (the last three finished
and the one in progress). It reports a marker as soon as the far white frame has reached `reportMm` (`MARKER_FRAME_REPORT_MM`, 4 mm; 0 means
the frame's minimum width of 25) and
the two runs before it match: a black box of the right size, preceded by a white frame of the right size. The
event carries the middle of the black box. A long white run (the START tile) never matches, because each
frame has to be 25 to 55 mm.

A change shorter than `glitchMm` is noise and does not start a new run. A `Mixed` sweep (an edge, or the line
on the ramp) never changes the run in progress, so a robot crossing at an angle is not fooled.

The gap is air for at least `gapMinMm`, reported once per run. The array reads 1016 to 1023 on every channel
over the gap. The black floor reads 976 to 1013 and the black tape up to 1023 on the few channels it covers, so
air needs most of the array saturated at once; the margin is a few counts.

## Bridge handover

`bridgeHandoverStep` decides when the approach hands over to the line follower. After the gap has been reported it
waits until the array has been off the air for `BRIDGE_HANDOVER_MIN_MM` (4 mm; air again restarts the wait), then
takes the first line view (`onLine`). The ~8 mm of dark floor just past the gap (seen on the robot) reads all
black, not a line, so it cannot trigger it. [mission](mission.md) uses it in the Bridge phase.

## Invariants

- Both events fire once per run of the class that produced them.
- `centreMm` is in the same coordinates as `distMm`.
- `airCounts == 0` turns `Air` off.

## Status

18 host tests, with the proportions measured on the robot. The three marker scans on the robot (white 36-42,
black 116-119, white 37-41 mm) are all inside the windows. The gap and the black strip were measured in the
same scans. Not yet fed from the live array: nothing calls `classifyFloor()` yet, and the thin saturation
margin has been measured on only one floor and one lighting.
