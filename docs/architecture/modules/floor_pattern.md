# floor_pattern

What the IR array sees as the robot drives out of the maze: the approach marker and the gap before the
ramp. Pure logic in `include/sense/FloorPattern.h`, tested by `test/test_floor_pattern/`. It sits on the
line view ([line_view](line_view.md)) and takes the odometer position of the array's sensing line.

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
and the one in progress). It reports a marker as soon as the far white frame has reached its minimum width and
the two runs before it match: a black box of the right size, preceded by a white frame of the right size. The
event carries the middle of the black box. A long white run (the START tile) never matches, because each
frame has to be 25 to 55 mm.

A change shorter than `glitchMm` is noise and does not start a new run. A `Mixed` sweep (an edge, or the line
on the ramp) never changes the run in progress, so a robot crossing at an angle is not fooled.

The gap is air for at least `gapMinMm`, reported once per run. The array reads 1016 to 1023 on every channel
over the gap. The black floor reads 976 to 1013 and the black tape up to 1023 on the few channels it covers, so
air needs most of the array saturated at once; the margin is a few counts.

## Invariants

- Both events fire once per run of the class that produced them.
- `centreMm` is in the same coordinates as `distMm`.
- `airCounts == 0` turns `Air` off.

## Status

Host tests (19) with the proportions measured on the robot (white 36-42, black 116-119, white 37-41
mm). `PatternParams::farFrameMinMm` lets the far frame count at 12 mm: the array is 86 mm ahead of
the axle, so a robot stopping on the tile's centre has only ~27 mm of it. The mission
([section_a](section_a.md)) feeds `Marker`; `Gap` and `BridgeHandover` take the mission from the approach to
the line follower; `floorPatternMarkerPending` accepts a marker the robot stopped a little short of. The gap detection's margin is a few counts, measured on one
floor and one lighting.
