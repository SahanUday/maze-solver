# line_follow

Following the black line on the white bridge tiles, and telling the approach marker and the
line zone apart from the bare floor. Pure logic in `include/LineFollow.h`, tested by
`test/test_line_follow/`.

## Interface

- `lineView(norm, mask, usable, pitchMm)` - one sweep: `allBlack`, `allWhite`, `onLine`, and
  `errMm`, the centroid of the black channels, positive to the **left** (the CCW side).
- `FloorDetector` / `floorDetectorStep(d, allBlack, distMm, params, centerMm)` - runs of
  non-black floor along the path. Events: `RunStart`, `Marker` (with the run's centre in odometer
  mm), `LineZone`.
- `LineFollow` / `lineFollowBegin(...)`, `lineFollowStep(...)` - the follower; returns
  `Following`, `Searching`, `Ended`, `Lost`.

## How it works

**The maze floor is black**, so "every channel black" is the normal state; on the white bridge
tiles the line is the black part. A run starts when the array first sees non-black after bare
floor and ends when bare floor lasts `FLOOR_RUN_GAP_MM`. A short run (`MARKER_RUN_MIN/MAX_MM`) is
the approach marker; one at least `LINE_ENTER_RUN_MM` long is a line zone. The gap is wider than
the 30 mm line (so the line crossing a white tile does not split it - the remainder would
otherwise look like a marker) and narrower than the 80 mm of floor between the marker and the
bridge tile. A run in progress at power-up is never measured: the robot starts on the white
START tile.

The follower steers on the centroid: the line error becomes a heading offset
(`KP_LINE`, `KD_LINE`, clamped to `STEER_MAX_DEG`) **relative to the current heading**, so it
follows bends. It slows linearly to `LINE_SLOWDOWN_PCT` as the error grows. All white means the
line is lost: it stops and spins toward the side it was last seen on, and gives up after
`LINE_LOST_TIMEOUT_MS` (0 = never). All black after the line means the zone has ended once bare floor has lasted
`LINE_END_ZONE_MM` under the array (longer than the 30 mm line, which can cross the array inside a zone).

**Then it drives on into Section B.** The end is measured under the array, which sits
`IR_ARRAY_FORWARD_OFFSET_MM` ahead of the axle, so stopping there would leave the wheels still on the line.
The follower records where the array first met the floor, adds the array offset (that is where the line
ends, in axle terms) and `LINE_EXIT_MM`, and runs a heading-held profile to that point, ignoring the floor, then
stops (`Ended`). A close front reading (`frontBlocked`, at or inside `FRONT_STOP_MM`) cuts this run short; it is
looked at only here, because on the ramp the front sensor reads the ground. The two constants do different jobs on
purpose: one confirms the end quickly, the other decides how far into B to go.

**On the ramp** (accelerometer tilt above `RAMP_TILT_DEG`) all black is ignored, because a lifted array reads black
whatever is below. `RAMP_TILT_DEG` is `0` ("never") until calibrated, and **must be set**: left at 0 a lifted array at
the crest would read as the end of the zone.

## Invariants

- Bare floor before the line is seen is not the end (`sawLine` gates it).
- The array ahead of the axle damps the loop; with the array at the axle a pure P loop rings.
  In the tests `kp = 1`, `kd = 0` converges within 8 mm from 20 mm off with the array 60 mm
  ahead.
- Unusable channels never count as black or white evidence.

## Status

32 host tests, 99% line coverage (one unreachable fall-through). Closed loop against a simulated
line and in the full-arena simulation. **Nothing here has seen a real line**: the thresholds are 0
in `RobotSpec.h`, and how a real array responds to a 30 mm line over painted plywood, a bend, or the
ramp lift is untested.
