# line_follow

Following the black line on the white bridge tiles, and telling the approach marker and the
line zone apart from the bare floor. Pure logic in `include/LineFollow.h`, tested by
`test/test_line_follow/`.

## Interface

- `lineView(norm, mask, usable, pitchMm, d1OnRight)` - one sweep: `allBlack`, `allWhite`, `onLine`, and
  `errMm`, the centroid of the black channels, positive to the **left** (the CCW side).
  Channel 0 is D1 and on this robot D1 is the **right** end (`IR_D1_ON_RIGHT`, found with a card under
  one end of the array); `d1OnRight` places each channel accordingly, so the sign of the error is right
  whichever way round the array is mounted.
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
the 24 mm line (so the line crossing a white tile does not split it - the remainder would
otherwise look like a marker) and narrower than the 80 mm of floor between the marker and the
bridge tile. A run in progress at power-up is never measured: the robot starts on the white
START tile.

The follower steers on the centroid: the line error becomes a heading offset
(`KP_LINE`, `KD_LINE`, clamped to `STEER_MAX_DEG`) **relative to the current heading**, so it
follows bends. It slows linearly to `LINE_SLOWDOWN_PCT` as the error grows.

**All black** under the whole array is never the end: it is a lifted array at the ramp's crest (white floor reads
dark when the array is high, measured as ~42 mm of `########` just after the crest), or dark floor. The robot
carries on along the last heading.

**All white** means the line is lost. The robot turns on the spot toward the side it was last seen on
(`LINE_SEARCH_RATE_DPS`, aiming 30 degrees ahead), at `LINE_SEARCH_SPEED_MM_S` forward speed (0 today), until a
channel sees the line again, and gives up with `Lost` after `LINE_LOST_TIMEOUT_MS`. This recovered the robot at the
90-degree bends of the flat test track. An optional end rule exists: with `LINE_END_LOST_MM` above 0, a line gone for
that much travel after it was seen is the end of the line (`Ended`, the robot brakes to a stop); it needs a forward
search speed to accumulate distance, and it is off (0) because how the run stops is not decided.

**On the ramp** (accelerometer tilt above `RAMP_TILT_DEG`) an all-white array is ignored, because a lifted array can
read white as dark or dark as white. `RAMP_TILT_DEG` is not set yet: the tilt reading spikes above the ramp's own
slope on flat floor when the robot accelerates or turns, and the dark stretch at the crest happens at 2 to 8
degrees, before the tilt rises (to ~12 degrees on the slope). Left at 0 ("never"), nothing is ignored.

## Invariants

- A line that was never seen cannot end (`sawLine` gates it).
- The array ahead of the axle damps the loop; with the array at the axle a pure P loop rings.
  In the tests `kp = 1`, `kd = 0` converges within 8 mm from 20 mm off with the array 60 mm
  ahead.
- Unusable channels never count as black or white evidence.

## Status

34 host tests, 99% line coverage (one unreachable fall-through). Closed loop against a simulated
line and in the full-arena simulation.

**On the robot** (2026-10-15, a 24 mm black line on a white floor, straight, 50 cm runs from about 11 mm
off-centre, 170 mm/s, `tools/probe/motion.cpp` `L`):
- The follower steers by adding a heading offset to the **current** heading, so with proportional gain alone
  the line error behaves like an integrator and oscillates. `KP_LINE` 0.3 with no D swung +-20 mm and did
  not die away; 0.12 with no D was worse (overshoot to +24 mm, the edge of the array), because a lower
  gain is less damped. `KP_LINE` 0.3 with `KD_LINE` 0.15 reaches zero in 90 to 140 mm and holds
  +-5 mm (standard deviation 2.5 to 3.3 mm) from either side.
- The line view had the channel order the wrong way round until the card test (D1 is the robot's right
  end); the error sign is now checked: a line toward the robot's right end reads negative.
- Below about 100 mm/s the robot does not move: the follower is speed-controlled, so the feedforward duty
  at 60 mm/s is about 70 and the wheels stick below about 160. At 170 mm/s the actual speed is 177 to
  215 mm/s (feedforward alone runs about 20% over).

Bends (90 degrees and an arc) were followed at 130 mm/s; at 170 mm/s the line was lost at the first 90-degree
bend. Switching from the maze to the line was run on the real course (marker, gap, hand over, down the ramp, smooth;
see [floor_pattern](floor_pattern.md)).

**Not yet tested on the robot:** the end of the line (`LINE_END_LOST_MM`) at the foot of the ramp, and the tilt rule
for `RAMP_TILT_DEG`.
