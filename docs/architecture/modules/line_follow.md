# line_follow

Following the black line on the white ramp and bridge tiles. Pure logic in `include/motion/LineFollow.h`,
tested by `test/test_line_follow/` and, closed loop, by `test/test_section_a/`. It sits on the line
view ([line_view](line_view.md)); the mission ([section_a](section_a.md)) hands over to it at the
first line view past the gap ([floor_pattern](floor_pattern.md)).

## Interface

- `LineParams` - gains, speed, acceleration, slowdown, search rate and speed, lost timeout, end
  distance, ramp incline. Every value is in `spec/LineFollow.h` (`KP_LINE`, `LINE_SPEED_MM_S`, ...).
- `lineFollowBegin(lf, fb, alreadyOnLine)`, `lineFollowStep(lf, p, fb, view, inclineCounts, dtMs,
  out)` - returns `Following`, `Searching`, `Ended` or `Lost`, and fills a `MotionTarget`.

## How it works

The follower is speed-controlled (`distRefMm` is the current odometer): the line error, filtered,
becomes a heading offset from the CURRENT heading (`KP_LINE`, `KD_LINE` on the error rate, clamped
to `LINE_STEER_MAX_DEG`), so bends are followed. With proportional gain alone this behaves like an
integrator and oscillates; the derivative term settles it (measured: KP 0.3 + KD 0.15 reached zero
in 90-140 mm and held +-5 mm).

- **All black** under the array is never "lost": a lifted array at the ramp's crest, or bare floor.
  It carries on along the last heading.
- **All white** is a lost line: it turns on the spot toward the side the line was last seen on
  (`LINE_SEARCH_RATE_DPS`, aiming 30 deg ahead) until a channel sees it, and is `Lost` after
  `LINE_LOST_TIMEOUT_MS`. On the ramp (nose-up slope above `RAMP_INCLINE_COUNTS`) an all-white view
  is ignored: a lifted array can read dark floor as white and white as dark.
- **End rule:** with `LINE_END_LOST_MM` above 0, a line gone for that far after it was seen is
  `Ended` and the robot brakes. It is 0 (off): the robot is powered off at the end of the line.

## Invariants

- A line that was never seen cannot end (`sawLine` gates it).
- The first tick of a follow has `newMove` set (zero integrals); the rest do not.
- Below ~100 mm/s the wheels do not start: the follower is entered while rolling.

## Status

11 unit tests; the whole path from a maze tile over the strip, the gap and the ramp onto the line
and round a 90 degree bend is in `test_section_a` (host model: line 24 mm, array 86 mm ahead,
D1 on the right). Tuned on the robot in PR #28 (straight line, 50 cm runs, bends at 130 mm/s); the
S-curve and the ramp itself are only simulated here.
