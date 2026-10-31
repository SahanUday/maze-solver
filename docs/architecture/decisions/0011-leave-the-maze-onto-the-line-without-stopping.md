# 0011: Leave the maze over the gap and onto the line, without stopping

- **Status:** Accepted
- **Date:** 2026-10-15

## Context

After the approach marker ([0010](0010-finish-on-the-approach-marker.md)) the first task continues
out of the maze: ~20 mm of black strip, a gap of ~37 mm (nothing under the IR array), a ramp
carrying a black line, then the line (bends, an S-curve). PR #28 measured all of it on the robot and
ran the hand-over on the real course.

## Decision

- **Approach:** at the marker event the robot drives straight on along the grid heading at the
  line speed (130 mm/s). If the way out is not straight ahead (a wall ahead on the marker's tile),
  it stops, reads both sides, turns to the open side that answers from far or not at all (the
  other open side leads to a maze tile whose far wall is within 2.5 m), and goes.
- **Hand-over:** at the first line view `BRIDGE_HANDOVER_MIN_MM` past the end of the gap
  (`BridgeHandover`): the dark patch after the gap must not count. No line within
  `APPROACH_NO_LINE_MM` of the marker's centre is `NoLine`.
- **Follower:** PID-ish on the line centroid as a heading offset from the current heading
  (`KP_LINE` 0.3, `KD_LINE` 0.15, as tuned on the robot).
- **No stop rule:** the run does not stop at the line's end; the robot is powered off there.
  `LINE_END_LOST_MM` exists and is 0. If it is left on, a lost line is searched for by turning
  toward the side it was last seen on, for `LINE_LOST_TIMEOUT_MS` (3 s), then the motors are cut
  (`LineLost`).
- **Ramp:** an all-white view is ignored above `RAMP_INCLINE_COUNTS` (a lifted array reads dark).
- `SECTION_A_FOLLOW_LINE = false` stops on the marker's tile instead (`Done`), for a first run that
  does not leave the maze.

## Consequences

- The mission has two more phases (`Approach`, `Line`), appended so the logged numbers of the
  older ones stay.
- The 8-minute run limit applies through the line.
- Section B and the bridge's far end are not handled; nothing stops the robot there.

## Alternatives considered

- **Stop on the marker and start the follower from rest:** the follower needs ~100 mm/s to move
  (the wheels stick below it), and a standing start on a slope is the worst case.
- **Take the line view alone as the hand-over:** the strip and the dark patch after the gap both
  read as a line for a moment.
