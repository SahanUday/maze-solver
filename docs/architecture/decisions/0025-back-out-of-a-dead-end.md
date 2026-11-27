# 0025: Back out of a dead end instead of turning round in it

- **Status:** Accepted
- **Date:** 2026-11-26

## Context

On the team's practice maze (2026-11-26) the robot's tail hit a wall while it turned round in a dead
end. It did not happen on every run.

How the explorer handled a dead end before this decision: the step reads the cell's three walls 50 mm
before its centre and stops on the centre, with the stop placed by the end wall. The planner then
says to go back the way the robot came, and the robot pivots 180 degrees in the cell, reads the walls
again and drives out. A stop by the end wall lands 109 to 126 mm from it (`spec/tuning/WallFollow.h`),
and a half turn ends with the tail pointing at that wall, after sweeping it past both side walls.

The room is small (SolidWorks model, 2026-11-27, read from its measurements and views; not measured on
the robot):

| From the middle of the axle | Distance |
|---|---|
| To the rear plate / to the back of the parts on it | 72 mm / about 90 mm |
| To the front of the IR board | 87 mm |
| To a tyre's outer face (186 mm across) | 93 mm |
| To the farthest points, which a pivot sweeps in a circle | about 99 mm at the tyres, up to about 110 mm at the rear corners |

How far a centred robot is from each side wall is not settled. The sensors read 231-235 mm across
(about 117 mm a side), which leaves a pivot about 7 to 20 mm to spare. The team's tape measurements of
the practice maze (2026-11-27) give 246 to 265 mm between wall faces (123 to 132 mm a side). Either
way a robot that stopped well off the centre can touch, which fits a hit on some runs only.

The tiles are not one size, and the code's pitch (250 mm) is nominal. A reverse of a fixed length
ends off by the difference.

The way the robot came is known: it drove it a moment ago, along the line it would reverse on, and the
step still holds where that move began (`StepCell::originMm`) and how many tiles it covered. Straight
moves already run backwards (`straightBegin` with a negative distance, tested in
`test_motion_primitives`).

Other mice mostly turn round in place (UKMARS `mazerunner-core`, `turn_back`), stopping by the wall
ahead first; their bodies pivot well inside a cell. Ours may not. A wheel-slip study
(micromouseonline.com, 2022-05-31) finds slip while speeding up makes a robot read long and while
braking short, which largely cancels over a move from rest to rest.

## Decision

**Where the way on from a tile the robot has driven into is straight back over the edge it came in
by, the robot reverses instead of turning round: a tile at a time, planning again after each, as far
back as the move that drove in began.**

- No separate dead-end test: a dead end is the case where the planner turns back, because its other
  three edges are walls. A tile whose other open edges lead only to known places is left the same way.
- No maximum number of tiles. Planning after each tile stops the reverse where the way turns; past the
  start of that move the robot turns round. `SECTION_A_BACK_OUT_MAX_TILES` can set a limit.
- It holds while exploring and while driving to a chosen tile (Section B's proof). The explorer
  remembers that it stands on the way of the last move until it turns, so it still knows after the
  stand-and-look that begins a drive.
- Not after a move that was cut short or ended without a verdict on the walls, and not from where a
  run or a part starts: the robot has not driven in.
- **Length:** what the move that drove in really covered, divided by the tiles still to go back, at
  most `BACK_OUT_MAX_OVER_MM` over the pitch (`stepCellBeginBack`), with no least length. Unless that
  limit cuts it, the last tile ends where the move began. The arena's own tile size is used and a
  steady odometer error cancels.
- **Steering:** the gyro holds the heading, trimmed by the forward move's wall steering
  (`wallSteerTrim`, same gains) with the two sides swapped, because backing, a heading to the left
  carries the robot to the right. The heading error it has learned is the same either way, so it is
  carried into the reverse and out of it.
- On the map the robot moves back a tile still facing the same way (`explorerRetreat`), and plans
  from there with what it already knows of that tile.
- A reverse that stops making progress is accepted close to its end, standing still, as a step is;
  otherwise the run ends in a fault.
- `SECTION_A_BACK_OUT` (`spec/tuning/CellExplorer.h`) switches it off.

## Consequences

All figures below are from the host world, most from scratch harnesses that are not in the repo.

- No half turn in a dead end the robot drove into, in either maze, as long as the way back is part of
  one move. A way back that was driven in two moves is reversed to the start of the last, and the robot
  turns round there.
- Against reversing one tile of a fixed pitch on the gyro alone, over 12 fault settings (uneven
  tiles, odometer, gyro, ultrasonic noise, dropouts and false echoes, wheel slip), runs succeeded at
  least as often in each. With uneven tiles they rubbed a side wall less often (114 against 211 of
  1,100 with the team's tile sizes and the code still saying 250 mm); with the wheels slipping one way
  on the way in, more often (133 against none of 1,100 at 5 %).
- In the same comparison with the team's tile sizes, the typical end of a reverse went from 25 mm off
  the tile's centre to 7 mm.
- It ends where the move began. A move begun off the centre is reversed to off the centre, so in a
  perfectly even maze it ends slightly farther out than a fixed pitch would (typically 0.4 mm).
- Faster. Exploring every tile of the photo maze takes 91.5 s instead of 101.4 s, with four reverses.
- **Wheel slip one way is the risk, and it grows with the length of the reverse.** If the wheels slip
  on the way in and not on the way back, the measured length is too long. Over a corridor of eight
  tiles, 2 % ended with the axle 80 mm from the wall behind and 5 % with it 16 mm from it, against about
  90 mm of tail. Slip of 5 % only while speeding up and braking stayed 110 mm clear. How much this robot slips
  has not been measured; if it does, set `SECTION_A_BACK_OUT_MAX_TILES` to 1.
- The wall steering matters over a long reverse. With the gyro 4 degrees wrong over eight tiles, the
  robot rubbed a side wall in 200 of 200 runs on the gyro alone, in 127 with a proportional
  correction only, and in none with the learned one.
- A reverse does not look behind: there is no sensor there, and no wall to place its end.
- Every other pivot beside a wall has the same small margin as before. This removes the ones the robot
  can do without; it does not make the body's turning circle smaller.
- A new status-line phase, `BackOut` (8), added after the older ones.
- The finish (#69) has to be told during the step onto it, as `FinishSource.h` already requires: a
  reverse does not ask for it.
- On the robot (practice maze, 2026-11-26), with an earlier version that reversed one fixed pitch on
  the gyro alone: two runs reversed once each, both from a cell walled on both sides whose way on was
  already explored. Each reversed 254-255 mm in 2.1 s with the heading within 2 degrees of square.
- On the robot (practice maze, 2026-11-27), as decided here but still with a least length, which
  acted in none of these: one exploring run, walls only with the exit blocked, ended without a fault
  after about 98 s and reversed three times.

  | Move driven in (odometer) | Reversed | Ended, from where the move began |
  |---|---|---|
  | 529 mm, two tiles | 528 mm | within 5 mm |
  | 282 mm, one tile | 280 mm | within 5 mm |
  | 292 mm, one tile | 279 mm | 10-13 mm short, cut by the limit |

  The heading stayed within 7 degrees of square and ended within 4. Moves in that run averaged more
  than the 250 mm pitch a tile (#70), so the limit can cut a true length. The robot still turned round
  once in a dead end, after a step cut short at 190 mm when the front reading jumped. Not recorded:
  whether the body touched a wall. Not seen: a reverse of three tiles or more. Not measured: wheel
  slip.

## Alternatives considered

- **Stop farther from the end wall before the half turn.** Cheap, but keeps the 2 s half turn and the
  sweep past both side walls.
- **Turn round, as most mice do.** Needs a body that can pivot in the tile.
- **Reverse a fixed pitch.** Ends off by however much the tile differs from 250 mm.
- **Also a least length, as far under the pitch as the limit is over it.** A move that drove less than
  that would be reversed past where it began, toward a wall no sensor sees. In the host world,
  leaving it out changed little either way.
- **Reverse one tile, then turn round.** Leaves the half turn between side walls in every deeper dead
  end. It is the setting to use if the wheels slip.
- **Cap the length at the nominal pitch.** Meant for the slip case; with the code's pitch below the
  arena's it ended short every tile and rubbed more.
- **Keep a list of the odometer at every tile.** Nothing measures where the tiles driven through are,
  so the list would hold assumed positions; the start of the move and the tile count are enough.
- **The gyro alone, or a proportional wall correction.** Drifts over a long reverse (above).
- **Check the length against the front sensor.** It reads true to about 3 mm only up to 300 mm, so it
  covers the first 18 cm of one reverse.
- **Stop 10 mm early when the map shows a wall behind.** Keeps a little farther from it, at about 3 mm
  of centring in every reverse.
- **Reverse only straight after the step that drove in.** The proof starts every drive by standing and
  looking, so it never reversed: in 400 simulated runs of Section B the robot turned round on a
  dead-end finish in 199 of them.
- **A smaller turning circle (round the chassis's corners).** Helps every turn, not only in dead ends,
  but needs a change to the chassis. Not instead of this.
