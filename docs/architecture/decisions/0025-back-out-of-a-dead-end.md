# 0025: Back out of a dead end instead of turning round in it

- **Status:** Accepted
- **Date:** 2026-11-26

## Context

On the team's practice maze (2026-11-26) the robot's tail hit a wall while it turned round in a dead
end. It did not happen on every run. The maze's cells measure 24.6 to 26.5 cm, not one size.

How the explorer handled a dead end before this change: the step reads the cell's three walls 50 mm
before its centre and stops on the centre, with the stop placed by the end wall. The planner then
says to go back the way the robot came, and the robot pivots 180 degrees in the cell, reads the walls
again and drives out. A stop by the end wall lands 109 to 126 mm from it (`spec/tuning/WallFollow.h`),
and a half turn ends with the tail pointing at that wall, after sweeping it past both side walls.
How far the tail reaches from the axle has not been measured.

The cell the robot came from is known: it drove through it a moment ago, along the line it would
reverse on. Straight moves already run backwards (`straightBegin` with a negative distance, tested in
`test_motion_primitives`).

## Decision

**When a step stops in a cell and the planner's next direction is straight back over the edge it
came in by, the robot reverses one cell instead of turning round.** It does not turn in that cell.

- No separate dead-end test: a dead end is the case where the planner turns back, because its other
  three edges are walls. A cell whose open edges lead only to known places is backed out of the same
  way.
- Only after a step that drove in and stopped on its decided centre. Not after a stop that was asked
  for (the marker tile) or a step that was cut short.
- The reverse is one cell (`TILE_PITCH_MM`) from where the robot stands, not to where the odometer
  put the centre before. A dead end's stop is placed by its end wall, and the centre before may have
  been rolled through on the odometer alone. In the host tests this lands within 11 mm of the
  centre; aiming at the odometer's centre landed up to 17 mm off.
- The gyro holds the heading. The wall centring is off: steering from the side walls while reversing
  turns the wrong way, and it is one cell along a line just driven.
- On the map the robot moves back a cell still facing the same way (`explorerRetreat`), and plans
  from there with what it already knows of that cell.
- `SECTION_A_BACK_OUT` (`spec/tuning/CellExplorer.h`) switches it off.

## Consequences

- No half turn in a dead end the robot drove into. The turn it makes instead, in the cell before, is
  usually a quarter turn.
- Faster. Exploring every tile of the photo maze in the host world took 91.8 s instead of 101.4 s,
  with four back-outs: about 2.4 s each (a half turn, a look and a drive back, against a drive back).
- A dead end two or more cells deep still ends in a half turn, in the cell before the last one, where
  the side walls are the same but the end wall is a cell further away.
- A new status-line phase, `Back` (8), added after the older ones.
- The explorer is shared, so Section B's maze backs out the same way once it is explored. If the
  finish can be a dead end, the robot must not back out of it: the finish has to be told during the
  step, as the marker is, so that the stop is asked for before the step ends.
- The reverse keeps whatever sideways offset the robot drove in with.
- On the robot (practice maze, 2026-11-26) two runs backed out once each, both from a cell walled on
  both sides whose way on was already explored, not from a dead end of three walls. Each reversed
  254-255 mm in 2.1 s with the heading within 2 degrees of square after the move (84-92 and -2 to
  +2 degrees) and the side walls read at 104-129 mm throughout.

## Alternatives considered

- **Stop farther from the end wall before the half turn.** Cheap, but keeps the 2 s half turn and the
  sweep past both side walls.
- **Decide a few centimetres into the cell and reverse without reaching the centre.** The walls are
  already read 50 mm before the centre, so it saves only about 0.3 to 0.6 s per dead end, on top of
  this.
- **Reverse until the planned path bends.** Removes the half turn at the end of a deeper dead end too.
  Left for later: it is a longer reverse with no wall to place its stop.
- **A smaller turning circle (round the chassis's corners).** Helps every turn, not only in dead ends,
  but needs the tail's reach measured first. Not instead of this.
