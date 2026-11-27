# 0025: Back out of a dead end instead of turning round in it

- **Status:** Accepted
- **Date:** 2026-11-26

## Context

On the team's practice maze (2026-11-26) the robot's tail hit a wall while it turned round in a dead
end. It did not happen on every run. The maze's cells measure 24.6 to 26.5 cm, not one size.

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

A centred robot has about 117 mm to each side wall (the sensors read 231-235 mm across), so a pivot
has about 7 to 20 mm to spare, and a robot that stopped off the centre touches. That fits a hit on
some runs only.

The cell the robot came from is known: it drove through it a moment ago, along the line it would
reverse on. Straight moves already run backwards (`straightBegin` with a negative distance, tested in
`test_motion_primitives`).

## Decision

**Where the way on from a tile the robot has driven into is straight back over the edge it came in
by, the robot reverses one tile instead of turning round.**

- No separate dead-end test: a dead end is the case where the planner turns back, because its other
  three edges are walls. A tile whose other open edges lead only to known places is left the same way.
- It holds while exploring and while driving to a chosen tile (Section B's proof). The explorer
  remembers that it stands on the centre a step drove to until it turns or reverses, so it still knows
  after the stand-and-look that begins a drive: the proof leaves the finish, and a tile it looked
  from, in reverse.
- Not after a step that was cut short or ended without a verdict on the walls, and not from where a
  run or a part starts: the robot has not driven in.
- The reverse is one pitch (`TILE_PITCH_MM`) from where the robot stands, not to where the odometer
  put the centre before. A dead end's stop is placed by its end wall, and the centre before may have
  been rolled through on the odometer alone. In the host tests aiming at the odometer's centre landed
  farther off.
- The gyro holds the heading. The wall centring is off: steering from the side walls while reversing
  turns the wrong way, and it is one tile along a line just driven.
- On the map the robot moves back a tile still facing the same way (`explorerRetreat`), and plans
  from there with what it already knows of that tile.
- A reverse that stops making progress is accepted close to its end, standing still, as a step is;
  otherwise the run ends in a fault.
- `SECTION_A_BACK_OUT` (`spec/tuning/CellExplorer.h`) switches it off.

## Consequences

- No half turn in a dead end the robot drove into, in either maze.
- A reverse needs room behind only for the back of the body. In simulation (a scratch harness) it ends
  with the axle at least 109 mm from a wall behind in the nominal world, and at least 96 mm with
  tiles of 246 to 260 mm or an odometer 2 % off, against about 90 mm of tail. Runs succeeded as often
  with it on as off, odometer and tile-size errors included.
- Every other pivot beside a wall has the same small margin as before. This removes the one the robot
  can do without; it does not make the body's turning circle smaller.
- Faster. Exploring every tile of the photo maze in the host world takes 91.8 s instead of 101.4 s,
  with four reverses: about 2.4 s each (a half turn, a look and a drive back, against a drive back).
- A dead end two or more tiles deep still ends in a half turn, in the tile before the last one, where
  the side walls are the same but the end wall is a tile farther away.
- A new status-line phase, `BackOut` (8), added after the older ones.
- The reverse keeps whatever sideways offset the robot drove in with, and has no wall to place its
  end: an error of the odometer or of the tile's size goes into where it stops, until the next step
  meets a wall.
- The finish (#69) has to be told during the step onto it, as `FinishSource.h` already requires: a
  reverse does not ask for it.
- On the robot (practice maze, 2026-11-26) two runs reversed once each, both from a cell walled on
  both sides whose way on was already explored, not from a dead end of three walls. Each reversed
  254-255 mm in 2.1 s with the heading within 2 degrees of square after the move (84-92 and -2 to
  +2 degrees) and the side walls read at 104-129 mm throughout. A reverse in Section B's proof has not
  run on the robot.

## Alternatives considered

- **Stop farther from the end wall before the half turn.** Cheap, but keeps the 2 s half turn and the
  sweep past both side walls.
- **Reverse only straight after the step that drove in.** The proof starts every drive by standing and
  looking, so it never reversed: in 400 simulated runs of Section B (a scratch harness, not in the
  repo) the robot turned round on a dead-end finish in 199 of them.
- **Decide a few centimetres into the cell and reverse without reaching the centre.** The walls are
  already read 50 mm before the centre, so it saves only about 0.3 to 0.6 s per dead end, on top of
  this.
- **Reverse until the planned path bends.** Removes the half turn at the end of a deeper dead end too.
  Left for later: it is a longer reverse with no wall to place its stop.
- **A smaller turning circle (round the chassis's corners).** Helps every turn, not only in dead ends,
  but needs a change to the chassis. Not instead of this.
