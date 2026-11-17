# 0016: Cut the run into parts where the robot's logic changes

- **Status:** Accepted
- **Date:** 2026-11-17

## Context

To a person the course is three regions: Section A, the bridge, Section B. The robot does not run one
logic per region: what it steers on, which sensors it trusts and what it is looking for change at
other points. Cutting the run in the wrong places would mean a part that runs two kinds of logic, or
a boundary the robot cannot tell. The run also has to be testable in pieces and in stretches (Section A
and the bridge, the bridge and Section B), on the host and later on the robot.

Checked before deciding:

- **Rules (`docs/Maze_Solver.pdf`, p. 3-4 and Figure 1).** Each maze has a corner tile *inside* it with
  the approach marker on it, and the white board with the 30 mm line attaches to that tile through the
  wall opening. So the line zone has a marker tile at both ends: Section A's is left through, Section
  B's is entered by.
- **Code, per stage.** Exploring steers on the walls and snaps its heading to them (`StepCell`),
  drives at the search speed and reads the marker with the floor pattern. The way out of the marker
  tile reads both sides while standing, then drives straight along the map heading with no wall
  steering, reading the floor for the gap. The line follower steers on the IR line centroid, takes
  its heading from the gyro plus that offset, has no map, and reads the tilt.
- **Team design document, section 7.** Its mission phases (explore A, prove A, exit A, bridge, explore B,
  prove B) agree with these cuts; it has no phase for the entry onto Section B.

## Decision

The run is cut into seven **parts**, in course order, at the points where the logic changes. A
boundary is an **event the robot senses**, never a clock or a position it was told.

| Part | Where | Logic | Ends when (the event) | Carried to the next |
|---|---|---|---|---|
| `AExplore` | Section A's maze | explore cell by cell, wall steering, start-relative map, marker watched | the marker pattern is told (the robot stands on its tile, or goes straight through it) | the map, the marker's tile |
| `AProve` | Section A's maze | explore the rest, prove the path, return to the marker tile (#62) | no unknown edge on the shortest path, and the robot is on the marker tile | the map |
| `AExit` | marker tile to the board | stand and read both sides if a wall is ahead, turn, drive straight, watch the floor for the gap | the line is in view past the gap | nothing: the robot is moving |
| `Line` | the line zone: bridge, ramp, bends | steer on the IR line centroid, tilt-aware, no map | the line ends onto the maze floor (#60) | nothing |
| `BEntry` | Section B's marker tile | off the line, onto the maze floor, to the tile's centre, a new map origin (#60, #61) | the robot stands centred on the tile | the map origin |
| `BExplore` | Section B's maze | the same explorer on a 17x17 map; the finish is watched, **the marker is not** (the entry tile has one) | the finish is told (#69) | the map |
| `BProve` | Section B's maze | after the finish: prove the shortest path, save the maps (#61, #63) | proved, or the run limit | nothing |

What is **not** a part:

- The ramp, the gap before it and the bends are *conditions inside* a part, told by the tilt, by the air
  reading under the array and by the line view. The logic (follow the line) does not change.
- Waiting for the start, done and fault belong to the manager.
- Exploring and *running a stored path* are two modes of `AExplore` and `BExplore`, not parts (the
  speed run, #66).

**How the robot knows which part it is in.** A run begins at the part chosen for it (a competition run:
`AExplore`, on the start tile) and moves on only when the running part reports its own end event. The
sensed floor alone cannot name the part: the maze floor with walls is Section A and Section B alike, so
the order tells them apart.

**The manager** (`MissionManager`) runs the parts from `first` to `last`, so one part alone and any
stretch of parts are the same thing as the whole course:

- `first` must be built and able to start alone: it needs nothing from the part before it. `AProve`,
  `AExit` and `BProve` continue a map and cannot begin a run. A selection that cannot run is refused
  with `BadConfig` before the robot moves.
- `last` is the last *built* part up to the one asked for, so unbuilt parts are skipped. With no line
  calibration (`wallsOnly`) the run ends with `AExplore`: there is no marker to find or line to follow.
- A part ends with **Done** (the next part begins on the next tick) or **HandOver** (it ended while
  moving on, so the next part acts in the same tick and the robot does not stop between them).
- The manager owns the start, the run limit and the one `motionControlStep()`; a part only fills a
  `MotionTarget`.

## Consequences

- Each part has an entry and an exit event that can be tested in its own simulated world; the bridge and
  Section B worlds are #59.
- The marker is looked for only in `AExplore`. Section B's entry tile has a marker on it and it is not a
  goal.
- `SectionA` is renamed `MissionManager`; `followLine` is gone, because `last = AExplore` says the same.
  The phase numbers in the status line are unchanged, and a part number is added to the phase line.
- Starting a part alone relies on where the robot is placed. A part that is started where it does not
  belong is not recognised from the floor (see above); it fails through its own limits (no line, a
  stall).
- The change moves code and changes no behaviour: the mission's output on every tick of
  `test_section_a` and `test_run_loop` (about 1.13 million) is identical before and after.

## Alternatives considered

- **Three parts (A, bridge, B).** Rejected: both marker tiles have their own logic and their own
  boundary event, and Section B's entry cannot be told from Section B's exploring if they are one.
- **Boundaries by distance or time.** Rejected: tile pitch and speeds vary; the events are sensed.
- **A classifier that names the part from the sensors at any moment.** Rejected for now: the floor and
  the walls do not separate Section A from Section B, and a wrong guess mid-run is worse than none. The
  order of events carries the knowledge.
- **A part per sub-condition (ramp, gap, bends).** Rejected: no change of logic, and the boundaries are
  not clean events.
