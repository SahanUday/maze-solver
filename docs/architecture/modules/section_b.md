# section_b

Section B's maze: explore from the entry tile until a stop source says "finish", then prove that the
path to it is the shortest. Pure logic in `include/mission/SectionB.h` and `FinishSource.h`, run by the
mission manager as the parts `BEntry`, `BExplore` and `BProve` ([mission_manager](mission_manager.md)).
Tested without a world in `test/unit/test_section_b_proof/` and in the host world's Section B preset in
`test/integration/test_section_b/`. It uses the same explorer as Section A ([cell_explorer](cell_explorer.md)) on
the start-relative 17x17 map of [0014](../decisions/0014-explore-section-b-in-a-start-relative-map.md) and
[0019](../decisions/0019-section-b-worst-case-map-and-fold-for-planning.md), and the proof of
[maze](maze.md) (`explorerProof`, by tile count). The parts are cut in
[0016](../decisions/0016-cut-the-run-into-parts.md).

## Interface

- `FinishSource` (`FinishSource.h`) - what says the robot is on the finish: a plain function pointer
  `told(ctx, perception)` and its context. In `MissionParams::finish`, none by default. The host tests
  plug in the world's finish tile; the real detector (#69) takes its place later.
- `SectionB` - the state: whether the finish is known and its map tile, the proof's step, the edge being
  looked at and its tries, how many looks, the length proved.
- `sectionBEnter(sb, ce, fb, closeBehind)` - begin the map on the tile the robot stands on (North is its
  heading). `closeBehind` marks the edge behind it a wall: it came through the bridge's opening, which
  is no way into the maze.
- `sectionBExploreStep(sb, ce, params, finish, perception, target)` - one tick of the exploring. Returns
  `BStatus::Done` when the finish was told and the robot stands on its tile (its map tile is
  `sb.finish`), `Fault` (`sb.fault`) or `Running`.
- `sectionBProveStep(sb, ce, params, perception, target)` - one tick of the proof.

## How it works

**BEntry** begins the map with the edge behind the robot closed, and ends at once. It does not move: how the
robot gets from the line onto the tile is #60.

**BExplore** is `cellExplorerStep`. While it steps, each tick it asks the finish source; when it says
"finish" the explorer is asked to stop on the tile it is heading for, as the marker is in Section A. The
marker is not watched: the entry tile has one on it and it is not a goal. With no source plugged in the
search ends when every reachable tile has been entered (`wallsOnly`) and no finish is known; with one
that never says so the search ends in `NoRoute`, after the rescans.

A rescan forgets every wall. The explorer's caller puts the closed edge back after it, or the explorer
would take the tiles outside the maze for unvisited ones and drive out of the opening.

**BProve** runs after the finish, by tile count: each round it reads `explorerProof(map, start, finish)`.

1. `Proved`: done, `sb.proved` is the length.
2. `NoRoute`: no path even with every unchecked edge open; a fault.
3. `NotYet`: an unchecked edge on the shortest open path could still shorten it. The robot drives to that
   edge's tile over checked edges (`cellExplorerDriveTo`), turns to face the edge if it does not, and
   stands to read it; the standing read marks the edge checked, as a wall or open. Then the next round.

An edge nothing answers for stays unchecked, which would repeat the same look for ever. After
`frontTries` looks at the same edge it is taken as a wall, as the explorer takes a front that stays silent.

**No time limit.** The firmware ends no run on time (ADR 0022): the officials time the trial. The proof
therefore runs until it is proved or a fault ends it, however long that takes.

## Invariants

- The marker is never watched in these parts.
- The edge the robot came in through is a wall on the map for the whole run, rescans included.
- The proof's look at an edge ends: a silent one becomes a wall after `frontTries` looks.
- Only the manager calls the motion controller; these functions fill a `MotionTarget`.

## Limits

- **A part placed alone has no closed edge.** `BExplore` as the first part begins the map where the robot
  stands and cannot tell the bridge's opening from an exit of the maze: it can drive out through it
  before the finish is found, whenever the opening becomes the nearest unvisited tile. The 9x9 fold
  (#75) is what bounds this. Section B alone is run from `BEntry`.
- **The proof is by tile count on the 17x17 map**, so it may look at edges of the maze's outer wall that
  the folded 9x9 map would not need. The time version is #76.
- **The finish is the robot's centre over the tile in the tests.** The real detector reads the white
  under the array, ahead of the axle; it is #69.
- **Host only.** The marker B and tile sizes in the Section B preset are those of A until they are
  measured (#54). Nothing of Section B has run on the robot.

## Status

In the Section B preset, from every corner and either opening, on two seeded 9x9 mazes: the robot
explores from the entry tile, stops within 40 mm of the finish tile's centre with no wall contact and
never leaves the maze, and the path it holds after the proof is as long as the generator's own shortest
path. The longest of those 16 runs needs 455 s (explore 62-354 s), and one needs more than 8 minutes (the officials' limit; the firmware does not enforce it).
`BExplore` alone (facing the finish from a tile next to it), no source, a source that is never told
(`NoRoute`) and a turn that cannot move (`StuckTurn`) are each tested.
