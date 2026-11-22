# cell_explorer

Explores a maze cell by cell: stand and read the walls, turn to the tile the planner picks, drive
it. Pure logic in `include/mission/CellExplorer.h`, generic in the maze size (`CellExplorerT<N>`,
over [maze](maze.md)'s `ExplorerT<N>`; `CellExplorer` is the Section A one). It is tested through
the Section A mission (`test/test_section_a/`). It joins the planner ([maze](maze.md)), the cell
step ([step_cell](step_cell.md)), the turn ([motion_primitives](motion_primitives.md)) and
[perception](perception.md); the decisions are
[0009](../decisions/0009-explore-in-a-start-relative-map.md) and
[0015](../decisions/0015-split-the-section-a-mission.md).

## Interface

- `CellExplorerT<N>` - the state: the `ExplorePhase` (`Sense`, `Turn`, `Step`), the map, the
  `StepCell`, the `Turn`, the votes and the counters. `cellExplorerReset(ce)`.
- `CellExplorerParams` - `StepCellParams` and the explorer's limits (`senseMs`, `frontTries`,
  `rescans`, `resumes`, `turnRetries`, `maxBlocked`, `stuckAcceptMm`, `wallsOnly`, `dtMs`), at their
  `spec/tuning/CellExplorer.h` values.
- `cellExplorerStart(ce, fb)` - begin from where the robot stands: map North is its heading now.
- `cellExplorerStep(ce, params, input, perception, target)` - one tick. Fills the `MotionTarget`
  (the caller owns the motion controller) and returns an `ExploreStatus`:
  `Running`, `Explored` (every reachable tile entered, `wallsOnly`), `AtTarget` (standing on the tile
  `cellExplorerDriveTo` was given), `Stopped` (standing on the tile a stop was asked for), `Braked` (a stop that braked at once, wheels still) or `Fault` (`ce.fault`:
  `NoRoute`, `StuckTurn`, `StuckStep`, `TooManyStops`, `TargetUnreachable`).
- The caller's say in what ends the search: `cellExplorerStopHere(ce)` asks for a stop on the tile the
  step is heading for, and `ExploreInput` says each tick whether a stop brakes at once
  (`brakeOnStop`) and whether one is due only once the step has ended (`lateStopOffer`; the explorer
  sets `lateTaken` when it takes it).
- `cellExplorerDriveTo(ce, target)` - head for map tile `target` over checked edges instead of
  exploring. Call it standing on a centre, after `Explored` or `AtTarget`. It lasts until the next
  reset.
- `cellExplorerBeginTurn(ce, fb, dir)` and `cellExplorerTurnStep(...)` - a turn the caller starts
  (the marker exit uses it), with the same retries.
- `cellExplorerEdgeOpen(ce, dir)`, `cellExplorerHeadingRef(ce)` - the map's view of the tile the
  robot is on, and the gyro heading square to the map's current direction.
- The fault codes (`MissionFault`) are in `mission/Fault.h`.

## How it works

- **Sense:** stand still for `senseMs` and vote on the three walls. The explorer records what was
  heard (a wall only if heard and present) and plans.
- **Turn:** face the planned tile, then Sense again: the new front may be an edge nothing has
  looked at. A stuck turn is retried `turnRetries` times, then `StuckTurn`.
- **Step:** drive one tile. The step reads the walls of the tile ahead 50 mm before its centre;
  there the explorer moves into it, records them and plans. A tile straight on is rolled through
  (`stepCellContinue`), anything else stops on the centre and the planner turns.

**Drive-to-tile mode** (`cellExplorerDriveTo`) runs the same Sense, Turn and Step; only the next tile
differs. It comes from `explorerRoute` ([maze](maze.md)): the shortest path over checked edges,
keeping out of the map's start tile (Section A's START, Section B's entry tile) when another way
exists. The rules say only that re-entering "the START tile" before the FINISH aborts the attempt, so
the entry tile is avoided too. The way ahead is a checked edge, so a silent front is not retried or
peeked at, and a centre on a straight route is rolled through. The walls are still read and recorded
on the way, and a new wall changes the route. On the target tile the explorer stops on the centre and
returns `AtTarget`. With no checked path it faults with `TargetUnreachable`, with no rescan.

A requested stop (`cellExplorerStopHere`) means no wall or planner decides the centre any more: the
step ends on it, and the status is `Stopped`. With `brakeOnStop` the step brakes at once, wherever it
was going, and the status is `Braked` once the wheels have stood still `STOP_STILL_MS`.

### Rules that keep the map true

- A front that stays unheard after `frontTries` looks is a wall in the sensor's dead zone (below
  ~114 mm from the axle): it is recorded as one, not driven at. Unless the way is settled another
  way, which a tile open to nothing needs: an edge a side sensor measured open on the way in
  (`openHeard`), an edge the right-hand sensor found open after a quarter turn to look at it
  (`peekedOpen`: a side sensor has no dead zone), or a cell already entered
  (`explorerAheadVisited`). A silent front through a sampling window, once the sensor has worked,
  is also open.
- A verdict from a step that was cut short is not taken. After such a stop the robot stands and
  looks: with no wall ahead the step is resumed to its planned centre (`resumes` times), with
  one the explorer is put where the odometer says (`stepCellCells`, `explorerRetreat`).
- No route: the map is forgotten and explored again up to `rescans` times (a wall that was only a
  bad reading is the likeliest cause), then `NoRoute`.
- More than `maxBlocked` cut-short steps in a run: `TooManyStops`.

## Invariants

- The explorer never touches `RobotState` or the motion controller; it reads `Perception` and fills
  a `MotionTarget`.
- Map North is the gyro heading at start, and map directions run clockwise where the gyro runs
  counter-clockwise: `gridHeading` is `h0 - 90 * dir`.
- The map's cell and the robot's agree at every stop on a centre: the tests check it.

## Status

Tested through the mission manager in a simulated arena (ray-cast walls, a plant with stiction,
encoder and gyro noise, ultrasonic dropouts and strays): see [mission_manager](mission_manager.md). The `Braked`
path is not reached by those tests. Ran on the robot at the mid-evaluation (tag `v0.1-section-a`) as
part of the Section A run, before the mission was split. The split changed no output on the host:
every tick of `test_section_a` and `test_run_loop` is the same before and after. Not run on the
robot since.

Drive-to-tile mode is tested by exploring the Section A arena from four start tiles and driving to
the marker tile: it stops within 40 mm, touches no wall and does not cross START; with every checked
bit cleared the target is `TargetUnreachable`. It has not run on the robot, and its cost on the AVR
(up to four floods per tile) is not measured.

An edge the robot drove over but no sensor heard open stays unchecked ([maze](maze.md)), so a route
over it is refused. A one-off simulator check (script not kept) found none: in 16 explorations, clean
and with 30 % dropped readings and 5 % stray echoes, every open edge between two entered cells was
checked. Of 45 noisy runs, 9 were refused, all because a false wall cut the target off (walls are
never erased). What the Section A return (A-02) does on `TargetUnreachable` is for that issue.
