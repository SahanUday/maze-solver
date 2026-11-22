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
  `spec/Mission.h` values.
- `cellExplorerStart(ce, fb)` - begin from where the robot stands: map North is its heading now.
- `cellExplorerStep(ce, params, input, perception, target)` - one tick. Fills the `MotionTarget`
  (the caller owns the motion controller) and returns an `ExploreStatus`:
  `Running`, `Explored` (every reachable tile entered, `wallsOnly`), `AtTarget` (standing on the target
  tile, see `cellExplorerDriveTo`), `Stopped` (standing on the tile a
  stop was asked for), `Braked` (a stop that braked at once, wheels still) or `Fault` (`ce.fault`:
  `NoRoute`, `StuckTurn`, `StuckStep`, `TooManyStops`, `TargetUnreachable`).
- The caller's say in what ends the search: `cellExplorerStopHere(ce)` asks for a stop on the tile the
  step is heading for, and `ExploreInput` says each tick whether a stop brakes at once
  (`brakeOnStop`) and whether one is due only once the step has ended (`lateStopOffer`; the explorer
  sets `lateTaken` when it takes it).
- `cellExplorerDriveTo(ce, target)` - drive to the map tile `target` (an `ExplorerT<N>::Index`) instead of
  the nearest unvisited one. Call it standing on a centre, after `Explored` or `AtTarget`; it lasts
  until the next reset.
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

**Drive-to-tile mode** (`cellExplorerDriveTo`) runs the same Sense, Turn and Step; only the choice of the next
tile differs. The planner is replaced by `explorerRoute` ([maze](maze.md)): the first step of the
shortest path over checked edges, keeping out of the start tile when another way exists. The way
ahead is a checked edge, so the explorer does not look again at a silent front, peek or retry it, and
rolls through a centre when the route goes straight on. It still reads the walls on the way, and a
wall it finds changes the route. On the target tile it stops on the centre and returns `AtTarget`; with
no path over checked edges it faults with `TargetUnreachable` and forgets nothing (no rescan). A tile the
robot drove through but no sensor heard open is not on a route: see the Status.

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

Drive-to-tile mode is tested by exploring the Section A arena from four start tiles, then driving to the marker
tile: it stops within 40 mm, touches no wall and does not cross the start tile; with every checked bit
cleared the same target is `TargetUnreachable`. It has not run on the robot. Open question for the Section A
return (A-02): an edge the robot crossed but no sensor heard open stays unchecked ([maze](maze.md)),
so a route over it is refused. The noise-free simulation never hit one; a run with missed readings
has not been tried.
