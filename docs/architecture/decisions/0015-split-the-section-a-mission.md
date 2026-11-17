# 0015: Split the Section A mission into the explorer, the marker exit and the mission

- **Status:** Accepted
- **Date:** 2026-11-17

## Context

`SectionA.h` was 763 lines and about 50 state fields. It carried the explore loop (sense, turn,
step, votes, peeks, rescans, resumes), the approach marker and the way out of its tile, the drive over
the gap, the line follow, and the one motion controller. Ten of its booleans stood in for sub-states
(`entered`, `decided`, `resumeNext`, `afterStop`, `finishHere`, `brakingAtMarker`, `peeking`,
`exiting`, `choosingExit`, `gapSeen`), and the marker code reached into the explorer in about eight
places. Section B needs the same explore loop on a 17x17 map
([0014](0014-explore-section-b-in-a-start-relative-map.md)), with a different thing ending the
search (a full-white finish tile), so the loop could not stay inside a mission that ends on the
approach marker.

## Decision

Split by job, keeping `SectionA` as the mission that composes the parts:

- `CellExplorer<N>` (`mission/CellExplorer.h`): sense, turn, step on an `ExplorerT<N>`. It does not
  know what ends the search. The caller can ask it to stop on the tile it is heading for
  (`cellExplorerStopHere`), tells it each tick whether a stop brakes at once and whether one is due
  only once the step has ended (`ExploreInput`), and is told when it has explored everything or has
  stopped (`ExploreStatus`).
- `MarkerExit` (`mission/MarkerExit.h`): tell the marker, find the way out of its tile (`Stand`,
  `Turn`, `Approach`), drive over the gap and decide the hand-over. `BridgeHandover` moves here from
  `sense/FloorPattern.h`: it is mission logic, not sensing.
- `SectionA` (`mission/SectionA.h`): the stage (`WaitStart`, `Explore`, `Exit`, `Line`, `Done`,
  `Fault`), the single `motionControlStep()` call, and the line follower. The parts fill a
  `MotionTarget`; only the mission applies it.

The ten booleans become one sub-state enum per part (`ExplorePhase`, `ExitPhase`) and the mission's
stage. `finishHere` is now `marker.atMarker`: it meant the approach marker, not the finish tile
(ADR 0010 stands as written; Section B will have a real finish). The status line keeps its phase
numbers: the mission derives `SectionAPhase` from the stage and the parts' phases.

The split moves code and changes no behaviour. The check was the output of the mission on every tick
of `test_section_a` and `test_run_loop` (about 1.1 million): the phase, fault, motor duty and enable
are the same before and after.

## Consequences

- A Section B explorer is a `CellExplorer<9>` with its own stop source and mission; nothing in the
  explorer names the marker or the line.
- `SectionAParams` inherits `CellExplorerParams` and `MarkerExitParams`, so a parameter keeps its name
  and a part takes only the slice it needs.
- `wallsOnly` is read by both the explorer (the search ends when everything is explored) and the
  mission (no marker to look for).
- The line follower is not a part of its own: `LineFollow` already is that unit. The end of a
  Section B entry line (black floor instead of white) is added to it when Section B needs it.
- `CellExplorer.h` is about 550 lines, over the 300 the plan aimed for: it is one cohesive loop.
- The `Braked` stop (a stop that brakes at once, used with `followLine` false) is not reached by any
  test.
- The mission's name stays `SectionA`; a manager for more than one mission waits for the second.

## Alternatives considered

- **A policy template for the stop** (a detector type the explorer calls). The explorer would then
  contain the two hook points itself; a plain input struct says the same with no template machinery.
- **Keep one file and only name the sub-states.** Fixes the flags, not the reuse.
- **Rewrite the explore loop while splitting it.** Rejected: a move that also changes behaviour
  cannot be checked tick by tick.
