# Testing the maze logic with the simulator: plan

**Status: proposal. Nothing in this file is built.** It is here so work can continue from a shared
idea, and it is expected to change: edit it freely, and note below what changed and why. Once a
piece is built, its facts belong in `docs/architecture/` (see `CLAUDE.md`); delete the matching part
here.

## Goal and scope

Check the maze-solving part of the firmware before it goes on the robot:

- exploring an unknown section and building its map,
- finding the shortest path once the finish is known,
- running that path as fast as possible, and measuring how efficient each stage is.

Out of scope: the motion, sensing and line-following code, the bridge, and anything about timing
on the real hardware. Section A (4x4) and Section B (9x9) are tested **separately**, as different
maze files in one generic setup; joining them (A's exit into B's start tile) is a later step.

## Where it fits

| Layer | Runs where | Role |
|---|---|---|
| Unit and simulated-arena tests in `test/` | PC, headless, in CI | The authoritative, gating tests. |
| Simulator (this plan) | PC, GUI | Seeing the algorithm work, trying other mazes, finding where it wastes moves. Not a CI gate. |
| The robot and the real maze | Hardware | Everything the simulator cannot show. |

The simulator never replaces the first row. A bug it finds should become a test in `test/`.

## Proposed design

```
 maze file ──┬──────────────► adapter ◄── lib/maze (unchanged, no Arduino)
 (walls,     │                  │
  start,     │                  ├── backend 1: mms     (stdin/stdout protocol, to watch it)
  goal)      │                  └── backend 2: headless (in-memory maze, for many runs)
             └─ read by mms too                        │
                                                       └─► metrics report
```

- **Maze file.** The mms map format, so mms can load it and the adapter can read the same file.
  mms ignores text inside a cell, so the start and goal can be written there. Proposed: `^ > v <`
  for the start tile and heading, `G` for the goal tile(s). Alternative: pass them on the command
  line. The mms start and goal are fixed (cell `(0,0)` facing North; the centre), so they cannot be
  used.
- **Adapter (the adoption layer).** A host program that runs the maze logic against a small
  "mouse" interface: read the three walls, turn, move one tile. It includes `lib/maze` headers
  directly, so what is tested is the real code and not a copy. Where it lives is open. If it is
  outside `src/`, `include/` and `lib/`, the firmware bans (no heap, no STL) do not apply to it,
  but clang-format does. A plain `g++` build avoids adding a PlatformIO env, which would have to
  be described in `docs/testing.md`.
- **Backend 1, mms.** Wall readings come from `wallFront/Left/Right`; the logic's turn and advance
  become `turnLeft/Right` and `moveForward`. The adapter also draws what the logic believes with
  `setWall`, and cell numbers or paths with `setColor` and `setText`.
- **Backend 2, headless.** The same adapter against an in-memory copy of the maze. This is where
  batches belong (every start tile and heading, sensor-miss rates, many mazes), because mms is
  animated and GUI-only. `Sim` in `test/test_maze/test_maze.cpp` is the seed of it today.
- **Metrics.** Count tiles driven, turns, stops to sense, and tiles entered more than once; count how
  many straights could be one multi-tile move (what a fast run needs). Estimate time by giving each
  of those a cost from the constants in `include/spec/`, for example `SECTION_A_SENSE_MS` for a
  stop. Host CPU time is not AVR time; use operation counts (such as cells expanded per plan) and
  the existing RAM budget instead.

## Gaps between the simulator and the robot

| mms | Robot | Handling |
|---|---|---|
| Starts on `(0,0)` facing North | Any tile, any heading | Adapter drives to the chosen start first (it knows the maze file), then starts the logic fresh. Not tried. |
| Goal is the centre | Finish tile found by the floor colour | Adapter knows the goal and stops there. |
| Perfect wall reading | Ultrasonic with misses and a dead zone | Adapter drops readings at random, as `test_maze.cpp` does. |
| Does not say which maze is loaded | n/a | Adapter checks size and the first readings against its own copy and stops on a mismatch. Not tried. |
| One rectangle | Two sections joined by a bridge | One file per section for now. |

## What has to exist first (in the firmware code)

- `lib/maze/Maze.h` is fixed at a 7x7 map built from `SECTION_A_SIZE_TILES`, and its planner packs
  path length into 6 bits (`static_assert(kCells - 1 < 63)`). A 9x9 map has 81 cells, so Section B
  needs a configurable size and wider packing. `SectionA.h` and two tests use the current names.
- Shortest-path, flood-fill and fast-run code does not exist yet, so there is nothing to test there
  until it is written. Write it so it can run against the adapter.
- Whether Section B's map is anchored to its corner or relative to the start is undecided; the
  simulator can test either.
- Mazes for A and B still have to be drawn or generated (the rules say wall-hugging alone must not
  be enough, so include loops and islands).

## Order of work

1. Adapter and headless backend, checked on the existing 4x4 photo maze against `test_maze.cpp`.
2. mms backend, to watch it run.
3. Generalise `Maze.h`, then Section B on its own.
4. Shortest path and fast run, with the metrics.
5. Joining A and B in one file.

## Open questions

- Markers in the maze file or command-line arguments?
- Where does the adapter live, and how is it built?
- What time costs go into the estimate, and who measures them on the robot?

## Changes to this plan

_Add a line here when the plan changes: date, what, why._
