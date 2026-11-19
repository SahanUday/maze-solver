# 0017: Explore, then prove, then run the proven path

- **Status:** Accepted
- **Date:** 2026-11-19

## Context

The rules (`docs/Maze_Solver.pdf`, p. 5-6, re-read 2026-11-19) give up to three trial runs, the best
time counts, each run lasts at most 8 min (+2 min grace), and the whole arena access is 30 min
including setup between runs. Touching the robot aborts the attempt only *during* a run; after a
completed run it may be removed, and batteries, sensors, speed settings and switches may be changed.
Both ends are unknown to the robot: the start tile is anywhere in Section A, the finish anywhere in
Section B, and "no external information may be provided to the robot" once the maze is revealed.
After the finish the robot "may continue exploring"; re-entering the start tile aborts the attempt and
a new one begins.

So a person moves the robot back to START between trials (no drive B -> bridge -> A), and the finish
is not known until it is found. A run that only explores until the finish gives a time, but not the
fastest route; a run that explores everything is slow and may run out of its 8 minutes.

## Decision

The three trials have three jobs.

1. **Trial 1 explores.** Section A is explored until the marker, then the line, then Section B until the
   finish is found. The finish is a goal the robot discovers, not one it is given. At the first finish
   the map is saved (see below). The robot then goes on exploring only the tiles that could still give
   a faster route (the proof, [0018](0018-route-by-estimated-time-and-prove-it.md)) and stops when the
   best known route is proved the best possible, or the run limit.
2. **Trial 2 runs the best route known, with safe settings.** It is the same path trial 3 will run, at
   a speed where a wrong map costs a stop, not a crash. Walls along it are checked as it goes: a wall
   that is there and the map says is open stops the robot, is marked, and the route is planned again.
3. **Trial 3 runs the same path with the maximum settings** found in the robot tests. The best time of
   the three counts.

Speed levels are a switch setting (rules allow changing switches between runs), not code that is
edited. Trial 2 uses the safe level and trial 3 the fast one.

Each part of a run is a part of the mission ([0016](0016-cut-the-run-into-parts.md)): `AExplore`,
`BExplore` and the proof are the exploring mode; the stored-path run is a second mode of the same
parts, not a new part.

**Keeping the map across trials depends on a rules answer that has not come.** The rules do not say
whether a robot may keep its own map in EEPROM across a power cycle. This ADR assumes it may. If it
may not, every trial explores again with the same proof loop, and the best time of the three still
counts; nothing in 0018 or 0019 changes, and trials 2 and 3 lose only the stored path. Until the
answer comes in writing, the map store (#63) is built so that using it is one switch.

## Consequences

- The robot needs a stored map (#63), a way to tell its start orientation in a later trial (#65), and
  a stored-path run (#66). The proof loop (#61, #62) is part of trial 1, not extra work.
- The run clock stops at the finish tile; what the robot does after it is not timed but still counts
  against the 8 minutes unless the rules say otherwise (not settled; to ask).
- Re-entering the start tile while exploring Section A aborts the attempt, so exploring all of
  Section A is expensive (#62 is low priority for that reason); how an aborted attempt counts toward
  the three trials is not settled either.
- Host simulation can prove the logic; nothing here has been run on the robot.

## Alternatives considered

- **Explore until the finish, then run the first route found.** Simple, but the first route is
  not always the fastest; in one simulated maze it took 25.8 s where the proved route took 22.4 s.
- **Explore everything in trial 1.** Slow, and re-entering START aborts the attempt.
- **Fast settings in trial 2.** A wall the map got wrong would be hit at the fastest speed with the
  trial wasted.
- **Different paths in trials 2 and 3.** Trial 2 would then verify a path trial 3 does not run.
