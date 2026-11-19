# 0017: Explore, then prove, then run the proven path

- **Status:** Accepted
- **Date:** 2026-11-19

## Context

What `docs/Maze_Solver.pdf` says, quoted, with its page:

- p. 5: "a maximum of three trial runs. The best (fastest) run among these will be taken as the team's
  official time." "Each run must not exceed 8 minutes", with a 2-minute grace if the robot is still
  navigating. "30 minutes total access ... including setup and adjustments between runs."
- p. 5: the race clock "starts when the robot leaves the START tile and stops when it enters the FINISH
  tile".
- p. 5: "If the robot re-enters the START tile before reaching the FINISH, that attempt is aborted and a
  new attempt begins." "If the operator touches the robot during a run, the attempt is immediately
  aborted." "If the robot has already completed a successful run, it may be removed without affecting
  the recorded time."
- p. 5: "After reaching the FINISH, the robot may continue exploring the maze as long as the total access
  time remains. However, time spent beyond the FINISH does not count toward the official run. A new
  official attempt begins only when the robot leaves the START tile again."
- p. 5-6: "Once the maze is revealed, no external information may be provided to the robot. However, teams
  may: change switch settings ...; replace batteries between runs; adjust sensors; modify speed
  settings ...". "Downloading new programs or re-programming the robot during trials is strictly
  prohibited."
- p. 3: the start orientation is "as instructed by officials before the run"; the finish and the start
  positions are variable.

Not stated, and so not decided here: whether a person may put a removed robot back on START for the
next trial (the rules say it may be removed, and that a new attempt begins when it leaves START again,
which implies it); whether the robot may keep its own map in EEPROM across trials; how an aborted
attempt counts toward the three trials; how the 8-minute limit applies to the exploring after the
FINISH (the "continue exploring" rule names only the total access time).

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

Speed levels are a switch setting (p. 5-6 allow changing switch and speed settings between runs, not
re-programming), not code that is edited. Trial 2 uses the safe level and trial 3 the fast one.

Each part of a run is a part of the mission ([0016](0016-cut-the-run-into-parts.md)): `AExplore`,
`BExplore` and the proof are the exploring mode; the stored-path run is a second mode of the same
parts, not a new part.

**Two things this plan depends on are not in the rules text** and are open questions for the
instructor, not decisions: that a removed robot may be placed on START for the next trial, and that
the robot may keep its own map in EEPROM across trials. If the second is refused, every trial explores
again with the same proof loop, and the best time of the three still counts; nothing in 0018 or 0019
changes, and trials 2 and 3 lose only the stored path. If the first is refused, the robot would have to
drive back to START itself after the finish, which the rules also allow ("may continue exploring ...
a new official attempt begins only when the robot leaves the START tile again"). Until both are
answered in writing, the map store (#63) is built so that using it is one switch.

## Consequences

- The robot needs a stored map (#63), a way to tell its start orientation in a later trial (#65), and
  a stored-path run (#66). The proof loop (#61, #62) is part of trial 1, not extra work.
- The run clock stops at the finish tile; the time spent after it "does not count toward the official
  run" (p. 5). What bounds the exploring after the FINISH is the total access time per the rules text; how the
  8-minute limit applies to it is not stated (to ask).
- Re-entering the start tile before the finish aborts the attempt and a new one begins (p. 5), so exploring all of
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
