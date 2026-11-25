# 0022: No run ends on time

- **Status:** Accepted
- **Date:** 2026-11-25
- **Supersedes:** the run limit in the firmware: [0016](0016-cut-the-run-into-parts.md) (the manager owns
  it, `BProve` ends at it), [0017](0017-explore-then-prove-then-run-the-proven-path.md) and
  [0018](0018-route-by-estimated-time-and-prove-it.md) (the proof stops at it), and
  [0011](0011-leave-the-maze-onto-the-line-without-stopping.md) (it applies through the line). The rest of
  those records stands.

## Context

What `docs/Maze_Solver.pdf` says, quoted from p. 5:

- "Each run must not exceed 8 minutes."
- "If the 8-minute limit is reached, a TIME-UP buzzer will sound, and the robot must be removed from the
  field."
- "If a robot is still actively navigating when the buzzer sounds, an additional 2-minute grace period will
  be given to allow the run to conclude without interruption."
- "Robots failing to reach the FINISH are ranked based on the maximum number of consecutive tiles
  successfully traversed without human intervention."
- "After reaching the FINISH, the robot may continue exploring the maze as long as the total access time
  remains."

What the code did before this change: `missionTick` failed any run still in progress 8 minutes after its
start (`TimeUp`) and cut the motors. A robot that was still navigating gave up the grace period and every
tile it had yet to reach, and one slow stretch ended the whole trial.

## Decision

**The firmware ends no run because of the time it has taken.** The officials time the trial and remove the
robot. A run ends when its last part is done, when a part gives up for its own reason, or when the motion
controller reports a stall. The mission is given no clock.

**Eight minutes is the target the algorithm is tuned to fit.** `RUN_LIMIT_MS` stays in
`spec/fixed/Rules.h` for the host tests: a Section A run in the simulated arena that does not end within
it fails them.

## Consequences

- A robot still navigating at the buzzer uses the grace period.
- A robot that goes wrong in a way no check catches runs until a person removes it. One case is known: the
  line follower carries straight on over an all-black view, so until the line has an end rule (#60) it
  stops there only on a stall.
- The limits inside the parts are unchanged: a move that makes no progress, turn retries, map rescans,
  blocked steps and the search for a lost line. Whether any of those should change is not decided here.
- `MissionFault::TimeUp` is gone. The later codes keep their numbers in the serial log.
- Gyro drift over a run longer than 8 minutes has not been measured.
- Nothing here has run on the robot.

## Alternatives considered

- **Keep the fault at 8 minutes.** It stops the robot exactly when the rules would still let it go on.
- **Fail the search at 8 minutes but end the proof without a fault.** The robot still stops itself, and
  every new part would need its own rule.
- **Stop at 10 minutes, after the grace period.** The officials already end the trial, and their clock
  starts when the robot leaves the START tile, which the firmware does not time.
