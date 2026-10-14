# mission

The run-phase state machine: find the bridge in Section A, follow the line across it, and stop where
the line ends. Pure logic in `include/Mission.h` (with `NavParams.h`), tested by `test/test_mission/`
against a simulated arena, plus two glue files in `src/` that are not host-testable
(`operator_io`, `line_cal_store`).

## Phases

`WaitStart -> SenseFront -> TurnAround -> SenseBack -> Explore -> LineFollow -> Done`, any phase `-> Fault`.

- **WaitStart** - motors off until START is pressed. `main.cpp` refuses to start (`Fault`) with no IMU or no
  stored line calibration.
- **SenseFront / TurnAround / SenseBack** - read the start cell's walls standing still; the turn round shows
  the wall behind.
- **Explore** - [step_cell](step_cell.md) to the next cell; at each cell the map is updated and
  [explorer](explorer.md) picks the move. Straight cells are not stopped for; a turn stops on the centre
  first.
- **LineFollow** - [line_follow](line_follow.md) until the line ends, then on into Section B by `LINE_EXIT_MM` past
  the end; then **Done** (driver enabled with zero duty - it holds the wheels). The full-arena test checks the
  axle stops past the line's end, not on it.
- **Fault** - motors off and the error LED on. Codes: `NoImu`, `NoLineCal`, `MoveTimeout`, `Stall`, `LineLost`,
  `NoRoute`.

## How the bridge is found

Nothing about the layout is fixed. The approach marker (a short run of white) is attributed to a cell from a
snapshot taken when the run began, because it is classified only after it ends and the robot may have moved on.
After the marker the goal is the neighbour beside it; the robot goes to the **marker cell** first and steps into
that tile along the marker's axis, because the line runs along that axis and met from the side it crosses the
robot's path. A white line tile met any other way is noted (`Cell::line`), not followed. Two more cases are
handled by looking at what the floor does when the robot **leaves** a white tile: if it stays white past the tile's
edge, leaving a line tile means the robot is going out along the line (hand over to the follower), and leaving
the START tile means the next tile is white too, so a line tile - the START tile is white and nothing about it is
measured.

## Operator controls

`src/operator_io` - the START button (polled every tick) and the DIP switches (read once at power-up): bit 0 picks
the hand for ties, bit 3 selects **line calibration mode**: slide the array over white and black, press START, and
the result is kept in EEPROM by `src/line_cal_store` (magic + checksum; a blank or torn record is rejected, and the
mission then refuses to start). Lighting at the arena is not the lighting in the lab and nothing can be re-flashed
there, so the calibration is taken there.

## Invariants

- No hardware, no `millis()`; the DriveCommand is the only output and `main.cpp` the only thing that turns it into
  `motorSet()`.
- The start cell is never taken for the marker or the line.
- A Fault is permanent for the run: nothing recovers it.

## Status

22 host tests. The headline ones drive the whole stack through `RobotState -> Perception -> missionStep ->
DriveCommand` against a simulated 4x4 arena (walls, ultrasonic rays refreshed one sensor per 20 ms, encoders, an IR
array over a painted floor with a start tile, marker, and bridge tile) from **every start cell and every heading,
for every bridge corner and direction, with both hands, on two mazes** - every run reaches `Done` without a
wall contact. Fault paths (no IMU, boxed in, wedged robot, stuck turn or step, lost line) each have a test.

**Not verified on the robot.** The simulation has no slip, no sensor noise, and ideal wheel response. Every
calibration value this depends on is `0` in `RobotSpec.h` (`<<TBD ...>>`), including the marker length window, the
line zone lengths, and `IR_ARRAY_FORWARD_OFFSET_MM`, which also has to exceed half the marker's length (~45 mm) so
the array is already past the marker when the robot stops on its centre. The marker is only classified once bare floor
has lasted `FLOOR_RUN_GAP_MM` after it, so a decision at that cell is taken before it is known; the next one uses it. If the marker is never detected and the bridge tile was never met
from the side, the robot explores everything and faults `NoRoute` rather than guessing. The EEPROM store and the
operator I/O are untested on hardware.
