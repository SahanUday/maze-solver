# mission

The run-phase state machine: find the bridge in Section A, follow the line across it, and stop where
the line ends. Pure logic in `include/Mission.h` (with `NavParams.h`), tested by `test/test_mission/`
against a simulated arena, plus two glue files in `src/` that are not host-testable
(`operator_io`, `line_cal_store`).

## Phases

`WaitStart -> SenseFront -> TurnAround -> SenseBack -> Explore -> Bridge -> LineFollow -> Done`, any phase `-> Fault`.

- **WaitStart** - motors off until START is pressed (or, with `RUN_USE_START_BUTTON` off, until
  `RUN_AUTOSTART_MS` after boot). `main.cpp` refuses to start (`Fault`) with no IMU, or with no stored line
  calibration when `RUN_USE_FLOOR_SENSING` is on.
- **SenseFront / TurnAround / SenseBack** - read the start cell's walls standing still; the turn round shows
  the wall behind.
- **Explore** - [step_cell](step_cell.md) to the next cell; at each cell the map is updated and
  [explorer](explorer.md) picks the move. Straight cells are not stopped for; a turn stops on the centre
  first.
- **Bridge** - entered from Explore the moment the marker is reported: a straight drive on the grid heading at line
  speed (`LINE_SPEED_MM_S`), carrying the current speed through, with no wall steering and no stop. It watches the
  floor ([floor_pattern](floor_pattern.md)) for the gap and hands over to the line follower at the first line view
  past the gap (`bridgeHandoverStep`, `BRIDGE_HANDOVER_MIN_MM`). A wall straight ahead (`frontBlocked`), or no line
  within `BRIDGE_MAX_MM` of the marker, is the `NoBridge` fault.
- **LineFollow** - [line_follow](line_follow.md) until the line has been gone for `LINE_END_LOST_MM`; then **Done**
  (driver enabled with zero duty - it holds the wheels). The full-arena test checks the robot stops on the
  line's axis, past the marker cell.
- **Fault** - motors off and the error LED on. Codes: `NoImu`, `NoLineCal`, `MoveTimeout`, `Stall`, `LineLost`,
  `NoRoute`, `NoBridge`, `Aborted`.

## How the bridge is found

Nothing about the layout is fixed, but the marker cell has exactly two openings: the side the robot comes in by
and the side straight across, which leaves the maze. So the robot never has to work out which way is out: when the
IR array reports the marker (white, black, white of the measured sizes) the robot is on the marker cell's centre
(the array sits `IR_ARRAY_FORWARD_OFFSET_MM` ahead of the axle, so it is over the far frame as the axle reaches the
centre), and the way out is straight on. The marker is reported once the far frame is `MARKER_FRAME_REPORT_MM` (4 mm)
long, shorter than the frame's own minimum: the white-black pair before it already identifies the marker, and the
robot stops on its odometer's cell centre, which was seen to be ~23 mm short of the marker's real centre after 6.8 m
of driving (the array 4 mm into the far frame).

Two fallbacks cover a marker that is not reported:
- **The gap.** A gap (about 22 mm of air) seen while exploring means the robot is out of the exit already; it goes
  straight into the Bridge phase with the gap counted, and hands over at the first line view.
- **The map.** When nothing is left to explore, `maze::exitCell` finds the one cell with a side observed open whose
  neighbour is outside the 4x4 (the extent is pinned by then). The robot goes there, turns to face the exit and
  starts the Bridge phase from the cell centre. The floor
runs are measured along one step and forgotten at the next, so nothing seen on the white START tile, or on a black
one, can combine into a marker.

The map's marker bookkeeping (`mapMarkMarker`, `bridgeCandidate`, the line tiles) and the old white-tile line zone
(`FloorDetector`) are no longer used by the mission; they stay in the library.

## Run options

`RobotSpec.h` has three switches, all defaulting to the simplest setup (a walls-only maze, nothing pressed):
`RUN_USE_FLOOR_SENSING`, `RUN_USE_START_BUTTON` (+ `RUN_AUTOSTART_MS`) and `RUN_USE_DIP_SWITCHES` (+ `RUN_HAND`).

With floor sensing off `MissionParams::wallsOnly` is set: the floor pattern is not read (so the marker never
starts the bridge; walls only cannot tell the open exit from an unexplored cell, so it may drive out of it), no line calibration is needed, and when nothing is left to explore the mission ends in `Done`
(a stand-still) instead of the `NoRoute` fault. A step that never read its walls is still a `NoRoute` fault.
With the button off the run begins on its own after the delay, so the robot must be placed at the start cell
before then. With the DIPs off there is no line-calibration mode and the hand is `RUN_HAND`.

## Supervision over serial

`main.cpp` prints `mission: phase N fault M` on every phase change and `mission: cell x,y facing d` on every cell
change (phases and faults are the enum values in `Mission.h`). Any byte received on the serial line stops the run
(`Fault`, `Aborted`). `DRIVE_MAX_DUTY` caps every wheel command in `applyDrive`, and the stall latch
(`STALL_*`) is on. The cap and the stall settings have not been exercised by a real stall.

## Running without the button or the switches

The robot has neither a START button nor DIP switches. With `RUN_USE_START_BUTTON` and `RUN_USE_DIP_SWITCHES` off
(the defaults), nothing reads them: the run starts `RUN_AUTOSTART_MS` after boot has finished (the gyro bias is
taken first, with the robot held still), and the hand is `RUN_HAND`. The line calibration is not taken on the robot:
it is captured with `probe_motion` (`W` on white, `B` on black), saved to the EEPROM, and loaded at boot; without a
stored one the mission faults `NoLineCal` when floor sensing is on.

## Operator controls (only with those switches fitted)

`src/operator_io` - the START button (polled every tick) and the DIP switches (read once at power-up): bit 0 picks
the hand for ties, bit 3 selects **line calibration mode**: slide the array over white and black, press START, and
the result is kept in EEPROM by `src/line_cal_store` (magic + checksum; a blank or torn record is rejected, and the
mission then refuses to start). Lighting at the arena is not the lighting in the lab and nothing can be re-flashed
there, so the calibration is taken there.

## Invariants

- No hardware, no `millis()`; the DriveCommand is the only output and `main.cpp` the only thing that turns it into
  `motorSet()`.
- The start cell is never taken for the marker: floor runs are per step, and the marker needs the measured
  white-black-white shape.
- A Fault is permanent for the run: nothing recovers it.

## Status

27 host tests. The headline ones drive the whole stack through `RobotState -> Perception -> missionStep ->
DriveCommand` against a simulated 4x4 arena (walls, ultrasonic rays refreshed one sensor per 20 ms, encoders, an IR
array over a painted floor: a white or black start tile, the measured marker, strip, gap and ramp line) from **every
start cell and every heading, for every edge and corner marker cell and every outward direction, with both hands, on
two mazes** - every run reaches `Done` without a wall contact. Fault paths (no IMU, boxed in, wedged robot, stuck turn
or step, lost line, no ramp after the marker, a wall where the exit should be) each have a test. The simulation uses
the robot's own array geometry (86 mm ahead of the axle, 8 mm pitch) and the measured marker sizes.

**Not verified on the robot as a whole.** The pieces were: the marker and gap detection and the handover to the line
follower (probe `m`, three runs from the cell before the marker), the line follower on the flat floor, the ramp and
the bends beyond it. Not yet: the full run from the start cell, the stop at the end of the line (`LINE_END_LOST_MM` is
off, so the run ends in the `LineLost` fault), the tilt rule for the ramp, and a marker reached while the robot is
turning rather than driving straight through. The simulation has no slip, no sensor noise, and ideal wheel response.
The EEPROM store and the operator I/O are untested on hardware.
