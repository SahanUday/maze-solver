# Postponed work

What the cleanup and refactoring of November 2026 (PRs #46-#52) left for later, why each piece
waits, and what is needed to finish it. This is a to-do list, not a design: when a piece is built,
its facts go into `architecture/` and this entry is deleted.

## Needs the robot or a decision first

| Item | Why it waits | Needed to finish it |
|---|---|---|
| **D1 - incline level at boot** | It changes what counts as "level", so it cannot be called safe without a ramp check. `InclineLevel` (`sense/Tilt.h`) and `perceiverSetLevel()` exist and are tested; nothing calls them. | Capture the level in the gyro-bias window (`src/app/heading_estimator.cpp`) and hand it to the perceiver. On the robot: read the tilt on level ground and on the ramp, and check `RAMP_INCLINE_COUNTS` still fits. |
| **D2 - IR calibration at the arena** | The only thing that ever wrote the stored calibration was the probe firmware, which is gone from the repo (it lives only at `refs/pull/28/head`). `main.cpp` can load a calibration from EEPROM but nothing can make a new one, so this is required before a new lighting. It needs hardware choices. | A switch on a free pin (`PIN_DIP`, pins 40-43, is reserved in `RobotConfig.h` but no driver reads it), a procedure for the operator, and an LED code for success or failure. The capture (`lineCalibrationReset/Accumulate/Finish`) and `lineCalSave()` exist and are tested. On the robot: calibrate and follow the line under two lightings. |
| **D3 - emitters off (ambient sweep)** | New behaviour, not cleanup. `lineSensorsSetEmitters()` is unused. Each switch waits `IR_EMITTER_SETTLE_US` (1 ms) inside a 10 ms tick. Nobody has shown that ambient light matters. | A robot test of the line sensor under different lighting. If it does not matter, delete the function and its line in `modules/line_sensors.md`. |
| **D4 - `dipSwitches`, `speedPotRaw`** | No driver writes them. | The switches of D2 and a pot driver, filled from `src/app/operator_io`. If no switch or pot will be fitted, delete the two fields. |
| **R5 - one EEPROM layout** | Only needed once maps are saved; building it earlier would be a guess. The line calibration now lives in one struct at address 0 with an additive checksum. Whether the rules let a robot keep its own map across trials is not answered ([0017](architecture/decisions/0017-explore-then-prove-then-run-the-proven-path.md)). | Issue #63: one layout in `src/app/storage.cpp`: header with magic and version, and per block a fixed address and a CRC-16; a version bump invalidates old data. Save at the first finish. Section B is stored folded to 9x9 ([0019](architecture/decisions/0019-section-b-map-17x9-folded-to-9x9.md)). Move the line calibration into it. On the robot: a power cycle keeps the calibration. |
| **R2 rest - maze library** | `ExplorerT<N>` is done. The rest has no user until the shortest-path run exists, and `lib/maze/Maze.h` is being changed by #55. | Known versus unknown edges and the shortest-path proof (#55, by tile count); the same by estimated time ([0018](architecture/decisions/0018-route-by-estimated-time-and-prove-it.md), #76); a rectangular map for Section B, 17x9 then folded to 9x9 ([0019](architecture/decisions/0019-section-b-map-17x9-folded-to-9x9.md), #75), after #55; map serialisation with a CRC-16 and rotation by 90 degrees for the speed-run start (#63, #65). |
| **R4 rest - telemetry and operator I/O** | The `Telemetry` struct would change again with the Section B mission. A status line is about 100 characters against a 64-byte transmit buffer, so a print can block about 4 ms every 200 ms; nobody has measured whether that disturbs control. DIP, pot and battery have no hardware. | A `Telemetry` struct filled by the mission and formatted in `src/app/telemetry.cpp`. On the robot: check the log's effect on control, then send it in pieces across ticks if it matters. A battery reading needs a divider on A8/A9 (the architecture's port table lists the pins, no divider is confirmed) and must not write `DIDR2` wholesale (see `architecture/ARCHITECTURE.md`). |

## To check on the robot

Nothing since the mid-evaluation run (tag `v0.1-section-a`) has run on the robot. One Section A run
should cover them all, with the same boot messages, status line, phase lines and LEDs as before:

- `src/app/` split out of `main.cpp` (#49).
- The maze library as a template on the maze size (#50).
- The Section A mission split into the explorer, the marker exit and the mission (#51). On the host
  every tick of the mission is identical before and after.
- `RunLoop` and `Perception` start handling (#52).
- The mission manager and the part model (INT-01): the start and the run limit now live in `RunStart` and
  `MissionManager`.

The host tests do not reach the explorer's "stop and brake at once" path (`ExploreStatus::Braked`,
used only when the run ends with `AExplore`, `Parts.h`); the firmware's default run goes on to the exit part.

## Decided not to do

| Item | Reason | Revisit when |
|---|---|---|
| **R6 - spec defaults in every Params struct** | Changing `MotionControlParams` alone broke 15 tests: they start from a zero baseline on purpose, so unit tests do not depend on tuning. | A second mission needs its own parameters: then move `sectionAParams()` somewhere mission-agnostic. Do not change the defaults. |
| **D9 - merge `LineFollow::Mode` and `LineStatus`** | Not the same: `Mode::Ended` reports `Searching` while braking. | The line follower is reworked for Section B's entry line. |
| **D10 - natural order for `MissionPhase`** | The phase numbers in the serial log would change. | Section B adds phases anyway; `phaseOf()` in `MissionManager.h` is the one place to renumber. |
| **D12 - one setter in `motors.cpp`** | Register code the host cannot test; about 20 lines of gain. | The robot is available to check the motors. |

## Quality targets not met

From the refactoring plan's definition of done; none is a functional gap.

- `src/main.cpp` is 103 lines against 80, mostly comments.
- `CellExplorer.h` is about 540 lines and `MissionManager.h` about 440, against a goal of about 300. Helpers
  (`planNext`, the vote handling) can move to a second header with no behaviour change.
- Some inner state flags are still booleans in `CellExplorer` (`entered`, `decided`, `resumeNext`,
  `afterStop`, `braking`); they can become enums. Either change can be checked by comparing every tick of
  `test_section_a` and `test_run_loop` before and after, as in #51 and #52.
- A few functions in `include/` are not called by the firmware: those of D1, D2 and D3 above, and
  `odometryReset()` and `wallSteerReset()` (kept on purpose).
