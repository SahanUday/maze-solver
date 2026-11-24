# 0021: The operator panel: a button, a DIP bank and two LEDs, built apart from the robot's logic

- **Status:** Accepted
- **Date:** 2026-11-24

## Context

What `docs/Maze_Solver.pdf` says, quoted, with its page:

- p. 6: "No human touch is allowed once the robot activation button is pressed."
- p. 5-6: between trials teams may "change switch settings", "modify speed settings" and replace batteries;
  "re-programming the robot during trials is strictly prohibited". p. 6: lighting is not controlled.
- p. 5: "If the operator touches the robot during a run, the attempt is immediately aborted"; a robot that
  has completed a successful run "may be removed".
- p. 7: "If students require any additional equipment not listed above, they must obtain prior approval
  from the course instructor." The parts list has no button, switches or LEDs.

What the code did before this change:

- The run began by itself 1 s after power-up (`RUN_START_BUTTON_FITTED` was false), so the power switch was
  the start. With a button fitted, `runStartTick` began the run on the same tick the debounce passed, with
  a hand still on the robot.
- `PIN_DIP`, `PIN_POT`, `RobotState::dipSwitches` and `speedPotRaw` existed and nothing used them.
- The status LED was a heartbeat for ever and the error LED followed a fault, so a finished run
  (`RunStage::Done`) looked like a running one: nothing told the operator when to take the robot.
- The IR calibration could not be redone at the arena, and a speed run (ADR 0017) needs a way to be chosen.
- ADR 0001 says switches are "read once at boot, never in the hot path".

Told by the team, not in the PDF: the start orientation is announced on the day and is the same for all
three trials (confirmed by the university, 2026-11-24).

## Decision

**Controls.** One momentary push button (D38) and one four-position DIP bank (D40-D43), all to GND with the
internal pull-ups, and two LEDs (D26 green, D27 red). The two power switches (the battery's main breaker and
the board's) stay hardware only. Part select stays a build-time constant.

**The DIP bank**, one meaning per switch:

| RUN | FAST | CAL | RES | Mode |
|---|---|---|---|---|
| off | any | off | off | explore (trial 1) |
| on | off | off | off | speed run, safe (trial 2) |
| on | on | off | off | speed run, fast (trial 3) |
| off | off | on | off | IR calibration |
| anything else | | | | refused |

All off is explore, the safest default, and a cut wire reads OFF. A mode that is not built yet is shown as
refused.

**The LEDs.** The status LED shows the mode code (N blinks) while waiting, a heartbeat while working and
SOLID once the run has ended; the error LED shows a refused setting (fast blink), a missing calibration
(slow blink) and an end gone wrong (SOLID). No heartbeat means the robot has ended, which is the signal to
take it. No buzzer: the rules already have an official TIME-UP buzzer.

**Built apart from the robot's logic.** The panel is pure logic in `include/panel/` that includes nothing from
`mission/`, `sense/` or `motion/`; `src/app/operator_io` reads and writes the pins. `main.cpp` is the one
place that knows both sides: it maps the mission's stage to the panel's four-value run status. The
mission will know nothing about switches or LEDs and will be told only `go`.

**One temporary flag.** `BUTTON_WIRED` (`spec/pins/Wired.h`, not `PinMap.h`, which stays pin numbers
only) keeps the no-button start until the button is wired. The DIP and the LEDs need none: an unwired bank
reads all OFF, and writing to an unconnected pin does nothing. The flag and the no-button start are deleted
once the button is wired.

**Reading the DIP.** The bank is read while the run waits for its start, never during the run, which
keeps ADR 0001's "never in the hot path".

**Built in two steps.** The first shows the LEDs and decodes the DIP and changes nothing in the mission.
The second moves the start rules (debounce, a press must follow a release, a hands-off delay after the
press, the no-button start) from `RunStart` into the panel and blocks `go` on a refused setting.

## Consequences

- The robot can be told what to do, and when it has ended, without re-programming.
- Switch and speed settings are a physical state the operator must set right: the mode code on the status LED
  is what lets them check it before pressing.
- The pot, the rotation bits and a clear-memory switch are not built (speed is the FAST switch, the
  orientation is the same in all trials, explore overwrites the stored maps).
- Speed run (#66) and calibration (#67) are refused until they are built; each removes its own refusal.
- Until the second step, a button that is wired while `BUTTON_WIRED` is false is ignored and the robot starts
  by itself.
- The button, the DIP bank and the LEDs need the instructor's prior approval (p. 7) before they are fitted
  for the final.
- Nothing here has run on the robot: the blink timings, the resistor values and how readable the LEDs are
  under the arena lighting are proposals.

## Alternatives considered

- **A DIP position as the start.** Fewer parts, but a latching switch stays ON after a run, so it needs an
  OFF-to-ON rule so that a robot powered up with it ON does not start in a hand, and the rules say "button".
- **A pot for the speed.** It cannot be set to the same value twice, and its pin is the battery sense's.
- **A coded two-bit mode with calibration at all OFF.** A DIP bank that is untouched, bumped or has a cut wire
  would start a calibration instead of exploring, and one switch would mean different things in different
  rows.
- **Orientation by switch or by wall matching.** Not needed once the orientation is the same in all trials.
- **Three wiring flags (button, DIP, LEDs), in `PinMap.h`.** Only the button changes behaviour when unwired,
  and `PinMap.h` is for pin numbers.
- **Panel code inside the mission.** Quicker, but every mission test would then depend on switch handling.
- **A buzzer for the end of a trial.** It could be taken for the official TIME-UP buzzer.
- **A separate calibration firmware, flashed beforehand.** Cannot be redone if the lighting at the arena
  differs, since re-flashing is banned once the maze is revealed.
