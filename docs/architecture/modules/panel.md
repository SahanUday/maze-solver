# panel

What the DIP bank means and what the two LEDs show. Pure logic in `include/panel/` (`PanelButtons.h`,
`PanelLeds.h`), blink timings in `spec/tuning/Panel.h`, tested by `test/unit/test_panel/`. It includes
nothing from `mission/`, `sense/` or `motion/`: `src/main.cpp` hands it the mission's stage. The pins are
read and written by [operator_io](operator_io.md). Why it is built apart from the robot's logic:
[0021](../decisions/0021-operator-panel-built-apart-from-the-robot-logic.md).

## Interface

- `panelDecode(dip)` - the `PanelMode` a DIP setting asks for. Bit 0 RUN, 1 FAST, 2 CAL, 3 reserved, a set
  bit = ON; bits above the fourth are ignored.

  | RUN | FAST | CAL | RES | Mode |
  |---|---|---|---|---|
  | off | any | off | off | `Explore` |
  | on | off | off | off | `SpeedSafe` |
  | on | on | off | off | `SpeedFast` |
  | off | off | on | off | `Calibrate` |
  | anything else | | | | `Refused` |

- `panelModeBuilt(mode)` - is the mode in this firmware? Only `Explore` is; the others are shown as refused
  until #66 and #67 build them.
- `panelModeCode(mode)` - the number of blinks that shows it: 1 explore, 2 speed safe, 3 speed fast,
  4 calibrate, 0 refused.
- `PanelView {run, mode, noCalibration}` and `panelLeds(view, nowMs)` - what the LEDs show now, as a
  `PanelLeds {status, error}`. `PanelRun` is `Waiting`, `Running`, `EndedOk` or `EndedBad`.
- `panelBlink(nowMs, halfPeriodMs)` and `panelCode(nowMs, n)` - the two patterns the table below is made of.

## What the LEDs show

| State | Status LED (green) | Error LED (red) |
|---|---|---|
| waiting, mode built | CODE N: N blinks, a pause, repeat | off; HEARTBEAT if no usable calibration |
| waiting, mode refused or not built | off | FAST BLINK |
| running, proving or saving | HEARTBEAT | off |
| ended normally | SOLID | off |
| ended badly | SOLID | SOLID |

HEARTBEAT, FAST BLINK and the CODE N timings are in `spec/tuning/Panel.h`. No heartbeat means the robot has
ended, which is the signal to take it.

## How it works

`panelLeds` is a function of the view and the time alone, so there is nothing to keep between ticks and a
test can ask for any instant. `main.cpp` maps `RunStage` to `PanelRun` (`WaitStart` to `Waiting`, `Running`,
`Done` to `EndedOk`, `Fault` to `EndedBad`), so a run that cannot start (no gyro) shows as ended badly.
"Ended normally" is the mission's `Done` for now; it will wait for the last map save once the maps are
stored (#63).

## Invariants

- Nothing here includes `Arduino.h` or anything of the mission, sense or motion layers.
- The DIP is read while the run waits for its start and not during the run (ADR 0001).

## Status

Host tests only; nothing is wired and none of this has run on the robot. The start rules (debounce, the press
after a release, the hands-off delay, the no-button start) are still in `mission/RunStart.h` and move here
in the next step.
