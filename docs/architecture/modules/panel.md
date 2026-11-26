# panel

What the DIP bank means, when the operator lets the run begin and what the two LEDs show. Pure logic in
`include/panel/` (`PanelMode.h`, `PanelStart.h`, `PanelLeds.h`), timings in `spec/tuning/Panel.h`, tests
in `test/unit/test_panel/`. It includes nothing from `mission/`, `sense/` or `motion/`; `src/main.cpp`
hands it the mission's stage. The pins are [operator_io](operator_io.md)'s. Why it is built apart from the
robot's logic: [0021](../decisions/0021-operator-panel-built-apart-from-the-robot-logic.md).

## The DIP bank

`panelDecode(dip)` gives the `PanelMode` of a setting: bits 0-3 in `PIN_DIP` order, a set bit = ON, bits
above the fourth ignored.

| RUN | FAST | CAL | RES | Mode |
|---|---|---|---|---|
| off | any | off | off | `Explore` |
| on | off | off | off | `SpeedSafe` |
| on | on | off | off | `SpeedFast` |
| off | off | on | off | `Calibrate` |
| anything else | | | | `Refused` |

All off is explore, and a cut wire reads OFF. `panelModeBuilt(mode)` is true only for `Explore`; the others
show as refused until #66 and #67 build them. `panelModeCode(mode)` is the blink count: 1 explore, 2 speed
safe, 3 speed fast, 4 calibrate, 0 refused.

## The start

`PanelStart ps(BUTTON_WIRED)` decides `go`, which `main.cpp` hands to the mission. Once per tick,
`panelStartTick(ps, nowMs, buttonDown, mode)` takes the raw button and the DIP setting.

- **Accept.** With a button wired: `PANEL_START_DEBOUNCE_TICKS` ticks down in a row, on a setting that is
  built, after the button has been seen up since boot (a stuck or shorted line never starts the run). A
  press on a setting that is not built is used up: it needs a release and a new press. With no button
  wired: the first tick on a setting that is built; the pin is not read.
- **Countdown.** An accepted start waits `PANEL_START_DELAY_MS` for the hands to come off, whatever the
  button or the DIP do meanwhile, then `go` latches. The mission is not started and the motors are off.

## The LEDs

`panelLeds(PanelView{run, mode, noCalibration}, nowMs)` gives the `PanelLeds{status, error}` to show, from
its inputs alone. `main.cpp` maps `RunStage` to `PanelRun`: `WaitStart` to `Waiting` (`Countdown` once
accepted), `Running`, `Done` to `EndedOk`, `Fault` to `EndedBad` (so a run that cannot start, with no gyro,
is ended badly).

| State | Status LED (green) | Error LED (red) |
|---|---|---|
| waiting, mode built | CODE N: N blinks, a pause, repeat | off; HEARTBEAT if no usable calibration |
| waiting, mode refused or not built | off | FAST BLINK |
| countdown after the start is accepted | FAST BLINK | off |
| running, proving or saving | HEARTBEAT | off |
| ended normally | SOLID | off |
| ended badly | SOLID | SOLID |

The timings are in `spec/tuning/Panel.h`. No heartbeat means the robot has ended: take it. "Ended
normally" is the mission's `Done`; it does not wait for a map save (#63).

## Status

Host tests only; nothing is wired and none of this has run on the robot. The accepted setting is not kept:
nothing reads it yet (#66, #67).
