# panel

What the DIP bank means and what the two LEDs show. Pure logic in `include/panel/` (`PanelControls.h`,
`PanelLeds.h`), timings in `spec/tuning/Panel.h`, tests in `test/unit/test_panel/`. It includes nothing
from `mission/`, `sense/` or `motion/`; `src/main.cpp` hands it the mission's stage. The pins are
[operator_io](operator_io.md)'s. Why it is built apart from the robot's logic:
[0021](../decisions/0021-operator-panel-built-apart-from-the-robot-logic.md).

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

## The LEDs

`panelLeds(PanelView{run, mode, noCalibration}, nowMs)` gives the `PanelLeds{status, error}` to show, from
its inputs alone. `main.cpp` maps `RunStage` to `PanelRun`: `WaitStart` to `Waiting`, `Running`, `Done` to
`EndedOk`, `Fault` to `EndedBad` (so a run that cannot start, with no gyro, is ended badly).

| State | Status LED (green) | Error LED (red) |
|---|---|---|
| waiting, mode built | CODE N: N blinks, a pause, repeat | off; HEARTBEAT if no usable calibration |
| waiting, mode refused or not built | off | FAST BLINK |
| running, proving or saving | HEARTBEAT | off |
| ended normally | SOLID | off |
| ended badly | SOLID | SOLID |

The timings are in `spec/tuning/Panel.h`. No heartbeat means the robot has ended: take it. "Ended
normally" is the mission's `Done`; it does not wait for a map save (#63).

## Status

Host tests only; nothing is wired and none of this has run on the robot. The start rules (debounce, delay,
no-button start) are still in `mission/RunStart.h`.
