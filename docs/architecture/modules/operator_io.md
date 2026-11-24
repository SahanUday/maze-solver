# operator_io

The pin side of the operator panel: sets the pin modes, reads the start button and the DIP bank, writes the
two LEDs. `src/app/operator_io.*`, non-critical glue that uses the Arduino framework
([0001](../decisions/0001-hybrid-hardware-abstraction.md)). What the DIP means and what the LEDs show is the
pure logic in [panel](panel.md); `src/main.cpp` joins the two.

## Interface

- `operatorIoInit()` - the LEDs as outputs; the four DIP pins as inputs with pull-ups; the start button as an
  input with a pull-up when it is wired (`BUTTON_WIRED`, `spec/pins/Wired.h`). Pins pick up their mode here
  because a Mega pin starts as an input with no pull-up, which an open switch would leave floating.
- `operatorIoDip()` - the DIP bank as bits, bit i set while position i (`PIN_DIP[i]`) is ON (pulled to GND).
  An unwired bank reads 0, which is explore.
- `operatorIoStartButton()` - the raw start button, true while it is held; always false without one wired.
- `operatorIoTick(nowMs, view)` - the LEDs show `view` (a `PanelView`); a pin is written only when its state
  changes.

## Pins

All in `spec/pins/PinMap.h`: `PIN_BTN_START` (D38), `PIN_DIP` (D40-D43), `PIN_LED_STATUS` (D26) and
`PIN_LED_ERROR` (D27), tagged `[PROPOSED]` until they are wired. A `static_assert` ties the size of `PIN_DIP`
to the panel's four positions.

## Invariants

- No `Serial` output here: the LEDs are the readout.
- `main.cpp` reads the DIP only while the run waits for its start.

## Status

Not wired and not run on the robot. The LEDs and the DIP need no flag: unwired, the DIP reads 0 and writing to
an unconnected LED pin does nothing. Wire the LEDs first (they are the only readout), then the DIP, then the
button.
