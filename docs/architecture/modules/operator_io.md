# operator_io

The panel's pins: sets their modes, reads the start button and the DIP bank, writes the two LEDs.
`src/app/operator_io.*`, non-critical glue on the Arduino framework
([0001](../decisions/0001-hybrid-hardware-abstraction.md)). What the DIP means and what the LEDs show is
[panel](panel.md); `src/main.cpp` joins the two.

## Interface

- `operatorIoInit()` - the LEDs as outputs; the DIP pins, and the start button if wired (`BUTTON_WIRED`,
  `spec/pins/Wired.h`), as inputs with pull-ups. A Mega pin starts as an input with no pull-up, so an open
  switch would float.
- `operatorIoDip()` - bit i set while `PIN_DIP[i]` is ON (pulled to GND). An unwired bank reads 0, which is
  explore. `main.cpp` reads it only while the run waits for its start.
- `operatorIoStartButton()` - the raw button; always false while it is not wired.
- `operatorIoTick(nowMs, view)` - shows a `PanelView` on the LEDs; a pin is written only when it changes.

## Pins

`PIN_BTN_START` (D38), `PIN_DIP` (D40-D43), `PIN_LED_STATUS` (D26), `PIN_LED_ERROR` (D27) in
`spec/pins/PinMap.h`, tagged `[PROPOSED]` until wired. A `static_assert` ties `PIN_DIP` to the panel's four
positions. There is no `Serial` output: the LEDs are the readout.

## Status

Not wired, not run on the robot. The DIP and the LEDs need no flag: unwired, the DIP reads 0 and a write to
an unconnected pin does nothing. Wire the LEDs first (the only readout), then the DIP, then the button.
