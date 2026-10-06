# line_sensors

8-channel analog IR reflectance array for the bridge between Section A and B,
where a 30mm black line (`BRIDGE_LINE_WIDTH_MM`) is the only lateral reference.
Code: `src/hal/line_sensors.{h,cpp}`. Decision and bench measurements:
[0005](../decisions/0005-analog-ir-array-adc-sampling.md), which supersedes
[0004](../decisions/0004-digital-ir-array-sensing.md).

## Interface

- `lineSensorsInit()` - takes over PORTF and the ADC, turns the emitters on and
  waits for them to settle.
- `lineSensorsRead(counts)` - one 10-bit conversion per channel into
  `uint16_t[8]`, index 0 = A0 = D1; lower = more reflected light. Called once per
  tick from `readSensors()` into `RobotState::irRaw`.
- `lineSensorsSetEmitters(on)` - switches the emitter bank and waits
  `IR_EMITTER_SETTLE_US` if the state changed. Only after `lineSensorsInit()`.

The module is labelled QYF-750 (no public datasheet found), silkscreen
`GND, IR, D1…D8, VCC`: D1…D8 go to A0…A7 (`ADC0`…`ADC7` = `PF0`…`PF7`), `IR` to
pin 36 (`PC1`, HIGH = emitters on). The silkscreen says `D`, but each output is
an analog voltage: a phototransistor collector with an on-board pull-up (probed
as 5-14 kΩ), so the count falls from ~1020 with nothing in range as more light
comes back. The module also pulls `IR` up, so a floating pin lights the emitters;
the driver drives it anyway.

## How it works

`lineSensorsInit()` clears `DDRF`/`PORTF` (the Mega's pull-ups stay off: they
would shift every reading ~20%), sets `DIDR0 = 0xFF` (`PINF` reads 0 from now on),
enables the ADC at `CPU_HZ`/`IR_ADC_PRESCALER` (500 kHz) with one throwaway
conversion, then drives pin 36 HIGH, `PORTC` before `DDRC` so there is no LOW
glitch. `lineSensorsRead()` converts channel by channel: each conversion clears
`MUX5`, writes `ADMUX` (AVcc reference), starts, polls `ADSC` and reads `ADC`.
There is no ISR. A sweep takes ~240 µs and is not one instant: the channels span
~0.24 mm of travel at 1 m/s. The wiring assumptions are `static_assert`-ed against
`RobotConfig.h`.

## Invariants

- A0-A7 stay on `ADC0`-`ADC7` in order; moving one fails the build until the
  assertion is updated with it.
- The module owns the ADC registers. A boot-time `analogRead(PIN_POT)` in
  `main.cpp` is fine: every conversion re-selects the reference, channel group
  (`MUX5`) and channel, because the Arduino core leaves `MUX5` set after reading
  A8-A15. A second register-level ADC driver must declare
  `// pin-check: shared adc - <reason>` in both modules.
- Counts are ratiometric to AVcc: power the module from the Mega's 5 V rail.
- A0-A7 are not usable as GPIO while `DIDR0` is set.

## Status

Compiles under `-Werror`; the pin-map, banned-pattern and ISR checks pass, and each
`static_assert` fires when its assumption is broken. On a bench board (numbers in
the ADR) the registers read back as intended, emitters off reads 1023, a sweep
takes 242 µs, all 8 channels respond in order, and white separates from black paper
at ~3 mm and at 22.77 mm. A host build against a fake ADC converts ADC0-7 even when
`MUX5` was left set the way `analogRead(A8)` leaves it; that is a simulation, not
yet confirmed on the board.

Nothing interprets the counts yet: no calibration or position math until the real
arena line material and the mount height (`IR_RIDE_HEIGHT_MM`, `<<TBD HARDWARE>>`)
are known. There is no ambient rejection, and `lineSensorsRead()` blocks ~240 µs of
the 10 ms tick.

Still to check: the real arena line material (black tape can reflect far more than
black paper), height and tilt (the bridge incline changes both), IR-rich ambient
light, emitter current draw, optical crosstalk, and a second board.
