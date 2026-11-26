# Mega shield PCB

A two-layer, through-hole board that plugs onto the Arduino Mega 2560 and gives every module a
labelled header, plus the operator panel (START, MODE DIP, two LEDs), a RESET button and the Mega's
5 V switch. Why it is built this way: [ADR 0023](../../docs/architecture/decisions/0023-shield-pcb-with-a-switched-mega-5v.md).

**Status:** rev A, designed 2026-11-26. Not yet made or tested.

## Files

| File | What it is |
|---|---|
| `MazeSolver.kicad_pro` / `.kicad_sch` / `.kicad_pcb` | The KiCad 10 project, schematic and board |
| `fp-lib-table`, `Arduino_MountingHole.pretty/` | The project footprint library (the Mega's mounting holes) |
| `fab/` | Gerber and drill files for the factory, generated from this board revision |

Open `MazeSolver.kicad_pro` in KiCad 10. Every other footprint is from KiCad's standard library.

## The board

- 91.1 × 53.3 mm, 2 layers, all through-hole. The headers to the Mega (J1–J7) sit on the underside;
  everything else is on top.
- Tracks are 0.3 mm for signals and 0.8 mm for `+5V` and `MEGA_5V`, with GND poured on both layers.
- The left end is trimmed, so nothing sits over the Mega's USB socket or power jack.

## Connectors

The pin order is printed on the board next to each header. Pin numbers are the firmware's
([`PinMap.h`](../../include/spec/pins/PinMap.h)).

| Header | Board label | Pins, pin 1 first | Mega pins |
|---|---|---|---|
| J8 | 5V IN | +5V, GND | From the LM2596 buck output |
| J9 / J10 / J11 | US FRONT / LEFT / RIGHT | 5V, TRG, ECH, GND | Trig 30 / 32 / 34, echo A10 / A11 / A12 |
| J12 | LINE SENSOR | GND, IR, D1–D8, 5V | Emitter 36, D1–D8 on A0–A7 (D1 = the robot's left) |
| J13 | IMU | 5V, GND, SCL, SDA, XDA, XCL, AD0, INT | SCL 21, SDA 20; the last four are not connected |
| J14 | MOTOR L | RPWM, LPWM, EN, EN, 5V, GND | 6, 7, EN 8 |
| J15 | MOTOR R | RPWM, LPWM, EN, EN, 5V, GND | 11, 12, EN 10 |
| J16 / J17 | ENC L / ENC R | 5V, A, B, GND | 2, 3 / 19, 18 |

Wire each cable to the module's own labels. In particular, the IBT-2 header is RPWM, LPWM, R_EN, L_EN,
R_IS, L_IS, VCC, GND: shield pins 5V and GND go to the IBT-2's **VCC and GND (its pins 7 and 8)**, not
its pins 5 and 6.

## Switches, buttons and LEDs

| Part | Board label | What it does |
|---|---|---|
| SW1 | START | Starts a run (D38). Ignored while `BUTTON_WIRED` is `false` in `spec/pins/Wired.h` |
| SW2 | MODE: 1 RUN 2 FAST 3 CAL 4 RES | The mode, read while the robot waits (D40–D43). Meanings: [panel](../../docs/architecture/modules/panel.md) |
| SW3 | RESET | Resets the Mega (RESET to GND). The shield covers the Mega's own button |
| SW4 | MEGA 5V | Connects the Mega's 5 V pins to the battery 5 V. See [Power](#power) |
| D1 + R1 | STATUS | Green LED on D26 |
| D2 + R2 | ERROR | Red LED on D27 |
| R3 / R4 | — | 10 kΩ pull-downs on D8 / D10: the motors stay off while the Mega is not driving them |

## Power

```
Buck 5V ── J8 ── +5V ── sensors, encoders, motor-driver logic   (always on the battery)
                  └──[SW4]── MEGA_5V ── Mega 5V pins (J1 pin 5, J7 pins 35/36)
```

| SW4 | USB | Use |
|---|---|---|
| ON | unplugged | **Running.** Everything on the battery |
| OFF | plugged in | **Upload and serial.** The Mega on USB, the sensors on the battery |
| ON | plugged in | **Never.** It joins the battery 5 V to the laptop's USB |

- **Before plugging in USB:** turn SW4 OFF. **Before running:** unplug USB, then turn SW4 ON.
- With SW4 OFF the sensors still need the battery: battery on first, then reset (the IMU).
- Calibrate the line sensor with SW4 ON on the battery. On USB the array and the Mega use different
  5 V supplies, so the readings are not ratiometric.
- Don't leave the battery on with SW4 OFF and no USB: the sensors are powered and the Mega is not.

## Parts

| Ref | Part | Qty |
|---|---|---|
| J1, J3, J4, J5, J6 | Male header 1×8, 2.54 mm (to the Mega) | 5 |
| J2 | Male header 1×10, 2.54 mm (to the Mega) | 1 |
| J7 | Male header 2×18, 2.54 mm (to the Mega) | 1 |
| J9, J10, J11, J16, J17 | Male header 1×4, 2.54 mm | 5 |
| J14, J15 | Male header 1×6, 2.54 mm | 2 |
| J13 | Male header 1×8, 2.54 mm | 1 |
| J12 | Male header 1×11, 2.54 mm | 1 |
| J8 | Screw terminal, 2-way, 5.08 mm (Phoenix MKDS 1,5 or similar) | 1 |
| SW1, SW3 | Tactile push button, 6 × 6 mm | 2 |
| SW2 | DIP switch, 4-way, 2.54 mm | 1 |
| SW4 | DIP switch, 1-way, 2.54 mm, rated 100 mA or more | 1 |
| D1 | LED 5 mm, green | 1 |
| D2 | LED 5 mm, red | 1 |
| R1, R2 | Resistor 470 Ω, 0.25 W | 2 |
| R3, R4 | Resistor 10 kΩ, 0.25 W | 2 |

## Ordering

Upload a zip of `fab/` and choose 2 layers, 1.6 mm, HASL, 1 oz copper. Any mask colour works.

Before ordering:

1. Print the board at 100 % and lay it on a Mega: the headers, holes and the USB end must line up.
2. Check SW4's datasheet: 100 mA or more.
3. Get the instructor's approval for the added controls (rules, p. 7).

## Changing the board

Edit in KiCad, then in the PCB editor refill the zones (**B**) and run the DRC with the schematic
parity check. The expected report is:

- 2 unconnected items: the J6 copy of SDA/SCL, left unrouted on purpose (the Mega joins it to J2
  inside). Exclude them.
- 4 silkscreen warnings on the Mega headers' reference text (J2, J4, J7).

Then regenerate `fab/` in the same commit, so it always matches the board:

```
cd hardware/shield
kicad-cli pcb export gerbers -o fab/ --layers F.Cu,B.Cu,F.Mask,B.Mask,F.SilkS,B.SilkS,Edge.Cuts MazeSolver.kicad_pcb
kicad-cli pcb export drill -o fab/ --format excellon --excellon-separate-th --excellon-units mm --excellon-oval-format route MazeSolver.kicad_pcb
```

A pin moved in `PinMap.h` needs the same change here, and a new board revision.
