# 0023: A shield PCB, with the Mega's 5 V on its own switch

- **Status:** Accepted
- **Date:** 2026-11-26

## Context

- The robot is wired with jumper wires from the Mega to every module. In the first bench session the
  sensors ran from the battery rail while the Mega was also on USB.
- The LM2596 buck output (5 V) feeds the Mega's 5V pin; VIN is not used. On the Mega 2560 the USB 5 V
  reaches the board's 5 V rail through a MOSFET that cuts USB off only when VIN is above about 6.6 V.
  With VIN unused it stays on, so plugging in USB with the buck on joins the two supplies, and current
  can flow back into the laptop's USB port (limited only by the Mega's 500 mA polyfuse).
- The IR array's counts are ratiometric to AVcc, so the array and the Mega must share one 5 V rail
  while the robot runs ([line_sensors](../modules/line_sensors.md)).
- During a reset, a firmware upload, or while the Mega has no power, its pins float, including the
  motor drivers' enable pins D8 and D10.
- The panel ([0021](0021-operator-panel-built-apart-from-the-robot-logic.md)) adds a button, a DIP
  bank and two LEDs that also need wiring. A shield over the Mega hides the Mega's own reset button.
- The Mega joins pins 20/21 to its SDA/SCL header internally.

## Decision

**A two-layer, through-hole shield** (`hardware/shield/`, KiCad 10) plugs onto the Mega and carries
every pin in `spec/pins/PinMap.h` to a labelled header, plus the panel controls and a reset button.
It is factory-made: 91.1 × 53.3 mm (trimmed at the USB/power-jack end so nothing sits over them),
0.3 mm signal tracks, 0.8 mm power tracks, GND poured on both layers.

**Two 5 V nets.** `+5V` comes from the buck through J8 and feeds the sensors, encoders and motor-driver
logic directly. The Mega's 5 V pins (J1 pin 5, J7 pins 35/36) are a separate net, `MEGA_5V`, reached
only through SW4, a 1-position DIP switch:

| SW4 | USB | Use |
|---|---|---|
| ON | unplugged | Running: everything on the battery, one shared rail |
| OFF | plugged in | Upload and serial: the Mega on USB, the sensors on the battery |

The board prints "MEGA 5V" and "USB: SET OFF" beside SW4.

**Motor pull-downs.** 10 kΩ from D8 (R3) and from D10 (R4) to GND hold both drivers disabled while
the Mega is not driving those pins.

**Reset button.** SW3 shorts RESET to GND, because the shield covers the Mega's own button.

**I2C through J2 only.** SDA/SCL reach the IMU header from the J2 copy of pins 20/21. The J6 copy is
left unrouted on purpose; KiCad reports it as two unconnected items.

## Consequences

- Every pin in `PinMap.h` is now fixed in copper. Moving one needs a new board revision. The
  ultrasonic and panel pins stay `[PROPOSED]` until checked on the built board.
- SW4 ON with USB plugged in joins the two supplies. That is an operator rule, printed on the board.
- With SW4 OFF the sensors still need the battery, so battery first, then reset, still applies to
  the IMU.
- With SW4 OFF on USB, the IR array and the Mega's AVcc sit on different 5 V supplies, so line
  readings taken that way are not ratiometric. Calibrate with SW4 ON, on the battery.
- SW4 OFF, with the battery on and no USB, powers the sensors but not the Mega. R3 and R4 keep the
  motors off; the state is still to be avoided.
- SW4 carries only the Mega, about 0.1 A, so a small DIP switch rated 100 mA or more is enough. The
  pull-downs draw 0.5 mA each while EN is high.
- The shield and its controls need the instructor's prior approval (rules, p. 7) before the final.
- Nothing here has been made or run yet: footprint fit, cable pin orders and the DIP's rating are to
  be checked on the first boards.

## Alternatives considered

- **Switch the whole 5 V rail** (a toggle between J8 and everything). On USB the laptop would then
  power all of the logic, about 0.35–0.5 A, at USB's 500 mA limit, and the switch would carry the
  full load.
- **No switch; unplug the battery before USB.** One slip feeds the laptop.
- **A Schottky diode from the buck.** It drops the rail to about 4.7 V for the whole robot and still
  leaves USB joined to it through the Mega's MOSFET.
- **Battery into VIN.** It lets the Mega's own USB changeover work, but puts the Mega on its
  regulator and the sensors on the buck, so the IR readings are no longer ratiometric while running.
- **A 2-pin jumper instead of SW4.** It works, but it is small and easy to lose at the arena.
