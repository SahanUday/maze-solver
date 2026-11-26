# 0023: A shield PCB for the Mega, built from the pin map

- **Status:** Accepted
- **Date:** 2026-11-26

## Context

- The robot is wired with jumper wires from the Mega to every module, and only `PinMap.h` records
  which wire goes where.
- The panel ([0021](0021-operator-panel-built-apart-from-the-robot-logic.md)) adds a button, a DIP
  bank and two LEDs that also need wiring.
- Any board over the Mega hides the Mega's own reset button.
- The Mega joins pins 20/21 to its SDA/SCL header internally, so the same I2C lines appear on two
  headers.
- The Mega's USB socket and power jack stand taller than the headers a board would sit on.

## Decision

**A shield, from `spec/pins/PinMap.h`.** A two-layer, through-hole board (`hardware/shield/`,
KiCad 10) plugs onto the Mega. Every pin in `PinMap.h` goes to a header whose name and pin order are
printed beside it. Connectors sit under the Mega pins they use, so most tracks are short and straight.

**The panel on the board.** START (D38), the 4-way MODE DIP (D40–D43), and the STATUS and ERROR LEDs
(D26, D27, through 470 Ω), each labelled.

**A reset button.** SW3 shorts RESET to GND, because the shield covers the Mega's own button. It sits
at the opposite end from START, so the two are hard to confuse.

**I2C through J2 only.** SDA/SCL reach the IMU header from the J2 copy of pins 20/21. The J6 copy is
left unrouted on purpose; KiCad reports it as two unconnected items.

**Factory-made.** 91.1 × 53.3 mm, trimmed at the USB/power-jack end so nothing sits over them.
Tracks are 0.3 mm for signals and 0.8 mm for `+5V`, with GND poured on both layers. `fab/` holds the
Gerber and drill files generated from the board, regenerated in the same commit as any board change.

## Consequences

- Every pin in `PinMap.h` is now fixed in copper. Moving one needs a new board revision. The
  ultrasonic and panel pins stay `[PROPOSED]` until checked on the built board.
- `BUTTON_WIRED` can be set once the board is built and START is checked on the bench.
- The buck's 5 V feeds the Mega's 5V pin directly, as with the jumper wiring, so USB must not be
  plugged in while the battery is on.
- The shield and its controls need the instructor's prior approval (rules, p. 7) before the final.
- Nothing here has been made yet: the footprint fit and the cable pin orders are to be checked on the
  first boards.

## Alternatives considered

- **Keep the jumper wires.** Nothing to make, but every connection stays a loose jumper that can be
  pulled or swapped, and the panel would add more of them.
- **A perfboard shield, hand-wired.** No factory wait, but it is slow to build, hard to check
  against `PinMap.h`, and hard to repeat for a spare.
- **A single-sided board, home-etched.** The headers to the Mega go on the underside, so their pads
  must be soldered from the top. That needs plated holes, which a home board does not have.
- **Routing both I2C copies.** It adds a track across the whole board to join two pins that the Mega
  already joins.
