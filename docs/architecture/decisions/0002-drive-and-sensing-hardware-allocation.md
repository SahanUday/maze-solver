# 0002: Pin, timer and interrupt allocation for drive and sensing

- **Status:** Accepted
- **Date:** 2026-10-03

## Context

The drive is two JGA25-370 DC12 gear motors (Hall quadrature encoder, A/B per
motor) behind two HW-039 / IBT_2 boards. Those boards carry the BTS7960
dual half-bridge: header `RPWM`, `LPWM`, `R_EN`, `L_EN`, `VCC`, `GND` (no
current-sense pins broken out) and screw terminals `B+/B-/M+/M-`. Direction is
chosen by *which* of RPWM/LPWM carries PWM, so each motor needs two PWM
outputs: four in total.

Nothing is wired yet, so the pin map is free. Two earlier inputs were set aside:
`RobotConfig.h`'s motor block assumed an L298N-style driver (one PWM + IN1/IN2),
and the team's earlier motor/encoder design notes assumed the same driver and put
encoder channel B on polled pins. Neither matches this hardware.

## Decision

| Resource | Assignment |
|---|---|
| Left encoder | A = D2 (PE4, INT4), B = D3 (PE5, INT5) |
| Right encoder | A = D19 (PD2, INT2), B = D18 (PD3, INT3) |
| Left motor | RPWM D6 (OC4A), LPWM D7 (OC4B), EN D8, all Timer4 |
| Right motor | RPWM D11 (OC1A), LPWM D12 (OC1B), EN D10, all Timer1 |
| Timer0 | Arduino `millis()`, untouched |

Rules behind it:

- **Each encoder's A and B sit on one port, A on the lower bit.** One `PINx`
  read gives a coherent `(B<<1)|A` snapshot, so the ISR never sees A and B from
  different instants.
- **Encoders get the four dedicated external-interrupt vectors** (INT2-INT5).
  INT0/INT1 are the I2C pins, so no external-interrupt vector is left over.
  Consequence: Serial1 (D18/D19) is unavailable.
- **Both legs of one motor share a timer**, so direction changes never straddle
  two counters. The two motors use different timers (a timer has only three
  compare channels). Timer4 and Timer1 put all six motor wires on the top
  header, in two adjacent groups.
- **PWM: Fast PWM mode 14, `ICRn` = TOP = 799, clk/1 = 20 kHz.** Above audible
  range, under the BTS7960's ~25 kHz limit, 800 duty steps.
- **EN: `R_EN` and `L_EN` of each board are tied and driven by one GPIO**, so
  firmware can coast the motors (EN low) independently of PWM state.

## Consequences

- `motors` and `encoders` HAL code is hand-mapped to these registers;
  `static_assert`s on the `RobotConfig.h` pin constants fail the build if the map
  moves without the code.
- Duty is in timer counts (0..799), not Arduino's 0..255. Dead-zone values
  (`MOTOR_MIN_PWM_*`) must be measured at 20 kHz; numbers measured with
  `analogWrite()`'s default ~490 Hz do not transfer.
- Idle-leg handling: a leg at duty 0 is disconnected from its timer and held
  low, so duty 0 is a true constant low. From the BTS7960's input truth table,
  both legs low with EN high should brake and EN low should coast; not yet
  confirmed on hardware.
- Wiring the robot to this map is a manual step; the physical connections are
  not verified by any test in this repo.

## Alternatives considered

- **The earlier notes' map** (D5/D6 PWM, D7-D10 direction, encoder B on D11/D12): written
  for an L298N-style driver; the D11/D12 B-channel idea also leaves A and B in
  different ports, and its interrupt-number labels for the Mega (D2/D3 as
  INT0/INT1) are wrong.
- **Motors on Timer4 + Timer5 (D6/D7 + D44/D45):** equivalent electrically, but
  scatters the harness across two header areas and consumes Timer5 for no gain.
- **Motors on Timer3 (OC3B/OC3C = D2/D3):** the timer guide's example choice;
  collides with the encoder pins.
- **Encoders on the PCINT2 bank (one shared ISR):** frees external-interrupt
  pins, but every edge pays for diffing four channels.
- **EN tied to VCC (no GPIO):** saves two pins, loses the firmware kill switch.
