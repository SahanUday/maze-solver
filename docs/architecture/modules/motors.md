# motors

Two HW-039 / IBT_2 (BTS7960) drivers, register-level PWM. Code:
`src/hal/motors.{h,cpp}`, pure duty split in `include/MotorDrive.h`. Decision:
[0002](../decisions/0002-drive-and-sensing-hardware-allocation.md).

## Interface

- `motorsInit()` - all pins low, EN low (coast), timers running at duty 0. Does
  not enable the drivers.
- `motorsEnable(bool)` - EN pins. Low = both half-bridges off, motors coast.
- `motorSet(Motor, int16_t)` - signed duty in timer counts, clamped to
  +-`MOTOR_PWM_TOP` (799). `> 0` is forward after `MOTOR_*_INVERT`.

## How it works

Left on Timer4 (OC4A = RPWM D6, OC4B = LPWM D7, EN D8), right on Timer1 (OC1A =
RPWM D11, OC1B = LPWM D12, EN D10). Fast PWM mode 14, `ICRn` = 799, clk/1 =
20 kHz. Positive command PWMs RPWM with LPWM low; negative does the reverse. The
leg at zero is disconnected from its timer and held low, so duty 0 is a true
constant low.

From the BTS7960 truth table, with EN high a command of 0 should brake (both
low-side FETs on) and EN low should coast. Not yet confirmed on hardware.

## Invariants

- Reversing is a change of which leg carries PWM; no direction pin is sequenced.
  Reversal under load has not been characterised, so ramp command changes in
  the control layer rather than flipping full-speed.
- Duty units are timer counts. `MOTOR_MIN_PWM_L/R` must be measured at 20 kHz.
- Nothing enables the drivers at boot; the control layer owns `motorsEnable`.

## Status

Compiles; the duty split is covered by `test/test_motor_drive` on the host.
`motorSet` / `motorsEnable` are not called by anything yet and have not been
run on hardware. Bring-up: check the boards' EN pins have a pull-down (else add
one, since the Mega pins float during reset); with EN high, step one motor
0 -> +100 -> 0 -> -100 and confirm direction and that the other leg stays low on
a scope or meter; set `MOTOR_*_INVERT` so positive is forward; then measure the
dead zone.
