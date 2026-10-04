# encoders

JGA25-370 quadrature encoders, 4x decoded. Code: `src/hal/encoders.{h,cpp}`,
pure decode in `include/Quadrature.h`. Decisions:
[0002](../decisions/0002-drive-and-sensing-hardware-allocation.md),
[0003](../decisions/0003-four-x-quadrature-decoding.md).

## Interface

- `encodersInit()` - pins to input + pull-up, edge interrupts on, counts zeroed.
- `encodersRead(left, right)` - atomic snapshot of both signed `int32_t` counts.
  Called once per tick from `readSensors()` into `RobotState`.

## How it works

Left = D2/D3 (INT4/INT5), right = D19/D18 (INT2/INT3), A on the lower port bit.
All four vectors do the same thing: snapshot the motor's port bits as
`(B<<1)|A`, add `quadratureDelta(lastState, state)` to the count, store the new
state. `+` means A leads B; `ENC_L_INVERT` / `ENC_R_INVERT` flip it per side.

## Invariants

- A two-state jump counts 0 (not a guess); the next legal step resumes counting.
- The counts are the only encoder data - speed is derived by the control layer
  from count deltas per tick.

## Status

Compiles; the decode logic is covered by `test/test_quadrature` on the host.
Channel A has been seen counting on a real motor; channel B, and so full 4x
decoding, has not been verified on hardware.

Bring-up: spin each wheel by hand and check both channels toggle and the count
rises forward (else set `ENC_*_INVERT`); turn one output revolution and record
the count to get `ENCODER_COUNTS_PER_REV`; confirm no phantom counts with the
motor drivers running.
