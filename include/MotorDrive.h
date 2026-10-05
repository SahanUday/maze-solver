// ============================================================================
//  MotorDrive.h  --  signed command -> per-leg duty for a BTS7960 half-bridge
//  pair. No hardware, no Arduino.h - testable under env:native.
// ============================================================================

#pragma once

#include <stdint.h>

struct MotorDuty {
    uint16_t rpwm; // forward leg
    uint16_t lpwm; // reverse leg
};

// command > 0 drives RPWM, < 0 drives LPWM, the other leg is 0. Magnitude is
// clamped to `top`. Both legs 0 = both driver inputs low.
constexpr MotorDuty motorDutyFromCommand(int16_t command, uint16_t top)
{
    const int32_t c = command;
    const uint32_t mag = static_cast<uint32_t>(c < 0 ? -c : c);
    const uint16_t duty = static_cast<uint16_t>(mag > top ? top : mag);
    return c < 0 ? MotorDuty{0, duty} : MotorDuty{duty, 0};
}
