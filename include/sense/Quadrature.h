// ============================================================================
//  Quadrature.h  --  4x quadrature step decoding. No hardware, no Arduino.h -
//  testable under env:native.
//
//  State is (B << 1) | A. "+1" means A leads B (00 -> 01 -> 11 -> 10 -> 00).
//  | B | A | (B << 1) | A | State |
//  |---|---|--------------|-------|
//  | 0 | 0 |       00     |    0  |
//  | 0 | 1 |       01     |    1  |
//  | 1 | 0 |       10     |    2  |
//  | 1 | 1 |       11     |    3  |
// ============================================================================

#pragma once

#include <stdint.h>

// Position of a 2-bit Gray-code state in the cycle 00, 01, 11, 10 -> 0..3.
constexpr uint8_t quadraturePosition(uint8_t state)
{
    return static_cast<uint8_t>((state ^ (state >> 1)) & 3); // & 3 : keeps only the lowest two bits
}

// +1 / -1 for a single legal step, 0 for no change. A two-state jump (missed
// edge or noise) is also 0: its direction is unknowable, so it is not counted.
// d = 0 → no movement
// d = 1 → forward one step
// d = 3 → backward one step
// d = 2 → jumped two states
constexpr int8_t quadratureDelta(uint8_t prevState, uint8_t currState)
{
    const uint8_t d =
        static_cast<uint8_t>((quadraturePosition(currState) - quadraturePosition(prevState)) & 3);
    return (d & 1) ? static_cast<int8_t>(2 - d) : static_cast<int8_t>(0);
}
