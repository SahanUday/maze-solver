// ============================================================================
//  core/MathUtil.h  --  the small arithmetic helpers the headers share. No hardware, no
//  Arduino.h - testable under env:native.
// ============================================================================

#pragma once

#include <stdint.h>

inline float absf(float x)
{
    return x < 0.0f ? -x : x;
}

// `a + b`, held at UINT16_MAX instead of wrapping: for time counters that run until something
// resets them.
inline uint16_t satAddU16(uint16_t a, uint16_t b)
{
    return a > UINT16_MAX - b ? UINT16_MAX : static_cast<uint16_t>(a + b);
}
