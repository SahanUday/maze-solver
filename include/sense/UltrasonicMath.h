// ============================================================================
//  UltrasonicMath.h  --  HC-SR04 echo duration to distance.
//  No Arduino.h: builds and unit-tests under env:native.
//  docs/architecture/decisions/0007-non-blocking-ultrasonic-ranging.md
// ============================================================================

#pragma once

#include <stdint.h>

#include "RobotSpec.h"

static_assert(US_RANGE_CAP_MM <= US_MAX_RANGE_MM, "cap must be within sensor range");
static_assert(US_RANGE_CAP_MM > US_MIN_RANGE_MM, "cap must exceed the dead zone");

struct UltrasonicReading {
    // Meaningful only when valid. Never a guess on timeout.
    uint16_t distanceMm = 0;
    bool valid = false;
};

// Echo timer ticks (0.5us) to millimetres, plus a per-sensor offset.
// mm = ticks * speed / 4000. See ADR 0007.
inline UltrasonicReading ultrasonicReadingFromTicks(uint16_t ticks, int16_t offsetMm = 0)
{
    UltrasonicReading r;

    // No echo, or past the deadline.
    if (ticks == 0 || ticks > US_ECHO_TIMEOUT_TICKS) {
        return r;
    }

    // Face-to-target, before any mounting offset.
    const uint32_t rawMm = (static_cast<uint32_t>(ticks) * SPEED_OF_SOUND_M_S) / 4000u;

    // Dead zone is a sensor property: judged on raw.
    if (rawMm < US_MIN_RANGE_MM) {
        return r;
    }

    const int32_t adjusted = static_cast<int32_t>(rawMm) + offsetMm;

    // A negative offset can push a near reading past zero.
    if (adjusted <= 0 || adjusted > 0xFFFF) {
        return r;
    }

    r.distanceMm = static_cast<uint16_t>(adjusted);
    r.valid = true;
    return r;
}
