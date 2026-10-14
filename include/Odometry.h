// ============================================================================
//  Odometry.h  --  encoder counts to distance and speed. No hardware, no Arduino.h -
//  testable under env:native.
//
//  Distance is the mean of the two wheels (a pivot adds none), cumulative and signed.
//  docs/architecture/modules/odometry.md
// ============================================================================

#pragma once

#include <stdint.h>

struct Odometry {
    int32_t lastL = 0;
    int32_t lastR = 0;
    bool primed = false; // the first update only records the counts
    float distMm = 0.0f;
    float speedMmS = 0.0f; // low-pass filtered
};

// Weight of the newest speed sample.
constexpr float ODOM_SPEED_FILTER = 0.5f;

inline void odometryReset(Odometry &odo)
{
    odo = Odometry{};
}

// Fold in one tick's counts. `mmPerCount` of 0 (uncalibrated) reads as not moving.
inline void odometryUpdate(Odometry &odo, int32_t countL, int32_t countR, float mmPerCount,
                           uint16_t dtMs)
{
    if (!odo.primed) {
        odo.lastL = countL;
        odo.lastR = countR;
        odo.primed = true;
        return;
    }
    const int32_t dL = countL - odo.lastL;
    const int32_t dR = countR - odo.lastR;
    odo.lastL = countL;
    odo.lastR = countR;

    const float stepMm = (static_cast<float>(dL) + static_cast<float>(dR)) * 0.5f * mmPerCount;
    odo.distMm += stepMm;
    if (dtMs == 0) {
        return;
    }
    const float rawMmS = stepMm * 1000.0f / static_cast<float>(dtMs);
    odo.speedMmS += ODOM_SPEED_FILTER * (rawMmS - odo.speedMmS);
}
