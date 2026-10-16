// ============================================================================
//  sense/Odometry.h  --  encoder counts to distance and speed. No hardware, no Arduino.h -
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
    bool primed = false;   // the first update only records the counts
    int32_t sumCounts = 0; // dL + dR since reset: exact, so distance does not drift over a run
    float distMm = 0.0f;
    float speedMmS = 0.0f; // low-pass filtered
};

// Weight of the newest speed sample.
constexpr float ODOM_SPEED_FILTER = 0.5f;

inline void odometryReset(Odometry &odo)
{
    odo = Odometry{};
}

// Fold in one tick's counts. `mmPerCount` of 0 (uncalibrated) reads as not moving; it applies to
// the whole sum, so keep it constant for a run.
inline void odometryUpdate(Odometry &odo, int32_t countL, int32_t countR, float mmPerCount,
                           uint16_t dtMs)
{
    if (!odo.primed) {
        odo.lastL = countL;
        odo.lastR = countR;
        odo.primed = true;
        return;
    }
    // Unsigned subtraction: the counts wrap, and signed overflow is undefined.
    const int32_t dL =
        static_cast<int32_t>(static_cast<uint32_t>(countL) - static_cast<uint32_t>(odo.lastL));
    const int32_t dR =
        static_cast<int32_t>(static_cast<uint32_t>(countR) - static_cast<uint32_t>(odo.lastR));
    odo.lastL = countL;
    odo.lastR = countR;

    const int32_t stepCounts = dL + dR;
    odo.sumCounts += stepCounts;
    odo.distMm = static_cast<float>(odo.sumCounts) * 0.5f * mmPerCount;
    if (dtMs == 0) {
        return;
    }
    const float stepMm = static_cast<float>(stepCounts) * 0.5f * mmPerCount;
    const float rawMmS = stepMm * 1000.0f / static_cast<float>(dtMs);
    odo.speedMmS += ODOM_SPEED_FILTER * (rawMmS - odo.speedMmS);
}
