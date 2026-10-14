// ============================================================================
//  Odometry.h  --  encoder counts to travelled distance and speed. No hardware,
//  no Arduino.h - testable under env:native.
//
//  Distance is the mean of the two wheels, so a pivot in place adds nothing to
//  it. It is cumulative and signed; a move records the value it started at.
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

// Weight of the newest speed sample. Encoder counts arrive in whole steps, so one tick's raw
// speed is quantised and noisy at the speeds this robot runs.
constexpr float ODOM_SPEED_FILTER = 0.5f;

inline void odometryReset(Odometry &odo)
{
    odo = Odometry{};
}

// Fold in one tick's counts. `mmPerCount` of 0 (not yet calibrated) leaves the robot "not
// moving" rather than dividing by it.
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
