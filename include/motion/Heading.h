// ============================================================================
//  Heading.h  --  gyro bias estimate and yaw integration. No hardware, no
//  Arduino.h - testable under env:native.
//
//  The gyro measures a rate, not an angle, so heading is the integral of that
//  rate and any uncorrected bias accumulates without bound. Bias removal is
//  therefore not optional: see decisions/0008-mpu6050-heading-and-tilt.md.
//  docs/architecture/modules/heading.md
// ============================================================================

#pragma once

#include <math.h>
#include <stdint.h>

#include "spec/Imu.h"

// A gyro reads slightly non-zero while still. Averaging that offset is the heart
// of this struct; `counts` is in raw gyro counts, like the samples it came from.
// It also tracks how much the samples scatter (`spread`, their standard
// deviation in counts): a robot that was being moved during the capture scatters
// far more than a still one, and its mean is a wrong zero.
struct GyroBias {
    int32_t sum = 0;
    uint16_t samples = 0;
    float mean = 0.0f;   // running mean and sum of squared deviations (Welford),
    float m2 = 0.0f;     // which stay accurate where sum-of-squares would not
    float spread = 0.0f; // set by gyroBiasFinish()
    float counts = 0.0f;
    bool ready = false;
};

struct Heading {
    float deg = 0.0f; // 0 = the heading held when the bias was finished
};

// Start a calibration: nothing sampled yet, and not usable until finished.
inline void gyroBiasReset(GyroBias &bias)
{
    bias.sum = 0;
    bias.samples = 0;
    bias.mean = 0.0f;
    bias.m2 = 0.0f;
    bias.spread = 0.0f;
    bias.counts = 0.0f;
    bias.ready = false;
}

// One sample of the yaw-rate count, taken with the robot held still. Saturates at
// GYRO_BIAS_SAMPLES so a caller that over-runs cannot overflow the sum.
inline void gyroBiasAccumulate(GyroBias &bias, int16_t yawRateCounts)
{
    if (bias.samples >= GYRO_BIAS_SAMPLES) {
        return;
    }
    bias.sum += yawRateCounts;
    ++bias.samples;
    const float x = static_cast<float>(yawRateCounts);
    const float delta = x - bias.mean;
    bias.mean += delta / static_cast<float>(bias.samples);
    bias.m2 += delta * (x - bias.mean);
}

// End the calibration. False (and not ready) if too few samples got through, or
// if they scatter more than GYRO_BIAS_MAX_SPREAD_COUNTS (the robot was moving):
// either way the average is not to be trusted, and a half-failed calibration is
// worse than an obvious one. `spread` is filled in for the caller to report.
// A perfectly steady rotation has no scatter and is not caught by this.
inline bool gyroBiasFinish(GyroBias &bias)
{
    bias.spread = 0.0f;
    if (bias.samples < GYRO_BIAS_MIN_SAMPLES) {
        bias.counts = 0.0f;
        bias.ready = false;
        return false;
    }
    bias.spread = sqrtf(bias.m2 / static_cast<float>(bias.samples));
    if (bias.spread > GYRO_BIAS_MAX_SPREAD_COUNTS) {
        bias.counts = 0.0f;
        bias.ready = false;
        return false;
    }
    bias.counts = static_cast<float>(bias.sum) / static_cast<float>(bias.samples);
    bias.ready = true;
    return true;
}

// Fold a heading back into [-180, 180). The per-tick change is small, so the loops
// run at most once in normal use.
inline float headingWrapDeg(float deg)
{
    while (deg >= 180.0f) {
        deg -= 360.0f;
    }
    while (deg < -180.0f) {
        deg += 360.0f;
    }
    return deg;
}

// Forget the accumulated angle and call the current direction `deg`. Used to zero
// at the start of a run and, later, to snap to the maze's 90 deg grid.
inline void headingSet(Heading &heading, float deg)
{
    heading.deg = headingWrapDeg(deg);
}

// Raw yaw count -> deg/s, bias removed and polarity applied. Meaningless before
// the bias is ready, which is why headingUpdate() checks first.
inline float gyroYawRateDps(const GyroBias &bias, int16_t yawRateCounts)
{
    const float corrected = static_cast<float>(yawRateCounts) - bias.counts;
    const float dps = corrected / GYRO_COUNTS_PER_DPS;
    if constexpr (GYRO_YAW_INVERT) {
        return -dps;
    }
    return dps;
}

// Integrate one tick and return the yaw rate used (deg/s, CCW positive). Does nothing and returns 0
// until the bias is ready: integrating an unknown offset is what this module exists to prevent.
inline float headingUpdate(Heading &heading, const GyroBias &bias, int16_t yawRateCounts,
                           uint16_t dtMs)
{
    if (!bias.ready) {
        return 0.0f;
    }
    const float dps = gyroYawRateDps(bias, yawRateCounts);
    heading.deg = headingWrapDeg(heading.deg + dps * (static_cast<float>(dtMs) / 1000.0f));
    return dps;
}
