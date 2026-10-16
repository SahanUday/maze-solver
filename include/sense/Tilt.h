// ============================================================================
//  Tilt.h  --  nose-up slope from the accelerometer's forward axis, in counts. No hardware, no
//  Arduino.h - testable under env:native.
//
//  Gravity on the forward axis is IMU_ACCEL_ONE_G_COUNTS * sin(incline), so a threshold in counts
//  answers "on the bridge" without trigonometry (decisions/0008). Valid at constant speed or at
//  rest only: the robot's own acceleration reads as a slope (500 mm/s^2 is about 875 counts).
//  docs/architecture/modules/perception.md
// ============================================================================

#pragma once

#include <stdint.h>

#include "spec/Imu.h"
#include "core/RobotState.h"

static_assert(IMU_ACCEL_AXIS_FORWARD < 3, "IMU_ACCEL_AXIS_FORWARD indexes ImuRaw::accel");

// Slope in accel counts, positive = nose up, against `levelCounts` (the forward-axis reading on
// level ground). Saturates at +-INT16_MAX.
inline int16_t inclineCounts(const ImuRaw &imu, int16_t levelCounts)
{
    int32_t slope = static_cast<int32_t>(imu.accel[IMU_ACCEL_AXIS_FORWARD]) - levelCounts;
    if constexpr (IMU_ACCEL_NOSE_UP_NEGATIVE) {
        slope = -slope;
    }
    if (slope > INT16_MAX) {
        return INT16_MAX;
    }
    return static_cast<int16_t>(slope < -INT16_MAX ? -INT16_MAX : slope);
}

// The forward-axis reading with the robot level and still, averaged over a capture at boot on
// the start surface. A capture that was too short, or whose samples spread wider than
// INCLINE_LEVEL_MAX_RANGE_COUNTS (the robot was being moved), is refused: `counts` keeps the
// bench value and `ready` stays false.
struct InclineLevel {
    int32_t sum = 0;
    uint16_t samples = 0;
    int16_t low = INT16_MAX;
    int16_t high = INT16_MIN;
    int16_t counts = IMU_ACCEL_FORWARD_LEVEL_COUNTS;
    bool ready = false;
};

inline void inclineLevelReset(InclineLevel &level)
{
    level = InclineLevel{};
}

// One sample, taken with the robot still. Saturates at UINT16_MAX samples.
inline void inclineLevelAccumulate(InclineLevel &level, const ImuRaw &imu)
{
    if (level.samples == UINT16_MAX) {
        return;
    }
    const int16_t v = imu.accel[IMU_ACCEL_AXIS_FORWARD];
    level.sum += v;
    ++level.samples;
    level.low = v < level.low ? v : level.low;
    level.high = v > level.high ? v : level.high;
}

inline bool inclineLevelFinish(InclineLevel &level)
{
    const int32_t range = static_cast<int32_t>(level.high) - level.low;
    level.ready = level.samples >= INCLINE_LEVEL_MIN_SAMPLES &&
                  range <= static_cast<int32_t>(INCLINE_LEVEL_MAX_RANGE_COUNTS);
    level.counts = level.ready ? static_cast<int16_t>(level.sum / level.samples)
                               : static_cast<int16_t>(IMU_ACCEL_FORWARD_LEVEL_COUNTS);
    return level.ready;
}
