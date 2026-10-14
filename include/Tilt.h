// ============================================================================
//  Tilt.h  --  how far the board is tipped from flat, from the accelerometer. No hardware, no
//  Arduino.h - testable under env:native.
//
//  The angle between gravity and the board's Z axis, so which way X and Y point does not matter.
//  It reads high while accelerating: compare it to a threshold well above a few degrees.
// ============================================================================

#pragma once

#include <math.h>

#include "RobotState.h"

inline float tiltDeg(const ImuRaw &imu)
{
    const float x = static_cast<float>(imu.accel[0]);
    const float y = static_cast<float>(imu.accel[1]);
    const float z = static_cast<float>(imu.accel[2]);
    return atan2f(sqrtf(x * x + y * y), z) * 57.29578f;
}
