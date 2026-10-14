// ============================================================================
//  MotionProfile.h  --  trapezoid speed profile, one axis (mm and mm/s, or deg and deg/s).
//  No hardware, no Arduino.h - testable under env:native.
//
//  Stepped on the reference trajectory, not the measured robot.
//  docs/architecture/modules/motion_profile.md
// ============================================================================

#pragma once

#include <math.h>

// Next signed speed. `remaining` is the signed distance left; accelerates at `accel` up to `vMax`
// and brakes as late as possible to arrive at `vEnd` (0 = stop, vMax = pass through).
inline float profileStep(float v, float remaining, float vMax, float accel, float vEnd, float dtS)
{
    const float dir = remaining >= 0.0f ? 1.0f : -1.0f;
    const float rem = remaining * dir;
    const float cap = vMax > 0.0f ? vMax : 0.0f;
    const float end = vEnd < 0.0f ? 0.0f : (vEnd > cap ? cap : vEnd);

    // Fastest speed from which `accel` can still reach `end` within `rem`.
    const float brake = sqrtf(end * end + 2.0f * accel * rem);
    const float target = brake < cap ? brake : cap;

    float along = v * dir; // negative if it points away from the target
    const float step = accel * dtS;
    // The braking envelope only creeps toward zero: finish the stop once one step is enough.
    if (end == 0.0f && target <= step && along <= step && along >= -step) {
        return 0.0f;
    }
    if (along < target) {
        along = along + step < target ? along + step : target;
    } else {
        along = along - step > target ? along - step : target;
    }
    return along * dir;
}
