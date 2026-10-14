// ============================================================================
//  MotionProfile.h  --  trapezoid speed profile, one axis. No hardware, no
//  Arduino.h - testable under env:native.
//
//  Used for distance (mm, mm/s) and for angle (deg, deg/s) alike: the maths does
//  not care. The profile is stepped on the REFERENCE trajectory, not the measured
//  robot, so a slipping wheel cannot make it stall; MotionControl closes the gap.
// ============================================================================

#pragma once

#include <math.h>

// Next signed speed. `remaining` is the signed distance still to cover along this axis; the
// speed carries its sign. It accelerates at `accel` up to `vMax`, and starts braking as late as
// it can while still arriving at `vEnd` (0 = stop; = vMax to pass through without slowing).
inline float profileStep(float v, float remaining, float vMax, float accel, float vEnd, float dtS)
{
    const float dir = remaining >= 0.0f ? 1.0f : -1.0f;
    const float rem = remaining * dir;
    const float cap = vMax > 0.0f ? vMax : 0.0f;
    const float end = vEnd < 0.0f ? 0.0f : (vEnd > cap ? cap : vEnd);

    // Fastest speed from which `accel` can still bring the axis down to `end` within `rem`.
    const float brake = sqrtf(end * end + 2.0f * accel * rem);
    const float target = brake < cap ? brake : cap;

    float along = v * dir; // speed in the direction of travel; negative if it points away
    const float step = accel * dtS;
    // The braking envelope shrinks with the root of the distance left, so it would creep toward
    // the target for ever. Once one tick of braking is enough, finish the stop.
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
