// ============================================================================
//  MotionPrimitives.h  --  Straight and Turn moves. No hardware, no Arduino.h -
//  testable under env:native.
//
//  begin, then step once per tick: step writes the MotionTarget for MotionControl and
//  returns a MoveStatus. Never touches the motors.
//    Straight   a distance along a held heading
//    Turn       an in-place pivot to an absolute heading
//  docs/architecture/modules/motion_primitives.md
// ============================================================================

#pragma once

#include <stdint.h>

#include "Heading.h"
#include "MotionProfile.h"
#include "MotionTypes.h"

struct PlanParams {
    float accelMmS2 = 0.0f;
    float turnRateMaxDps = 0.0f;
    float turnAccelDps2 = 0.0f;
    float stepToleranceMm = 0.0f;
    float turnToleranceDeg = 0.0f;
    uint16_t turnSettleMs = 0;
    uint16_t moveTimeoutMs = 0; // per leg; 0 = no limit
};

enum class MoveStatus : uint8_t {
    Running,
    Done,
    Timeout, // ran longer than moveTimeoutMs
};

// ---- Straight ----------------------------------------------------------------

struct Straight {
    float targetMm = 0.0f; // absolute odometer position to reach
    float refMm = 0.0f;    // where the profile says the robot is
    float refSpeedMmS = 0.0f;
    float vMaxMmS = 0.0f;
    float vEndMmS = 0.0f; // arrival speed; 0 = stop
    float headingRefDeg = 0.0f;
    uint16_t elapsedMs = 0;
};

// Start a move of `distMm` (signed) from the current position, at the measured speed.
inline void straightBegin(Straight &s, const MotionFeedback &fb, float distMm, float vMaxMmS,
                          float vEndMmS, float headingRefDeg)
{
    s.targetMm = fb.distMm + distMm;
    s.refMm = fb.distMm;
    s.refSpeedMmS = fb.speedMmS;
    s.vMaxMmS = vMaxMmS;
    s.vEndMmS = vEndMmS;
    s.headingRefDeg = headingRefDeg;
    s.elapsedMs = 0;
}

// Move the finish line `addMm` on and change the arrival speed. Restarts the timeout.
inline void straightExtend(Straight &s, float addMm, float vEndMmS)
{
    s.targetMm += addMm;
    s.vEndMmS = vEndMmS;
    s.elapsedMs = 0;
}

// Replace the finish line.
inline void straightRetarget(Straight &s, float targetMm, float vEndMmS)
{
    s.targetMm = targetMm;
    s.vEndMmS = vEndMmS;
}

// One tick. `trimDeg` is added to the held heading (wall centring). Done: stopped within
// tolerance (vEnd 0), or reference past the target (pass-through).
inline MoveStatus straightStep(Straight &s, const PlanParams &plan, const MotionFeedback &fb,
                               float trimDeg, uint16_t dtMs, MotionTarget &out)
{
    const float dtS = static_cast<float>(dtMs) / 1000.0f;
    s.refSpeedMmS =
        profileStep(s.refSpeedMmS, s.targetMm - s.refMm, s.vMaxMmS, plan.accelMmS2, s.vEndMmS, dtS);
    s.refMm += s.refSpeedMmS * dtS;
    s.elapsedMs = static_cast<uint16_t>(s.elapsedMs + dtMs);

    out.distRefMm = s.refMm;
    out.speedRefMmS = s.refSpeedMmS;
    out.headingRefDeg = headingWrapDeg(s.headingRefDeg + trimDeg);
    out.yawRateRefDps = 0.0f;
    out.pivot = false;

    float err = s.targetMm - fb.distMm;
    err = err < 0.0f ? -err : err;
    if (s.vEndMmS == 0.0f) {
        if (s.refSpeedMmS == 0.0f && err <= plan.stepToleranceMm) {
            return MoveStatus::Done;
        }
    } else if (s.refMm >= s.targetMm) {
        return MoveStatus::Done;
    }
    if (plan.moveTimeoutMs > 0 && s.elapsedMs >= plan.moveTimeoutMs) {
        return MoveStatus::Timeout;
    }
    return MoveStatus::Running;
}

// ---- Turn --------------------------------------------------------------------

struct Turn {
    float startHeadingDeg = 0.0f;
    float targetHeadingDeg = 0.0f; // absolute, what the gyro must read at the end
    float deltaDeg = 0.0f;         // signed sweep; its sign picks the direction
    float refAngleDeg = 0.0f;      // sweep done so far, per the profile
    float refRateDps = 0.0f;
    float holdMm = 0.0f; // odometer position held while pivoting
    uint16_t settledMs = 0;
    uint16_t elapsedMs = 0;
};

// Pivot by `deltaDeg` (CCW positive) to end on `targetHeadingDeg`. Both are given: a 180 is
// ambiguous from the headings alone.
inline void turnBegin(Turn &t, const MotionFeedback &fb, float targetHeadingDeg, float deltaDeg)
{
    t.startHeadingDeg = fb.headingDeg;
    t.targetHeadingDeg = headingWrapDeg(targetHeadingDeg);
    t.deltaDeg = deltaDeg;
    t.refAngleDeg = 0.0f;
    t.refRateDps = 0.0f;
    t.holdMm = fb.distMm;
    t.settledMs = 0;
    t.elapsedMs = 0;
}

inline MoveStatus turnStep(Turn &t, const PlanParams &plan, const MotionFeedback &fb, uint16_t dtMs,
                           MotionTarget &out)
{
    const float dtS = static_cast<float>(dtMs) / 1000.0f;
    t.refRateDps = profileStep(t.refRateDps, t.deltaDeg - t.refAngleDeg, plan.turnRateMaxDps,
                               plan.turnAccelDps2, 0.0f, dtS);
    t.refAngleDeg += t.refRateDps * dtS;
    t.elapsedMs = static_cast<uint16_t>(t.elapsedMs + dtMs);

    // Profile stopped within a degree: aim at the grid heading, not start + sweep.
    const float left = t.deltaDeg - t.refAngleDeg;
    const bool profileDone = t.refRateDps == 0.0f && left < 1.0f && left > -1.0f;
    out.distRefMm = t.holdMm;
    out.speedRefMmS = 0.0f;
    out.headingRefDeg =
        profileDone ? t.targetHeadingDeg : headingWrapDeg(t.startHeadingDeg + t.refAngleDeg);
    out.yawRateRefDps = t.refRateDps;
    out.pivot = true;

    float err = headingWrapDeg(t.targetHeadingDeg - fb.headingDeg);
    err = err < 0.0f ? -err : err;
    if (profileDone && err <= plan.turnToleranceDeg) {
        t.settledMs = static_cast<uint16_t>(t.settledMs + dtMs);
        if (t.settledMs >= plan.turnSettleMs) {
            return MoveStatus::Done;
        }
    } else {
        t.settledMs = 0;
    }
    if (plan.moveTimeoutMs > 0 && t.elapsedMs >= plan.moveTimeoutMs) {
        return MoveStatus::Timeout;
    }
    return MoveStatus::Running;
}
