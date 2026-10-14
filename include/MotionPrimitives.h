// ============================================================================
//  MotionPrimitives.h  --  the moves the mission is built from. No hardware, no
//  Arduino.h - testable under env:native.
//
//  A primitive is a small per-tick state machine: begin it, then call its step
//  function once per tick. It writes the MotionTarget for MotionControl and says
//  whether it has finished. It never touches the motors itself.
//    Straight   a distance along a held heading (StepCell, below, builds on it)
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
    uint16_t moveTimeoutMs = 0; // 0 = no limit
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
    float vEndMmS = 0.0f; // speed to arrive with; 0 = stop
    float headingRefDeg = 0.0f;
    uint16_t elapsedMs = 0;
};

// Start a move of `distMm` (signed) from where the robot is now. The profile starts at the
// measured speed, so a move begun while rolling carries on without a jerk.
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

// Move the finish line `addMm` further on and change the arrival speed: how a pass-through cell
// becomes a stop, or the next cell is chained on without slowing.
inline void straightExtend(Straight &s, float addMm, float vEndMmS)
{
    s.targetMm += addMm;
    s.vEndMmS = vEndMmS;
    s.elapsedMs = 0; // the timeout is per leg: a chain of cells must not add up to one
}

// Replace the finish line (a wall ahead told us where the cell centre really is).
inline void straightRetarget(Straight &s, float targetMm, float vEndMmS)
{
    s.targetMm = targetMm;
    s.vEndMmS = vEndMmS;
}

// One tick. `trimDeg` is added to the held heading: wall centring steers through it.
// Done means stopped on the target for a stop move (vEnd 0), or the reference passed the target
// for a pass-through move.
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
    float holdMm = 0.0f; // odometer position to hold while pivoting
    uint16_t settledMs = 0;
    uint16_t elapsedMs = 0;
};

// Pivot by `deltaDeg` (CCW positive) to end on `targetHeadingDeg`. Both are given because a
// 180 is ambiguous from the headings alone, and because the caller's target is on the maze's
// 90 deg grid, so gyro error does not accumulate from turn to turn.
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

    // Once the profile has stopped within a degree of its sweep, aim at the grid heading itself
    // rather than at the sweep, which is only as exact as the start heading was.
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
