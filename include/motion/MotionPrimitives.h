// ============================================================================
//  motion/MotionPrimitives.h  --  Straight and Turn moves. No hardware, no Arduino.h -
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

#include "motion/Heading.h"
#include "motion/MotionProfile.h"
#include "motion/MotionTypes.h"

struct PlanParams {
    float accelMmS2 = 0.0f;
    float turnRateMaxDps = 0.0f;
    float turnAccelDps2 = 0.0f;
    float stepToleranceMm = 0.0f;
    float turnToleranceDeg = 0.0f;
    uint16_t turnSettleMs = 0;
    // Stuck: no new low in the error, by at least progressMin, for patienceMs. 0 = off.
    float progressMinMm = 0.0f;
    float progressMinDeg = 0.0f;
    uint16_t patienceMs = 0;
};

enum class MoveStatus : uint8_t {
    Running,
    Done,
    Stuck, // no progress for patienceMs
};

// ---- Progress ----------------------------------------------------------------

// A move's length and speed never make it stuck; only a lack of progress does. Progress is a new
// low in the error by at least `minStep`; time without one is idle. Within one step of the
// tolerance the approach is slow by nature, so idle time is held there, not counted and not reset:
// an oscillation that sweeps through the target still runs out.
struct ProgressWatch {
    float bestErr = 0.0f;
    uint16_t idleMs = 0;
};

constexpr float PROGRESS_NO_LOW = 3.0e38f; // no low yet: the next tick sets one

inline void progressBegin(ProgressWatch &w, float err)
{
    w.bestErr = err;
    w.idleMs = 0;
}

// The finish line moved: forget the low and the idle time.
inline void progressRestart(ProgressWatch &w)
{
    progressBegin(w, PROGRESS_NO_LOW);
}

// Fold in one tick's error. True once `patienceMs` of idle time has passed; 0 = never.
inline bool progressStep(ProgressWatch &w, float err, float tolerance, float minStep,
                         uint16_t patienceMs, uint16_t dtMs)
{
    if (err <= tolerance + minStep) {
        w.bestErr = err < w.bestErr ? err : w.bestErr;
        return false;
    }
    if (err < w.bestErr - minStep) {
        w.bestErr = err;
        w.idleMs = 0;
    } else {
        w.idleMs = static_cast<uint16_t>(w.idleMs + dtMs);
    }
    return patienceMs > 0 && w.idleMs >= patienceMs;
}

// ---- Straight ----------------------------------------------------------------

struct Straight {
    float targetMm = 0.0f; // absolute odometer position to reach
    bool forward = true;   // direction of travel, fixed at begin
    float refMm = 0.0f;    // where the profile says the robot is
    float refSpeedMmS = 0.0f;
    float vMaxMmS = 0.0f;
    float vEndMmS = 0.0f; // arrival speed; 0 = stop
    float headingRefDeg = 0.0f;
    bool fresh = true; // the next step is the first of the move
    ProgressWatch watch;
};

// Start a move of `distMm` (signed) from the current position, at the measured speed.
inline void straightBegin(Straight &s, const MotionFeedback &fb, float distMm, float vMaxMmS,
                          float vEndMmS, float headingRefDeg)
{
    s.targetMm = fb.distMm + distMm;
    s.forward = distMm >= 0.0f;
    s.refMm = fb.distMm;
    s.refSpeedMmS = fb.speedMmS;
    s.vMaxMmS = vMaxMmS;
    s.vEndMmS = vEndMmS;
    s.headingRefDeg = headingRefDeg;
    s.fresh = true;
    progressBegin(s.watch, distMm < 0.0f ? -distMm : distMm);
}

// Move the finish line `addMm` on (signed like `distMm`) and change the arrival speed.
inline void straightExtend(Straight &s, float addMm, float vEndMmS)
{
    s.targetMm += addMm;
    s.vEndMmS = vEndMmS;
    progressRestart(s.watch);
}

// Replace the finish line. It cannot come closer than the braking distance in the direction of the
// move (the move would run through it and come back), and a line already nearer is never pushed
// out. A move never reverses: a line behind the reference is taken as the current line.
inline void straightRetarget(Straight &s, float targetMm, float vEndMmS, float accelMmS2)
{
    const float dir = s.forward ? 1.0f : -1.0f;
    const float v = s.refSpeedMmS * dir; // along the move; negative while creeping back
    const float excess = v * v - vEndMmS * vEndMmS;
    const float brakeMm =
        v > 0.0f && excess > 0.0f && accelMmS2 > 0.0f ? excess / (2.0f * accelMmS2) : 0.0f;
    const float nearestMm = s.refMm + dir * brakeMm;
    if (s.forward) {
        const float limit = nearestMm < s.targetMm ? nearestMm : s.targetMm;
        s.targetMm = targetMm > limit ? targetMm : limit;
    } else {
        const float limit = nearestMm > s.targetMm ? nearestMm : s.targetMm;
        s.targetMm = targetMm < limit ? targetMm : limit;
    }
    s.vEndMmS = vEndMmS;
}

// The reference has reached or passed the finish line in the direction of travel.
inline bool straightPassed(const Straight &s)
{
    return s.forward ? s.refMm >= s.targetMm : s.refMm <= s.targetMm;
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

    out.distRefMm = s.refMm;
    out.speedRefMmS = s.refSpeedMmS;
    out.headingRefDeg = headingWrapDeg(s.headingRefDeg + trimDeg);
    out.yawRateRefDps = 0.0f;
    out.pivot = false;
    out.newMove = s.fresh;
    s.fresh = false;

    float err = s.targetMm - fb.distMm;
    err = err < 0.0f ? -err : err;
    if (s.vEndMmS == 0.0f) {
        if (s.refSpeedMmS == 0.0f && err <= plan.stepToleranceMm) {
            return MoveStatus::Done;
        }
    } else if (straightPassed(s)) {
        return MoveStatus::Done;
    }
    if (progressStep(s.watch, err, plan.stepToleranceMm, plan.progressMinMm, plan.patienceMs,
                     dtMs)) {
        return MoveStatus::Stuck;
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
    bool fresh = true; // the next step is the first of the move
    ProgressWatch watch;
};

// Pivot by `deltaDeg` (CCW positive, within +-180) to end on `targetHeadingDeg`. Both are given: a
// 180 is ambiguous from the headings alone.
inline void turnBegin(Turn &t, const MotionFeedback &fb, float targetHeadingDeg, float deltaDeg)
{
    t.startHeadingDeg = fb.headingDeg;
    t.targetHeadingDeg = headingWrapDeg(targetHeadingDeg);
    t.deltaDeg = deltaDeg;
    t.refAngleDeg = 0.0f;
    t.refRateDps = 0.0f;
    t.holdMm = fb.distMm;
    t.settledMs = 0;
    t.fresh = true;
    const float startErr = headingWrapDeg(t.targetHeadingDeg - fb.headingDeg);
    progressBegin(t.watch, startErr < 0.0f ? -startErr : startErr);
}

inline MoveStatus turnStep(Turn &t, const PlanParams &plan, const MotionFeedback &fb, uint16_t dtMs,
                           MotionTarget &out)
{
    const float dtS = static_cast<float>(dtMs) / 1000.0f;
    t.refRateDps = profileStep(t.refRateDps, t.deltaDeg - t.refAngleDeg, plan.turnRateMaxDps,
                               plan.turnAccelDps2, 0.0f, dtS);
    t.refAngleDeg += t.refRateDps * dtS;

    // Profile stopped within a degree: aim at the grid heading, not start + sweep.
    const float left = t.deltaDeg - t.refAngleDeg;
    const bool profileDone = t.refRateDps == 0.0f && left < 1.0f && left > -1.0f;
    out.distRefMm = t.holdMm;
    out.speedRefMmS = 0.0f;
    out.headingRefDeg =
        profileDone ? t.targetHeadingDeg : headingWrapDeg(t.startHeadingDeg + t.refAngleDeg);
    out.yawRateRefDps = t.refRateDps;
    out.pivot = true;
    out.newMove = t.fresh;
    t.fresh = false;

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
    if (progressStep(t.watch, err, plan.turnToleranceDeg, plan.progressMinDeg, plan.patienceMs,
                     dtMs)) {
        return MoveStatus::Stuck;
    }
    return MoveStatus::Running;
}
