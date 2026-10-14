// Differential-drive plant for closed-loop motion tests: duty in, first-order lagged wheel speed
// out, integrated into MotionFeedback.
#pragma once

#include <math.h>

#include "Heading.h"
#include "MotionControl.h"
#include "MotionPrimitives.h"

struct Sim {
    static constexpr float MM_S_PER_DUTY = 0.5f; // wheel speed per duty count
    static constexpr float TRACK_MM = 100.0f;
    static constexpr float LAG = 0.5f; // share of the way to the commanded speed per tick

    float vL = 0.0f, vR = 0.0f;
    float distMm = 0.0f;
    float headingDeg = 0.0f;
    float yawRateDps = 0.0f;
    float speedMmS = 0.0f;
    float yMm = 0.0f; // sideways position, + = left of the start line

    void step(const MotionOutput &o, float dtS)
    {
        vL += LAG * (o.left * MM_S_PER_DUTY - vL);
        vR += LAG * (o.right * MM_S_PER_DUTY - vR);
        speedMmS = 0.5f * (vL + vR);
        yawRateDps = (vR - vL) / TRACK_MM * 57.29578f;
        distMm += speedMmS * dtS;
        yMm += speedMmS * dtS * sinf(headingDeg * 0.0174533f);
        headingDeg = headingWrapDeg(headingDeg + yawRateDps * dtS);
    }

    MotionFeedback feedback() const
    {
        MotionFeedback fb;
        fb.distMm = distMm;
        fb.speedMmS = speedMmS;
        fb.headingDeg = headingDeg;
        fb.yawRateDps = yawRateDps;
        return fb;
    }
};

inline MotionControlParams simControlParams()
{
    MotionControlParams p;
    p.pwmTop = 799;
    p.ffPwmPerMmS = 1.0f / Sim::MM_S_PER_DUTY;
    p.ffPwmPerDps = Sim::TRACK_MM * 0.0174533f / (2.0f * Sim::MM_S_PER_DUTY);
    p.kpDist = 4.0f;
    p.kdDist = 0.5f;
    p.kpTurn = 4.0f;
    p.kdTurn = 0.2f;
    return p;
}

inline PlanParams simPlanParams()
{
    PlanParams p;
    p.accelMmS2 = 1500.0f;
    p.turnRateMaxDps = 360.0f;
    p.turnAccelDps2 = 2500.0f;
    p.stepToleranceMm = 2.0f;
    p.turnToleranceDeg = 2.0f;
    p.turnSettleMs = 30;
    p.progressMinMm = 2.0f;
    p.progressMinDeg = 1.0f;
    p.patienceMs = 1000;
    return p;
}
