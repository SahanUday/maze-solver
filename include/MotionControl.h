// ============================================================================
//  MotionControl.h  --  MotionTarget + MotionFeedback to left/right duty. No hardware,
//  no Arduino.h - testable under env:native.
//
//    linear   PID on the odometer + speed feedforward
//    angular  PID on the gyro heading (not the encoders) + yaw-rate feedforward
//  left = linear - angular, right = (linear + angular) * ffScaleR; CCW positive.
//  docs/architecture/modules/motion_control.md
// ============================================================================

#pragma once

#include <stdint.h>

#include "Heading.h"
#include "MotionTypes.h"

// Gains are duty counts per mm, deg, mm/s or deg/s (P, D) and per mm*s or deg*s (I).
struct MotionControlParams {
    float kpDist = 0.0f;
    float kiDist = 0.0f;
    float kdDist = 0.0f; // on the speed error
    float kpTurn = 0.0f;
    float kiTurn = 0.0f;   // heading integral while pivoting
    float kdTurn = 0.0f;   // on the yaw-rate error
    float kiHold = -1.0f;  // heading integral while driving; negative = kiTurn
    float ffScaleR = 1.0f; // multiplies the whole right command
    float ffPwmPerMmS = 0.0f;
    float ffPwmPerDps = 0.0f;
    uint16_t minPwmL = 0; // non-zero commands are lifted to this duty; 0 = no lift
    uint16_t minPwmR = 0;
    uint16_t pwmTop = 0;
    // Stall: duty >= stallDuty with speed and yaw rate under their minimums for stallTimeMs.
    // stallDuty or stallTimeMs at 0 disables it.
    uint16_t stallDuty = 0;
    uint16_t stallTimeMs = 0;
    float stallMinSpeedMmS = 0.0f;
    float stallMinYawDps = 0.0f;
};

struct MotionControl {
    float distIntegral = 0.0f;
    float turnIntegral = 0.0f;
    uint16_t stallMs = 0;
    bool stalled = false; // latched until motionControlReset()
};

struct MotionOutput {
    int16_t left = 0;
    int16_t right = 0;
};

// Most duty one integral term may contribute (anti-windup).
constexpr float MOTION_INTEGRAL_MAX_DUTY = 200.0f;

inline void motionControlReset(MotionControl &mc)
{
    mc = MotionControl{};
}

// Forget both integrals, keep the stall latch. Call when a move starts, so a turn's wound-up
// integral does not carry into the next straight.
inline void motionControlClearIntegrals(MotionControl &mc)
{
    mc.distIntegral = 0.0f;
    mc.turnIntegral = 0.0f;
}

namespace motion_detail
{

inline float absf(float x)
{
    return x < 0.0f ? -x : x;
}

inline float clampSymmetric(float x, float limit)
{
    return x > limit ? limit : (x < -limit ? -limit : x);
}

// Integrate `err`, keeping gain * integral inside +-MOTION_INTEGRAL_MAX_DUTY.
inline void accumulate(float &integral, float err, float dtS, float gain)
{
    if (gain <= 0.0f) {
        integral = 0.0f;
        return;
    }
    integral = clampSymmetric(integral + err * dtS, MOTION_INTEGRAL_MAX_DUTY / gain);
}

// Duty command -> signed timer counts: non-zero commands are rescaled onto [minPwm, top];
// |cmd| < 0.5 is 0.
inline int16_t toDuty(float cmd, uint16_t minPwm, uint16_t top)
{
    if (top == 0 || absf(cmd) < 0.5f) {
        return 0;
    }
    const float topF = static_cast<float>(top);
    const float minF = static_cast<float>(minPwm < top ? minPwm : top);
    float mag = minF + absf(cmd) * (topF - minF) / topF;
    mag = mag > topF ? topF : mag;
    return static_cast<int16_t>(cmd < 0.0f ? -mag : mag);
}

} // namespace motion_detail

inline MotionOutput motionControlStep(MotionControl &mc, const MotionControlParams &p,
                                      const MotionTarget &target, const MotionFeedback &fb,
                                      uint16_t dtMs)
{
    using namespace motion_detail;
    const float dtS = static_cast<float>(dtMs) / 1000.0f;

    const float distErr = target.distRefMm - fb.distMm;
    accumulate(mc.distIntegral, distErr, dtS, p.kiDist);
    const float linear = p.ffPwmPerMmS * target.speedRefMmS + p.kpDist * distErr +
                         p.kiDist * mc.distIntegral + p.kdDist * (target.speedRefMmS - fb.speedMmS);

    const float headErr = headingWrapDeg(target.headingRefDeg - fb.headingDeg);
    const float kiTurnNow = (target.pivot || p.kiHold < 0.0f) ? p.kiTurn : p.kiHold;
    accumulate(mc.turnIntegral, headErr, dtS, kiTurnNow);
    const float angular = p.ffPwmPerDps * target.yawRateRefDps + p.kpTurn * headErr +
                          kiTurnNow * mc.turnIntegral +
                          p.kdTurn * (target.yawRateRefDps - fb.yawRateDps);

    MotionOutput out;
    out.left = toDuty(linear - angular, p.minPwmL, p.pwmTop);
    out.right = toDuty((linear + angular) * p.ffScaleR, p.minPwmR, p.pwmTop);

    // Stall: pushing hard, not moving.
    if (p.stallDuty > 0 && p.stallTimeMs > 0) {
        const int16_t l = out.left < 0 ? static_cast<int16_t>(-out.left) : out.left;
        const int16_t r = out.right < 0 ? static_cast<int16_t>(-out.right) : out.right;
        const int16_t peak = l > r ? l : r;
        const bool pushing = peak >= static_cast<int16_t>(p.stallDuty);
        const bool still =
            absf(fb.speedMmS) < p.stallMinSpeedMmS && absf(fb.yawRateDps) < p.stallMinYawDps;
        if (pushing && still) {
            mc.stallMs = static_cast<uint16_t>(mc.stallMs + dtMs);
            if (mc.stallMs >= p.stallTimeMs) {
                mc.stalled = true;
            }
        } else {
            mc.stallMs = 0;
        }
    }
    if (mc.stalled) {
        return MotionOutput{};
    }
    return out;
}
