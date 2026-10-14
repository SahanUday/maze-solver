// ============================================================================
//  MotionControl.h  --  turns a MotionTarget into left/right duty. No hardware,
//  no Arduino.h - testable under env:native.
//
//  Two channels, mixed:
//    linear   position PID on the odometer + speed feedforward
//    angular  heading PID on the GYRO       + rate feedforward
//  left = linear - angular, right = linear + angular (CCW positive, so a CCW turn
//  speeds the right wheel). The gyro, not the encoders, closes the angular loop,
//  so turns do not depend on the measured track width.
//  docs/architecture/modules/motion_control.md
// ============================================================================

#pragma once

#include <stdint.h>

#include "Heading.h"
#include "MotionTypes.h"

struct MotionControlParams {
    float kpDist = 0.0f;
    float kiDist = 0.0f;
    float kdDist = 0.0f; // acts on the speed error
    float kpTurn = 0.0f;
    float kiTurn =
        0.0f; // heading integral while turning in place: it has to break the wheels loose
    float kdTurn = 0.0f; // acts on the yaw-rate error
    // Heading integral while driving (not pivoting). Negative = same as kiTurn. A turn-sized
    // integral on a straight rings: it is what the zig-zag was.
    float kiHold = -1.0f;
    // The right wheel needs more duty than the left for the same speed on the floor (~20%): its
    // whole command is multiplied by this, so the integral has nothing constant left to supply.
    float ffScaleR = 1.0f;
    float ffPwmPerMmS = 0.0f;
    float ffPwmPerDps = 0.0f;
    uint16_t minPwmL = 0; // duty that just starts the wheel; 0 = none
    uint16_t minPwmR = 0;
    uint16_t pwmTop = 0;
    // Stall: duty at or above stallDuty with the robot neither translating nor rotating for
    // stallTimeMs. Either of those two at 0 turns it off.
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

// An integral term may contribute at most this much duty, so a long stall cannot wind it up.
constexpr float MOTION_INTEGRAL_MAX_DUTY = 200.0f;

inline void motionControlReset(MotionControl &mc)
{
    mc = MotionControl{};
}

// A new move starts from a clean slate: the heading integral wound up at the end of a turn (to
// break the wheels loose) must not carry into the straight that follows, where it shows up as a
// zig-zag. The stall latch is left alone.
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

// Duty command -> signed counts: a non-zero command is lifted past the wheel's start-up
// threshold, and the result is clamped to the timer's TOP.
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

    // Stall: the motors are being pushed hard and nothing moves.
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
