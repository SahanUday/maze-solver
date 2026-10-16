// ============================================================================
//  WallSense.h  --  ultrasonic readings to "is there a wall" and a centring trim. No hardware,
//  no Arduino.h - testable under env:native.
//
//  Readings carry their US_OFFSET_*, so (left - right) / 2 is the offset from the corridor centre.
//  Each sensor refreshes every US_SENSOR_COUNT * US_SLOT_MS, so a reading can be that old.
//  docs/architecture/modules/wall_sense.md
// ============================================================================

#pragma once

#include <stdint.h>

#include "motion/Heading.h"
#include "spec/WallFollow.h"
#include "core/RobotState.h"

// The defaults are the values in spec/WallFollow.h.
struct WallParams {
    uint16_t sideMm = WALL_PRESENT_SIDE_MM;      // side reading below this = wall
    uint16_t frontMm = WALL_PRESENT_FRONT_MM;    // front reading below this = wall
    uint16_t centerDistMm = WALL_CENTER_DIST_MM; // side reading when centred; one wall only
    uint16_t pairTolMm = WALL_PAIR_TOL_MM;       // both walls: |l + r - 2 * centre| above = wrong
    float kp = KP_WALL;                          // deg of heading offset per mm of lateral error
    float ki = KI_WALL;                          // ... per mm of lateral error per second
    float biasMaxDeg = WALL_BIAS_MAX_DEG;
    float biasResetDeg = WALL_BIAS_RESET_DEG;
    uint16_t biasHoldMs = WALL_BIAS_HOLD_MS;
    float biasFadeDegPerS = WALL_BIAS_FADE_DEG_S;
    float steerMaxDeg = STEER_MAX_DEG;
};

// One tick's view of the three sensors.
struct WallView {
    // Echo heard (usValid) for each sensor.
    bool frontValid = false;
    bool leftValid = false;
    bool rightValid = false;
    uint16_t frontMm = 0;
    uint16_t leftMm = 0;
    uint16_t rightMm = 0;
    // Valid and under the threshold. A sensor with no echo is neither wall nor open.
    bool front = false;
    bool left = false;
    bool right = false;
};

inline WallView wallView(const RobotState &state, const WallParams &p)
{
    WallView v;
    v.frontValid = (state.usValid & 0x01) != 0;
    v.leftValid = (state.usValid & 0x02) != 0;
    v.rightValid = (state.usValid & 0x04) != 0;
    v.frontMm = state.distFrontMm;
    v.leftMm = state.distLeftMm;
    v.rightMm = state.distRightMm;
    v.front = v.frontValid && v.frontMm < p.frontMm;
    v.left = v.leftValid && v.leftMm < p.sideMm;
    v.right = v.rightValid && v.rightMm < p.sideMm;
    return v;
}

// Lateral error in mm, positive when the robot sits right of centre (steer CCW to fix it).
// False when neither side wall is in view, or when both are but disagree with the corridor width:
// one of them is wrong and there is no telling which.
inline bool wallCenterError(const WallView &v, const WallParams &p, float &errMm)
{
    const float center = static_cast<float>(p.centerDistMm);
    if (v.left && v.right) {
        const float sum = static_cast<float>(v.leftMm) + static_cast<float>(v.rightMm);
        const float off = sum - 2.0f * center;
        if ((off < 0.0f ? -off : off) > static_cast<float>(p.pairTolMm)) {
            return false;
        }
        errMm = (static_cast<float>(v.leftMm) - static_cast<float>(v.rightMm)) * 0.5f;
        return true;
    }
    if (v.left) {
        errMm = static_cast<float>(v.leftMm) - center;
        return true;
    }
    if (v.right) {
        errMm = center - static_cast<float>(v.rightMm);
        return true;
    }
    return false;
}

// Which walls the last error came from; the filter restarts when it changes.
enum class WallMode : uint8_t { None, Both, LeftOnly, RightOnly };

struct WallSteer {
    float filtMm = 0.0f;    // filtered lateral error
    float biasDeg = 0.0f;   // what the integral has learned: the heading reference's error
    float anchorDeg = 0.0f; // the heading the bias was learned at
    uint16_t unseenMs = 0;  // how long no wall has confirmed the bias
    bool anchored = false;
    WallMode mode = WallMode::None;
};

// Weight of the newest error in the filter. There is no derivative term: a sensor refreshes every
// 60 ms but is read every tick, so a derivative only spikes at each refresh.
constexpr float WALL_ERR_FILTER = 0.5f;

inline void wallSteerReset(WallSteer &w)
{
    w = WallSteer{};
}

// The caller moved the heading frame by `deltaDeg` (headingSet): keep the bias anchored to it.
inline void wallSteerShiftFrame(WallSteer &w, float deltaDeg)
{
    w.anchorDeg = headingWrapDeg(w.anchorDeg + deltaDeg);
}

// Heading offset (deg, CCW positive) toward the centre line. A proportional term alone leaves an
// offset of (heading error / kp), so the integral learns the error of the heading reference and
// keeps applying it through gaps in the walls. The learned bias belongs to the heading it was
// learned at: a turn drops it by itself. With no wall to confirm it, it is held for a gap in the
// walls, then faded out: a bias learned from a bad reading must not be applied for ever.
// `headingDeg` is the gyro heading.
inline float wallSteerTrim(WallSteer &w, const WallView &v, const WallParams &p, float headingDeg,
                           uint16_t dtMs)
{
    const float moved = headingWrapDeg(headingDeg - w.anchorDeg);
    if (!w.anchored || (moved < 0.0f ? -moved : moved) > p.biasResetDeg) {
        w.biasDeg = 0.0f;
        w.anchorDeg = headingDeg;
        w.anchored = true;
        w.mode = WallMode::None;
    }

    float err = 0.0f;
    float trim = w.biasDeg;
    if (!wallCenterError(v, p, err)) {
        w.mode = WallMode::None;
        w.unseenMs =
            w.unseenMs > UINT16_MAX - dtMs ? UINT16_MAX : static_cast<uint16_t>(w.unseenMs + dtMs);
        if (w.unseenMs > p.biasHoldMs) {
            const float step = p.biasFadeDegPerS * (static_cast<float>(dtMs) / 1000.0f);
            w.biasDeg =
                w.biasDeg > step ? w.biasDeg - step : (w.biasDeg < -step ? w.biasDeg + step : 0.0f);
        }
    } else {
        w.unseenMs = 0;
        const WallMode mode = (v.left && v.right)
                                  ? WallMode::Both
                                  : (v.left ? WallMode::LeftOnly : WallMode::RightOnly);
        if (mode != w.mode) {
            w.mode = mode;
            w.filtMm = err;
        }
        w.filtMm += WALL_ERR_FILTER * (err - w.filtMm);
        trim += p.kp * w.filtMm;

        // Learn only while the output has room, so a saturated trim cannot wind the bias up.
        const bool pinned =
            (trim > p.steerMaxDeg && w.filtMm > 0.0f) || (trim < -p.steerMaxDeg && w.filtMm < 0.0f);
        if (!pinned) {
            w.biasDeg += p.ki * w.filtMm * (static_cast<float>(dtMs) / 1000.0f);
            w.biasDeg = w.biasDeg > p.biasMaxDeg
                            ? p.biasMaxDeg
                            : (w.biasDeg < -p.biasMaxDeg ? -p.biasMaxDeg : w.biasDeg);
        }
    }
    return trim > p.steerMaxDeg ? p.steerMaxDeg : (trim < -p.steerMaxDeg ? -p.steerMaxDeg : trim);
}
