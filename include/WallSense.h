// ============================================================================
//  WallSense.h  --  ultrasonic readings to "is there a wall" and a centring
//  trim. No hardware, no Arduino.h - testable under env:native.
//
//  Readings carry their US_OFFSET_* correction, so left and right are measured
//  from the same reference line and (left - right) / 2 is the robot's offset from
//  the corridor's centre. Each sensor only refreshes every
//  US_SENSOR_COUNT * US_SLOT_MS, so a reading can be that stale; the filter and
//  the clamps below exist because of that.
//  docs/architecture/modules/wall_sense.md
// ============================================================================

#pragma once

#include <stdint.h>

#include "Heading.h"
#include "RobotState.h"

struct WallParams {
    uint16_t sideMm = 0;       // side reading below this = wall
    uint16_t frontMm = 0;      // front reading below this = wall
    uint16_t centerDistMm = 0; // side reading when centred; used when only one wall is seen
    float kp = 0.0f;           // deg of heading offset per mm of lateral error
    float kd = 0.0f;           // ... per mm/s of lateral error rate
    float steerMaxDeg = 0.0f;
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
// False when neither side wall is in view, so there is nothing to centre on.
inline bool wallCenterError(const WallView &v, const WallParams &p, float &errMm)
{
    const float center = static_cast<float>(p.centerDistMm);
    if (v.left && v.right) {
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

// Centring state. `mode` records which walls the last error came from, so the filter restarts
// instead of spiking when a wall appears or ends.
struct WallSteer {
    float filtMm = 0.0f;
    uint8_t mode = 0; // 0 none, 1 both, 2 left only, 3 right only
};

// Weight of the newest error in the filter.
constexpr float WALL_ERR_FILTER = 0.5f;

inline void wallSteerReset(WallSteer &w)
{
    w = WallSteer{};
}

// Heading offset (deg, CCW positive) that steers back toward the centre line; 0 with no wall.
inline float wallSteerTrim(WallSteer &w, const WallView &v, const WallParams &p, uint16_t dtMs)
{
    float err = 0.0f;
    if (!wallCenterError(v, p, err)) {
        w.mode = 0;
        return 0.0f;
    }
    const uint8_t mode = (v.left && v.right) ? 1 : (v.left ? 2 : 3);
    if (mode != w.mode) {
        w.mode = mode;
        w.filtMm = err;
    }
    const float prev = w.filtMm;
    w.filtMm += WALL_ERR_FILTER * (err - w.filtMm);
    const float rate = dtMs > 0 ? (w.filtMm - prev) * 1000.0f / static_cast<float>(dtMs) : 0.0f;

    const float trim = p.kp * w.filtMm + p.kd * rate;
    return trim > p.steerMaxDeg ? p.steerMaxDeg : (trim < -p.steerMaxDeg ? -p.steerMaxDeg : trim);
}
