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

#include "RobotSpec.h"
#include "RobotState.h"

// The defaults are the measured values in RobotSpec.h.
struct WallParams {
    uint16_t sideMm = WALL_PRESENT_SIDE_MM;      // side reading below this = wall
    uint16_t frontMm = WALL_PRESENT_FRONT_MM;    // front reading below this = wall
    uint16_t centerDistMm = WALL_CENTER_DIST_MM; // side reading when centred; one wall only
    float kp = KP_WALL;                          // deg of heading offset per mm of lateral error
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
// False when neither side wall is in view.
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

// Which walls the last error came from; the filter restarts when it changes.
enum class WallMode : uint8_t { None, Both, LeftOnly, RightOnly };

struct WallSteer {
    float filtMm = 0.0f;
    WallMode mode = WallMode::None;
};

// Weight of the newest error in the filter. There is no derivative term: a sensor refreshes every
// 60 ms but is read every tick, so a derivative only spikes at each refresh.
constexpr float WALL_ERR_FILTER = 0.5f;

inline void wallSteerReset(WallSteer &w)
{
    w = WallSteer{};
}

// Heading offset (deg, CCW positive) toward the centre line; 0 with no wall.
inline float wallSteerTrim(WallSteer &w, const WallView &v, const WallParams &p)
{
    float err = 0.0f;
    if (!wallCenterError(v, p, err)) {
        w.mode = WallMode::None;
        return 0.0f;
    }
    const WallMode mode =
        (v.left && v.right) ? WallMode::Both : (v.left ? WallMode::LeftOnly : WallMode::RightOnly);
    if (mode != w.mode) {
        w.mode = mode;
        w.filtMm = err;
    }
    w.filtMm += WALL_ERR_FILTER * (err - w.filtMm);

    const float trim = p.kp * w.filtMm;
    return trim > p.steerMaxDeg ? p.steerMaxDeg : (trim < -p.steerMaxDeg ? -p.steerMaxDeg : trim);
}
