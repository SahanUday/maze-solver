// ============================================================================
//  Perception.h  --  RobotState in, what the control layer acts on out. No
//  hardware, no Arduino.h - testable under env:native.
//
//  RobotState is raw: counts, deg, mm, ADC counts. The control layer wants
//  distance travelled, a yaw RATE, which walls exist and where the line is, so
//  this derives them once per tick and hands over one struct. RobotState itself is
//  never written here - drivers own it.
//  docs/architecture/modules/perception.md
// ============================================================================

#pragma once

#include <stdint.h>

#include "Heading.h"
#include "LineSense.h"
#include "LineView.h"
#include "MotionTypes.h"
#include "Odometry.h"
#include "RobotState.h"
#include "Tilt.h"
#include "WallSense.h"

struct Perception {
    MotionFeedback motion;
    float tiltDeg = 0.0f;
    WallView walls;
    LineView line;
    bool startPressed = false;
};

struct PerceiveParams {
    float mmPerCount = 0.0f;
    uint16_t dtMs = 10;
    WallParams wall;
    float irPitchMm = 0.0f;
};

struct Perceiver {
    Odometry odo;
    float lastHeadingDeg = 0.0f;
    bool headingPrimed = false;
    float yawRateDps = 0.0f;
    LineCalibration cal{};
    uint8_t usable = 0; // channels the calibration accepted
    uint8_t mask = 0;   // black channels, with hysteresis
    uint8_t norm[IR_CHANNEL_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0};
};

// Weight of the newest yaw-rate sample; the gyro is integrated into a heading at 100 Hz, so its
// difference is noisy at the resolution of one tick.
constexpr float YAW_RATE_FILTER = 0.5f;

inline void perceiverReset(Perceiver &pc)
{
    pc = Perceiver{};
}

// Install a calibration (from the store, or just taken) and work out which channels it accepts.
inline void perceiverSetCalibration(Perceiver &pc, const LineCalibration &cal)
{
    pc.cal = cal;
    pc.usable = 0;
    for (uint8_t ch = 0; ch < IR_CHANNEL_COUNT; ++ch) {
        if (cal.scale[ch] != 0) {
            pc.usable = static_cast<uint8_t>(pc.usable | (1u << ch));
        }
    }
    pc.mask = 0;
}

inline void perceive(Perceiver &pc, const RobotState &state, const PerceiveParams &p,
                     Perception &out)
{
    odometryUpdate(pc.odo, state.encoderCountL, state.encoderCountR, p.mmPerCount, p.dtMs);

    if (pc.headingPrimed && p.dtMs > 0) {
        const float raw = headingWrapDeg(state.headingDeg - pc.lastHeadingDeg) * 1000.0f /
                          static_cast<float>(p.dtMs);
        pc.yawRateDps += YAW_RATE_FILTER * (raw - pc.yawRateDps);
    }
    pc.lastHeadingDeg = state.headingDeg;
    pc.headingPrimed = true;

    out.motion.distMm = pc.odo.distMm;
    out.motion.speedMmS = pc.odo.speedMmS;
    out.motion.headingDeg = state.headingDeg;
    out.motion.yawRateDps = pc.yawRateDps;
    out.tiltDeg = tiltDeg(state.imu);
    out.walls = wallView(state, p.wall);

    lineNormalize(pc.cal, state.irRaw, pc.norm);
    pc.mask = lineMaskUpdate(pc.norm, pc.mask);
    out.line = lineView(pc.norm, pc.mask, pc.usable, p.irPitchMm);

    out.startPressed = state.startButtonPressed;
}
