// ============================================================================
//  Perception.h  --  RobotState in, one derived Perception out per tick. No hardware, no
//  Arduino.h - testable under env:native.
//
//  RobotState is raw (counts, deg, mm); this derives distance, walls, the line and the incline,
//  and passes the yaw rate through. It never writes RobotState.
//  docs/architecture/modules/perception.md
// ============================================================================

#pragma once

#include <stdint.h>

#include "FloorPattern.h"
#include "LineSense.h"
#include "LineView.h"
#include "MotionTypes.h"
#include "Odometry.h"
#include "RobotSpec.h"
#include "RobotState.h"
#include "Tilt.h"
#include "WallSense.h"

struct Perception {
    MotionFeedback motion;
    int16_t inclineCounts = 0; // nose-up slope in accel counts; see Tilt.h
    WallView walls;
    // How long neither side sensor has given a usable reading (no echo, or frozen). An open side
    // reads far and is valid; this is a sensor failure, for the mission to act on.
    uint16_t sideBlindMs = 0;
    LineView line;
    FloorClass floor = FloorClass::Black; // the line view, and "nothing under the array"
    bool startPressed = false;
};

// The defaults are the values in RobotSpec.h.
struct PerceiveParams {
    float mmPerCount = MM_PER_ENCODER_COUNT;
    uint16_t dtMs = CONTROL_LOOP_PERIOD_MS;
    uint16_t frozenMs = WALL_FROZEN_MS; // an unchanging side reading this long is a frozen sensor
    float frontFrozenMm = FRONT_FROZEN_MM; // an unchanging front reading over this travel is frozen
    WallParams wall;
    float irPitchMm = IR_SENSOR_PITCH_MM;
    bool irD1OnRight = IR_D1_ON_RIGHT;
    uint16_t irAirCounts = IR_AIR_COUNTS; // see classifyFloor(); 0 = no air detection
    uint8_t irAirChannels = IR_AIR_CHANNELS;
};

// The last three distinct echoes of one side sensor. A reading that is not the median of its
// neighbours is a stray echo; the median costs one echo (60 ms) before a real change comes through.
struct EchoMedian {
    uint16_t v[3] = {0, 0, 0}; // newest first
    uint8_t n = 0;
    uint16_t last = 0; // the raw reading that was last pushed
};

struct Perceiver {
    Odometry odo;
    int16_t levelCounts = IMU_ACCEL_FORWARD_LEVEL_COUNTS; // forward-axis reading on level ground
    uint16_t sideLastMm[2] = {0, 0}; // left, right: last reading and how long it has not changed
    uint16_t sideSameMs[2] = {0, 0};
    uint16_t sideBlindMs = 0;
    uint16_t frontLastMm = 0; // last front reading, and the odometer where it last changed
    float frontSinceMm = 0.0f;
    EchoMedian median[2]; // left, right
    LineCalibration cal{};
    uint8_t usable = 0; // channels the calibration accepted
    uint8_t mask = 0;   // black channels, with hysteresis
    uint8_t norm[IR_CHANNEL_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0};
};

inline void perceiverReset(Perceiver &pc)
{
    pc = Perceiver{};
}

// Install the level reading captured at boot (InclineLevel::counts).
inline void perceiverSetLevel(Perceiver &pc, int16_t levelCounts)
{
    pc.levelCounts = levelCounts;
}

// Install a calibration and work out which channels it accepts.
inline void perceiverSetCalibration(Perceiver &pc, const LineCalibration &cal)
{
    pc.cal = cal;
    pc.usable = lineCalibrationUsable(cal);
    pc.mask = 0;
}

// True once a valid reading has not changed at all for `frozenMs`: a real reading scatters by a few
// mm, so a sensor repeating one value is stuck. Counts through the frozen stretch.
inline bool sideFrozen(uint16_t &lastMm, uint16_t &sameMs, bool valid, uint16_t mm, uint16_t dtMs,
                       uint16_t frozenMs)
{
    if (!valid || mm != lastMm) {
        lastMm = mm;
        sameMs = 0;
        return false;
    }
    sameMs = sameMs > UINT16_MAX - dtMs ? UINT16_MAX : static_cast<uint16_t>(sameMs + dtMs);
    return sameMs >= frozenMs;
}

// Feed one tick's reading; returns the median of the last three echoes (the reading itself until
// there are three). A repeated reading is not a new echo.
inline uint16_t echoMedian(EchoMedian &e, bool valid, uint16_t mm)
{
    if (valid && (e.n == 0 || mm != e.last)) {
        e.v[2] = e.v[1];
        e.v[1] = e.v[0];
        e.v[0] = mm;
        e.last = mm;
        if (e.n < 3) {
            ++e.n;
        }
    }
    if (e.n < 3) {
        return mm;
    }
    const uint16_t a = e.v[0];
    const uint16_t b = e.v[1];
    const uint16_t c = e.v[2];
    const uint16_t lo = a < b ? a : b;
    const uint16_t hi = a < b ? b : a;
    return c < lo ? lo : (c > hi ? hi : c);
}

inline void perceive(Perceiver &pc, const RobotState &state, const PerceiveParams &p,
                     Perception &out)
{
    odometryUpdate(pc.odo, state.encoderCountL, state.encoderCountR, p.mmPerCount, p.dtMs);

    out.motion.distMm = pc.odo.distMm;
    out.motion.speedMmS = pc.odo.speedMmS;
    out.motion.headingDeg = state.headingDeg;
    out.motion.yawRateDps = state.yawRateDps;
    out.inclineCounts = inclineCounts(state.imu, pc.levelCounts);
    out.walls = wallView(state, p.wall);
    if (sideFrozen(pc.sideLastMm[0], pc.sideSameMs[0], out.walls.leftValid, out.walls.leftMm,
                   p.dtMs, p.frozenMs)) {
        out.walls.leftValid = false;
        out.walls.left = false;
    }
    if (sideFrozen(pc.sideLastMm[1], pc.sideSameMs[1], out.walls.rightValid, out.walls.rightMm,
                   p.dtMs, p.frozenMs)) {
        out.walls.rightValid = false;
        out.walls.right = false;
    }
    // Front: frozen only over travel; at rest a steady wall reads steady.
    if (out.walls.frontValid) {
        if (out.walls.frontMm != pc.frontLastMm) {
            pc.frontLastMm = out.walls.frontMm;
            pc.frontSinceMm = pc.odo.distMm;
        }
        const float moved = pc.odo.distMm - pc.frontSinceMm;
        if ((moved < 0.0f ? -moved : moved) >= p.frontFrozenMm) {
            out.walls.frontValid = false;
            out.walls.front = false;
        }
    }
    // Stray echoes on the sides: the median of the last three distinct ones, on the sensors still
    // believed. The front is left as it comes: its stray echoes are the step cell's to refuse
    // (two echoes must agree), and a median there would lag a ramp by an echo and hide the strays
    // from that check.
    {
        WallView &w = out.walls;
        w.leftMm = echoMedian(pc.median[0], w.leftValid, w.leftMm);
        w.rightMm = echoMedian(pc.median[1], w.rightValid, w.rightMm);
        w.left = w.leftValid && w.leftMm < p.wall.sideMm;
        w.right = w.rightValid && w.rightMm < p.wall.sideMm;
    }
    const bool blind = !out.walls.leftValid && !out.walls.rightValid;
    pc.sideBlindMs = !blind ? 0
                            : (pc.sideBlindMs > UINT16_MAX - p.dtMs
                                   ? UINT16_MAX
                                   : static_cast<uint16_t>(pc.sideBlindMs + p.dtMs));
    out.sideBlindMs = pc.sideBlindMs;

    lineNormalize(pc.cal, state.irRaw, pc.norm);
    pc.mask = lineMaskUpdate(pc.norm, pc.mask);
    out.line = lineView(pc.norm, pc.mask, pc.usable, p.irPitchMm, p.irD1OnRight);
    out.floor = classifyFloor(out.line, state.irRaw, p.irAirCounts, p.irAirChannels);

    out.startPressed = state.startButtonPressed;
}
