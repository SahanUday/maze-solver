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
    LineView line;
    bool startPressed = false;
};

// The defaults are the values in RobotSpec.h.
struct PerceiveParams {
    float mmPerCount = MM_PER_ENCODER_COUNT;
    uint16_t dtMs = CONTROL_LOOP_PERIOD_MS;
    WallParams wall;
    float irPitchMm = IR_SENSOR_PITCH_MM;
};

struct Perceiver {
    Odometry odo;
    int16_t levelCounts = IMU_ACCEL_FORWARD_LEVEL_COUNTS; // forward-axis reading on level ground
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

    lineNormalize(pc.cal, state.irRaw, pc.norm);
    pc.mask = lineMaskUpdate(pc.norm, pc.mask);
    out.line = lineView(pc.norm, pc.mask, pc.usable, p.irPitchMm);

    out.startPressed = state.startButtonPressed;
}
