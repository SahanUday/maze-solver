// ============================================================================
//  RunLoop.h  --  one control tick of a run: sensors in, drive command out. No hardware, no
//  Arduino.h - testable under env:native.
//
//  What main.cpp does around the mission, kept here so it can be tested: RobotState goes through
//  perceive(), the mission manager takes the tick (start, run limit, parts), and the mission's
//  heading correction is handed back for the caller to apply.
//  docs/architecture/modules/run_loop.md
// ============================================================================

#pragma once

#include <stdint.h>

#include "core/RobotState.h"
#include "mission/MissionManager.h"
#include "motion/DriveCommand.h"
#include "sense/Perception.h"

struct RunLoop {
    Perceiver perceiver;
    PerceiveParams perceive;
    Perception perception;
    MissionManager mission;
    MissionParams params;
};

// `lineCal` is the IR array's calibration, or null if there is none. Without a usable channel the
// floor cannot be read, so the run explores every reachable tile and stops (wallsOnly). `parts` is
// which parts of the course to run: all of them by default.
inline void runLoopInit(RunLoop &rl, bool imuPresent, const LineCalibration *lineCal,
                        const PartRange &parts = PartRange{})
{
    perceiverReset(rl.perceiver);
    if (lineCal != nullptr) {
        perceiverSetCalibration(rl.perceiver, *lineCal);
    }
    rl.perceive = PerceiveParams{};
    missionReset(rl.mission);
    rl.params = missionParams();
    rl.params.parts = parts;
    rl.params.wallsOnly = rl.perceiver.usable == 0;
    if (!imuPresent) {
        missionFail(rl.mission, MissionFault::NoImu);
    }
}

// One tick. `buttonDown` is the raw start button (ignored without one); `headingFixDeg` is what to
// subtract from the gyro heading (headingSet(hd, hd.deg - fix)).
inline DriveCommand runLoopTick(RunLoop &rl, const RobotState &state, uint32_t nowMs,
                                bool buttonDown, float &headingFixDeg)
{
    perceive(rl.perceiver, state, rl.perceive, rl.perception);
    const DriveCommand cmd = missionTick(rl.mission, rl.params, rl.perception, nowMs, buttonDown);
    headingFixDeg = missionTakeHeadingFix(rl.mission);
    return cmd;
}
