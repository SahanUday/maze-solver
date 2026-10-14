// ============================================================================
//  RunLoop.h  --  one control tick of a Section A run: sensors in, drive command out. No
//  hardware, no Arduino.h - testable under env:native.
//
//  What main.cpp does around the mission, kept here so it can be tested: the start button is
//  debounced and latched, RobotState goes through perceive(), the mission steps, the run limit is
//  enforced, and the mission's heading correction is handed back for the caller to apply.
//  docs/architecture/modules/run_loop.md
// ============================================================================

#pragma once

#include <stdint.h>

#include "DriveCommand.h"
#include "Perception.h"
#include "RobotSpec.h"
#include "RobotState.h"
#include "SectionA.h"

struct RunLoop {
    Perceiver perceiver;
    PerceiveParams perceive;
    Perception perception;
    SectionA mission;
    SectionAParams params;
    uint8_t pressedTicks = 0;
    bool started = false;
    uint32_t startedAtMs = 0;
};

// `lineCal` is the IR array's calibration, or null if there is none. Without a usable channel the
// floor cannot be read, so the run explores every reachable tile and stops (wallsOnly).
inline void runLoopInit(RunLoop &rl, bool imuPresent, const LineCalibration *lineCal)
{
    perceiverReset(rl.perceiver);
    if (lineCal != nullptr) {
        perceiverSetCalibration(rl.perceiver, *lineCal);
    }
    rl.perceive = PerceiveParams{};
    sectionAReset(rl.mission);
    rl.params = sectionAParams();
    rl.params.wallsOnly = rl.perceiver.usable == 0;
    rl.pressedTicks = 0;
    rl.started = false;
    rl.startedAtMs = 0;
    if (!imuPresent) {
        sectionAFail(rl.mission, SectionAFault::NoImu);
    }
}

// One tick. `buttonDown` is the raw start button; `headingFixDeg` is what to subtract from the
// gyro heading (headingSet(hd, hd.deg - fix)).
inline DriveCommand runLoopTick(RunLoop &rl, const RobotState &state, uint32_t nowMs,
                                bool buttonDown, float &headingFixDeg)
{
    headingFixDeg = 0.0f;
    if (!rl.started) {
        if (buttonDown) {
            if (rl.pressedTicks < RUN_START_DEBOUNCE_TICKS) {
                ++rl.pressedTicks;
            }
            if (rl.pressedTicks >= RUN_START_DEBOUNCE_TICKS) {
                rl.started = true; // a run is begun by one press and not by its release
                rl.startedAtMs = nowMs;
            }
        } else {
            rl.pressedTicks = 0;
        }
    }

    RobotState seen = state;
    seen.startButtonPressed = rl.started;
    perceive(rl.perceiver, seen, rl.perceive, rl.perception);

    if (rl.started && rl.mission.phase != SectionAPhase::Done &&
        rl.mission.phase != SectionAPhase::Fault && nowMs - rl.startedAtMs >= RUN_LIMIT_MS) {
        sectionAFail(rl.mission, SectionAFault::TimeUp);
    }
    const DriveCommand cmd = sectionAStep(rl.mission, rl.params, rl.perception);
    headingFixDeg = sectionATakeHeadingFix(rl.mission);
    return cmd;
}
