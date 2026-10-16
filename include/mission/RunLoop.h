// ============================================================================
//  RunLoop.h  --  one control tick of a Section A run: sensors in, drive command out. No
//  hardware, no Arduino.h - testable under env:native.
//
//  What main.cpp does around the mission, kept here so it can be tested: the run is started (by
//  itself a moment after power-up, or by a debounced button press where one is fitted),
//  RobotState goes through perceive(), the mission steps, the run limit is enforced, and the
//  mission's heading correction is handed back for the caller to apply.
//  docs/architecture/modules/run_loop.md
// ============================================================================

#pragma once

#include <stdint.h>

#include "motion/DriveCommand.h"
#include "sense/Perception.h"
#include "RobotSpec.h"
#include "core/RobotState.h"
#include "mission/SectionA.h"

struct RunLoop {
    Perceiver perceiver;
    PerceiveParams perceive;
    Perception perception;
    SectionA mission;
    SectionAParams params;
    bool waitForButton = RUN_START_BUTTON_FITTED;
    uint8_t pressedTicks = 0;
    bool armed = false; // the first tick has been seen
    uint32_t armedAtMs = 0;
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
    rl.waitForButton = RUN_START_BUTTON_FITTED;
    rl.pressedTicks = 0;
    rl.armed = false;
    rl.armedAtMs = 0;
    rl.started = false;
    rl.startedAtMs = 0;
    if (!imuPresent) {
        sectionAFail(rl.mission, SectionAFault::NoImu);
    }
}

// One tick. `buttonDown` is the raw start button (ignored without one: the run begins
// RUN_START_DELAY_MS after the first tick); `headingFixDeg` is what to subtract from the gyro
// heading (headingSet(hd, hd.deg - fix)).
inline DriveCommand runLoopTick(RunLoop &rl, const RobotState &state, uint32_t nowMs,
                                bool buttonDown, float &headingFixDeg)
{
    headingFixDeg = 0.0f;
    if (!rl.started && !rl.waitForButton) {
        if (!rl.armed) {
            rl.armed = true;
            rl.armedAtMs = nowMs;
        }
        if (nowMs - rl.armedAtMs >= RUN_START_DELAY_MS) {
            rl.started = true;
            rl.startedAtMs = nowMs;
        }
    } else if (!rl.started) {
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
