// ============================================================================
//  mission/RunStart.h  --  when the run begins: by itself a moment after power-up, or by a
//  debounced button press where one is fitted. No hardware, no Arduino.h - testable under
//  env:native. docs/architecture/modules/run_loop.md
// ============================================================================

#pragma once

#include <stdint.h>

#include "spec/Mission.h"

struct RunStart {
    bool waitForButton = RUN_START_BUTTON_FITTED;
    uint8_t pressedTicks = 0;
    bool armed = false; // the first tick has been seen
    uint32_t armedAtMs = 0;
    bool started = false;
    uint32_t startedAtMs = 0;
};

// One tick. `buttonDown` is the raw start button; without one the run begins RUN_START_DELAY_MS
// after the first tick.
inline void runStartTick(RunStart &rs, uint32_t nowMs, bool buttonDown)
{
    if (rs.started) {
        return;
    }
    if (!rs.waitForButton) {
        if (!rs.armed) {
            rs.armed = true;
            rs.armedAtMs = nowMs;
        }
        if (nowMs - rs.armedAtMs >= RUN_START_DELAY_MS) {
            rs.started = true;
            rs.startedAtMs = nowMs;
        }
        return;
    }
    if (buttonDown) {
        if (rs.pressedTicks < RUN_START_DEBOUNCE_TICKS) {
            ++rs.pressedTicks;
        }
        if (rs.pressedTicks >= RUN_START_DEBOUNCE_TICKS) {
            rs.started = true; // a run is begun by one press and not by its release
            rs.startedAtMs = nowMs;
        }
    } else {
        rs.pressedTicks = 0;
    }
}
