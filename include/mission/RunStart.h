// ============================================================================
//  mission/RunStart.h  --  the run's start: latched when told `go`. When to tell is the operator
//  panel's (panel/PanelStart.h). No hardware, no Arduino.h - testable under env:native.
//  docs/architecture/modules/run_loop.md
// ============================================================================

#pragma once

#include <stdint.h>

struct RunStart {
    bool started = false;
    uint32_t startedAtMs = 0;
};

// One tick: the run begins the first time `go` is true.
inline void runStartTick(RunStart &rs, uint32_t nowMs, bool go)
{
    if (!rs.started && go) {
        rs.started = true;
        rs.startedAtMs = nowMs;
    }
}
