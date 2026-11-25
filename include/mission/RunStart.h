// ============================================================================
//  mission/RunStart.h  --  the run's start, latched when told `go` (when: panel/PanelStart.h).
//  No hardware, no Arduino.h. docs/architecture/modules/run_loop.md
// ============================================================================

#pragma once

struct RunStart {
    bool started = false;
};

// One tick: the run begins the first time `go` is true.
inline void runStartTick(RunStart &rs, bool go)
{
    rs.started = rs.started || go;
}
