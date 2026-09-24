// ============================================================================
//  Scheduler.h  --  fixed-period tick timing. No hardware, no Arduino.h -
//  testable under env:native.
// ============================================================================

#pragma once

#include <stdint.h>

// True if `periodMs` have elapsed since the last tick target. Advances
// `nextTickMs` to the next target on a hit; resyncs to `nowMs` instead of
// advancing if more than one period behind (avoids a catch-up burst).
inline bool tickDue(uint32_t nowMs, uint32_t &nextTickMs, uint16_t periodMs)
{
    if (static_cast<int32_t>(nowMs - nextTickMs) < 0) {
        return false;
    }
    nextTickMs += periodMs;
    if (static_cast<int32_t>(nowMs - nextTickMs) > 0) {
        nextTickMs = nowMs;
    }
    return true;
}
