// ============================================================================
//  mission/FinishSource.h  --  what tells Section B's exploring it has reached the finish. Pure
//  logic, no hardware, no Arduino.h.
//
//  The mission does not know how a finish is recognised: a caller plugs in a function that is asked
//  every tick, with its own context pointer (a plain function pointer, no std::function, so it
//  costs nothing on the AVR). The host tests plug in the world's finish tile; the real detector
//  (#69) takes its place later. Without one, Section B is explored until nothing reachable is left.
//  docs/architecture/modules/section_b.md
// ============================================================================

#pragma once

#include "sense/Perception.h"

struct FinishSource {
    // True while the robot is on the finish tile (or the finish pattern is under the array).
    bool (*told)(const void *ctx, const Perception &pc) = nullptr;
    const void *ctx = nullptr;
};

inline bool finishPlugged(const FinishSource &f)
{
    return f.told != nullptr;
}

inline bool finishTold(const FinishSource &f, const Perception &pc)
{
    return f.told != nullptr && f.told(f.ctx, pc);
}
