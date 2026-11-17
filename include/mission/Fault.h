// ============================================================================
//  mission/Fault.h  --  why a mission stopped itself. No hardware, no Arduino.h.
// ============================================================================

#pragma once

#include <stdint.h>

enum class MissionFault : uint8_t {
    None,
    NoImu,        // set by the caller: no gyro, no heading, no turns
    NoRoute,      // nothing left to explore, the rescans used, the white tile not found
    StuckTurn,    // a turn made no progress, retried
    StuckStep,    // a step made no progress
    Stall,        // motors pushed hard and nothing moved
    TooManyStops, // blocked steps over the limit: something keeps stopping the robot
    TimeUp,       // set by the caller: the run limit passed before the run ended
    NoLine,       // drove on from the marker and found no line past the gap
    LineLost,     // the line was gone, and searched for in vain
    BadConfig     // the parts asked for cannot run: not built, or not able to start alone
};
