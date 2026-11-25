// ============================================================================
//  mission/Fault.h  --  why a mission stopped itself. No hardware, no Arduino.h.
// ============================================================================

#pragma once

#include <stdint.h>

// The serial log prints these as numbers, so a code keeps its number.
enum class MissionFault : uint8_t {
    None,
    NoImu,            // set by the caller: no gyro, no heading, no turns
    NoRoute,          // nothing left to explore and the rescans used
    StuckTurn,        // a turn made no progress, retried
    StuckStep,        // a step made no progress
    Stall,            // motors pushed hard and nothing moved
    TooManyStops,     // blocked steps over the limit: something keeps stopping the robot
    NoLine = 8,       // drove on from the marker and found no line past the gap
    LineLost,         // the line was gone, and searched for in vain
    BadConfig,        // the parts asked for cannot run: not built, or not able to start alone
    TargetUnreachable // no path to the target over checked edges
};
