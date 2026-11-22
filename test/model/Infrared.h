// ============================================================================
//  model/Infrared.h  --  what the IR array reads over each kind of floor, in ADC counts, as the
//  host world draws it. Only the simulator reads these. Tags: include/spec/README.md.
// ============================================================================

#pragma once

#include <stdint.h>

#include "spec/fixed/Hardware.h"

namespace model
{

// Counts on channels D1 to D8: `low` on D1, rising evenly to `high` on D8.
struct Level {
    float low;
    float high;

    uint16_t on(uint8_t channel) const
    {
        return static_cast<uint16_t>(low + (high - low) * channel / (IR_CHANNEL_COUNT - 1));
    }
};

// ---- Floor levels ----------------------------------------- <<TBD HARDWARE>> --
// Typed-in stand-ins. Log each floor at line speed on the robot (#54).
constexpr Level WHITE{500.0f, 542.0f};
constexpr Level BLACK{990.0f, 990.0f};
constexpr Level AIR{1020.0f, 1020.0f}; // nothing under the array

} // namespace model
