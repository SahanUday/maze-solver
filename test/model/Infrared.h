// ============================================================================
//  model/Infrared.h  --  what the IR array reads over each kind of floor, in ADC counts, as the
//  host world draws it. Only the simulator reads these. Tags: include/spec/README.md.
// ============================================================================

#pragma once

#include <stdint.h>

#include "spec/fixed/Hardware.h"

namespace model
{

// Counts on channels D1 to D8: `low` on D1, rising evenly to `high` on D8, and the 1 sigma of one
// reading.
struct Level {
    float low;
    float high;
    float sd;

    float on(uint8_t channel) const
    {
        return low + (high - low) * channel / (IR_CHANNEL_COUNT - 1);
    }
};

// ---- Floor levels --------------------------------------------- [MEASURED] --
// The arena's lighting, 2026-10-15: PR #40 and the comment in spec/tuning/FloorPattern.h.
// Channel means, and the spread of a reading where it was measured: the maze floor (sd 3.4, so it
// reads 1009-1020) and air (1021-1023 on every channel).
constexpr Level MAZE_FLOOR{1009.0f, 1016.0f, 3.4f};
constexpr Level AIR{1021.0f, 1023.0f, 0.7f}; // nothing under the array

// ---- Other floors --------------------------------------------- [MEASURED] --
// The same runs; the spread of a reading is not known for them, so it is the maze floor's.
constexpr Level TAPE{1002.0f, 1009.0f, 3.4f}; // black tape on the board
constexpr Level WHITE{778.0f, 851.0f, 3.4f};  // the calibration's white
constexpr Level BOARD{830.0f, 880.0f, 3.4f};  // the white board

// ---- Gap and finish --------------------------------------- <<TBD HARDWARE>> --
// The gap before the ramp read 915-1010 on the first real run (PR #40) and 1016-1023 on PR #28's
// scans; one of them is wrong (#54). The finish tile is a guess: it reads like the marker's white.
constexpr Level GAP_DARK{915.0f, 1010.0f, 3.4f};
constexpr Level FINISH = WHITE;

} // namespace model
