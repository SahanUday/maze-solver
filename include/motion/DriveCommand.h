// ============================================================================
//  DriveCommand.h  --  what the algorithm layer asks of the drivetrain, one per tick.
//  Duties are signed timer counts, the unit motorSet() takes.
// ============================================================================

#pragma once

#include <stdint.h>

struct DriveCommand {
    int16_t left = 0;
    int16_t right = 0;
    bool enable = false; // false = both drivers coast
};
