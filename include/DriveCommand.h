// ============================================================================
//  DriveCommand.h  --  what the algorithm layer asks of the drivetrain.
//
//  The output counterpart of RobotState: runAlgorithm() returns one per tick and
//  only main.cpp turns it into motorSet() / motorsEnable() calls, so everything
//  above the HAL stays testable on the host. Duties are signed timer counts, the
//  unit motorSet() takes.
// ============================================================================

#pragma once

#include <stdint.h>

struct DriveCommand {
    int16_t left = 0;
    int16_t right = 0;
    bool enable = false; // false = both drivers coast
};
