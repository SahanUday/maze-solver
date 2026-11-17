// ============================================================================
//  telemetry.h  --  what the robot prints over serial while it runs, so a run can be read
//  afterwards. Integers only (no float printing).
//  docs/architecture/modules/run_loop.md
// ============================================================================

#pragma once

#include "core/RobotState.h"
#include "mission/RunLoop.h"
#include "motion/DriveCommand.h"

// Once per tick, after the mission has stepped: the status line every RUN_LOG_EVERY_MS while the
// run is going, and a line whenever the mission's phase changes.
void telemetryTick(const RunLoop &run, const RobotState &state, const DriveCommand &cmd);
