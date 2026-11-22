// ============================================================================
//  spec/tuning/MarkerExit.h  --  where the mission hands over to the line follower after the marker
//  (mission/MarkerExit.h).
//  Layout, tags and the TBD legend: docs/architecture/ARCHITECTURE.md ("Spec layout").
//  The floor it drives over: BRIDGE_OUT_* in measurements/Arena.h.
// ============================================================================

#pragma once

#include <stdint.h>

// ---- Hand-over to the line ------------------------------------------- [TUNED] --
// After the marker the mission drives on and hands over to the line follower. The hand-over is the
// first line view BRIDGE_HANDOVER_MIN_MM past the end of the gap (the ~8 mm dark patch right after
// it must not count); no line within APPROACH_NO_LINE_MM of the marker's centre is `NoLine`
// (marker centre to the ramp's line is ~300 mm).
constexpr float BRIDGE_HANDOVER_MIN_MM = 4.0f;
// Without a seen gap, hand over at the first line view this far past the marker's box centre (the
// gap ends ~182 mm and the dark patch ~190 mm from the tile's centre; the line is on the board
// from there). Measured on the robot 2026-10-15: the gap read 915-1010, not air.
constexpr float BRIDGE_HANDOVER_NO_GAP_MM = 230.0f;
constexpr float APPROACH_NO_LINE_MM = 400.0f;
