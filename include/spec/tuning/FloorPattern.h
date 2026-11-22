// ============================================================================
//  spec/tuning/FloorPattern.h  --  what the IR array must see to call the approach marker, the gap
//  and the air (sense/FloorPattern.h).
//  Layout, tags and the TBD legend: docs/architecture/ARCHITECTURE.md ("Spec layout").
//  The sizes the windows surround: measurements/Arena.h.
// ============================================================================

#pragma once

#include <stdint.h>
#include "spec/measurements/Arena.h"

// ---- Approach marker A: detection windows --------------------------------- [TUNED] --
// Crossing the marker along its middle the array sees white, black, white (sizes in Arena.h).
constexpr float MARKER_A_FRAME_MIN_MM = 25.0f; // each white frame
constexpr float MARKER_A_FRAME_MAX_MM = 55.0f;
// The far frame counts once this much of it is under the array: with the array 86 mm ahead of the
// axle, a robot at the tile's centre has only ~27 mm of it, and one 10 mm short has 17.
constexpr float MARKER_A_FAR_FRAME_MIN_MM = 12.0f;
constexpr float MARKER_A_CENTRE_MIN_MM = 95.0f; // the black box between them
constexpr float MARKER_A_CENTRE_MAX_MM = 140.0f;
static_assert(MARKER_A_FRAME_MM >= MARKER_A_FRAME_MIN_MM &&
                  MARKER_A_FRAME_MM <= MARKER_A_FRAME_MAX_MM &&
                  MARKER_A_BOX_MM >= MARKER_A_CENTRE_MIN_MM &&
                  MARKER_A_BOX_MM <= MARKER_A_CENTRE_MAX_MM,
              "marker A's measured sizes must lie inside its detection windows");

// ---- Approach marker B: detection windows ------------------ <<TBD CALIBRATION>> --
// A's windows until marker B is measured (#54).
constexpr float MARKER_B_FRAME_MIN_MM = 25.0f;
constexpr float MARKER_B_FRAME_MAX_MM = 55.0f;
constexpr float MARKER_B_FAR_FRAME_MIN_MM = 12.0f;
constexpr float MARKER_B_CENTRE_MIN_MM = 95.0f;
constexpr float MARKER_B_CENTRE_MAX_MM = 140.0f;
static_assert(MARKER_B_FRAME_MM >= MARKER_B_FRAME_MIN_MM &&
                  MARKER_B_FRAME_MM <= MARKER_B_FRAME_MAX_MM &&
                  MARKER_B_BOX_MM >= MARKER_B_CENTRE_MIN_MM &&
                  MARKER_B_BOX_MM <= MARKER_B_CENTRE_MAX_MM,
              "marker B's measured sizes must lie inside its detection windows");

// ---- Gap and air ------------------------------------------------------- [TUNED] --
constexpr float GAP_MIN_MM = 12.0f;     // air for this long = the gap
constexpr float FLOOR_GLITCH_MM = 4.0f; // a change shorter than this is noise
// Air: at least IR_AIR_CHANNELS channels at IR_AIR_COUNTS or more. On the robot (2026-10-15, bench,
// the arena's lighting): the array lifted clear of everything reads 1021-1023 on every channel; the
// black maze floor reads 1009-1020 (channel means 1009-1016, sd 3.4), so a threshold of 1010 on 7
// channels called the bare floor air. 1018 on 7 channels needs the whole array to be saturated: the
// floor gets there on one or two channels at a time, not seven. The gap before the ramp read
// 1016-1023 in PR #28's scans; recheck it against this threshold on the real course. 0 turns the
// detection off.
constexpr uint16_t IR_AIR_COUNTS = 1018;
constexpr uint8_t IR_AIR_CHANNELS = 7;
