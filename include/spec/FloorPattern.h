// ============================================================================
//  spec/FloorPattern.h  --  the approach marker and gap as the IR array sees them.
//  No pins (see config/RobotConfig.h), no Arduino.h (needed for env:native).
//
//  `<<TBD HARDWARE>>` / `<<TBD CALIBRATION>>` = not yet measured/tuned.
//  See scripts/list-tbds.sh.
// ============================================================================

#pragma once

#include <stdint.h>

// ---- Marker and gap on the floor ------------------------------- [MEASURED] --
// What the array sees along the path out of the maze (include/sense/FloorPattern.h). The approach
// marker is a white frame around a black box; crossing it along its middle the array saw (three
// scans on the robot): white 36-42 mm, black 116-119 mm, white 37-41 mm. Then ~20 mm of black strip
// and the gap before the ramp: every channel at 1016-1023 for ~21 mm (the gap is ~37 mm; its edges
// blur).
constexpr float MARKER_FRAME_MIN_MM = 25.0f; // each white frame
constexpr float MARKER_FRAME_MAX_MM = 55.0f;
// The far frame counts once this much of it is under the array: with the array 86 mm ahead of the
// axle, a robot at the tile's centre has only ~27 mm of it, and one 10 mm short has 17.
constexpr float MARKER_FAR_FRAME_MIN_MM = 12.0f;
constexpr float MARKER_CENTRE_MIN_MM = 95.0f; // the black box between them
constexpr float MARKER_CENTRE_MAX_MM = 140.0f;
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
