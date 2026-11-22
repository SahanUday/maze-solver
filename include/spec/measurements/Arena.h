// ============================================================================
//  spec/measurements/Arena.h  --  the arena as built: tiles, walls, approach markers, bridge floor.
//  Layout, tags and how to update: spec/README.md.
//
//  The rules give nominal sizes; the arena is hand-built, so each marker and each bridge line has
//  its own constants even where the rules say they match. A is the mid-evaluation arena (tag
//  v0.1-section-a, ran on the robot); B is Section B's, not yet measured.
// ============================================================================

#pragma once

#include <stdint.h>

// ---- Tiles and walls ---------------------------------------------------- [RULES] --
constexpr uint16_t TILE_PITCH_MM = 250; // centre-to-centre; 25 cm tiles, held at the mid-evaluation
constexpr uint16_t WALL_HEIGHT_MM = 100;

// ---- Wall thickness ----------------------------------------- <<TBD HARDWARE>> --
// Tape-measure it (#54). The sensors' corridor of 231-235 mm on a 250 mm pitch implies 15-19 mm.
constexpr float WALL_THICKNESS_MM = 17.0f;

// ---- Approach marker A (Section A, before the bridge-out) ------------ [MEASURED] --
// A white frame around a black box; sizes along the path. Three scans on the robot: white 36-42 mm,
// black 116-119 mm, white 37-41 mm. Detection windows: tuning/FloorPattern.h.
constexpr float MARKER_A_FRAME_MM = 36.5f; // white, each side of the box
constexpr float MARKER_A_BOX_MM = 118.0f;

// ---- Approach marker B (Section B, before the bridge-in) ---- <<TBD HARDWARE>> --
// Not measured: A's sizes until the arena visit (#54).
constexpr float MARKER_B_FRAME_MM = 36.5f;
constexpr float MARKER_B_BOX_MM = 118.0f;

// ---- Bridge-out floor (beyond marker A's open edge) ------------------ [MEASURED] --
// Along the exit direction from the marker tile's centre: the tile edge, ~20 mm of black strip, the
// ~37 mm gap (its edges blur), a dark patch, then the white board with the black line (PR #28
// scans).
constexpr float BRIDGE_OUT_STRIP_FROM_MM = 125.0f;
constexpr float BRIDGE_OUT_GAP_FROM_MM = 145.0f;
constexpr float BRIDGE_OUT_GAP_TO_MM = 182.0f;
constexpr float BRIDGE_OUT_BOARD_FROM_MM = 190.0f;
constexpr float BRIDGE_OUT_LINE_WIDTH_MM = 24.0f; // the rules say 30

// ---- Bridge-in floor ---------------------------------------- <<TBD HARDWARE>> --
// Not measured: the bridge-out line's width until the arena visit (#54).
constexpr float BRIDGE_IN_LINE_WIDTH_MM = 24.0f;
