// ============================================================================
//  spec/measurements/Arena.h  --  the arena as built: tiles, walls, markers, bridge floor.
//  Each marker and bridge line has its own constants, even where the rules say they match.
//  A is the mid-evaluation arena (tag v0.1-section-a); B is Section B's, not yet measured.
// ============================================================================

#pragma once

#include <stdint.h>

// ---- Tiles and walls -------------------------------------------- [RULES] --
constexpr uint16_t TILE_PITCH_MM = 250; // centre to centre; held at the mid-evaluation
constexpr uint16_t WALL_HEIGHT_MM = 100;

// ---- Wall thickness ------------------------------------ <<TBD HARDWARE>> --
// Tape-measure it (#54): the sensors' 231-235 mm corridor on a 250 mm pitch implies 15-19 mm.
constexpr float WALL_THICKNESS_MM = 17.0f;

// ---- Marker A ------------------------------------------------ [MEASURED] --
// Section A, before the bridge-out: a white frame around a black box, sizes along the path (three
// scans: white 36-42, black 116-119, white 37-41 mm). Detection windows: tuning/FloorPattern.h.
constexpr float MARKER_A_FRAME_MM = 36.5f; // white, each side of the box
constexpr float MARKER_A_BOX_MM = 118.0f;

// ---- Marker B ------------------------------------------ <<TBD HARDWARE>> --
// Section B, before the bridge-in. A's sizes until measured (#54).
constexpr float MARKER_B_FRAME_MM = 36.5f;
constexpr float MARKER_B_BOX_MM = 118.0f;

// ---- Bridge-out floor ---------------------------------------- [MEASURED] --
// Along the exit direction from marker A's tile centre (PR #28 scans): the tile edge, ~20 mm of
// black strip, the ~37 mm gap, a dark patch, then the white board with the line.
constexpr float BRIDGE_OUT_STRIP_FROM_MM = 125.0f;
constexpr float BRIDGE_OUT_GAP_FROM_MM = 145.0f;
constexpr float BRIDGE_OUT_GAP_TO_MM = 182.0f;
constexpr float BRIDGE_OUT_BOARD_FROM_MM = 190.0f;
constexpr float BRIDGE_OUT_LINE_WIDTH_MM = 24.0f; // the rules say 30

// ---- Bridge-in floor ----------------------------------- <<TBD HARDWARE>> --
// The bridge-out line's width until measured (#54).
constexpr float BRIDGE_IN_LINE_WIDTH_MM = 24.0f;
