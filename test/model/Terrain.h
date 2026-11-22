// ============================================================================
//  model/Terrain.h  --  the ramp and the bridge-in board's end, as the host world lays them. Tags:
//  include/spec/README.md.
// ============================================================================

#pragma once

namespace model
{

// ---- Ramp -------------------------------------------------- <<TBD HARDWARE>> --
// Measure the slope and the length of the real ramp (#54).
constexpr float RAMP_SLOPE_DEG = 10.0f;
constexpr float RAMP_LENGTH_MM = 300.0f;

// ---- Bridge-in board --------------------------------------- <<TBD HARDWARE>> --
// Where the board meets the entry tile, from the tile's centre outwards, and how far the array,
// lifted at the ramp's crest, reads air (#54).
constexpr float BRIDGE_IN_BOARD_END_MM = 125.0f;
constexpr float CREST_AIR_MM = 20.0f;

} // namespace model
