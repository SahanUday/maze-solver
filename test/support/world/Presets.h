// Ready-made arenas and floors: fill a World with one.
#pragma once

#include "../../model/Terrain.h"
#include "Floor.h"
#include "World.h"
#include "spec/measurements/Arena.h"

namespace world
{

enum class Gap : uint8_t {
    Air, // nothing under the array
    Dark // the floor below reads dark, as the first real course did
};

// Past marker A's tile the black line runs straight to this far from the tile's centre, then bends
// to the left along a bar of BAR_LENGTH_MM.
constexpr float LINE_STRAIGHT_TO_MM = 800.0f;
constexpr float BAR_LENGTH_MM = 1500.0f;

// Marker A on tile (markerI, markerJ), the way out through its `exitDir` side: the strip, the gap,
// the white board with its line and the ramp.
inline Floor sectionAFloor(int markerI, int markerJ, int exitDir, Gap gap = Gap::Air)
{
    Floor f;
    f.frame = {PITCH * static_cast<float>(markerI), PITCH * static_cast<float>(markerJ), exitDir};
    const float box = MARKER_A_BOX_MM / 2;
    f.addSquare({0.0f, 0.0f}, box + MARKER_A_FRAME_MM, Surface::Marker);
    f.addSquare({0.0f, 0.0f}, box, Surface::MazeFloor);
    f.addBand(BRIDGE_OUT_GAP_FROM_MM, BRIDGE_OUT_GAP_TO_MM,
              gap == Gap::Dark ? Surface::GapDark : Surface::Air);
    f.addBand(BRIDGE_OUT_BOARD_FROM_MM, FAR_MM, Surface::Board);
    const Point line[] = {{BRIDGE_OUT_BOARD_FROM_MM, 0.0f},
                          {LINE_STRAIGHT_TO_MM, 0.0f},
                          {LINE_STRAIGHT_TO_MM, BAR_LENGTH_MM}};
    f.addLine(line, 3, BRIDGE_OUT_LINE_WIDTH_MM);
    f.addRamp(BRIDGE_OUT_BOARD_FROM_MM, BRIDGE_OUT_BOARD_FROM_MM + model::RAMP_LENGTH_MM,
              model::RAMP_SLOPE_DEG);
    return f;
}

// The team's Section A maze (the photo): the marker tile is its top-left corner, closed on both
// sides and open to the north, where the line leaves the maze.
inline void photo(World &w, Gap gap = Gap::Air)
{
    Arena &a = w.arena;
    a.clear(SECTION_A_SIZE_TILES);
    a.closeOuterWalls();
    a.side(0, a.n - 1, 1) = false;
    const struct {
        int row; // from the top of the photo
        int col;
        char side; // E = its east side, S = its south side
    } edges[] = {{0, 0, 'E'}, {1, 0, 'S'}, {3, 0, 'E'}, {0, 2, 'S'},
                 {1, 1, 'E'}, {2, 1, 'E'}, {1, 2, 'S'}, {2, 2, 'E'}};
    for (const auto &e : edges) {
        a.side(e.col, a.n - 1 - e.row, e.side == 'E' ? 0 : 3) = true;
    }
    a.build();
    w.floor = sectionAFloor(0, a.n - 1, 1, gap);
}

} // namespace world
