// Ready-made arenas and floors: fill a World with one.
#pragma once

#include "../../model/Terrain.h"
#include "../MazeGen.h"
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
// to the left along a bar of BRIDGE_OUT_BAR_MM.
constexpr float BRIDGE_OUT_STRAIGHT_TO_MM = 800.0f;
constexpr float BRIDGE_OUT_BAR_MM = 1500.0f;

// Marker A on tile (markerI, markerJ), the way out through its `exitDir` side: the strip, the gap,
// the white board with its line and the ramp.
inline Floor bridgeOutFloor(int markerI, int markerJ, int exitDir, Gap gap = Gap::Dark)
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
                          {BRIDGE_OUT_STRAIGHT_TO_MM, 0.0f},
                          {BRIDGE_OUT_STRAIGHT_TO_MM, BRIDGE_OUT_BAR_MM}};
    f.addLine(line, BRIDGE_OUT_LINE_WIDTH_MM);
    f.addRamp(BRIDGE_OUT_BOARD_FROM_MM, BRIDGE_OUT_BOARD_FROM_MM + model::RAMP_LENGTH_MM,
              model::RAMP_SLOPE_DEG);
    return f;
}

// The team's Section A maze (the photo): the marker tile is its top-left corner, closed on both
// sides and open to the north, where the line leaves the maze.
inline void photoMaze(World &w, Gap gap = Gap::Dark)
{
    Arena &a = w.arena;
    a.reset(SECTION_A_SIZE_TILES);
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
    w.floor = bridgeOutFloor(0, a.n - 1, 1, gap);
}

// ---- The way into Section B ------------------------------------------------------------------
// The line board ends at a corner tile of an n x n maze, through one of the tile's two outer walls.
struct Entry {
    int size = SECTION_B_SIZE_TILES; // 1: the board and a single tile
    int corner = 0;                  // 0 south-west, 1 south-east, 2 north-east, 3 north-west
    int opening = 0;                 // 0 or 1: which of the corner tile's outer walls is open
    uint32_t seed = 1;               // the maze
};

// The two sides of each corner tile that face out of the arena (0 east, 1 north, 2 west, 3 south).
constexpr int CORNER_OUTER_SIDES[4][2] = {{2, 3}, {0, 3}, {0, 1}, {2, 1}};

// The line, from its far end to the entry tile, in the floor's frame: the way out of the entry tile
// and across it. The ramp climbs to a crest at BRIDGE_IN_CREST_MM; layout, not measured.
constexpr Point BRIDGE_IN_LINE[] = {{2300.0f, 0.0f},   {1500.0f, 0.0f},
                                    {1250.0f, 250.0f}, {900.0f, 250.0f},
                                    {650.0f, 0.0f},    {model::BRIDGE_IN_BOARD_END_MM, 0.0f}};
constexpr float BRIDGE_IN_CREST_MM = 1900.0f;

// A maze tile's side, as the world numbers it, for each maze::Dir.
constexpr int WORLD_SIDE[4] = {1, 0, 3, 2};

// The maze, its entry through the board, and marker B on the entry tile. The finish is the tile
// farthest from the entry, white all over; the robot sees it only on it.
inline void bridgeIn(World &w, const Entry &e)
{
    Arena &a = w.arena;
    const int n = e.size;
    a.reset(n);
    const mazegen::RandomMaze m =
        mazegen::randomMaze(static_cast<uint8_t>(n), e.seed, static_cast<uint8_t>(n));
    for (int r = 0; r < n; ++r) {
        for (int c = 0; c < n; ++c) {
            for (int d = 0; d < 4; ++d) {
                if ((m.wall[r * n + c] & maze::wallBit(static_cast<maze::Dir>(d))) != 0) {
                    a.side(c, n - 1 - r, WORLD_SIDE[d]) = true;
                }
            }
        }
    }
    const int ei = (e.corner == 1 || e.corner == 2) ? n - 1 : 0;
    const int ej = (e.corner == 2 || e.corner == 3) ? n - 1 : 0;
    const int out = CORNER_OUTER_SIDES[e.corner][e.opening];
    a.side(ei, ej, out) = false;
    a.build();

    Floor f;
    f.frame = {PITCH * static_cast<float>(ei), PITCH * static_cast<float>(ej), out};
    const int entry = (n - 1 - ej) * n + ei;
    int finish = -1;
    int farthest = 0;
    for (int t = 0; t < n * n; ++t) {
        const int steps = mazegen::trueDistance(m, entry, t);
        if (steps > farthest) {
            farthest = steps;
            finish = t;
        }
    }
    if (finish >= 0) {
        Point centre;
        f.frame.to(PITCH * static_cast<float>(finish % n),
                   PITCH * static_cast<float>(n - 1 - finish / n), centre.along, centre.lat);
        f.addSquare(centre, PITCH / 2, Surface::Finish);
    }
    const float box = MARKER_B_BOX_MM / 2;
    f.addSquare({0.0f, 0.0f}, box + MARKER_B_FRAME_MM, Surface::Marker);
    f.addSquare({0.0f, 0.0f}, box, Surface::MazeFloor);
    f.addBand(model::BRIDGE_IN_BOARD_END_MM, FAR_MM, Surface::Board);
    f.addLine(BRIDGE_IN_LINE, BRIDGE_IN_LINE_WIDTH_MM);
    f.addRamp(BRIDGE_IN_CREST_MM, BRIDGE_IN_CREST_MM + model::RAMP_LENGTH_MM,
              model::RAMP_SLOPE_DEG);
    f.addBand(BRIDGE_IN_CREST_MM - model::CREST_AIR_MM, BRIDGE_IN_CREST_MM, Surface::Air);
    w.floor = f;
}

// On the line's far end, facing the entry tile.
inline void startOnLine(World &w)
{
    float x;
    float y;
    w.floor.frame.from(BRIDGE_IN_LINE[0].along, BRIDGE_IN_LINE[0].lat, x, y);
    w.startAt(x, y, 90.0 * w.floor.frame.dir + 180.0);
}

// On the entry tile, facing into the maze.
inline void startOnEntryTile(World &w)
{
    w.start(w.floor.originI(), w.floor.originJ(), (w.floor.frame.dir + 2) & 3);
}

} // namespace world
