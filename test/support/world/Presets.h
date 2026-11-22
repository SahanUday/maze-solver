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
    Dark // something dark under it, as the first real course read
};

// ---- Section A ---------------------------------------------------------------------------------
// Past marker A's tile the line runs straight to this far from the tile's centre, then bends left
// along a bar.
constexpr float BRIDGE_OUT_STRAIGHT_TO_MM = 800.0f;
constexpr float BRIDGE_OUT_BAR_MM = 1500.0f;

// Marker A on tile (markerI, markerJ), the way out through its `exitDir` side: the strip, the gap,
// the white board with its line, and the ramp.
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

// The team's Section A maze (the photo): marker A on its top-left tile, open to the north.
inline void photoMaze(World &w, Gap gap = Gap::Dark)
{
    Arena &a = w.arena;
    a.reset(SECTION_A_SIZE_TILES);
    a.closeOuterWalls();
    a.side(0, a.n - 1, 1) = false;
    const struct {
        int row; // from the top of the photo
        int col;
        char side; // E east, S south
    } edges[] = {{0, 0, 'E'}, {1, 0, 'S'}, {3, 0, 'E'}, {0, 2, 'S'},
                 {1, 1, 'E'}, {2, 1, 'E'}, {1, 2, 'S'}, {2, 2, 'E'}};
    for (const auto &e : edges) {
        a.side(e.col, a.n - 1 - e.row, e.side == 'E' ? 0 : 3) = true;
    }
    a.build();
    w.floor = bridgeOutFloor(0, a.n - 1, 1, gap);
}

// ---- Section B ---------------------------------------------------------------------------------
// The line board ends at a corner tile of an n x n maze, through one of the tile's two outer walls.
struct Entry {
    int size = SECTION_B_SIZE_TILES; // 1: the board and a single tile
    int corner = 0;                  // 0 south-west, 1 south-east, 2 north-east, 3 north-west
    int opening = 0;                 // which of the corner tile's two outer walls is open
    uint32_t seed = 1;               // the maze
};

// The two sides of each corner tile that face out of the arena.
constexpr int CORNER_OUTER_SIDES[4][2] = {{2, 3}, {0, 3}, {0, 1}, {2, 1}};

// The line from its far end to the entry tile, in the floor's frame: layout, not measured.
constexpr Point BRIDGE_IN_LINE[] = {{2300.0f, 0.0f},   {1500.0f, 0.0f},
                                    {1250.0f, 250.0f}, {900.0f, 250.0f},
                                    {650.0f, 0.0f},    {model::BRIDGE_IN_BOARD_END_MM, 0.0f}};
constexpr float BRIDGE_IN_CREST_MM = 1900.0f; // the ramp's top

// A maze tile's side as the world numbers it, for each maze::Dir.
constexpr int WORLD_SIDE[4] = {1, 0, 3, 2};

// The world's walls from a generated maze (row 0 at the top).
inline void wallsOf(const mazegen::RandomMaze &m, Arena &a)
{
    for (int r = 0; r < m.n; ++r) {
        for (int c = 0; c < m.n; ++c) {
            for (int d = 0; d < 4; ++d) {
                if ((m.wall[r * m.n + c] & maze::wallBit(static_cast<maze::Dir>(d))) != 0) {
                    a.side(c, m.n - 1 - r, WORLD_SIDE[d]) = true;
                }
            }
        }
    }
}

// The tile farthest from `from` by steps; -1 in a maze of one tile.
inline int farthestTile(const mazegen::RandomMaze &m, int from)
{
    int farthest = -1;
    int steps = 0;
    for (int t = 0; t < m.n * m.n; ++t) {
        const int d = mazegen::trueDistance(m, from, t);
        if (d > steps) {
            steps = d;
            farthest = t;
        }
    }
    return farthest;
}

// The maze behind the board, marker B on its entry tile, and a white finish tile (the farthest
// from the entry) the robot sees only on it.
inline void bridgeIn(World &w, const Entry &e)
{
    const int n = e.size;
    const mazegen::RandomMaze m =
        mazegen::randomMaze(static_cast<uint8_t>(n), e.seed, static_cast<uint8_t>(n));
    Arena &a = w.arena;
    a.reset(n);
    wallsOf(m, a);
    const int ei = (e.corner == 1 || e.corner == 2) ? n - 1 : 0;
    const int ej = (e.corner == 2 || e.corner == 3) ? n - 1 : 0;
    const int out = CORNER_OUTER_SIDES[e.corner][e.opening];
    a.side(ei, ej, out) = false;
    a.build();

    Floor f;
    f.frame = {PITCH * static_cast<float>(ei), PITCH * static_cast<float>(ej), out};
    const int finish = farthestTile(m, (n - 1 - ej) * n + ei);
    if (finish >= 0) {
        const Point centre = f.frame.at(PITCH * static_cast<float>(finish % n),
                                        PITCH * static_cast<float>(n - 1 - finish / n));
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
    const Xy p = w.floor.frame.from(BRIDGE_IN_LINE[0].along, BRIDGE_IN_LINE[0].lat);
    w.startAt(p.x, p.y, 90.0 * w.floor.frame.dir + 180.0);
}

// On the entry tile, facing into the maze.
inline void startOnEntryTile(World &w)
{
    w.start(w.floor.originI(), w.floor.originJ(), (w.floor.frame.dir + 2) & 3);
}

} // namespace world
