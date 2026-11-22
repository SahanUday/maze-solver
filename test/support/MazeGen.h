// Random mazes for the host tests: a depth-first maze (one way between any two tiles) with a few
// walls knocked out for loops, from a seed. Row 0 is the top, the outer edges are walls.
#pragma once

#include <stdint.h>

#include "Maze.h"

namespace mazegen
{

using maze::Dir;
using maze::opposite;
using maze::wallBit;

// Row and column step of each Dir.
constexpr int kDRow[4] = {-1, 0, 1, 0};
constexpr int kDCol[4] = {0, 1, 0, -1};

struct RandomMaze {
    uint8_t n;
    uint8_t wall[81]; // wallBit() of each tile, row-major; the outer edges are walls
};

inline uint32_t nextRandom(uint32_t &s)
{
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s << 5;
    return s;
}

// The tile next to `tile` in direction `d`, or -1 outside the maze.
inline int mazeNeighbour(const RandomMaze &m, int tile, Dir d)
{
    const int r = tile / m.n + kDRow[static_cast<uint8_t>(d)];
    const int c = tile % m.n + kDCol[static_cast<uint8_t>(d)];
    return (r < 0 || c < 0 || r >= m.n || c >= m.n) ? -1 : r * m.n + c;
}

// Knock out the wall between `tile` and its neighbour `next` in direction `d`.
inline void openWall(RandomMaze &m, int tile, Dir d, int next)
{
    m.wall[tile] = static_cast<uint8_t>(m.wall[tile] & ~wallBit(d));
    m.wall[next] = static_cast<uint8_t>(m.wall[next] & ~wallBit(opposite(d)));
}

// A maze with one way between any two tiles (depth-first), then `loops` more walls knocked out.
inline RandomMaze randomMaze(uint8_t n, uint32_t seed, uint8_t loops)
{
    RandomMaze m{n, {}};
    for (int i = 0; i < n * n; ++i) {
        m.wall[i] = 0x0F;
    }
    uint32_t rng = seed * 2654435761u + 1u;
    bool seen[81] = {};
    uint8_t stack[81];
    uint8_t top = 0;
    stack[top++] = 0;
    seen[0] = true;
    while (top > 0) {
        const int cur = stack[top - 1];
        Dir options[4];
        uint8_t count = 0;
        for (uint8_t k = 0; k < 4; ++k) {
            const int next = mazeNeighbour(m, cur, static_cast<Dir>(k));
            if (next >= 0 && !seen[next]) {
                options[count++] = static_cast<Dir>(k);
            }
        }
        if (count == 0) {
            --top;
            continue;
        }
        const Dir d = options[nextRandom(rng) % count];
        const int next = mazeNeighbour(m, cur, d);
        openWall(m, cur, d, next);
        seen[next] = true;
        stack[top++] = static_cast<uint8_t>(next);
    }
    for (uint8_t k = 0; k < loops; ++k) {
        const int tile = static_cast<int>(nextRandom(rng) % (n * n));
        const Dir d = static_cast<Dir>(nextRandom(rng) & 3);
        const int next = mazeNeighbour(m, tile, d);
        if (next >= 0) {
            openWall(m, tile, d, next);
        }
    }
    return m;
}

// Steps from `from` to `goal` by a search over the maze itself.
inline uint16_t trueDistance(const RandomMaze &m, int from, int goal)
{
    int dist[81];
    int queue[81];
    for (int i = 0; i < m.n * m.n; ++i) {
        dist[i] = -1;
    }
    int head = 0;
    int tail = 0;
    dist[from] = 0;
    queue[tail++] = from;
    while (head < tail) {
        const int cur = queue[head++];
        for (uint8_t k = 0; k < 4; ++k) {
            const Dir d = static_cast<Dir>(k);
            const int next = mazeNeighbour(m, cur, d);
            if ((m.wall[cur] & wallBit(d)) == 0 && dist[next] < 0) {
                dist[next] = dist[cur] + 1;
                queue[tail++] = next;
            }
        }
    }
    return static_cast<uint16_t>(dist[goal]);
}

} // namespace mazegen
