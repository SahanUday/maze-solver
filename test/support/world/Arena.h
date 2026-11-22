// The walls of an n x n arena, and what the ultrasonic modules and the nose check ask of them.
#pragma once

#include <math.h>

#include "spec/fixed/Rules.h"
#include "spec/measurements/Arena.h"

namespace world
{

constexpr int MAX_TILES = SECTION_B_SIZE_TILES;
constexpr float PITCH = TILE_PITCH_MM;
constexpr float WALL_T = WALL_THICKNESS_MM;
constexpr float FAR_MM = 1.0e9f; // no wall

struct Rect {
    float x0, x1, y0, y1;
};

// Tile (i, j): column i from the west, row j from the south; the centre of (0, 0) is the origin.
// A side is 0 east, 1 north, 2 west, 3 south.
struct Arena {
    int n = SECTION_A_SIZE_TILES;
    bool vw[(MAX_TILES + 1) * MAX_TILES] = {}; // vw[i * n + j]: a wall on the west side of column i
    bool hw[MAX_TILES * (MAX_TILES + 1)] = {}; // hw[j * n + i]: a wall on the south side of row j
    Rect rect[2 * (MAX_TILES + 1) * MAX_TILES];
    int rects = 0;

    void clear(int size)
    {
        *this = Arena{};
        n = size;
    }

    bool &side(int i, int j, int dir)
    {
        switch (dir) {
            case 0:
                return vw[(i + 1) * n + j];
            case 1:
                return hw[(j + 1) * n + i];
            case 2:
                return vw[i * n + j];
            default:
                return hw[j * n + i];
        }
    }

    const bool &side(int i, int j, int dir) const
    {
        return const_cast<Arena *>(this)->side(i, j, dir);
    }

    void closeOuterWalls()
    {
        for (int k = 0; k < n; ++k) {
            side(0, k, 2) = true;
            side(n - 1, k, 0) = true;
            side(k, 0, 3) = true;
            side(k, n - 1, 1) = true;
        }
    }

    // The wall flags as rectangles; call it after the last change.
    void build()
    {
        rects = 0;
        for (int i = 0; i <= n; ++i) {
            for (int j = 0; j < n; ++j) {
                if (vw[i * n + j]) {
                    const float bx = PITCH * i - PITCH / 2;
                    const float cy = PITCH * j;
                    rect[rects++] = {bx - WALL_T / 2, bx + WALL_T / 2, cy - PITCH / 2 - WALL_T / 2,
                                     cy + PITCH / 2 + WALL_T / 2};
                }
            }
        }
        for (int j = 0; j <= n; ++j) {
            for (int i = 0; i < n; ++i) {
                if (hw[j * n + i]) {
                    const float by = PITCH * j - PITCH / 2;
                    const float cx = PITCH * i;
                    rect[rects++] = {cx - PITCH / 2 - WALL_T / 2, cx + PITCH / 2 + WALL_T / 2,
                                     by - WALL_T / 2, by + WALL_T / 2};
                }
            }
        }
    }

    // Distance along a ray (unit direction) to the nearest wall face; FAR_MM if none.
    float ray(float ox, float oy, float dx, float dy) const
    {
        float best = FAR_MM;
        for (int k = 0; k < rects; ++k) {
            float t0 = 0.0f;
            float t1 = FAR_MM;
            bool hit = true;
            const float o[2] = {ox, oy};
            const float d[2] = {dx, dy};
            const float lo[2] = {rect[k].x0, rect[k].y0};
            const float hi[2] = {rect[k].x1, rect[k].y1};
            for (int a = 0; a < 2 && hit; ++a) {
                if (fabsf(d[a]) < 1.0e-9f) {
                    hit = o[a] >= lo[a] && o[a] <= hi[a];
                    continue;
                }
                float ta = (lo[a] - o[a]) / d[a];
                float tb = (hi[a] - o[a]) / d[a];
                if (ta > tb) {
                    const float t = ta;
                    ta = tb;
                    tb = t;
                }
                t0 = ta > t0 ? ta : t0;
                t1 = tb < t1 ? tb : t1;
                hit = t0 <= t1;
            }
            if (hit && t0 > 0.0f && t0 < best) {
                best = t0;
            }
        }
        return best;
    }

    // Distance from a point to the nearest wall face; FAR_MM if none.
    float clearance(float px, float py) const
    {
        float best = FAR_MM * FAR_MM;
        for (int k = 0; k < rects; ++k) {
            const float gx = fmaxf(fmaxf(rect[k].x0 - px, px - rect[k].x1), 0.0f);
            const float gy = fmaxf(fmaxf(rect[k].y0 - py, py - rect[k].y1), 0.0f);
            const float d2 = gx * gx + gy * gy;
            best = d2 < best ? d2 : best;
        }
        return sqrtf(best);
    }
};

// The clearance of a point, kept while the point stays well inside it: no ray from there can be
// shorter, so most asks for "is a wall within reach?" need no ray. Forget it when the walls change.
struct Clearance {
    float x = 0.0f;
    float y = 0.0f;
    float mm = -1.0f; // < 0: nothing kept

    // True only if no wall is within `reach` of the point; false means "look and see".
    bool beyond(const Arena &arena, float px, float py, float reach)
    {
        constexpr float MARGIN_MM = 0.05f;
        if (mm >= 0.0f && mm - hypotf(px - x, py - y) > reach + MARGIN_MM) {
            return true;
        }
        x = px;
        y = py;
        mm = arena.clearance(px, py);
        return mm > reach + MARGIN_MM;
    }

    void forget()
    {
        mm = -1.0f;
    }
};

} // namespace world
