// What lies on the floor: patches of surface in a frame, drawn in order, the last one on top, over
// the maze's black floor. Surfaces say what a point is; what the IR array reads over each is the
// sensor's business (Sensors.h).
#pragma once

#include <math.h>
#include <stdint.h>

#include "Arena.h"

namespace world
{

enum class Surface : uint8_t { MazeFloor, Marker, Board, Line, GapDark, Air, Finish };

struct Point {
    float along;
    float lat;
};

// Along a direction (0 east, 1 north, 2 west, 3 south) and across it, left positive, from an
// origin.
struct Frame {
    float x = 0.0f;
    float y = 0.0f;
    int dir = 1;

    static constexpr float EX[4] = {1.0f, 0.0f, -1.0f, 0.0f};
    static constexpr float EY[4] = {0.0f, 1.0f, 0.0f, -1.0f};

    void to(float px, float py, float &along, float &lat) const
    {
        const float dx = px - x;
        const float dy = py - y;
        along = dx * EX[dir] + dy * EY[dir];
        lat = -dx * EY[dir] + dy * EX[dir];
    }

    void from(float along, float lat, float &px, float &py) const
    {
        px = x + along * EX[dir] - lat * EY[dir];
        py = y + along * EY[dir] + lat * EX[dir];
    }
};

// Everything past `from` up to `to`, all across.
struct Band {
    float from;
    float to;

    bool has(float along, float) const
    {
        return along >= from && along < to;
    }
};

// A square of half side `half` around `centre`.
struct Square {
    Point centre;
    float half;

    bool has(float along, float lat) const
    {
        return fabsf(along - centre.along) < half && fabsf(lat - centre.lat) < half;
    }
};

// A line from `a` along the unit direction (ux, uy) for `len`, of width 2 * half, reaching `back`
// behind `a`.
struct Strip {
    Point a;
    float ux;
    float uy;
    float len;
    float half;
    float back;

    static Strip between(Point from, Point to, float width, float back)
    {
        const float dx = to.along - from.along;
        const float dy = to.lat - from.lat;
        const float len = sqrtf(dx * dx + dy * dy);
        return {from, dx / len, dy / len, len, width / 2, back};
    }

    // Distance along the line and across it, of a point.
    void measure(float along, float lat, float &t, float &across) const
    {
        const float da = along - a.along;
        const float dl = lat - a.lat;
        t = da * ux + dl * uy;
        across = -da * uy + dl * ux;
    }

    bool has(float along, float lat) const
    {
        float t;
        float across;
        measure(along, lat, t, across);
        return fabsf(across) < half && t >= -back && t <= len;
    }

    // Distance from a point to the line's own centre.
    float distance(float along, float lat) const
    {
        float t;
        float across;
        measure(along, lat, t, across);
        const float ct = t < 0.0f ? 0.0f : (t > len ? len : t);
        return sqrtf((t - ct) * (t - ct) + across * across);
    }
};

// The forward axis' tilt while the axle is between `from` and `to`, nose up positive for a robot
// driving the way the run goes.
struct Ramp {
    float from;
    float to;
    float slopeDeg;
};

struct Patch {
    enum class Kind : uint8_t { Band, Square, Strip };

    Kind kind;
    Surface surface;
    union {
        Band band;
        Square square;
        Strip strip;
    };

    static Patch of(const Band &shape, Surface s)
    {
        Patch p{Kind::Band, s, {}};
        p.band = shape;
        return p;
    }
    static Patch of(const Square &shape, Surface s)
    {
        Patch p{Kind::Square, s, {}};
        p.square = shape;
        return p;
    }
    static Patch of(const Strip &shape, Surface s)
    {
        Patch p{Kind::Strip, s, {}};
        p.strip = shape;
        return p;
    }

    bool has(float along, float lat) const
    {
        switch (kind) {
            case Kind::Band:
                return band.has(along, lat);
            case Kind::Square:
                return square.has(along, lat);
            default:
                return strip.has(along, lat);
        }
    }
};

struct Floor {
    static constexpr int MAX_PATCHES = 32;
    static constexpr int MAX_RAMPS = 4;

    Frame frame;
    Patch patch[MAX_PATCHES];
    int patches = 0;
    Ramp ramp[MAX_RAMPS];
    int ramps = 0;

    void addBand(float from, float to, Surface s)
    {
        patch[patches++] = Patch::of(Band{from, to}, s);
    }
    void addSquare(Point centre, float half, Surface s)
    {
        patch[patches++] = Patch::of(Square{centre, half}, s);
    }
    // A line through `points`; a bend has no notch: each strip after the first reaches half a width
    // behind its start.
    template <int N> void addLine(const Point (&points)[N], float width)
    {
        for (int k = 0; k + 1 < N; ++k) {
            const float back = k > 0 ? width / 2 : 0.0f;
            patch[patches++] =
                Patch::of(Strip::between(points[k], points[k + 1], width, back), Surface::Line);
        }
    }
    void addRamp(float from, float to, float slopeDeg)
    {
        ramp[ramps++] = {from, to, slopeDeg};
    }

    // Take every patch of a surface away, e.g. the line.
    void erase(Surface s)
    {
        int kept = 0;
        for (int k = 0; k < patches; ++k) {
            if (patch[k].surface != s) {
                patch[kept++] = patch[k];
            }
        }
        patches = kept;
    }

    Surface surfaceAt(float px, float py) const
    {
        float along;
        float lat;
        frame.to(px, py, along, lat);
        for (int k = patches - 1; k >= 0; --k) {
            if (patch[k].has(along, lat)) {
                return patch[k].surface;
            }
        }
        return Surface::MazeFloor;
    }

    // Tilt under a point, 0 on the level.
    float slopeDeg(float px, float py) const
    {
        float along;
        float lat;
        frame.to(px, py, along, lat);
        for (int k = 0; k < ramps; ++k) {
            if (along >= ramp[k].from && along <= ramp[k].to) {
                return ramp[k].slopeDeg;
            }
        }
        return 0.0f;
    }

    // How far a point is from the nearest line's centre; FAR_MM with no line.
    float lineErrorMm(float px, float py) const
    {
        float along;
        float lat;
        frame.to(px, py, along, lat);
        float best = FAR_MM;
        for (int k = 0; k < patches; ++k) {
            if (patch[k].surface == Surface::Line) {
                const float d = patch[k].strip.distance(along, lat);
                best = d < best ? d : best;
            }
        }
        return best;
    }

    float alongMm(float px, float py) const
    {
        float along;
        float lat;
        frame.to(px, py, along, lat);
        return along;
    }

    // The tile the frame's origin lies on.
    int originI() const
    {
        return static_cast<int>(lroundf(frame.x / PITCH));
    }
    int originJ() const
    {
        return static_cast<int>(lroundf(frame.y / PITCH));
    }
};

} // namespace world
