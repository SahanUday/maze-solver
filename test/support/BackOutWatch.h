// What the explorer does where the way on is straight back, watched after each tick of a run:
// whether it turns round in a dead end, and how its reverses end.
#pragma once

#include <math.h>

#include "mission/CellExplorer.h"
#include "world/World.h"

namespace backout
{

// Walls on the sides of arena tile (i, j); three make it a dead end.
inline int wallsAround(const world::Arena &a, int i, int j)
{
    int k = 0;
    for (int d = 0; d < 4; ++d) {
        k += a.side(i, j, d) ? 1 : 0;
    }
    return k;
}

struct Watch {
    ExplorePhase was = ExplorePhase::Sense;
    int backOuts = 0;
    int longestRun = 0;            // tiles reversed in a row
    uint16_t seen = 0;             // the explorer's count at the last tick
    int halfTurnsInDeadEnds = 0;   // begun in a dead end the robot had driven into
    float worstEndMm = 0.0f;       // a reverse's end from the centre of its tile
    float worstFromStartMm = 0.0f; // ... and, along the way, from where the move it undoes began
    float originMm = -1.0f;        // the move in progress, as the explorer holds it
    float startX = 0.0f, startY = 0.0f;    // ... and where the robot was when it began
    float nearestBehindMm = world::FAR_MM; // ... and from the axle to a wall behind it
    bool openHeardCleared = true; // nothing heard in the tile it left stays with the explorer
    bool statusShowsBackOut = true;

    // True on the tick a tile has been reversed, whether or not the next one follows at once.
    template <uint8_t N> bool see(const world::World &w, const CellExplorerT<N> &ce)
    {
        const ExplorePhase now = ce.phase;
        if (ce.step.originMm != originMm) { // a move has just begun
            originMm = ce.step.originMm;
            startX = static_cast<float>(w.plant.x);
            startY = static_cast<float>(w.plant.y);
        }
        const bool ended = ce.backOuts != seen;
        seen = ce.backOuts;
        if (ended) {
            ++backOuts;
            longestRun = ce.reversed > longestRun ? ce.reversed : longestRun;
            const float off = hypotf(static_cast<float>(w.plant.x) - world::PITCH * w.cellI(),
                                     static_cast<float>(w.plant.y) - world::PITCH * w.cellJ());
            worstEndMm = fmaxf(worstEndMm, off);
            if (ce.advanced == 0) { // the whole move undone
                const float a = w.plant.headingRad();
                const float along = (static_cast<float>(w.plant.x) - startX) * cosf(a) +
                                    (static_cast<float>(w.plant.y) - startY) * sinf(a);
                worstFromStartMm = fmaxf(worstFromStartMm, fabsf(along));
            }
            const float behind = w.plant.ray(w.arena, static_cast<float>(w.plant.yawDeg) + 180.0f);
            nearestBehindMm = fminf(nearestBehindMm, behind);
            openHeardCleared = openHeardCleared && ce.openHeard == 0;
        }
        if (now != was && now == ExplorePhase::Turn && absf(ce.turn.deltaDeg) > 135.0f &&
            ce.moves > 0 && wallsAround(w.arena, w.cellI(), w.cellJ()) == 3) {
            ++halfTurnsInDeadEnds;
        }
        was = now;
        return ended;
    }
};

} // namespace backout
