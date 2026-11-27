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
    int halfTurnsInDeadEnds = 0;           // begun in a dead end the robot had driven into
    float worstEndMm = 0.0f;               // a reverse's end from the centre of its tile
    float nearestBehindMm = world::FAR_MM; // ... and from the axle to a wall behind it
    bool openHeardCleared = true; // nothing heard in the tile it left stays with the explorer
    bool statusShowsBackOut = true;

    // True on the tick a reverse has ended.
    template <uint8_t N> bool see(const world::World &w, const CellExplorerT<N> &ce)
    {
        const ExplorePhase now = ce.phase;
        const bool ended = was == ExplorePhase::BackOut && now != was;
        if (ended) {
            ++backOuts;
            const float off = hypotf(static_cast<float>(w.plant.x) - world::PITCH * w.cellI(),
                                     static_cast<float>(w.plant.y) - world::PITCH * w.cellJ());
            worstEndMm = fmaxf(worstEndMm, off);
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
