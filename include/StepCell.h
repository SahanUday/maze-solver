// ============================================================================
//  StepCell.h  --  drive from one cell centre to the next, reading its walls on the
//  way. No hardware, no Arduino.h - testable under env:native.
//
//  Runs to the centre of the next cell. `sampleLeadMm` before it, the cell's three walls
//  have been voted on and `wallsReady` is up; the caller answers with
//    stepCellContinue()  roll on through the centre into the next cell, or
//    nothing             the move stops on the centre.
//  A wall ahead moves the stop onto the front reading, cancelling odometer drift.
//  docs/architecture/modules/step_cell.md
// ============================================================================

#pragma once

#include <stdint.h>

#include "MotionPrimitives.h"
#include "RobotSpec.h"
#include "WallSense.h"

struct StepCellParams {
    PlanParams plan;
    WallParams wall;
    float pitchMm = TILE_PITCH_MM;
    float cruiseMmS = SEARCH_SPEED_MM_S;
    float sampleLeadMm = WALL_SAMPLE_LEAD_MM;     // sampling ends this far before the centre
    float sampleWindowMm = WALL_SAMPLE_WINDOW_MM; // ...and starts this much earlier
    float frontStopMm = FRONT_STOP_MM;            // front reading on a centre; 0 = no anchoring
    float frontAgreeMm = FRONT_AGREE_MM;          // echoes this close count as agreeing
    float anchorFilter = FRONT_ANCHOR_FILTER;     // weight of each accepted echo
};

struct CellWalls {
    bool front = false;
    bool left = false;
    bool right = false;
    bool unreliable = false; // a sensor heard no echo all window; its wall is reported present
};

struct StepCell {
    Straight straight;
    WallSteer steer;
    // Votes per sensor: [0] present, [1] open, [2] no echo.
    uint8_t votes[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    bool wallsReady = false; // raised once per cell
    CellWalls walls;         // valid while wallsReady
    // The move ended short of its cell because a wall stood inside the stop distance.
    bool blocked = false;
    // Latest front echo, placed on the odometer where the robot would stand with it at
    // frontStopMm. The position is the one the echo CHANGED at, not the current tick's.
    uint16_t lastFrontMm = 0;
    float candidateMm = 0.0f;
    bool haveFront = false;
    bool frontAgreed = false; // the latest echo put the wall where the one before did
    float anchorMm = 0.0f;    // the stop, smoothed over the agreeing echoes
    bool haveAnchor = false;
};

// The per-cell state, cleared on begin and on continue.
inline void stepCellClearCell(StepCell &sc)
{
    for (auto &row : sc.votes) {
        for (auto &v : row) {
            v = 0;
        }
    }
    sc.wallsReady = false;
    sc.walls = CellWalls{};
    sc.blocked = false;
    sc.haveAnchor = false;
}

// Begin a move to the centre of the next cell along `headingRefDeg`.
inline void stepCellBegin(StepCell &sc, const StepCellParams &p, const MotionFeedback &fb,
                          float headingRefDeg)
{
    straightBegin(sc.straight, fb, p.pitchMm, p.cruiseMmS, 0.0f, headingRefDeg);
    stepCellClearCell(sc); // the wall steer keeps its bias: a turn drops it
    sc.haveFront = false;
}

// The next cell is straight on: roll through this centre and read the cell after it.
inline void stepCellContinue(StepCell &sc, const StepCellParams &p)
{
    straightExtend(sc.straight, p.pitchMm, 0.0f);
    stepCellClearCell(sc);
}

namespace step_detail
{

inline void vote(uint8_t (&v)[3], bool valid, bool present)
{
    const uint8_t i = !valid ? 2 : (present ? 0 : 1);
    if (v[i] < 255) {
        ++v[i];
    }
}

// Present when seen at least as often as open; a sensor with no echo counts as present.
inline bool verdict(const uint8_t (&v)[3], bool &unreliable)
{
    if (v[2] > 0) {
        unreliable = true;
    }
    return v[0] > 0 ? v[0] >= v[1] : v[1] == 0;
}

inline float absf(float x)
{
    return x < 0.0f ? -x : x;
}

// Take in the front echo. Only one that agrees with the echo before it moves the stop, so a stray
// reading cannot. A wall inside the stop distance is a safety stop: any amount, shortening only.
// A wall this cell's vote confirmed is a drift correction, believed only while small.
inline void frontAnchor(StepCell &sc, const StepCellParams &p, const MotionFeedback &fb,
                        const WallView &wv)
{
    const bool fresh = !sc.haveFront || wv.frontMm != sc.lastFrontMm;
    const float front = static_cast<float>(wv.frontMm);
    if (fresh) {
        const float candidate = fb.distMm + (front - p.frontStopMm);
        sc.frontAgreed = sc.haveFront && absf(candidate - sc.candidateMm) <= p.frontAgreeMm;
        sc.candidateMm = candidate;
        sc.lastFrontMm = wv.frontMm;
        sc.haveFront = true;
    }

    const bool inside = front <= p.frontStopMm;
    const bool confirmed = sc.wallsReady && sc.walls.front && wv.front;
    if (!inside && !confirmed) {
        return;
    }
    // Only a new echo moves the position; between echoes it holds.
    if (sc.frontAgreed && (fresh || !sc.haveAnchor)) {
        sc.anchorMm = sc.haveAnchor ? sc.anchorMm + p.anchorFilter * (sc.candidateMm - sc.anchorMm)
                                    : sc.candidateMm;
        sc.haveAnchor = true;
    }
    if (!sc.haveAnchor) {
        return;
    }
    const float shift = sc.anchorMm - sc.straight.targetMm;
    const float believed = p.pitchMm * 0.5f; // a bigger correction is something else
    if (inside && shift < 0.0f) {
        sc.blocked = sc.blocked || shift <= -believed;
        straightRetarget(sc.straight, sc.anchorMm, 0.0f, p.plan.accelMmS2);
    } else if (confirmed && shift > -believed && shift < believed) {
        straightRetarget(sc.straight, sc.anchorMm, 0.0f, p.plan.accelMmS2);
    }
}

} // namespace step_detail

inline MoveStatus stepCellStep(StepCell &sc, const StepCellParams &p, const MotionFeedback &fb,
                               const WallView &wv, uint16_t dtMs, MotionTarget &out)
{
    using namespace step_detail;

    const float remaining = sc.straight.targetMm - fb.distMm;

    // Vote while the centre is lead..lead+window ahead; the verdict lands as the window closes.
    if (!sc.wallsReady && remaining <= p.sampleLeadMm + p.sampleWindowMm) {
        vote(sc.votes[0], wv.frontValid, wv.front);
        vote(sc.votes[1], wv.leftValid, wv.left);
        vote(sc.votes[2], wv.rightValid, wv.right);
        if (remaining <= p.sampleLeadMm) {
            sc.walls.front = verdict(sc.votes[0], sc.walls.unreliable);
            sc.walls.left = verdict(sc.votes[1], sc.walls.unreliable);
            sc.walls.right = verdict(sc.votes[2], sc.walls.unreliable);
            sc.wallsReady = true;
        }
    }

    if (p.frontStopMm > 0.0f && wv.frontValid) {
        frontAnchor(sc, p, fb, wv);
    }

    const float trim = wallSteerTrim(sc.steer, wv, p.wall, fb.headingDeg, dtMs);
    return straightStep(sc.straight, p.plan, fb, trim, dtMs, out);
}
