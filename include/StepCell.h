// ============================================================================
//  StepCell.h  --  drive from one cell centre to the next, reading its walls on the
//  way. No hardware, no Arduino.h - testable under env:native.
//
//  A StepCell runs toward the centre of the cell ahead. Shortly before that centre
//  (`sampleLeadMm` away) it has voted on that cell's three walls and raised
//  `wallsReady`; the caller decides what comes next and answers with either
//    stepCellContinue()  drive on through the centre into the next cell, or
//    nothing             the move stops on the centre by itself.
//  The default is to stop, so a caller that never answers is merely slow.
//  A wall ahead re-anchors the stop on the front reading, which also cancels the
//  distance drift the odometer has built up.
//  docs/architecture/modules/step_cell.md
// ============================================================================

#pragma once

#include <stdint.h>

#include "MotionPrimitives.h"
#include "WallSense.h"

struct StepCellParams {
    PlanParams plan;
    WallParams wall;
    float pitchMm = 0.0f; // centre to centre
    float cruiseMmS = 0.0f;
    float sampleLeadMm = 0.0f;   // sampling ends this far before the target centre
    float sampleWindowMm = 0.0f; // ...and starts this much earlier
    float frontStopMm = 0.0f;    // front reading when stopped on a centre; 0 disables anchoring
};

struct CellWalls {
    bool front = false;
    bool left = false;
    bool right = false;
    // Some sensor never heard an echo through the whole window. Its wall is reported present
    // (the safe answer) but the reading deserves less trust.
    bool unreliable = false;
};

struct StepCell {
    Straight straight;
    WallSteer steer;
    // Votes taken in the sample window, per sensor: [0] present, [1] open, [2] no echo.
    uint8_t votes[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    bool sampling = false;
    bool wallsReady = false; // raised once per cell; cleared by stepCellContinue()
    CellWalls walls;
    // The front reading and the odometer position when it last changed. A sensor only refreshes
    // every few ticks, so a reading is older than the tick it is read on; pairing it with the
    // position it was TAKEN at (not the current one) keeps the anchor from drifting with the robot.
    uint16_t lastFrontMm = 0;
    float lastFrontDistMm = 0.0f;
    bool haveFront = false;
    // The anchored stop position, smoothed over the readings. The front reading jitters by +-4 mm
    // (110 and 119 alternated at a dead end on the robot); retargeting to each one kept the
    // profile restarting, so the move never settled.
    float anchorMm = 0.0f;
    bool haveAnchor = false;
};

// Weight of each new front reading in the smoothed stop position.
constexpr float STEP_ANCHOR_FILTER = 0.3f;

inline void stepCellClearVotes(StepCell &sc)
{
    for (auto &row : sc.votes) {
        for (auto &v : row) {
            v = 0;
        }
    }
    sc.sampling = false;
    sc.wallsReady = false;
    sc.haveAnchor = false;
}

// Begin a move to the centre of the next cell along `headingRefDeg`. `fb` is the robot at the
// centre it is leaving (or wherever it is, if a turn just ended).
inline void stepCellBegin(StepCell &sc, const StepCellParams &p, const MotionFeedback &fb,
                          float headingRefDeg)
{
    straightBegin(sc.straight, fb, p.pitchMm, p.cruiseMmS, 0.0f, headingRefDeg);
    wallSteerReset(sc.steer);
    stepCellClearVotes(sc);
    sc.walls = CellWalls{};
    sc.haveFront = false;
}

// The next cell is straight on: carry on through this centre without stopping, and start
// reading the walls of the cell after it.
inline void stepCellContinue(StepCell &sc, const StepCellParams &p)
{
    straightExtend(sc.straight, p.pitchMm, 0.0f);
    stepCellClearVotes(sc);
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

// Present when it was seen at least as often as it was seen open; a sensor that never heard an
// echo counts as present.
inline bool verdict(const uint8_t (&v)[3], bool &unreliable)
{
    if (v[2] > 0) {
        unreliable = true;
    }
    return v[0] > 0 ? v[0] >= v[1] : v[1] == 0;
}

} // namespace step_detail

inline MoveStatus stepCellStep(StepCell &sc, const StepCellParams &p, const MotionFeedback &fb,
                               const WallView &wv, uint16_t dtMs, MotionTarget &out)
{
    using namespace step_detail;

    const float remaining = sc.straight.targetMm - fb.distMm;

    // Vote on the walls while the centre is between lead+window and lead ahead; the verdict
    // lands as the window closes.
    if (!sc.wallsReady && remaining <= p.sampleLeadMm + p.sampleWindowMm) {
        sc.sampling = true;
        vote(sc.votes[0], wv.frontValid, wv.front);
        vote(sc.votes[1], wv.leftValid, wv.left);
        vote(sc.votes[2], wv.rightValid, wv.right);
        if (remaining <= p.sampleLeadMm) {
            sc.walls = CellWalls{};
            sc.walls.front = verdict(sc.votes[0], sc.walls.unreliable);
            sc.walls.left = verdict(sc.votes[1], sc.walls.unreliable);
            sc.walls.right = verdict(sc.votes[2], sc.walls.unreliable);
            sc.wallsReady = true;
            sc.sampling = false;
        }
    }

    // Anchor the stop on a wall ahead: the centre is frontStopMm short of it. Trusted when this
    // cell's own vote confirmed the wall, or when the reading is already at or inside the stop
    // distance (so a wall that was never voted on still cannot be driven into). Only a small
    // correction is believed: a big one means the reading is something else.
    if (p.frontStopMm > 0.0f && wv.frontValid) {
        const bool fresh = !sc.haveFront || wv.frontMm != sc.lastFrontMm;
        if (fresh) {
            sc.lastFrontMm = wv.frontMm;
            sc.lastFrontDistMm = fb.distMm;
            sc.haveFront = true;
        }
        const float front = static_cast<float>(wv.frontMm);
        const bool confirmed = sc.wallsReady && sc.walls.front && wv.front;
        if (confirmed || front <= p.frontStopMm) {
            const float anchored = sc.lastFrontDistMm + (front - p.frontStopMm);
            // Only a new reading moves the smoothed position; between readings it holds.
            if (fresh || !sc.haveAnchor) {
                sc.anchorMm = sc.haveAnchor
                                  ? sc.anchorMm + STEP_ANCHOR_FILTER * (anchored - sc.anchorMm)
                                  : anchored;
                sc.haveAnchor = true;
            }
            const float shift = sc.anchorMm - sc.straight.targetMm;
            if (shift > -p.pitchMm * 0.5f && shift < p.pitchMm * 0.5f) {
                straightRetarget(sc.straight, sc.anchorMm, 0.0f);
            }
        }
    }

    const float trim = wallSteerTrim(sc.steer, wv, p.wall, dtMs);
    return straightStep(sc.straight, p.plan, fb, trim, dtMs, out);
}
