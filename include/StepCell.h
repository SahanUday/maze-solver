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
#include "RobotSpec.h"
#include "WallSense.h"

struct StepCellParams {
    PlanParams plan;
    WallParams wall;
    float pitchMm = TILE_PITCH_MM; // centre to centre
    float cruiseMmS = SEARCH_SPEED_MM_S;
    float sampleLeadMm = WALL_SAMPLE_LEAD_MM;     // sampling ends this far before the target centre
    float sampleWindowMm = WALL_SAMPLE_WINDOW_MM; // ...and starts this much earlier
    float frontStopMm =
        FRONT_STOP_MM; // front reading when stopped on a centre; 0 disables anchoring
    float frontAgreeMm = FRONT_AGREE_MM;      // echoes this close, in one frame, count as agreeing
    float anchorFilter = FRONT_ANCHOR_FILTER; // weight of each accepted echo in the stop position
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
    CellWalls walls;         // valid while wallsReady
    // A wall stood inside the stop distance, further than a drift correction is believed: the
    // move ended short of the cell and the map must not advance.
    bool blocked = false;
    // The front reading, and the odometer position where the robot would stand with that reading at
    // frontStopMm, taken at the position the reading CHANGED at. A sensor only refreshes every few
    // ticks, so a reading is older than the tick it is read on; pairing it with the position it
    // was taken at (not the current one) keeps the stop from drifting with the robot.
    uint16_t lastFrontMm = 0;
    float candidateMm = 0.0f;
    bool haveFront = false;
    bool frontAgreed = false; // the latest echo put the wall where the one before did
    // The stop position, smoothed over the agreeing echoes. The front jitters by +-4 mm (110 and
    // 119 alternated at a dead end on the robot); retargeting to each one kept the profile
    // restarting, so the move never settled.
    float anchorMm = 0.0f;
    bool haveAnchor = false;
};

// What stepCellBegin() and stepCellContinue() both clear: the votes and the verdict of a cell.
inline void stepCellClearCell(StepCell &sc)
{
    for (auto &row : sc.votes) {
        for (auto &v : row) {
            v = 0;
        }
    }
    sc.sampling = false;
    sc.wallsReady = false;
    sc.walls = CellWalls{};
    sc.blocked = false;
    sc.haveAnchor = false;
}

// Begin a move to the centre of the next cell along `headingRefDeg`. `fb` is the robot at the
// centre it is leaving (or wherever it is, if a turn just ended).
inline void stepCellBegin(StepCell &sc, const StepCellParams &p, const MotionFeedback &fb,
                          float headingRefDeg)
{
    straightBegin(sc.straight, fb, p.pitchMm, p.cruiseMmS, 0.0f, headingRefDeg);
    stepCellClearCell(sc); // the wall steer keeps what it learned: a turn drops it by itself
    sc.haveFront = false;
}

// The next cell is straight on: carry on through this centre without stopping, and start
// reading the walls of the cell after it.
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

// Present when it was seen at least as often as it was seen open; a sensor that never heard an
// echo counts as present.
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

// Take in the front reading. A new echo is placed on the odometer and compared with the one before:
// only an echo that agrees with its predecessor is believed, so one stray reading never moves the
// stop. The stop then follows the smoothed accepted position.
//
// A wall inside the stop distance is a safety stop: the robot is already past where it should be
// braking, whatever the odometer says, so the stop is pulled in by any amount. A wall that cell's
// vote confirmed is a drift correction, believed only while small (a big one means the reading is
// something else).
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
    // Only a new echo moves the smoothed position; between echoes it holds.
    if (sc.frontAgreed && (fresh || !sc.haveAnchor)) {
        sc.anchorMm = sc.haveAnchor ? sc.anchorMm + p.anchorFilter * (sc.candidateMm - sc.anchorMm)
                                    : sc.candidateMm;
        sc.haveAnchor = true;
    }
    if (!sc.haveAnchor) {
        return;
    }
    const float shift = sc.anchorMm - sc.straight.targetMm;
    const float believed = p.pitchMm * 0.5f;
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

    if (p.frontStopMm > 0.0f && wv.frontValid) {
        frontAnchor(sc, p, fb, wv);
    }

    const float trim = wallSteerTrim(sc.steer, wv, p.wall, fb.headingDeg, dtMs);
    return straightStep(sc.straight, p.plan, fb, trim, dtMs, out);
}
