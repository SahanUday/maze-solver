// ============================================================================
//  StepCell.h  --  drive from one cell centre to the next, reading its walls on the
//  way. No hardware, no Arduino.h - testable under env:native.
//
//  Runs to the centre of the next cell. `sampleLeadMm` before it, the cell's three walls
//  have been voted on and `wallsReady` is up; the caller answers with
//    stepCellContinue()  roll on through the centre into the next cell, or
//    nothing             the move stops on the centre.
//  A wall ahead moves the stop onto the front reading, cancelling odometer drift. The corridor's
//  direction, measured along the walls, is handed back as a correction to the gyro heading:
//  stepCellTakeHeadingFix().
//  docs/architecture/modules/step_cell.md
// ============================================================================

#pragma once

#include <math.h>
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
    float frontStopMm = FRONT_STOP_MM;            // front reading on a centre; 0 = no front logic
    float frontAgreeMm = FRONT_AGREE_MM;          // echoes this close count as agreeing
    float anchorFilter = FRONT_ANCHOR_FILTER;     // weight of each accepted echo
    float echoMs = US_SLOT_MS * US_SENSOR_COUNT; // a sensor's refresh period: a reading is this old
    uint16_t blindMs = FRONT_BLIND_MS; // no front echoes agree this long at the start: blocked
    float snapMm = WALL_SNAP_MM;     // travel along one wall mode to measure the corridor's heading
    float snapGain = WALL_SNAP_GAIN; // share of the measured frame error taken out per stretch
    float snapMinDeg = WALL_SNAP_MIN_DEG;
    float snapMaxDeg = WALL_BIAS_MAX_DEG; // a bigger one is not the gyro frame
};

// A sensor that heard no echo all window is reported as a wall (the safe answer) and is not
// `heard`: its reading says nothing, so a caller that keeps "not heard" apart from "open" uses the
// flags.
struct CellWalls {
    bool front = false;
    bool left = false;
    bool right = false;
    bool frontHeard = false;
    bool leftHeard = false;
    bool rightHeard = false;
    bool unreliable = false; // some sensor was not heard
};

struct StepCell {
    Straight straight;
    WallSteer steer;
    // Votes per sensor: [0] present, [1] open, [2] no echo.
    uint8_t votes[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    bool wallsReady = false; // raised once per cell
    CellWalls walls;         // valid while wallsReady
    // The move ended short of its cell: a wall stood inside the stop distance, or no front echo
    // came at all. The map must not advance on it.
    bool blocked = false;
    // The odometer position the move left (a cell centre) and the centre it is planned to reach.
    float originMm = 0.0f;
    float plannedMm = 0.0f;
    // Latest front echo, placed on the odometer where the robot would stand with it at
    // frontStopMm. The position is the one the echo CHANGED at, not the current tick's.
    uint16_t lastFrontMm = 0;
    float candidateMm = 0.0f;
    bool haveFront = false;   // an echo has come this move
    bool frontAgreed = false; // the latest echo put the wall where the one before did
    // Where the last agreeing echoes put the wall's stop point. It stays true when the sensor goes
    // quiet: the wall does not move, the odometer does.
    float wallMm = 0.0f;
    bool haveWall = false;
    float anchorMm = 0.0f; // the stop, smoothed over the agreeing echoes
    bool haveAnchor = false;
    uint16_t blindMs = 0; // time spent waiting at the start for the front sensor
    // The caller knows the way ahead is open to beyond the sensor's range (a side sensor measured
    // it): silence from the front is then no sign of a wall in the dead zone, and the move does
    // not wait for an echo.
    bool openAhead = false;
    uint16_t sameMs = 0; // how long the front reading has stood unchanged
    // Heading correction owed to the caller, and the stretch it is measured over: the lateral
    // error against the odometer along one wall mode (a least-squares line), and the mean heading.
    float snapDeg = 0.0f;
    WallMode trackMode = WallMode::None;
    float lastDistMm = 0.0f;
    float trackStartMm = 0.0f;
    uint16_t trackN = 0;
    float trackS = 0.0f, trackY = 0.0f, trackSS = 0.0f, trackSY = 0.0f, trackH = 0.0f;
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

namespace step_detail
{

inline void trackReset(StepCell &sc)
{
    sc.trackMode = WallMode::None;
    sc.trackN = 0;
    sc.trackS = sc.trackY = sc.trackSS = sc.trackSY = sc.trackH = 0.0f;
}

inline void begin(StepCell &sc, const StepCellParams &p, const MotionFeedback &fb,
                  float headingRefDeg, float distMm)
{
    straightBegin(sc.straight, fb, distMm, p.cruiseMmS, 0.0f, headingRefDeg);
    stepCellClearCell(sc); // the wall steer keeps its bias: a turn drops it
    sc.plannedMm = sc.straight.targetMm;
    sc.haveFront = false;
    sc.haveWall = false;
    sc.frontAgreed = false;
    sc.blindMs = 0;
    sc.openAhead = false;
    sc.sameMs = 0;
    step_detail::trackReset(sc);
    sc.lastDistMm = fb.distMm;
}

} // namespace step_detail

// Begin a move to the centre of the next cell along `headingRefDeg`.
inline void stepCellBegin(StepCell &sc, const StepCellParams &p, const MotionFeedback &fb,
                          float headingRefDeg)
{
    step_detail::begin(sc, p, fb, headingRefDeg, p.pitchMm);
    sc.originMm = fb.distMm;
}

// Begin again toward the centre a blocked move was planned to reach (stepCellRemainingMm()).
inline void stepCellResume(StepCell &sc, const StepCellParams &p, const MotionFeedback &fb,
                           float headingRefDeg)
{
    const float planned = sc.plannedMm;
    const float origin = sc.originMm;
    step_detail::begin(sc, p, fb, headingRefDeg, planned - fb.distMm);
    sc.originMm = origin;
}

// The next cell is straight on: roll through this centre and read the cell after it.
inline void stepCellContinue(StepCell &sc, const StepCellParams &p)
{
    straightExtend(sc.straight, p.pitchMm, 0.0f);
    sc.plannedMm += p.pitchMm;
    stepCellClearCell(sc);
}

// Whole cells the robot has covered since the move began, from the odometer. After a blocked move
// that stopped on a wall's stop point this is the cell it is in.
inline int stepCellCells(const StepCell &sc, const StepCellParams &p, const MotionFeedback &fb)
{
    const float cells = (fb.distMm - sc.originMm) / p.pitchMm;
    return static_cast<int>(cells + (cells < 0.0f ? -0.5f : 0.5f));
}

// Signed distance still to the centre the move was planned to reach.
inline float stepCellRemainingMm(const StepCell &sc, const MotionFeedback &fb)
{
    return sc.plannedMm - fb.distMm;
}

// Degrees to subtract from the gyro heading (headingSet(hd, hd.deg - fix)): the wall steer has
// learned, steady on the walls, how far the heading frame is from the corridor. Taking it returns
// 0 until the next stretch earns another.
inline float stepCellTakeHeadingFix(StepCell &sc)
{
    const float fix = sc.snapDeg;
    sc.snapDeg = 0.0f;
    return fix;
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
    if (v[0] == 0 && v[1] == 0 && v[2] > 0) {
        unreliable = true;
    }
    return v[0] > 0 ? v[0] >= v[1] : v[1] == 0;
}

// Some present or open vote: an echo was heard.
inline bool heard(const uint8_t (&v)[3])
{
    return v[0] > 0 || v[1] > 0;
}

inline float absf(float x)
{
    return x < 0.0f ? -x : x;
}

// Take in the front echo; true while the move should wait for a pair. Only an echo that
// agrees with the one before it moves the wall, so a stray reading cannot. A wall the robot can no
// longer stop short of the stop point for (braking distance and one echo of travel included) is a
// safety stop: any amount, shortening only. A wall this cell's vote confirmed is a drift
// correction, believed only while small.
//
// The wall is kept as a position on the odometer, so a sensor gone quiet (the dead zone below
// ~114 mm from the axle, a dropout) does not make it vanish. A move with no agreeing pair of echoes
// yet is held at rest until one forms; if none does within blindMs the wall is taken to be inside
// the dead zone.
inline bool frontLogic(StepCell &sc, const StepCellParams &p, const MotionFeedback &fb,
                       const WallView &wv, uint16_t dtMs)
{
    const bool echo = wv.frontValid;
    const bool fresh = echo && (!sc.haveFront || wv.frontMm != sc.lastFrontMm);
    // A reading that has stood unchanged past a refresh period was met by another echo that agrees
    // (a still robot at 300 mm reads sd 0.0): without it a stopped move never forms a pair.
    if (fresh || !echo) {
        sc.sameMs = 0;
    } else if (sc.sameMs < UINT16_MAX - dtMs) {
        sc.sameMs = static_cast<uint16_t>(sc.sameMs + dtMs);
    }
    if (echo && !fresh && sc.haveFront && !sc.haveWall &&
        static_cast<float>(sc.sameMs) >= 1.5f * p.echoMs) {
        sc.frontAgreed = true;
        sc.wallMm = sc.candidateMm;
        sc.haveWall = true;
    }
    if (fresh) {
        const float candidate = fb.distMm + (static_cast<float>(wv.frontMm) - p.frontStopMm);
        sc.frontAgreed = sc.haveFront && absf(candidate - sc.candidateMm) <= p.frontAgreeMm;
        sc.candidateMm = candidate;
        sc.lastFrontMm = wv.frontMm;
        sc.haveFront = true;
        if (sc.frontAgreed) {
            sc.wallMm = candidate;
            sc.haveWall = true;
        }
    }

    const float believed = p.pitchMm * 0.5f; // a bigger correction is something else
    if (!sc.haveWall) {
        // No pair of echoes agree yet. An echo well clear of the stop point is enough to go on; if
        // not, wait at rest for one, and if none comes the wall is in the dead zone.
        const bool clear = sc.haveFront && sc.lastFrontMm > p.frontStopMm + p.sampleLeadMm;
        if (clear || p.blindMs == 0 || sc.openAhead) {
            return false;
        }
        if (sc.blindMs < UINT16_MAX - dtMs) {
            sc.blindMs = static_cast<uint16_t>(sc.blindMs + dtMs);
        }
        if (sc.blindMs < p.blindMs) {
            return true;
        }
        sc.blocked = true; // no word from the front sensor: a wall in the dead zone, or a dead one
        straightRetarget(sc.straight, fb.distMm, 0.0f, p.plan.accelMmS2);
        return false;
    }

    const float v = sc.straight.refSpeedMmS > 0.0f ? sc.straight.refSpeedMmS : 0.0f;
    const float brakeMm = p.plan.accelMmS2 > 0.0f ? v * v / (2.0f * p.plan.accelMmS2) : 0.0f;
    const bool inside = sc.wallMm <= fb.distMm + brakeMm + v * p.echoMs * 0.001f;
    const bool confirmed = sc.wallsReady && sc.walls.front && sc.lastFrontMm < p.wall.frontMm;
    if (!inside && !confirmed) {
        return false;
    }
    // Only a new agreeing echo moves the smoothed position; between echoes it holds.
    if (!sc.haveAnchor || (fresh && sc.frontAgreed)) {
        sc.anchorMm =
            sc.haveAnchor ? sc.anchorMm + p.anchorFilter * (sc.wallMm - sc.anchorMm) : sc.wallMm;
        sc.haveAnchor = true;
    }
    const float shift = sc.anchorMm - sc.straight.targetMm;
    if (inside && shift < 0.0f) {
        sc.blocked = sc.blocked || shift <= -believed;
        straightRetarget(sc.straight, sc.anchorMm, 0.0f, p.plan.accelMmS2);
    } else if (confirmed && shift > -believed && shift < believed) {
        straightRetarget(sc.straight, sc.anchorMm, 0.0f, p.plan.accelMmS2);
    }
    return false;
}

// The corridor's direction, measured: along one wall mode the lateral error changes at the rate of
// the robot's heading against it, whatever the controller is doing. Fitted over a stretch, that
// slope puts the corridor in the gyro frame, and its distance from the grid heading is the frame's
// error (drift, scale, a turn short of its target). A share of it is handed to the caller as a
// heading correction, and taken out of the steer's bias, which was making up for it.
inline void headingSnap(StepCell &sc, const StepCellParams &p, const MotionFeedback &fb,
                        const WallView &wv)
{
    const float d = fb.distMm - sc.lastDistMm;
    sc.lastDistMm = fb.distMm;
    float err = 0.0f;
    const WallMode mode = sc.steer.mode;
    if (mode == WallMode::None || d <= 0.0f || !wallCenterError(wv, p.wall, err)) {
        trackReset(sc);
        return;
    }
    if (mode != sc.trackMode) {
        trackReset(sc);
        sc.trackMode = mode;
        sc.trackStartMm = fb.distMm;
    }
    const float s = fb.distMm - sc.trackStartMm;
    ++sc.trackN;
    sc.trackS += s;
    sc.trackY += err;
    sc.trackSS += s * s;
    sc.trackSY += s * err;
    sc.trackH += headingWrapDeg(fb.headingDeg - sc.straight.headingRefDeg);
    if (s < p.snapMm || sc.trackN < 10) {
        return;
    }
    const float n = static_cast<float>(sc.trackN);
    const float denom = n * sc.trackSS - sc.trackS * sc.trackS;
    if (denom > 1.0f) {
        const float slope = (n * sc.trackSY - sc.trackS * sc.trackY) / denom; // mm of error per mm
        // Error is positive right of centre, so a heading CCW of the corridor lowers it.
        const float againstCorridorDeg = -atanf(slope) * 57.29578f;
        const float frameErrDeg = sc.trackH / n - againstCorridorDeg;
        if (absf(frameErrDeg) >= p.snapMinDeg && absf(frameErrDeg) <= p.snapMaxDeg) {
            const float fix = p.snapGain * frameErrDeg;
            sc.snapDeg += fix;
            wallSteerShiftFrame(sc.steer, -fix);
            sc.steer.biasDeg -= fix;
        }
    }
    trackReset(sc);
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
            sc.walls.frontHeard = heard(sc.votes[0]);
            sc.walls.leftHeard = heard(sc.votes[1]);
            sc.walls.rightHeard = heard(sc.votes[2]);
            sc.wallsReady = true;
        }
    }

    const bool hold = p.frontStopMm > 0.0f && frontLogic(sc, p, fb, wv, dtMs);

    const float trim = wallSteerTrim(sc.steer, wv, p.wall, fb.headingDeg, dtMs);
    if (hold) {
        out.distRefMm = fb.distMm;
        out.speedRefMmS = 0.0f;
        out.headingRefDeg = headingWrapDeg(sc.straight.headingRefDeg + trim);
        out.yawRateRefDps = 0.0f;
        out.pivot = false;
        out.newMove = true; // nothing is moving: no integral to carry
        return MoveStatus::Running;
    }
    headingSnap(sc, p, fb, wv);
    return straightStep(sc.straight, p.plan, fb, trim, dtMs, out);
}
