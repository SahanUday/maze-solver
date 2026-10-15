// ============================================================================
//  LineFollow.h  --  following the black line on the white bridge tiles, and
//  telling the approach marker and the line zone apart from the bare floor.
//  No hardware, no Arduino.h - testable under env:native.
//
//  The maze floor is BLACK, so "every channel black" is the normal state. On the
//  white bridge tiles the line is the black part and the rest is white.
//    lineView()        one sweep of the array -> where is the line
//    FloorDetector     runs of non-black floor along the path: a short one is the
//                      approach marker, a long one the line zone
//    LineFollow        the follower: steer on the centroid, spin to find a lost
//                      line, stop once the zone has ended
//  docs/architecture/modules/line_follow.md
// ============================================================================

#pragma once

#include <stdint.h>

#include "Heading.h"
#include "MotionProfile.h"
#include "MotionTypes.h"
#include "RobotSpec.h"

// ---- One sweep ----------------------------------------------------------------

struct LineView {
    bool allBlack = false; // every usable channel black: bare maze floor (or the array lifted)
    bool allWhite = false; // no usable channel black: off the line, or a plain white tile
    bool onLine = false;   // some black and some white
    // Centroid of the black channels, mm from the array centre, positive to the LEFT (the CCW
    // side, like heading). Only meaningful when allWhite is false.
    float errMm = 0.0f;
};

// `usable` = bit per channel the calibration accepted; `mask` = bit per black channel.
// `d1OnRight` = channel 0 (D1) is the robot's RIGHT end, as it is on this robot (IR_D1_ON_RIGHT);
// the error is positive to the LEFT either way.
inline LineView lineView(const uint8_t (&norm)[IR_CHANNEL_COUNT], uint8_t mask, uint8_t usable,
                         float pitchMm, bool d1OnRight = false)
{
    LineView v;
    const uint8_t black = static_cast<uint8_t>(mask & usable);
    v.allBlack = usable != 0 && black == usable;
    v.allWhite = black == 0;
    v.onLine = !v.allBlack && !v.allWhite;

    float weight = 0.0f;
    float moment = 0.0f;
    for (uint8_t ch = 0; ch < IR_CHANNEL_COUNT; ++ch) {
        if ((black & (1u << ch)) == 0) {
            continue;
        }
        // Position from the array centre, positive to the left. Channel 0 is D1: the leftmost end
        // unless the array is mounted the other way round.
        const float fromLeft =
            static_cast<float>(IR_CHANNEL_COUNT - 1) * 0.5f - static_cast<float>(ch);
        const float pos = (d1OnRight ? -fromLeft : fromLeft) * pitchMm;
        const float w = static_cast<float>(norm[ch]);
        weight += w;
        moment += w * pos;
    }
    v.errMm = weight > 0.0f ? moment / weight : 0.0f;
    return v;
}

// ---- Marker and line-zone detection -------------------------------------------

enum class FloorEvent : uint8_t {
    None,
    RunStart, // non-black floor began after bare floor; what it turns out to be comes later
    Marker,   // a run of non-black floor as long as the approach marker
    LineZone, // a run longer than any marker: white tiles with a line
};

struct FloorParams {
    float markerMinMm = 0.0f;
    float markerMaxMm = 0.0f;
    float lineEnterMm = 0.0f;
    float gapMm = 0.0f; // bare floor must last this far to end a run
};

struct FloorDetector {
    enum class State : uint8_t {
        WaitFloor, // have not seen bare floor yet: a run in progress at power-up is not measured
        Floor,
        Run,
        InLine, // LineZone already reported; waits for the floor to return
    };
    State state = State::WaitFloor;
    float runStartMm = 0.0f;
    float lastNonBlackMm = 0.0f;
    float blackSinceMm = 0.0f;
};

inline void floorDetectorReset(FloorDetector &d)
{
    d = FloorDetector{};
}

// Feed it the odometer position and whether the array sees bare floor. On a Marker the centre
// of the run, in odometer mm of the array's sensing line, comes back in `centerMm`.
inline FloorEvent floorDetectorStep(FloorDetector &d, bool allBlack, float distMm,
                                    const FloorParams &p, float &centerMm)
{
    switch (d.state) {
        case FloorDetector::State::WaitFloor:
            if (allBlack) {
                d.state = FloorDetector::State::Floor;
            }
            return FloorEvent::None;

        case FloorDetector::State::Floor:
            if (!allBlack) {
                d.state = FloorDetector::State::Run;
                d.runStartMm = distMm;
                d.lastNonBlackMm = distMm;
                d.blackSinceMm = distMm;
                return FloorEvent::RunStart;
            }
            return FloorEvent::None;

        case FloorDetector::State::Run:
            if (!allBlack) {
                d.lastNonBlackMm = distMm;
                d.blackSinceMm = distMm;
                if (distMm - d.runStartMm >= p.lineEnterMm) {
                    d.state = FloorDetector::State::InLine;
                    return FloorEvent::LineZone;
                }
                return FloorEvent::None;
            }
            // Black again. Only a black stretch of gapMm ends the run: a shorter one is a glitch,
            // or the black line crossing a white tile.
            if (distMm - d.blackSinceMm < p.gapMm) {
                return FloorEvent::None;
            }
            d.state = FloorDetector::State::Floor;
            {
                const float len = d.lastNonBlackMm - d.runStartMm;
                if (len >= p.markerMinMm && len <= p.markerMaxMm) {
                    centerMm = (d.runStartMm + d.lastNonBlackMm) * 0.5f;
                    return FloorEvent::Marker;
                }
            }
            return FloorEvent::None;

        case FloorDetector::State::InLine:
            // The zone is over only when bare floor lasts gapMm. The black line crossing a white
            // tile is bare black for a moment too, and the rest of the tile is not a new run.
            if (!allBlack) {
                d.blackSinceMm = distMm;
            } else if (distMm - d.blackSinceMm >= p.gapMm) {
                d.state = FloorDetector::State::Floor;
            }
            return FloorEvent::None;
    }
    return FloorEvent::None;
}

// ---- Following ------------------------------------------------------------------

struct LineParams {
    float kp = 0.0f; // deg of heading offset per mm of line error
    float kd = 0.0f; // ... per mm/s of error rate
    float steerMaxDeg = 0.0f;
    float speedMmS = 0.0f;
    float accelMmS2 = 0.0f;
    float slowdownErrMm = 0.0f; // error at which the speed has dropped to slowdownPct
    uint8_t slowdownPct = 100;
    float searchRateDps = 0.0f;  // spin rate while the line is lost
    float searchSpeedMmS = 0.0f; // forward speed while it searches; 0 = stop and turn on the spot
    uint16_t lostTimeoutMs = 0;  // 0 = search for ever
    // The line is over when it has been gone (white floor, no line) for this far after it was seen.
    // The robot keeps rolling at searchSpeedMmS while it checks, so the distance can add up; 0 =
    // never.
    float endLostMm = 0.0f;
    float rampTiltDeg =
        0.0f; // above this an all-white sweep is ignored (a lifted array); 0 = never
};

enum class LineStatus : uint8_t {
    Following,
    Searching, // line lost, spinning toward the side it was last seen on
    Ended,     // the line ended (gone for endLostMm) and the robot has stopped
    Lost,      // searched for lostTimeoutMs without finding it
};

struct LineFollow {
    enum class Mode : uint8_t { Follow, Search, Ended, Lost };
    Mode mode = Mode::Follow;
    float refSpeedMmS = 0.0f;
    float holdHeadingDeg = 0.0f; // last heading with the line in view
    float filtErrMm = 0.0f;
    bool haveErr = false; // the filter holds a real error
    float side = 1.0f;    // +1 line last seen left, -1 right
    bool sawLine = false;
    float lostRunMm = 0.0f; // travelled over white floor since the line was last seen
    float lastDistMm = 0.0f;
    uint16_t lostMs = 0;
};

constexpr float LINE_ERR_FILTER = 0.5f;
constexpr float LINE_SEARCH_LEAD_DEG = 30.0f; // how far ahead of the robot the search aims
constexpr float LINE_STOPPED_MM_S = 10.0f;    // below this the robot counts as stopped

// `alreadyOnLine`: the line is already under the array, so losing it later can mean its end.
inline void lineFollowBegin(LineFollow &lf, const MotionFeedback &fb, bool alreadyOnLine)
{
    lf = LineFollow{};
    lf.refSpeedMmS = fb.speedMmS;
    lf.holdHeadingDeg = fb.headingDeg;
    lf.lastDistMm = fb.distMm;
    lf.sawLine = alreadyOnLine;
}

// `frontBlocked` is not used any more (it cut the old drive past the line's end short); the callers
// still pass it.
inline LineStatus lineFollowStep(LineFollow &lf, const LineParams &p, const MotionFeedback &fb,
                                 const LineView &lv, float tilt, bool frontBlocked, uint16_t dtMs,
                                 MotionTarget &out)
{
    const float dtS = static_cast<float>(dtMs) / 1000.0f;
    const bool onRamp = p.rampTiltDeg > 0.0f && tilt > p.rampTiltDeg;
    const float travelled = fb.distMm - lf.lastDistMm;
    lf.lastDistMm = fb.distMm;

    (void)frontBlocked;

    // Mode transitions.
    if (lf.mode == LineFollow::Mode::Follow || lf.mode == LineFollow::Mode::Search) {
        if (lv.onLine) {
            lf.mode = LineFollow::Mode::Follow;
            lf.sawLine = true;
            lf.lostRunMm = 0.0f;
            lf.lostMs = 0;
        } else if (lv.allBlack) {
            // Black under the whole array: a lifted array at the ramp's crest, or bare dark floor.
            // Neither is "the line is gone": carry on along the last heading.
            lf.mode = LineFollow::Mode::Follow;
            lf.lostRunMm = 0.0f;
            lf.lostMs = 0;
        } else if (lv.allWhite && !onRamp) {
            lf.mode = LineFollow::Mode::Search;
            lf.lostRunMm += travelled > 0.0f ? travelled : 0.0f;
            lf.lostMs = static_cast<uint16_t>(lf.lostMs + dtMs);
            if (lf.sawLine && p.endLostMm > 0.0f && lf.lostRunMm >= p.endLostMm) {
                lf.mode = LineFollow::Mode::Ended;
            } else if (p.lostTimeoutMs > 0 && lf.lostMs >= p.lostTimeoutMs) {
                lf.mode = LineFollow::Mode::Lost;
            }
        }
    }

    out = MotionTarget{};
    out.distRefMm = fb.distMm; // speed-controlled: no position to chase

    switch (lf.mode) {
        case LineFollow::Mode::Follow: {
            float speed = p.speedMmS;
            float trim = 0.0f;
            if (lv.onLine) {
                const float prev = lf.haveErr ? lf.filtErrMm : lv.errMm;
                lf.filtErrMm = prev + LINE_ERR_FILTER * (lv.errMm - prev);
                lf.haveErr = true;
                const float rate =
                    dtMs > 0 ? (lf.filtErrMm - prev) * 1000.0f / static_cast<float>(dtMs) : 0.0f;
                trim = p.kp * lf.filtErrMm + p.kd * rate;
                trim = trim > p.steerMaxDeg ? p.steerMaxDeg
                                            : (trim < -p.steerMaxDeg ? -p.steerMaxDeg : trim);
                if (lv.errMm > 0.5f || lv.errMm < -0.5f) {
                    lf.side = lv.errMm > 0.0f ? 1.0f : -1.0f;
                }
                // Slow down as the line gets off-centre (a bend): linearly to slowdownPct.
                if (p.slowdownErrMm > 0.0f) {
                    float frac = (lv.errMm < 0.0f ? -lv.errMm : lv.errMm) / p.slowdownErrMm;
                    frac = frac > 1.0f ? 1.0f : frac;
                    speed *= 1.0f - frac * (1.0f - static_cast<float>(p.slowdownPct) / 100.0f);
                }
                lf.holdHeadingDeg = fb.headingDeg;
            } else {
                lf.haveErr = false; // no line in view: carry on along the last good heading
            }
            lf.refSpeedMmS = profileStep(lf.refSpeedMmS, 1e9f, speed, p.accelMmS2, speed, dtS);
            out.speedRefMmS = lf.refSpeedMmS;
            out.headingRefDeg =
                headingWrapDeg((lv.onLine ? fb.headingDeg : lf.holdHeadingDeg) + trim);
            return LineStatus::Following;
        }
        case LineFollow::Mode::Search:
            // Roll on at the search speed. With a search rate, turn toward where the line was last
            // seen; without one, go straight on along the last heading (a line that just ends).
            lf.refSpeedMmS = profileStep(lf.refSpeedMmS, 1e9f, p.searchSpeedMmS, p.accelMmS2,
                                         p.searchSpeedMmS, dtS);
            out.speedRefMmS = lf.refSpeedMmS;
            if (p.searchRateDps > 0.0f) {
                out.headingRefDeg = headingWrapDeg(fb.headingDeg + lf.side * LINE_SEARCH_LEAD_DEG);
                out.yawRateRefDps = lf.side * p.searchRateDps;
            } else {
                out.headingRefDeg = headingWrapDeg(lf.holdHeadingDeg);
            }
            return LineStatus::Searching;

        case LineFollow::Mode::Ended:
            // The line is over: brake to a stop along the last heading.
            lf.refSpeedMmS = profileStep(lf.refSpeedMmS, 1e9f, 0.0f, p.accelMmS2, 0.0f, dtS);
            out.speedRefMmS = lf.refSpeedMmS;
            out.headingRefDeg = headingWrapDeg(lf.holdHeadingDeg);
            if (lf.refSpeedMmS == 0.0f && fb.speedMmS < LINE_STOPPED_MM_S &&
                fb.speedMmS > -LINE_STOPPED_MM_S) {
                return LineStatus::Ended;
            }
            return LineStatus::Searching;

        case LineFollow::Mode::Lost:
            break;
    }
    out.headingRefDeg = fb.headingDeg; // lost: do nothing; the mission decides
    return LineStatus::Lost;
}
