// ============================================================================
//  LineFollow.h  --  following the black line on the white bridge tiles. No hardware, no
//  Arduino.h - testable under env:native.
//
//  The line view (LineView.h) says where the line is; this steers on it: the line error becomes a
//  heading offset from the CURRENT heading, so bends are followed. A line that is lost is searched
//  for by turning toward the side it was last seen on.
//  docs/architecture/modules/line_follow.md
// ============================================================================

#pragma once

#include <stdint.h>

#include "motion/Heading.h"
#include "sense/LineView.h"
#include "motion/MotionProfile.h"
#include "motion/MotionTypes.h"

struct LineParams {
    float kp = 0.0f; // deg of heading offset per mm of line error
    float kd = 0.0f; // ... per mm/s of error rate
    float steerMaxDeg = 0.0f;
    float speedMmS = 0.0f;
    float accelMmS2 = 0.0f;
    float slowdownErrMm = 0.0f; // error at which the speed has dropped to slowdownPct; 0 = never
    uint8_t slowdownPct = 100;
    float searchRateDps = 0.0f;  // spin rate while the line is lost; 0 = straight on
    float searchSpeedMmS = 0.0f; // forward speed while it searches; 0 = stop and turn on the spot
    uint16_t lostTimeoutMs = 0;  // searched this long without finding it: Lost; 0 = for ever
    // The line is over when it has been gone (white floor, no line) for this far after it was
    // seen. The robot keeps rolling at searchSpeedMmS while it checks, so the distance adds up.
    // 0 = never.
    float endLostMm = 0.0f;
    // Above this nose-up slope (accel counts, Tilt.h) the array may be lifted off the floor, and an
    // all-white view is ignored; 0 = never.
    int16_t rampInclineCounts = 0;
};

enum class LineStatus : uint8_t {
    Following,
    Searching, // line lost, turning toward the side it was last seen on
    Ended,     // gone for endLostMm and the robot has stopped
    Lost,      // searched for lostTimeoutMs without finding it
};

struct LineFollow {
    enum class Mode : uint8_t { Follow, Search, Ended, Lost };
    Mode mode = Mode::Follow;
    float refSpeedMmS = 0.0f;
    float holdHeadingDeg = 0.0f; // last heading with the line in view
    float filtErrMm = 0.0f;
    bool haveErr = false;
    float side = 1.0f; // +1 line last seen left, -1 right
    bool sawLine = false;
    float lostRunMm = 0.0f; // travelled over white floor since the line was last seen
    float lastDistMm = 0.0f;
    uint16_t lostMs = 0;
    bool fresh = true;
};

constexpr float LINE_ERR_FILTER = 0.5f;
constexpr float LINE_SEARCH_LEAD_DEG = 30.0f; // how far ahead of the robot the search aims
constexpr float LINE_STOPPED_MM_S = 10.0f;    // below this the robot counts as stopped

// `alreadyOnLine`: the line is under the array, so losing it later can mean its end.
inline void lineFollowBegin(LineFollow &lf, const MotionFeedback &fb, bool alreadyOnLine)
{
    lf = LineFollow{};
    lf.refSpeedMmS = fb.speedMmS;
    lf.holdHeadingDeg = fb.headingDeg;
    lf.lastDistMm = fb.distMm;
    lf.sawLine = alreadyOnLine;
}

inline LineStatus lineFollowStep(LineFollow &lf, const LineParams &p, const MotionFeedback &fb,
                                 const LineView &lv, int16_t inclineCounts, uint16_t dtMs,
                                 MotionTarget &out)
{
    const float dtS = static_cast<float>(dtMs) / 1000.0f;
    const bool onRamp = p.rampInclineCounts > 0 && inclineCounts > p.rampInclineCounts;
    const float travelled = fb.distMm - lf.lastDistMm;
    lf.lastDistMm = fb.distMm;

    if (lf.mode == LineFollow::Mode::Follow || lf.mode == LineFollow::Mode::Search) {
        if (lv.onLine) {
            lf.mode = LineFollow::Mode::Follow;
            lf.sawLine = true;
            lf.lostRunMm = 0.0f;
            lf.lostMs = 0;
        } else if (lv.allBlack || !lv.valid) {
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
    out.newMove = lf.fresh;
    lf.fresh = false;

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
            // seen; without one, go straight on along the last heading.
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
