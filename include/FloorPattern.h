// ============================================================================
//  FloorPattern.h  --  what the IR array sees as the robot drives: the approach marker and the gap
//  before the bridge. No hardware, no Arduino.h - testable under env:native.
//
//  The marker is a white frame around a black box (measured on the robot: white ~36 mm, black
//  ~118 mm, white ~37 mm along the path), so crossing it gives white - black - white. After the
//  maze's exit the floor ends in a gap (~37 mm) before the ramp: every channel reads the top of
//  the ADC scale, because nothing is under the array.
//    classifyFloor()   one sweep -> Black / White / Air / Mixed
//    FloorPattern      runs of those along the path -> Marker, Gap
//  docs/architecture/modules/floor_pattern.md
// ============================================================================

#pragma once

#include <stdint.h>

#include "LineFollow.h"
#include "RobotSpec.h"

enum class FloorClass : uint8_t {
    Black, // bare maze floor (or a marker's black box)
    White, // every usable channel white
    Air,   // nothing under the array: a gap, or the array lifted clear of the floor
    Mixed, // an edge or the line: part black, part white
};

// `airCounts` and `airChannels`: a sweep is Air when at least `airChannels` of the channels read
// `airCounts` or more. The black floor reads ~1000 and the black tape ~1020 on a few channels, so
// it takes most of the array above ~1010 at once to be air. airCounts == 0 turns Air off.
inline FloorClass classifyFloor(const LineView &v, const uint16_t (&raw)[IR_CHANNEL_COUNT],
                                uint16_t airCounts, uint8_t airChannels)
{
    if (airCounts > 0) {
        uint8_t high = 0;
        for (uint8_t ch = 0; ch < IR_CHANNEL_COUNT; ++ch) {
            if (raw[ch] >= airCounts) {
                ++high;
            }
        }
        if (high >= airChannels) {
            return FloorClass::Air;
        }
    }
    if (v.allBlack) {
        return FloorClass::Black;
    }
    if (v.allWhite) {
        return FloorClass::White;
    }
    return FloorClass::Mixed;
}

// After the gap, the robot hands over to the line follower. Not at the first sweep that is not air:
// the dark patch just past the gap reads black, and the gap's trailing edge can show a false line.
// It waits until the array has been off the gap for `minMm`, and then takes the first real line
// view.
struct BridgeHandover {
    float offGapSinceMm = -1.0f; // array position where the floor stopped being air; -1 = on it
};

inline void bridgeHandoverReset(BridgeHandover &b)
{
    b.offGapSinceMm = -1.0f;
}

inline bool bridgeHandoverStep(BridgeHandover &b, bool gapSeen, FloorClass floor, const LineView &v,
                               float posMm, float minMm)
{
    if (!gapSeen) {
        return false;
    }
    if (floor == FloorClass::Air) {
        b.offGapSinceMm = -1.0f;
        return false;
    }
    if (b.offGapSinceMm < 0.0f) {
        b.offGapSinceMm = posMm;
    }
    return posMm - b.offGapSinceMm >= minMm && v.onLine;
}

enum class PatternEvent : uint8_t {
    None,
    Marker, // white, black, white of the marker's proportions; centreMm is the middle of the black
    Gap,    // air for at least gapMinMm
};

struct PatternParams {
    float frameMinMm = 0.0f; // each white frame of the marker
    float frameMaxMm = 0.0f;
    float centreMinMm = 0.0f; // the black box between them
    float centreMaxMm = 0.0f;
    float gapMinMm = 0.0f;
    float glitchMm = 4.0f; // a change shorter than this is noise, not a new run
};

struct PatternRun {
    FloorClass cls = FloorClass::Black;
    float startMm = 0.0f;
    float endMm = 0.0f;
};

struct FloorPattern {
    // The last three finished runs, oldest first, and the one in progress.
    PatternRun done[3];
    uint8_t doneCount = 0;
    FloorClass cls = FloorClass::Black; // the accepted class of the run in progress
    float runStartMm = 0.0f;
    bool started = false;
    // A different class seen but not yet accepted.
    FloorClass pending = FloorClass::Black;
    float pendingStartMm = 0.0f;
    bool havePending = false;
    bool markerSent = false; // for the run in progress
    bool gapSent = false;
};

inline void floorPatternReset(FloorPattern &f)
{
    f = FloorPattern{};
}

namespace pattern_detail
{

inline void pushDone(FloorPattern &f, FloorClass cls, float startMm, float endMm)
{
    f.done[0] = f.done[1];
    f.done[1] = f.done[2];
    f.done[2].cls = cls;
    f.done[2].startMm = startMm;
    f.done[2].endMm = endMm;
    if (f.doneCount < 3) {
        ++f.doneCount;
    }
}

inline bool within(float v, float lo, float hi)
{
    return v >= lo && v <= hi;
}

} // namespace pattern_detail

// Feed one sweep: the class and the odometer position of the array's sensing line. A Mixed sweep
// (an edge, the line) never changes the run in progress. On a Marker, `centreMm` is the middle of
// the marker's black box.
inline PatternEvent floorPatternStep(FloorPattern &f, FloorClass seen, float distMm,
                                     const PatternParams &p, float &centreMm)
{
    using namespace pattern_detail;
    if (!f.started) {
        if (seen == FloorClass::Mixed) {
            return PatternEvent::None;
        }
        f.started = true;
        f.cls = seen;
        f.runStartMm = distMm;
        return PatternEvent::None;
    }

    if (seen != FloorClass::Mixed && seen != f.cls) {
        if (!f.havePending || f.pending != seen) {
            f.pending = seen;
            f.pendingStartMm = distMm;
            f.havePending = true;
        }
        if (distMm - f.pendingStartMm >= p.glitchMm) {
            // The change is real: the old run ended where the new class first appeared.
            pushDone(f, f.cls, f.runStartMm, f.pendingStartMm);
            f.cls = f.pending;
            f.runStartMm = f.pendingStartMm;
            f.havePending = false;
            f.markerSent = false;
            f.gapSent = false;
        }
    } else if (seen == f.cls) {
        f.havePending = false; // a glitch that went back
    }

    const float length = distMm - f.runStartMm;

    if (f.cls == FloorClass::Air && !f.gapSent && length >= p.gapMinMm) {
        f.gapSent = true;
        return PatternEvent::Gap;
    }

    // The far frame is under way and already long enough: white, black, white.
    if (f.cls == FloorClass::White && !f.markerSent && f.doneCount >= 2 && length >= p.frameMinMm &&
        length <= p.frameMaxMm) {
        const PatternRun &black = f.done[2];
        const PatternRun &near = f.done[1];
        if (black.cls == FloorClass::Black && near.cls == FloorClass::White &&
            within(black.endMm - black.startMm, p.centreMinMm, p.centreMaxMm) &&
            within(near.endMm - near.startMm, p.frameMinMm, p.frameMaxMm)) {
            f.markerSent = true;
            centreMm = (black.startMm + black.endMm) * 0.5f;
            return PatternEvent::Marker;
        }
    }
    return PatternEvent::None;
}
