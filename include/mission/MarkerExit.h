// ============================================================================
//  MarkerExit.h  --  from the approach marker out of the maze: tell the marker, find the way out
//  of its tile, drive on over the strip and the gap, and hand over to the line follower. No
//  hardware, no Arduino.h - testable under env:native.
//
//  The marker (white frame, black box, white frame under the array) is a floor pattern read while
//  the robot steps. On its tile the way out is straight on or to a side: with a wall ahead the
//  robot stands and reads both sides (Stand), turns to the one that answers from far or not at all
//  (Turn), then drives on (Approach) until the first line view past the gap.
//  docs/architecture/modules/marker_exit.md
// ============================================================================

#pragma once

#include <stdint.h>

#include "Maze.h"
#include "motion/MotionPrimitives.h"
#include "motion/MotionTypes.h"
#include "sense/FloorPattern.h"
#include "sense/LineView.h"
#include "sense/Perception.h"
#include "spec/fixed/Hardware.h"
#include "spec/measurements/Robot.h"
#include "spec/tuning/LineFollow.h"
#include "spec/tuning/MarkerExit.h"

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

enum class ExitPhase : uint8_t { Stand, Turn, Approach };

enum class ApproachStatus : uint8_t { Running, HandOver, NoLine };

enum class ExitChoice : uint8_t { Wait, Left, Right, NoWay };

struct MarkerExitParams {
    PatternParams pattern;
    // After the marker: drive on at approachSpeedMmS through the gap, hand over to the follower at
    // the first line past it, give up if there is none within noLineMm of the marker's centre.
    float approachSpeedMmS = LINE_SPEED_MM_S;
    float noLineMm = APPROACH_NO_LINE_MM;
    float handoverMinMm = BRIDGE_HANDOVER_MIN_MM;
    // The gap may not read as air on this robot (it read 915-1010 on the real course, below the
    // black floor's 1009-1020): a line view this far past the marker's box centre is the line.
    float handoverNoGapMm = BRIDGE_HANDOVER_NO_GAP_MM;
};

struct MarkerExit {
    ExitPhase phase = ExitPhase::Stand;
    FloorPattern floor;
    bool atMarker = false;       // the marker has been seen
    float markerCentreMm = 0.0f; // the black box's middle, on the odometer of the array's line
    bool sideExit = false;       // the way out was to a side of the marker's tile
    // Standing on the marker's tile to find the way out: the side readings, a silent one counted
    // as the sensor's range, sampled every other tick.
    uint16_t standMs = 0;
    uint16_t exitMm[2][16] = {{0}, {0}};
    uint8_t exitN = 0;
    Straight approach;
    BridgeHandover bridge;
    bool gapSeen = false;
};

namespace marker_exit_detail
{

// Median of the first n of v (sorted in place).
inline uint16_t medianOf(uint16_t *v, uint8_t n)
{
    for (uint8_t i = 1; i < n; ++i) {
        const uint16_t x = v[i];
        uint8_t j = i;
        while (j > 0 && v[j - 1] > x) {
            v[j] = v[j - 1];
            --j;
        }
        v[j] = x;
    }
    return n == 0 ? 0 : v[n / 2];
}

} // namespace marker_exit_detail

// The array's sensing line on the odometer.
inline float markerArrayPosMm(const MotionFeedback &fb)
{
    return fb.distMm + IR_ARRAY_FORWARD_OFFSET_MM;
}

// Feed one step tick to the pattern. True on the tick the marker is told (the far frame has been
// crossed, which puts the robot's axle about at the tile's centre). Only while stepping: the
// pattern is a length along the path, and a pivot at a centre sweeps the array across the frame
// with the odometer standing still (or creeping by a wheel's slip).
inline bool markerWatch(MarkerExit &me, const Perception &pc, const PatternParams &pp)
{
    float centre = 0.0f;
    if (floorPatternStep(me.floor, pc.floor, markerArrayPosMm(pc.motion), pp, centre) ==
        PatternEvent::Marker) {
        me.markerCentreMm = centre;
        me.atMarker = true;
        return true;
    }
    return false;
}

// The marker is under the array as far as the path goes, for a robot that stopped a little short of
// the tile's centre (FloorPattern.h). `centreMm` is the box's middle; the caller makes it the
// marker's with markerTaken().
inline bool markerPending(const MarkerExit &me, const MotionFeedback &fb, const PatternParams &pp,
                          float &centreMm)
{
    return floorPatternMarkerPending(me.floor, pp, markerArrayPosMm(fb), centreMm);
}

inline void markerTaken(MarkerExit &me, float centreMm)
{
    me.markerCentreMm = centreMm;
    me.atMarker = true;
}

// Drive on from here, straight along `headingDeg`, over the strip and the gap. The pattern starts
// afresh: what the array has seen so far belongs to the marker.
inline void markerExitBeginApproach(MarkerExit &me, const MarkerExitParams &p,
                                    const MotionFeedback &fb, float headingDeg)
{
    straightBegin(me.approach, fb, 1000.0f, p.approachSpeedMmS, 0.0f, headingDeg);
    bridgeHandoverReset(me.bridge);
    floorPatternReset(me.floor);
    me.gapSeen = false;
    me.phase = ExitPhase::Approach;
}

// Stand on the marker's tile and read both sides.
inline void markerExitBeginStand(MarkerExit &me)
{
    me.phase = ExitPhase::Stand;
    me.standMs = 0;
    me.exitN = 0;
}

// One tick standing. Once `senseMs` have passed, both sides may be unwalled: the other tile of the
// maze, and the way out. The way out is the one that answers from far or not at all: a tile's far
// wall is within 2.5 m, the room beyond the exit is not. Medians, for the dropouts. `leftOpen` and
// `rightOpen`: the map holds no wall on that edge of the tile.
inline ExitChoice markerExitStandStep(MarkerExit &me, const Perception &pc, uint16_t senseMs,
                                      uint16_t dtMs, bool leftOpen, bool rightOpen)
{
    me.standMs = static_cast<uint16_t>(me.standMs + dtMs);
    if (me.exitN < 16 && ((me.standMs / dtMs) & 1u) != 0) {
        me.exitMm[0][me.exitN] = pc.walls.leftValid ? pc.walls.leftMm : US_RANGE_CAP_MM;
        me.exitMm[1][me.exitN] = pc.walls.rightValid ? pc.walls.rightMm : US_RANGE_CAP_MM;
        ++me.exitN;
    }
    if (me.standMs < senseMs) {
        return ExitChoice::Wait;
    }
    if (!leftOpen && !rightOpen) {
        return ExitChoice::NoWay;
    }
    const uint16_t leftMm = marker_exit_detail::medianOf(me.exitMm[0], me.exitN);
    const uint16_t rightMm = marker_exit_detail::medianOf(me.exitMm[1], me.exitN);
    const bool toLeft = leftOpen && (!rightOpen || leftMm >= rightMm);
    return toLeft ? ExitChoice::Left : ExitChoice::Right;
}

// One tick of the drive over the gap. HandOver: the line is in view past it, and the follower takes
// over, mid-stride. NoLine: none within `noLineMm` of the marker's centre.
inline ApproachStatus markerExitApproachStep(MarkerExit &me, const MarkerExitParams &p,
                                             const PlanParams &plan, const Perception &pc,
                                             uint16_t dtMs, MotionTarget &target)
{
    const MotionFeedback &fb = pc.motion;
    const float pos = markerArrayPosMm(fb);
    float centre = 0.0f;
    if (floorPatternStep(me.floor, pc.floor, pos, p.pattern, centre) == PatternEvent::Gap) {
        me.gapSeen = true;
    }
    const bool pastTheGap = pos - me.markerCentreMm >= p.handoverNoGapMm && pc.line.onLine &&
                            pc.floor == FloorClass::Mixed;
    if (bridgeHandoverStep(me.bridge, me.gapSeen, pc.floor, pc.line, pos, p.handoverMinMm) ||
        pastTheGap) {
        return ApproachStatus::HandOver;
    }
    if (pos - me.markerCentreMm >= p.noLineMm) {
        return ApproachStatus::NoLine;
    }
    straightStep(me.approach, plan, fb, 0.0f, dtMs, target);
    return ApproachStatus::Running;
}
