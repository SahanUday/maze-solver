// ============================================================================
//  MissionManager.h  --  the run, part by part. Pure logic, no hardware: testable under env:native.
//
//    WaitStart -> Running (the parts from `first` to `last`, in course order) -> Done; any -> Fault
//
//  The manager starts the run, moves on when a part reports its end and makes the one
//  motionControlStep() call: a part only fills a MotionTarget. A part ends with Done (the next
//  begins on the next tick) or HandOver (it ended while moving on: the next acts in the same tick,
//  so nothing stops in between). No run ends on time. Parts: Parts.h.
//  docs/architecture/modules/mission_manager.md
// ============================================================================

#pragma once

#include <stdint.h>

#include "mission/CellExplorer.h"
#include "mission/Fault.h"
#include "mission/MarkerExit.h"
#include "mission/MissionParams.h"
#include "mission/Parts.h"
#include "mission/RunStart.h"
#include "motion/DriveCommand.h"
#include "motion/LineFollow.h"
#include "motion/MotionControl.h"
#include "sense/Perception.h"
#include "spec/tuning/Run.h"

// The status line's phase numbers: Approach and Line come last so the older numbers stay.
enum class MissionPhase : uint8_t { WaitStart, Sense, Turn, Step, Done, Fault, Approach, Line };

enum class RunStage : uint8_t { WaitStart, Running, Done, Fault };

struct MissionManager {
    MissionPhase phase = MissionPhase::WaitStart; // as the status line numbers it
    MissionFault fault = MissionFault::None;
    RunStage stage = RunStage::WaitStart;
    Part part = Part::AExplore;     // running, or the last one that ran
    Part lastPart = Part::AExplore; // where this run ends, once started
    RunStart start;

    CellExplorer explore;
    MarkerExit marker;
    LineFollow line;
    MotionControl control;
};

inline void missionReset(MissionManager &ms)
{
    ms = MissionManager{};
    cellExplorerReset(ms.explore);
    motionControlReset(ms.control);
}

// A precondition was missing before the run, or a part gave up.
inline void missionFail(MissionManager &ms, MissionFault code)
{
    ms.stage = RunStage::Fault;
    ms.phase = MissionPhase::Fault;
    ms.fault = code;
}

// Degrees to subtract from the gyro heading, once (headingSet(hd, hd.deg - fix)); see StepCell.h.
inline float missionTakeHeadingFix(MissionManager &ms)
{
    const float fix = ms.explore.headingFixDeg;
    ms.explore.headingFixDeg = 0.0f;
    return fix;
}

namespace mission_detail
{

enum class PartStatus : uint8_t {
    Running,
    Done,     // the next part begins on the next tick
    HandOver, // ended while moving on: the next part acts in this tick
    DoneHold, // ended; hold the wheels with no control step this tick
    Fault,    // `fault` says why
};

inline MissionPhase phaseOf(const MissionManager &ms)
{
    switch (ms.stage) {
        case RunStage::WaitStart:
            return MissionPhase::WaitStart;
        case RunStage::Running:
            switch (ms.part) {
                case Part::AExplore:
                case Part::AProve:
                    switch (ms.explore.phase) {
                        case ExplorePhase::Sense:
                            return MissionPhase::Sense;
                        case ExplorePhase::Turn:
                            return MissionPhase::Turn;
                        case ExplorePhase::Step:
                            return MissionPhase::Step;
                    }
                    break;
                case Part::AExit:
                    switch (ms.marker.phase) {
                        case ExitPhase::Stand:
                            return MissionPhase::Sense;
                        case ExitPhase::Turn:
                            return MissionPhase::Turn;
                        case ExitPhase::Approach:
                            return MissionPhase::Approach;
                    }
                    break;
                case Part::Line:
                    return MissionPhase::Line;
                case Part::BEntry:
                case Part::BExplore:
                case Part::BProve:
                    break;
            }
            break;
        case RunStage::Done:
            return MissionPhase::Done;
        case RunStage::Fault:
            break;
    }
    return MissionPhase::Fault;
}

// The run's first and last part; false if the selection cannot run (the first is not built or
// cannot start alone, or the last is before it). Without a marker to find (wallsOnly) it ends with
// AExplore.
inline bool resolveRange(const MissionParams &p, Part &first, Part &last)
{
    first = p.parts.first;
    if (!partBuilt(first) || !partStartsAlone(first)) {
        return false;
    }
    Part cap = p.parts.last;
    if (p.wallsOnly && static_cast<uint8_t>(cap) > static_cast<uint8_t>(Part::AExplore)) {
        cap = Part::AExplore;
    }
    return partLastBuilt(cap, last) && static_cast<uint8_t>(last) >= static_cast<uint8_t>(first);
}

// Drive straight on along the map heading, over the strip and the gap.
inline void beginApproach(MissionManager &ms, const MissionParams &p, const MotionFeedback &fb)
{
    markerExitBeginApproach(ms.marker, p, fb, cellExplorerHeadingRef(ms.explore));
}

// Begin `next` from where the robot stands.
inline void partBegin(MissionManager &ms, Part next, const MissionParams &p,
                      const MotionFeedback &fb)
{
    ms.part = next;
    switch (next) {
        case Part::AExplore:
            cellExplorerStart(ms.explore, fb);
            break;
        case Part::AExit:
            if (cellExplorerEdgeOpen(ms.explore, ms.explore.map.heading)) {
                beginApproach(ms, p, fb);
            } else {
                // A wall ahead: the way out is to a side. Stand and read both sides first.
                markerExitBeginStand(ms.marker);
            }
            break;
        case Part::Line:
            lineFollowBegin(ms.line, fb, true);
            break;
        case Part::AProve:
        case Part::BEntry:
        case Part::BExplore:
        case Part::BProve:
            break; // not built: resolveRange() keeps a run out of them
    }
}

// Section A's maze. Done on the marker's tile; HandOver as soon as the marker is told when the way
// out is straight on and AExit follows (no stop).
inline PartStatus runAExplore(MissionManager &ms, const MissionParams &p, const Perception &pc,
                              MotionTarget &target)
{
    const MotionFeedback &fb = pc.motion;
    Part next = Part::AExplore;
    const bool hasNext = partAfter(Part::AExplore, ms.lastPart, next);
    const bool exitFollows = hasNext && next == Part::AExit;

    // The marker is told only while stepping, and not without a calibration.
    ExploreInput in;
    in.brakeOnStop = !hasNext;
    float lateCentreMm = 0.0f;
    const bool looking =
        !p.wallsOnly && ms.explore.phase == ExplorePhase::Step && !ms.marker.atMarker;
    if (looking && markerWatch(ms.marker, pc, p.pattern)) {
        cellExplorerStopHere(ms.explore);
        if (exitFollows && cellExplorerEdgeOpen(ms.explore, ms.explore.map.heading)) {
            return PartStatus::HandOver; // straight through: no stop
        }
    } else if (looking) {
        in.lateStopOffer = markerPending(ms.marker, fb, p.pattern, lateCentreMm);
    }

    const ExploreStatus st = cellExplorerStep(ms.explore, p, in, pc, target);
    if (ms.explore.lateTaken) {
        ms.explore.lateTaken = false;
        markerTaken(ms.marker, lateCentreMm);
    }
    switch (st) {
        case ExploreStatus::Running:
            return PartStatus::Running;
        case ExploreStatus::Explored: // every reachable tile has been entered
        case ExploreStatus::AtTarget: // no part drives to a target yet
        case ExploreStatus::Stopped:
            return PartStatus::Done;
        case ExploreStatus::Braked:
            return PartStatus::DoneHold;
        case ExploreStatus::Fault:
            ms.fault = ms.explore.fault;
            return PartStatus::Fault;
    }
    return PartStatus::Fault;
}

// The marker tile and the way out of it. HandOver when the line is in view past the gap.
inline PartStatus runAExit(MissionManager &ms, const MissionParams &p, const Perception &pc,
                           MotionTarget &target)
{
    const MotionFeedback &fb = pc.motion;
    switch (ms.marker.phase) {
        case ExitPhase::Stand: {
            const maze::Dir left = maze::turnLeft(ms.explore.map.heading);
            const maze::Dir right = maze::turnRight(ms.explore.map.heading);
            const ExitChoice c = markerExitStandStep(ms.marker, pc, p.senseMs, p.dtMs,
                                                     cellExplorerEdgeOpen(ms.explore, left),
                                                     cellExplorerEdgeOpen(ms.explore, right));
            if (c == ExitChoice::NoWay) {
                ms.fault = MissionFault::NoLine; // no way out of the tile
                return PartStatus::Fault;
            }
            if (c != ExitChoice::Wait) {
                ms.marker.sideExit = true;
                ms.marker.phase = ExitPhase::Turn;
                cellExplorerBeginTurn(ms.explore, fb, c == ExitChoice::Left ? left : right);
            }
            return PartStatus::Running;
        }
        case ExitPhase::Turn: {
            const TurnStatus ts = cellExplorerTurnStep(ms.explore, p, fb, target);
            if (ts == TurnStatus::Fault) {
                ms.fault = ms.explore.fault;
                return PartStatus::Fault;
            }
            if (ts == TurnStatus::Done) {
                beginApproach(ms, p, fb); // facing the open edge: out of the maze
            }
            return PartStatus::Running;
        }
        case ExitPhase::Approach: {
            const ApproachStatus as =
                markerExitApproachStep(ms.marker, p, p.step.plan, pc, p.dtMs, target);
            if (as == ApproachStatus::HandOver) {
                return PartStatus::HandOver;
            }
            if (as == ApproachStatus::NoLine) {
                ms.fault = MissionFault::NoLine;
                return PartStatus::Fault;
            }
            return PartStatus::Running;
        }
    }
    return PartStatus::Fault;
}

// The line zone. Done when the line is over and the robot has stopped.
inline PartStatus runLine(MissionManager &ms, const MissionParams &p, const Perception &pc,
                          MotionTarget &target)
{
    const LineStatus ls =
        lineFollowStep(ms.line, p.line, pc.motion, pc.line, pc.inclineCounts, p.dtMs, target);
    if (ls == LineStatus::Lost) {
        ms.fault = MissionFault::LineLost;
        return PartStatus::Fault;
    }
    return ls == LineStatus::Ended ? PartStatus::Done : PartStatus::Running;
}

inline PartStatus runPart(MissionManager &ms, const MissionParams &p, const Perception &pc,
                          MotionTarget &target)
{
    switch (ms.part) {
        case Part::AExplore:
            return runAExplore(ms, p, pc, target);
        case Part::AExit:
            return runAExit(ms, p, pc, target);
        case Part::Line:
            return runLine(ms, p, pc, target);
        case Part::AProve:
        case Part::BEntry:
        case Part::BExplore:
        case Part::BProve:
            break;
    }
    ms.fault = MissionFault::BadConfig; // not built
    return PartStatus::Fault;
}

inline DriveCommand tick(MissionManager &ms, const MissionParams &p, const Perception &pc)
{
    const MotionFeedback &fb = pc.motion;
    MotionTarget target;
    target.distRefMm = fb.distMm; // default: stand still
    // Parts with a map square to its directions; the others hold the heading they have.
    target.headingRefDeg = ms.stage == RunStage::WaitStart || !partUsesMap(ms.part)
                               ? fb.headingDeg
                               : cellExplorerHeadingRef(ms.explore);
    target.newMove = true;

    DriveCommand cmd;
    switch (ms.stage) {
        case RunStage::WaitStart: {
            if (ms.start.started) {
                Part first = Part::AExplore;
                Part last = Part::AExplore;
                if (!resolveRange(p, first, last)) {
                    missionFail(ms, MissionFault::BadConfig);
                    return DriveCommand{};
                }
                ms.lastPart = last;
                partBegin(ms, first, p, fb);
                ms.stage = RunStage::Running;
            }
            return cmd; // motors off until the run starts
        }

        case RunStage::Running: {
            cmd.enable = true;
            PartStatus st = runPart(ms, p, pc, target);
            for (uint8_t hops = 0;
                 hops < PART_COUNT && (st == PartStatus::Done || st == PartStatus::HandOver);
                 ++hops) {
                Part next = Part::AExplore;
                if (!partAfter(ms.part, ms.lastPart, next)) {
                    ms.stage = RunStage::Done; // the last part of the run has ended
                    break;
                }
                const bool movingOn = st == PartStatus::HandOver;
                partBegin(ms, next, p, fb);
                if (!movingOn) {
                    break;
                }
                st = runPart(ms, p, pc, target); // the next part acts in this tick
            }
            if (st == PartStatus::Fault) {
                missionFail(ms, ms.fault);
                return DriveCommand{};
            }
            if (st == PartStatus::DoneHold) {
                ms.stage = RunStage::Done;
                DriveCommand hold;
                hold.enable = true; // enabled at zero duty: holds the wheels
                return hold;
            }
            break;
        }

        case RunStage::Done:
            cmd.enable = true; // holds the wheels
            break;

        case RunStage::Fault:
            return cmd;
    }

    const MotionOutput out = motionControlStep(ms.control, p.control, target, fb, p.dtMs);
    if (ms.control.stalled) {
        missionFail(ms, MissionFault::Stall);
        return DriveCommand{};
    }
    cmd.left = out.left;
    cmd.right = out.right;
    return cmd;
}

} // namespace mission_detail

// One tick without the start latch, for a caller that sets `ms.start.started` itself.
inline DriveCommand missionStep(MissionManager &ms, const MissionParams &p, const Perception &pc)
{
    const DriveCommand cmd = mission_detail::tick(ms, p, pc);
    if (ms.stage != RunStage::Fault) {
        ms.phase = mission_detail::phaseOf(ms);
    }
    return cmd;
}

// One tick of a run: the start, then the parts. `go` is the operator's start.
inline DriveCommand missionTick(MissionManager &ms, const MissionParams &p, const Perception &pc,
                                bool go)
{
    runStartTick(ms.start, go);
    return missionStep(ms, p, pc);
}
