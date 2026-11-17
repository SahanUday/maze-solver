// ============================================================================
//  MissionManager.h  --  the run, part by part. No hardware, no Arduino.h - testable under
//  env:native.
//
//    WaitStart -> Running (one part at a time, in course order) -> Done
//                 any -> Fault
//
//  The parts are in Parts.h: Section A's maze (CellExplorer), the way out of the marker tile
//  (MarkerExit) and the line zone (LineFollow) are built. The manager starts the run, runs the
//  parts from `first` to `last` (the whole course, one part alone, or any stretch in between),
//  moves on when a part reports its own end event, enforces the run limit, and makes the one
//  motionControlStep() call: the parts only fill a MotionTarget.
//
//  A part ends in one of two ways. Done: the next part begins on the next tick. HandOver: the part
//  ended while the robot was moving on, and the next part acts in the same tick, so nothing
//  stops in between.
//  docs/architecture/modules/mission_manager.md
// ============================================================================

#pragma once

#include <stdint.h>

#include "mission/CellExplorer.h"
#include "mission/Fault.h"
#include "mission/MarkerExit.h"
#include "mission/Parts.h"
#include "mission/RunStart.h"
#include "motion/DriveCommand.h"
#include "motion/LineFollow.h"
#include "motion/MotionControl.h"
#include "motion/MotionPrimitives.h"
#include "sense/Perception.h"
#include "spec/Drivetrain.h"
#include "spec/FloorPattern.h"
#include "spec/Imu.h"
#include "spec/LineFollow.h"
#include "spec/LineSensor.h"
#include "spec/Mission.h"
#include "spec/Motion.h"
#include "spec/Platform.h"

// What the status line prints. Approach and Line come last so the numbers the log prints for the
// older phases stay.
enum class MissionPhase : uint8_t { WaitStart, Sense, Turn, Step, Done, Fault, Approach, Line };

enum class RunStage : uint8_t { WaitStart, Running, Done, Fault };

struct MissionParams : CellExplorerParams, MarkerExitParams {
    PartRange parts; // which parts to run: the whole course by default
    MotionControlParams control;
    LineParams line;
    // The marker is told by the floor sensor; without it (the array not calibrated) the run
    // explores every reachable tile and stops (wallsOnly, in CellExplorerParams).
};

// Every parameter at its spec/ value: what the robot runs with.
inline MissionParams missionParams()
{
    MissionParams p;
    PlanParams &plan = p.step.plan;
    plan.accelMmS2 = ACCEL_MM_S2;
    plan.turnRateMaxDps = TURN_RATE_MAX_DPS;
    plan.turnAccelDps2 = TURN_ACCEL_DPS2;
    plan.stepToleranceMm = STEP_TOLERANCE_MM;
    plan.turnToleranceDeg = TURN_TOLERANCE_DEG;
    plan.turnSettleMs = TURN_SETTLE_MS;
    plan.progressMinMm = PROGRESS_MIN_MM;
    plan.progressMinDeg = PROGRESS_MIN_DEG;
    plan.patienceMs = PATIENCE_MS;

    MotionControlParams &c = p.control;
    c.kpDist = KP_DISTANCE;
    c.kiDist = KI_DISTANCE;
    c.kdDist = KD_DISTANCE;
    c.kpTurn = KP_TURN;
    c.kiTurn = KI_TURN;
    c.kdTurn = KD_TURN;
    c.kiHold = KI_TURN_HOLD;
    c.ffScaleR = FEEDFORWARD_RIGHT_SCALE;
    c.ffPwmPerMmS = FEEDFORWARD_PWM_PER_MM_S;
    c.ffPwmPerDps = FEEDFORWARD_PWM_PER_DPS;
    c.ffPwmFriction = FEEDFORWARD_FRICTION_DUTY;
    c.minPwmL = CONTROL_MIN_PWM_L;
    c.minPwmR = CONTROL_MIN_PWM_R;
    c.pwmTop = MOTOR_PWM_TOP;
    c.stallDuty = STALL_DUTY;
    c.stallTimeMs = STALL_TIME_MS;
    c.stallMinSpeedMmS = STALL_MIN_SPEED_MM_S;
    c.stallMinYawDps = STALL_MIN_YAW_DPS;

    p.pattern.frameMinMm = MARKER_FRAME_MIN_MM;
    p.pattern.frameMaxMm = MARKER_FRAME_MAX_MM;
    p.pattern.farFrameMinMm = MARKER_FAR_FRAME_MIN_MM;
    p.pattern.centreMinMm = MARKER_CENTRE_MIN_MM;
    p.pattern.centreMaxMm = MARKER_CENTRE_MAX_MM;
    p.pattern.gapMinMm = GAP_MIN_MM;
    p.pattern.glitchMm = FLOOR_GLITCH_MM;

    LineParams &l = p.line;
    l.kp = KP_LINE;
    l.kd = KD_LINE;
    l.steerMaxDeg = LINE_STEER_MAX_DEG;
    l.speedMmS = LINE_SPEED_MM_S;
    l.accelMmS2 = LINE_ACCEL_MM_S2;
    l.slowdownErrMm = LINE_SLOWDOWN_ERR_MM;
    l.slowdownPct = LINE_SLOWDOWN_PCT;
    l.searchRateDps = LINE_SEARCH_RATE_DPS;
    l.searchSpeedMmS = LINE_SEARCH_SPEED_MM_S;
    l.lostTimeoutMs = LINE_LOST_TIMEOUT_MS;
    l.endLostMm = LINE_END_LOST_MM;
    l.rampInclineCounts = RAMP_INCLINE_COUNTS;
    return p;
}

struct MissionManager {
    MissionPhase phase = MissionPhase::WaitStart; // where the run is, as the log numbers it
    MissionFault fault = MissionFault::None;
    RunStage stage = RunStage::WaitStart;
    Part part = Part::AExplore;     // the part running, or the last one that ran
    Part lastPart = Part::AExplore; // where this run ends, once it has started
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

// The caller found a precondition missing before the run, or a part gave up.
inline void missionFail(MissionManager &ms, MissionFault code)
{
    ms.stage = RunStage::Fault;
    ms.phase = MissionPhase::Fault;
    ms.fault = code;
}

// Degrees to subtract from the gyro heading (headingSet(hd, hd.deg - fix)); see StepCell.h.
inline float missionTakeHeadingFix(MissionManager &ms)
{
    const float fix = ms.explore.headingFixDeg;
    ms.explore.headingFixDeg = 0.0f;
    return fix;
}

enum class PartStatus : uint8_t {
    Running,
    Done,     // the part has ended; the next one begins on the next tick
    HandOver, // ended while moving on: the next part acts in this tick
    DoneHold, // ended and the wheels are to be held with no control step this tick
    Fault,    // `fault` says why
};

namespace mission_detail
{

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

// The first and last part of this run: false if the selection cannot run. The first part must be
// built and able to start alone; the last is the last built part up to the one asked for. A run
// without a line calibration (wallsOnly) has no marker to find, so it ends with the exploring.
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

// Drive on from here, straight along the map direction the robot faces, over the strip and the gap.
inline void beginApproach(MissionManager &ms, const MissionParams &p, const MotionFeedback &fb)
{
    markerExitBeginApproach(ms.marker, p, fb, cellExplorerHeadingRef(ms.explore));
}

// Begin `next`: what a part needs to start from where the robot is.
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
                // With a wall ahead the way out is to a side: stand and read both sides before
                // choosing.
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
            break; // not built: a run never reaches them
    }
}

// Section A's maze. Ends when the explorer stands on the marker's tile (Done), or, with the way out
// straight on and the exit part next, as soon as the marker is told (HandOver: no stop).
inline PartStatus runAExplore(MissionManager &ms, const MissionParams &p, const Perception &pc,
                              MotionTarget &target)
{
    const MotionFeedback &fb = pc.motion;
    Part next = Part::AExplore;
    const bool hasNext = partAfter(Part::AExplore, ms.lastPart, next);
    const bool exitFollows = hasNext && next == Part::AExit;

    // The marker is told only while stepping; without a calibration there is none to tell.
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

// The marker tile and the way out of it. Ends (HandOver) when the line is in view past the gap.
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
                return PartStatus::HandOver; // the line is in view: the next part takes over
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

// The line zone. Ends (Done) when the line is over and the robot has stopped.
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
    target.distRefMm = fb.distMm; // default: stand where we are
    // ... square to the map direction where there is a map, else on the heading it has
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
                hold.enable = true; // zero duty with the driver enabled holds the wheels
                return hold;
            }
            break;
        }

        case RunStage::Done:
            cmd.enable = true; // zero duty with the driver enabled holds the wheels
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

// One tick. Returns what the drivetrain should do. The run begins when `ms.start.started` is set
// (missionTick does that from the button or the delay).
inline DriveCommand missionStep(MissionManager &ms, const MissionParams &p, const Perception &pc)
{
    const DriveCommand cmd = mission_detail::tick(ms, p, pc);
    if (ms.stage != RunStage::Fault) {
        ms.phase = mission_detail::phaseOf(ms);
    }
    return cmd;
}

// One tick of a run: the start, the run limit, then the parts. `buttonDown` is the raw start
// button (ignored without one: the run begins RUN_START_DELAY_MS after the first tick).
inline DriveCommand missionTick(MissionManager &ms, const MissionParams &p, const Perception &pc,
                                uint32_t nowMs, bool buttonDown)
{
    runStartTick(ms.start, nowMs, buttonDown);
    if (ms.start.started && ms.stage != RunStage::Done && ms.stage != RunStage::Fault &&
        nowMs - ms.start.startedAtMs >= RUN_LIMIT_MS) {
        missionFail(ms, MissionFault::TimeUp);
    }
    return missionStep(ms, p, pc);
}
