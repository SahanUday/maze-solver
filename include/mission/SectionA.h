// ============================================================================
//  SectionA.h  --  the Section A mission: explore from any tile until the approach marker, then
//  drive out of the maze over the gap and follow the line. No hardware, no Arduino.h - testable
//  under env:native.
//
//    WaitStart -> Explore -> (marker) -> Exit -> Line
//                 any -> Fault          (Done: see below)
//
//  The mission composes three parts and owns the one motion controller they all feed:
//    CellExplorer  sense, turn and step through the maze (CellExplorer.h)
//    MarkerExit    tell the marker, find the way out of its tile, drive over the gap (MarkerExit.h)
//    LineFollow    follow the line past the gap (motion/LineFollow.h)
//  The marker (white frame, black box, white frame under the array) ends the search. Done is only
//  reached without the line (`followLine` false: stop on the marker's tile) or without a
//  calibration (`wallsOnly`: everything explored).
//  docs/architecture/modules/section_a.md
// ============================================================================

#pragma once

#include <stdint.h>

#include "mission/CellExplorer.h"
#include "mission/Fault.h"
#include "mission/MarkerExit.h"
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
enum class SectionAPhase : uint8_t { WaitStart, Sense, Turn, Step, Done, Fault, Approach, Line };

enum class SectionAStage : uint8_t { WaitStart, Explore, Exit, Line, Done, Fault };

struct SectionAParams : CellExplorerParams, MarkerExitParams {
    MotionControlParams control;
    // followLine false stops on the marker's tile instead (Done).
    bool followLine = SECTION_A_FOLLOW_LINE;
    LineParams line;
    // The marker is told by the floor sensor; without it (the array not calibrated) the run
    // explores every reachable tile and stops (wallsOnly, in CellExplorerParams).
};

// Every parameter at its spec/ value: what the robot runs with.
inline SectionAParams sectionAParams()
{
    SectionAParams p;
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

struct SectionA {
    SectionAPhase phase = SectionAPhase::WaitStart; // where the stage is, as the log numbers it
    SectionAFault fault = SectionAFault::None;
    SectionAStage stage = SectionAStage::WaitStart;

    CellExplorer explore;
    MarkerExit marker;
    LineFollow line;
    MotionControl control;
};

inline void sectionAReset(SectionA &ms)
{
    ms = SectionA{};
    cellExplorerReset(ms.explore);
    motionControlReset(ms.control);
}

// The caller found a precondition missing before the run.
inline void sectionAFail(SectionA &ms, SectionAFault code)
{
    ms.stage = SectionAStage::Fault;
    ms.phase = SectionAPhase::Fault;
    ms.fault = code;
}

// Degrees to subtract from the gyro heading (headingSet(hd, hd.deg - fix)); see StepCell.h.
inline float sectionATakeHeadingFix(SectionA &ms)
{
    const float fix = ms.explore.headingFixDeg;
    ms.explore.headingFixDeg = 0.0f;
    return fix;
}

namespace section_a_detail
{

inline SectionAPhase phaseOf(const SectionA &ms)
{
    switch (ms.stage) {
        case SectionAStage::WaitStart:
            return SectionAPhase::WaitStart;
        case SectionAStage::Explore:
            switch (ms.explore.phase) {
                case ExplorePhase::Sense:
                    return SectionAPhase::Sense;
                case ExplorePhase::Turn:
                    return SectionAPhase::Turn;
                case ExplorePhase::Step:
                    return SectionAPhase::Step;
            }
            break;
        case SectionAStage::Exit:
            switch (ms.marker.phase) {
                case ExitPhase::Stand:
                    return SectionAPhase::Sense;
                case ExitPhase::Turn:
                    return SectionAPhase::Turn;
                case ExitPhase::Approach:
                    return SectionAPhase::Approach;
            }
            break;
        case SectionAStage::Line:
            return SectionAPhase::Line;
        case SectionAStage::Done:
            return SectionAPhase::Done;
        case SectionAStage::Fault:
            break;
    }
    return SectionAPhase::Fault;
}

// Drive on from here, straight along the map direction the robot faces, over the strip and the gap.
inline void beginApproach(SectionA &ms, const SectionAParams &p, const MotionFeedback &fb)
{
    markerExitBeginApproach(ms.marker, p, fb, cellExplorerHeadingRef(ms.explore));
    ms.stage = SectionAStage::Exit;
}

inline DriveCommand tick(SectionA &ms, const SectionAParams &p, const Perception &pc)
{
    const MotionFeedback &fb = pc.motion;
    MotionTarget target;
    target.distRefMm = fb.distMm; // default: stand where we are, square to the map direction
    target.headingRefDeg =
        ms.stage == SectionAStage::WaitStart ? fb.headingDeg : cellExplorerHeadingRef(ms.explore);
    target.newMove = true;

    // The marker is told only while stepping; without a calibration there is none to tell.
    ExploreInput in;
    in.brakeOnStop = !p.followLine;
    float lateCentreMm = 0.0f;
    const bool looking = !p.wallsOnly && ms.stage == SectionAStage::Explore &&
                         ms.explore.phase == ExplorePhase::Step && !ms.marker.atMarker;
    if (looking && markerWatch(ms.marker, pc, p.pattern)) {
        cellExplorerStopHere(ms.explore);
        if (p.followLine && cellExplorerEdgeOpen(ms.explore, ms.explore.map.heading)) {
            beginApproach(ms, p, fb); // straight through: no stop
        }
    } else if (looking) {
        in.lateStopOffer = markerPending(ms.marker, fb, p.pattern, lateCentreMm);
    }

    DriveCommand cmd;
    switch (ms.stage) {
        case SectionAStage::WaitStart:
            if (pc.startPressed) {
                cellExplorerStart(ms.explore, fb);
                ms.stage = SectionAStage::Explore;
            }
            return cmd; // motors off until the run starts

        case SectionAStage::Explore: {
            cmd.enable = true;
            const ExploreStatus st = cellExplorerStep(ms.explore, p, in, pc, target);
            if (ms.explore.lateTaken) {
                ms.explore.lateTaken = false;
                markerTaken(ms.marker, lateCentreMm);
            }
            switch (st) {
                case ExploreStatus::Running:
                    break;
                case ExploreStatus::Explored:
                    ms.stage = SectionAStage::Done; // every reachable tile has been entered
                    break;
                case ExploreStatus::Fault:
                    sectionAFail(ms, ms.explore.fault);
                    return DriveCommand{};
                case ExploreStatus::Braked: {
                    ms.stage = SectionAStage::Done;
                    DriveCommand hold;
                    hold.enable = true; // zero duty with the driver enabled holds the wheels
                    return hold;
                }
                case ExploreStatus::Stopped:
                    if (!p.followLine) {
                        ms.stage = SectionAStage::Done;
                    } else if (cellExplorerEdgeOpen(ms.explore, ms.explore.map.heading)) {
                        beginApproach(ms, p,
                                      fb); // it stopped short of the pattern's end: straight on
                    } else {
                        // With a wall ahead the way out is to a side: stand and read both sides
                        // before choosing.
                        markerExitBeginStand(ms.marker);
                        ms.stage = SectionAStage::Exit;
                    }
                    break;
            }
            break;
        }

        case SectionAStage::Exit: {
            cmd.enable = true;
            switch (ms.marker.phase) {
                case ExitPhase::Stand: {
                    const maze::Dir left = maze::turnLeft(ms.explore.map.heading);
                    const maze::Dir right = maze::turnRight(ms.explore.map.heading);
                    const ExitChoice c = markerExitStandStep(
                        ms.marker, pc, p.senseMs, p.dtMs, cellExplorerEdgeOpen(ms.explore, left),
                        cellExplorerEdgeOpen(ms.explore, right));
                    if (c == ExitChoice::NoWay) {
                        sectionAFail(ms, SectionAFault::NoLine); // no way out of the tile
                        return DriveCommand{};
                    }
                    if (c != ExitChoice::Wait) {
                        ms.marker.sideExit = true;
                        ms.marker.phase = ExitPhase::Turn;
                        cellExplorerBeginTurn(ms.explore, fb, c == ExitChoice::Left ? left : right);
                    }
                    break;
                }
                case ExitPhase::Turn: {
                    const TurnStatus ts = cellExplorerTurnStep(ms.explore, p, fb, target);
                    if (ts == TurnStatus::Fault) {
                        sectionAFail(ms, ms.explore.fault);
                        return DriveCommand{};
                    }
                    if (ts == TurnStatus::Done) {
                        beginApproach(ms, p, fb); // facing the open edge: out of the maze
                    }
                    break;
                }
                case ExitPhase::Approach: {
                    const ApproachStatus as =
                        markerExitApproachStep(ms.marker, p, p.step.plan, pc, p.dtMs, target);
                    if (as == ApproachStatus::HandOver) {
                        // The line is in view past the gap: the follower takes over, mid-stride.
                        lineFollowBegin(ms.line, fb, true);
                        ms.stage = SectionAStage::Line;
                        lineFollowStep(ms.line, p.line, fb, pc.line, pc.inclineCounts, p.dtMs,
                                       target);
                    } else if (as == ApproachStatus::NoLine) {
                        sectionAFail(ms, SectionAFault::NoLine);
                        return DriveCommand{};
                    }
                    break;
                }
            }
            break;
        }

        case SectionAStage::Line: {
            cmd.enable = true;
            const LineStatus ls =
                lineFollowStep(ms.line, p.line, fb, pc.line, pc.inclineCounts, p.dtMs, target);
            if (ls == LineStatus::Lost) {
                sectionAFail(ms, SectionAFault::LineLost);
                return DriveCommand{};
            }
            if (ls == LineStatus::Ended) {
                ms.stage = SectionAStage::Done;
            }
            break;
        }

        case SectionAStage::Done:
            cmd.enable = true; // zero duty with the driver enabled holds the wheels
            break;

        case SectionAStage::Fault:
            return cmd;
    }

    const MotionOutput out = motionControlStep(ms.control, p.control, target, fb, p.dtMs);
    if (ms.control.stalled) {
        sectionAFail(ms, SectionAFault::Stall);
        return DriveCommand{};
    }
    cmd.left = out.left;
    cmd.right = out.right;
    return cmd;
}

} // namespace section_a_detail

// One tick. Returns what the drivetrain should do.
inline DriveCommand sectionAStep(SectionA &ms, const SectionAParams &p, const Perception &pc)
{
    const DriveCommand cmd = section_a_detail::tick(ms, p, pc);
    if (ms.stage != SectionAStage::Fault) {
        ms.phase = section_a_detail::phaseOf(ms);
    }
    return cmd;
}
