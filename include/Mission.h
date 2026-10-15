// ============================================================================
//  Mission.h  --  the run-phase state machine: find the bridge in Section A, follow
//  the line across it, stop. No hardware, no Arduino.h - testable under env:native.
//
//    WaitStart -> SenseFront -> TurnAround -> SenseBack -> Explore -> Bridge -> LineFollow -> Done
//                                                            any -> Fault
//
//  Nothing about the layout is assumed: the start cell, its heading and the bridge
//  corner are discovered. The robot starts by reading the four walls of the start
//  cell (the turn-round shows the one behind), then explores toward the nearest
//  unseen cell. The marker cell has exactly two openings, the side the robot comes in by and
//  the one straight across; the IR array sees the marker (white, black, white) as the robot
//  arrives on the cell's centre, and the way out is straight on. Bridge drives straight on
//  the grid heading, past the marker's far strip and the gap, and hands over to the line
//  follower at the first line view past the gap.
//  docs/architecture/modules/mission.md
// ============================================================================

#pragma once

#include <stdint.h>

#include "DriveCommand.h"
#include "Explorer.h"
#include "LineFollow.h"
#include "MotionControl.h"
#include "MotionPrimitives.h"
#include "Perception.h"
#include "StepCell.h"

enum class Phase : uint8_t {
    WaitStart,
    SenseFront, // standing at the start, reading the three walls ahead
    TurnAround,
    SenseBack, // facing the other way: the same three sensors now show the wall behind
    Explore,
    Bridge, // the marker was seen: straight on, across the gap, to the first line view
    LineFollow,
    Done,
    Fault,
};

enum class FaultCode : uint8_t {
    None,
    NoImu,       // set by the caller: no gyro, no heading, no turns
    NoLineCal,   // set by the caller: no usable line calibration
    MoveTimeout, // a step or turn took longer than MOVE_TIMEOUT_MS
    Stall,       // motors pushed hard and nothing moved
    LineLost,    // searched for the line for LINE_LOST_TIMEOUT_MS
    NoRoute,     // nothing left to explore and no bridge found
    NoBridge,    // the marker was seen but no line followed the gap, or something blocked the way
    Aborted,     // set by the caller: stopped from the serial line
};

struct MissionParams {
    StepCellParams step;
    LineParams line;
    PatternParams pattern;        // the marker and the gap
    float bridgeMinLineMm = 0.0f; // the array off the gap this far before a line view counts
    float bridgeMaxMm = 0.0f;     // the first line view must come within this far of the marker
    MotionControlParams control;
    uint16_t senseMs = 0;   // how long to stand and read the start cell's walls
    int8_t hand = 1;        // +1 prefer left turns on a tie, -1 right
    bool wallsOnly = false; // no floor sensing: explore everything, then stop (Done)
    uint16_t dtMs = 10;
};

struct Mission {
    enum class Sub : uint8_t { Stepping, Turning };

    Phase phase = Phase::WaitStart;
    FaultCode fault = FaultCode::None;

    maze::MazeMap map;
    maze::Dir facing = maze::Dir::North;
    int8_t cx = maze::kStartX; // the cell whose walls were last read
    int8_t cy = maze::kStartY;
    int8_t tx = maze::kStartX; // the cell the current step is heading for
    int8_t ty = maze::kStartY;
    int8_t fromX = maze::kStartX; // the cell it started from
    int8_t fromY = maze::kStartY;
    float h0 = 0.0f; // gyro heading at the start; map North is this

    Sub sub = Sub::Stepping;
    maze::Move pending = maze::Move::None;
    bool decided = false; // the walls of the target are in the map and a move is chosen

    StepCell step;
    Turn turn;
    LineFollow line;
    FloorPattern pattern; // the runs of floor class along the current step
    BridgeHandover handover;
    Straight bridge;      // the straight drive from the marker to the line
    bool gapSeen = false; // the gap after the marker has been reported

    // Everything is explored and the marker was not reported: the exit is found from the map
    // (the one open side that leads out of the 4x4) and the robot goes there and takes it.
    bool exiting = false;
    bool bridgeNow = false; // at the exit cell, facing the exit: start the bridge drive
    int8_t exitX = 0, exitY = 0;
    maze::Dir exitDir = maze::Dir::North;
    MotionControl control;

    uint16_t phaseMs = 0;
    uint8_t votes[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
};

inline void missionReset(Mission &ms)
{
    ms = Mission{};
    maze::mapReset(ms.map);
    motionControlReset(ms.control);
    floorPatternReset(ms.pattern);
    bridgeHandoverReset(ms.handover);
}

// The caller found a precondition missing before the run.
inline void missionFail(Mission &ms, FaultCode code)
{
    ms.phase = Phase::Fault;
    ms.fault = code;
}

namespace mission_detail
{

// The sweep that ends exactly on `targetDeg`, going the way `dir` says (positive = CCW). The robot
// is rarely square to the grid when a turn starts (the last one stopped within the tolerance, not
// on the mark), and a nominal 90 would add that error to this turn's.
inline float sweepTo(float fromDeg, float targetDeg, float dir)
{
    float d = headingWrapDeg(targetDeg - fromDeg);
    if (dir > 0.0f && d <= 0.0f) {
        d += 360.0f;
    } else if (dir < 0.0f && d >= 0.0f) {
        d -= 360.0f;
    }
    return d;
}

// Heading the gyro reads when the robot is square to map direction `d`.
inline float gridHeading(const Mission &ms, maze::Dir d)
{
    return headingWrapDeg(ms.h0 + 90.0f * static_cast<float>(maze::dirIndex(d)));
}

inline void clearVotes(Mission &ms)
{
    for (auto &row : ms.votes) {
        for (auto &v : row) {
            v = 0;
        }
    }
    ms.phaseMs = 0;
}

// Read the three sensors into the votes; once the window is over, the verdict.
inline bool senseWindow(Mission &ms, const MissionParams &p, const Perception &pc, CellWalls &out)
{
    step_detail::vote(ms.votes[0], pc.walls.frontValid, pc.walls.front);
    step_detail::vote(ms.votes[1], pc.walls.leftValid, pc.walls.left);
    step_detail::vote(ms.votes[2], pc.walls.rightValid, pc.walls.right);
    ms.phaseMs = static_cast<uint16_t>(ms.phaseMs + p.dtMs);
    if (ms.phaseMs < p.senseMs) {
        return false;
    }
    out = CellWalls{};
    out.front = step_detail::verdict(ms.votes[0], out.unreliable);
    out.left = step_detail::verdict(ms.votes[1], out.unreliable);
    out.right = step_detail::verdict(ms.votes[2], out.unreliable);
    return true;
}

// Something close ahead: the front sensor hears an echo at or inside the stop distance.
inline bool frontBlocked(const MissionParams &p, const Perception &pc)
{
    return p.step.frontStopMm > 0.0f && pc.walls.frontValid &&
           static_cast<float>(pc.walls.frontMm) <= p.step.frontStopMm;
}

// Choose the next move from the cell the robot is at. None = nothing left to try.
inline maze::Move plan(Mission &ms, const MissionParams &p)
{
    maze::Move mv = maze::Move::None;
    if (maze::exploreMove(ms.map, ms.cx, ms.cy, ms.facing, p.hand, mv)) {
        return mv;
    }
    // Nothing left to explore. The marker was not reported (the robot stopped short of seeing it,
    // say), but the map still knows the way out: the one open side that leads out of the 4x4.
    if (!p.wallsOnly && maze::exitCell(ms.map, ms.exitX, ms.exitY, ms.exitDir)) {
        ms.exiting = true;
        if (ms.cx == ms.exitX && ms.cy == ms.exitY) {
            if (ms.facing == ms.exitDir) {
                ms.bridgeNow = true; // the step stops on this centre; the bridge starts from there
                return maze::Move::None;
            }
            return maze::detail::relativeMove(ms.facing, ms.exitDir);
        }
        if (maze::goToMove(ms.map, ms.cx, ms.cy, ms.facing, p.hand, ms.exitX, ms.exitY, mv)) {
            return mv;
        }
    }
    return maze::Move::None;
}

inline void startStep(Mission &ms, const MissionParams &p, const MotionFeedback &fb)
{
    motionControlClearIntegrals(ms.control);
    ms.fromX = ms.cx;
    ms.fromY = ms.cy;
    ms.tx = static_cast<int8_t>(ms.cx + maze::kDx[maze::dirIndex(ms.facing)]);
    ms.ty = static_cast<int8_t>(ms.cy + maze::kDy[maze::dirIndex(ms.facing)]);
    stepCellBegin(ms.step, p.step, fb, gridHeading(ms, ms.facing));
    ms.decided = false;
    ms.sub = Mission::Sub::Stepping;
    floorPatternReset(ms.pattern); // runs of floor are measured along one step
}

// Stopped on a cell centre with a move chosen: do it.
inline void startMove(Mission &ms, const MissionParams &p, const MotionFeedback &fb)
{
    using maze::Move;
    int8_t quarter = 0;
    switch (ms.pending) {
        case Move::Left:
            quarter = 1;
            break;
        case Move::Right:
            quarter = -1;
            break;
        case Move::Back:
            quarter = p.hand >= 0 ? 2 : -2; // a 180 either way; the hand picks which
            break;
        default:
            break;
    }
    ms.pending = Move::None;
    if (quarter == 0) {
        startStep(ms, p, fb);
        return;
    }
    const maze::Dir to = maze::dirRotate(ms.facing, quarter);
    motionControlClearIntegrals(ms.control);
    turnBegin(ms.turn, fb, gridHeading(ms, to),
              sweepTo(fb.headingDeg, gridHeading(ms, to), static_cast<float>(quarter)));
    ms.facing = to; // the turn ends square to it
    ms.sub = Mission::Sub::Turning;
}

// The step's walls are in: put them on the map and decide what happens at this centre.
inline void onWallsReady(Mission &ms, const MissionParams &p)
{
    using namespace maze;
    ms.cx = ms.tx;
    ms.cy = ms.ty;
    mapObserve(ms.map, ms.cx, ms.cy, ms.facing, ms.step.walls.front, ms.step.walls.left,
               ms.step.walls.right);
    mapSetEdge(ms.map, ms.cx, ms.cy, dirOpposite(ms.facing), false); // the way we came
    const Move mv = plan(ms, p);
    if (mv == Move::Forward) {
        stepCellContinue(ms.step, p.step);
        ms.fromX = ms.cx;
        ms.fromY = ms.cy;
        ms.tx = static_cast<int8_t>(ms.cx + kDx[dirIndex(ms.facing)]);
        ms.ty = static_cast<int8_t>(ms.cy + kDy[dirIndex(ms.facing)]);
        ms.decided = false;
        return;
    }
    ms.pending = mv; // the step stops on this centre by itself
    ms.decided = true;
}

// The marker was just reported: the robot is on the marker cell's centre and the way out is
// straight on, so drive on along the grid heading at line speed, carrying the current speed
// through. Also used when the gap is seen while exploring with no marker reported (`gapAlready`):
// the robot is then out of the maze's exit already and only the line past the gap is left to find.
inline void startBridge(Mission &ms, const MissionParams &p, const MotionFeedback &fb,
                        bool gapAlready = false)
{
    motionControlClearIntegrals(ms.control);
    bridgeHandoverReset(ms.handover);
    ms.gapSeen = gapAlready;
    straightBegin(ms.bridge, fb, p.bridgeMaxMm, p.line.speedMmS, p.line.speedMmS,
                  gridHeading(ms, ms.facing));
    ms.phase = Phase::Bridge;
}

// One tick of the bridge drive. False when it failed (the fault is set).
inline bool bridgeTick(Mission &ms, const MissionParams &p, const Perception &pc,
                       MotionTarget &target)
{
    const MotionFeedback &fb = pc.motion;
    float unused = 0.0f;
    if (floorPatternStep(ms.pattern, pc.floor, fb.distMm, p.pattern, unused) == PatternEvent::Gap) {
        ms.gapSeen = true;
    }
    if (bridgeHandoverStep(ms.handover, ms.gapSeen, pc.floor, pc.line, fb.distMm,
                           p.bridgeMinLineMm)) {
        // Hand over mid-stride: the follower starts from the current speed, and runs this very
        // tick so the drivetrain never sees a gap.
        lineFollowBegin(ms.line, fb, true);
        ms.phase = Phase::LineFollow;
        lineFollowStep(ms.line, p.line, fb, pc.line, pc.tiltDeg, false, p.dtMs, target);
        return true;
    }
    if (frontBlocked(p, pc)) {
        missionFail(ms, FaultCode::NoBridge); // a wall where the opening should be
        return false;
    }
    const MoveStatus st = straightStep(ms.bridge, p.step.plan, fb, 0.0f, p.dtMs, target);
    if (st == MoveStatus::Done || st == MoveStatus::Timeout) {
        missionFail(ms, FaultCode::NoBridge); // no line came after the gap
        return false;
    }
    return true;
}

} // namespace mission_detail

// One tick. Returns what the drivetrain should do.
inline DriveCommand missionStep(Mission &ms, const MissionParams &p, const Perception &pc)
{
    using namespace mission_detail;
    const MotionFeedback &fb = pc.motion;
    MotionTarget target;
    // Default: stand still where we are.
    target.distRefMm = fb.distMm;
    target.headingRefDeg = fb.headingDeg;

    DriveCommand cmd;
    switch (ms.phase) {
        case Phase::WaitStart:
            if (pc.startPressed) {
                ms.h0 = fb.headingDeg;
                clearVotes(ms);
                ms.phase = Phase::SenseFront;
            }
            return cmd; // motors off until we move

        case Phase::SenseFront:
        case Phase::SenseBack: {
            CellWalls w;
            if (senseWindow(ms, p, pc, w)) {
                maze::mapObserve(ms.map, ms.cx, ms.cy, ms.facing, w.front, w.left, w.right);
                if (ms.phase == Phase::SenseFront) {
                    const maze::Dir to = maze::dirOpposite(ms.facing);
                    motionControlClearIntegrals(ms.control);
                    turnBegin(
                        ms.turn, fb, gridHeading(ms, to),
                        sweepTo(fb.headingDeg, gridHeading(ms, to), p.hand >= 0 ? 1.0f : -1.0f));
                    ms.facing = to;
                    ms.phase = Phase::TurnAround;
                } else {
                    ms.pending = plan(ms, p);
                    if (ms.pending == maze::Move::None) {
                        if (p.wallsOnly) {
                            ms.phase = Phase::Done; // nothing to explore
                            return cmd;
                        }
                        missionFail(ms, FaultCode::NoRoute);
                        return cmd;
                    }
                    ms.phase = Phase::Explore;
                    startMove(ms, p, fb);
                }
            }
            cmd.enable = true;
            break;
        }

        case Phase::TurnAround: {
            const MoveStatus st = turnStep(ms.turn, p.step.plan, fb, p.dtMs, target);
            if (st == MoveStatus::Done) {
                clearVotes(ms);
                ms.phase = Phase::SenseBack;
            } else if (st == MoveStatus::Timeout) {
                missionFail(ms, FaultCode::MoveTimeout);
                return cmd;
            }
            cmd.enable = true;
            break;
        }

        case Phase::Explore: {
            cmd.enable = true;
            if (ms.sub == Mission::Sub::Turning) {
                const MoveStatus st = turnStep(ms.turn, p.step.plan, fb, p.dtMs, target);
                if (st == MoveStatus::Done) {
                    if (ms.exiting && ms.cx == ms.exitX && ms.cy == ms.exitY &&
                        ms.facing == ms.exitDir) {
                        startBridge(ms, p, fb); // turned to face the exit: take it
                        if (!bridgeTick(ms, p, pc, target)) {
                            return DriveCommand{};
                        }
                        break;
                    }
                    startStep(ms, p, fb);
                } else if (st == MoveStatus::Timeout) {
                    missionFail(ms, FaultCode::MoveTimeout);
                    return DriveCommand{};
                }
                break;
            }

            // Stepping. The floor tells us about the marker (walls-only: not read). The marker cell
            // has two openings, the one we came in by and the one straight across: go straight on.
            if (!p.wallsOnly) {
                float center = 0.0f;
                const PatternEvent ev =
                    floorPatternStep(ms.pattern, pc.floor, fb.distMm, p.pattern, center);
                if (ev == PatternEvent::Marker || ev == PatternEvent::Gap) {
                    startBridge(ms, p, fb, ev == PatternEvent::Gap);
                    if (!bridgeTick(ms, p, pc, target)) {
                        return DriveCommand{};
                    }
                    break;
                }
            }

            const MoveStatus st = stepCellStep(ms.step, p.step, fb, pc.walls, p.dtMs, target);
            if (ms.step.wallsReady && !ms.decided) {
                onWallsReady(ms, p);
            }
            if (st == MoveStatus::Timeout) {
                missionFail(ms, FaultCode::MoveTimeout);
                return DriveCommand{};
            }
            if (st == MoveStatus::Done) {
                if (p.wallsOnly && ms.decided && ms.pending == maze::Move::None) {
                    ms.phase = Phase::Done; // every reachable cell explored
                    cmd.enable = true;
                    break;
                }
                if (ms.bridgeNow) {
                    ms.bridgeNow = false;
                    startBridge(ms, p, fb);
                    if (!bridgeTick(ms, p, pc, target)) {
                        return DriveCommand{};
                    }
                    break;
                }
                if (!ms.decided || ms.pending == maze::Move::None) {
                    missionFail(ms, FaultCode::NoRoute);
                    return DriveCommand{};
                }
                startMove(ms, p, fb);
            }
            break;
        }

        case Phase::Bridge:
            cmd.enable = true;
            if (!bridgeTick(ms, p, pc, target)) {
                return DriveCommand{};
            }
            break;

        case Phase::LineFollow: {
            cmd.enable = true;
            const LineStatus st = lineFollowStep(ms.line, p.line, fb, pc.line, pc.tiltDeg,
                                                 frontBlocked(p, pc), p.dtMs, target);
            if (st == LineStatus::Ended) {
                ms.phase = Phase::Done;
            } else if (st == LineStatus::Lost) {
                missionFail(ms, FaultCode::LineLost);
                return DriveCommand{};
            }
            break;
        }

        case Phase::Done:
            cmd.enable = true; // zero duty with the driver enabled holds the wheels
            break;

        case Phase::Fault:
            return cmd;
    }

    const MotionOutput out = motionControlStep(ms.control, p.control, target, fb, p.dtMs);
    if (ms.control.stalled) {
        missionFail(ms, FaultCode::Stall);
        return DriveCommand{};
    }
    cmd.left = out.left;
    cmd.right = out.right;
    return cmd;
}
