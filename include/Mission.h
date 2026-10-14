// ============================================================================
//  Mission.h  --  the run-phase state machine: find the bridge in Section A, follow
//  the line across it, stop. No hardware, no Arduino.h - testable under env:native.
//
//    WaitStart -> SenseFront -> TurnAround -> SenseBack -> Explore -> LineFollow -> Done
//                                                            any -> Fault
//
//  Nothing about the layout is assumed: the start cell, its heading and the bridge
//  corner are discovered. The robot starts by reading the four walls of the start
//  cell (the turn-round shows the one behind), then explores toward the nearest
//  unseen cell. The marker redirects it to the cell beside the marker; entering the
//  line zone hands over to the line follower, which runs until the line ends.
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
};

struct MissionParams {
    StepCellParams step;
    LineParams line;
    FloorParams floor;
    MotionControlParams control;
    uint16_t senseMs = 0;       // how long to stand and read the start cell's walls
    float leaveMarginMm = 0.0f; // see LINE_LEAVE_MARGIN_MM
    int8_t hand = 1;            // +1 prefer left turns on a tie, -1 right
    float irAheadMm = 0.0f;     // IR array ahead of the axle (IR_ARRAY_FORWARD_OFFSET_MM)
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
    FloorDetector floor;
    MotionControl control;

    // Where the current run of white floor began: the step it was on then. Marker and line zone are
    // told apart only after the run ends, by which time the robot may have moved on (or turned), so
    // they are attributed from this snapshot.
    bool haveRun = false;
    int8_t runFromX = 0, runFromY = 0, runToX = 0, runToY = 0;
    float runTargetMm = 0.0f;
    maze::Dir runFacing = maze::Dir::North;

    // Leaving a white tile (the start tile, or a line tile met from the side): watch whether the
    // floor stays white past its edge. From a line tile that means the robot is going out along
    // the line (the bridge); from the start tile it means the next tile is white too, a line tile
    // that no run measured because the white began under the start tile. outCenterMm is where the
    // tile's centre was; outBlack records that bare floor showed up first.
    enum class Out : uint8_t { None, FromLine, FromStart };
    Out out = Out::None;
    bool outBlack = false;
    float outCenterMm = 0.0f;

    uint16_t phaseMs = 0;
    uint8_t votes[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
};

inline void missionReset(Mission &ms)
{
    ms = Mission{};
    maze::mapReset(ms.map);
    motionControlReset(ms.control);
    floorDetectorReset(ms.floor);
}

// The caller found a precondition missing before the run.
inline void missionFail(Mission &ms, FaultCode code)
{
    ms.phase = Phase::Fault;
    ms.fault = code;
}

namespace mission_detail
{

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

inline Mission::Out outKind(const Mission &ms)
{
    if (ms.cx == maze::kStartX && ms.cy == maze::kStartY) {
        return Mission::Out::FromStart;
    }
    return ms.map.cell[ms.cy][ms.cx].line ? Mission::Out::FromLine : Mission::Out::None;
}

// Choose the next move from the cell the robot is at. None = nothing left to try.
// With the marker seen the goal is the bridge-out tile beside it, and the way in is through the
// marker cell: the line runs along that axis, and met from the side it crosses the robot's path.
inline maze::Move plan(Mission &ms, const MissionParams &p)
{
    using namespace maze;
    Move mv = Move::None;
    int8_t gx = 0, gy = 0;
    if (ms.map.hasMarker && bridgeCandidate(ms.map, ms.facing, gx, gy)) {
        if (ms.cx == ms.map.markerX && ms.cy == ms.map.markerY) {
            mv = stepIntoNeighbour(ms.map, ms.facing, gx, gy);
            if (mv != Move::None) {
                return mv;
            }
        } else if (goToMove(ms.map, ms.cx, ms.cy, ms.facing, p.hand, ms.map.markerX, ms.map.markerY,
                            mv)) {
            return mv;
        }
    }
    if (exploreMove(ms.map, ms.cx, ms.cy, ms.facing, p.hand, mv)) {
        return mv;
    }
    return Move::None;
}

inline void startStep(Mission &ms, const MissionParams &p, const MotionFeedback &fb)
{
    ms.fromX = ms.cx;
    ms.fromY = ms.cy;
    ms.tx = static_cast<int8_t>(ms.cx + maze::kDx[maze::dirIndex(ms.facing)]);
    ms.ty = static_cast<int8_t>(ms.cy + maze::kDy[maze::dirIndex(ms.facing)]);
    stepCellBegin(ms.step, p.step, fb, gridHeading(ms, ms.facing));
    ms.decided = false;
    ms.sub = Mission::Sub::Stepping;
    ms.out = outKind(ms);
    ms.outBlack = false;
    ms.outCenterMm = fb.distMm;
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
    turnBegin(ms.turn, fb, gridHeading(ms, to), 90.0f * static_cast<float>(quarter));
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
        ms.out = outKind(ms);
        ms.outBlack = false;
        ms.outCenterMm = ms.step.straight.targetMm; // this centre, before the target moves on
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

inline bool atStart(int8_t x, int8_t y)
{
    return x == maze::kStartX && y == maze::kStartY;
}

// A run of white floor has just begun under the array: remember which step it is on. The start
// tile is white and would pass for the marker or the line, so a run that begins as the robot
// heads for it is not kept. (Leaving it starts no run: the one in progress at power-up is never
// measured, and a later one began on the way in.)
inline void onRunStart(Mission &ms)
{
    ms.haveRun = !atStart(ms.tx, ms.ty);
    ms.runFromX = ms.fromX;
    ms.runFromY = ms.fromY;
    ms.runToX = ms.tx;
    ms.runToY = ms.ty;
    ms.runTargetMm = ms.step.straight.targetMm;
    ms.runFacing = ms.facing;
}

// The marker run ended: which cell was it in? The detector reports where the array saw its centre;
// the axle was irAheadMm behind that, and the target centre of the step the run began on is known.
inline void onMarker(Mission &ms, const MissionParams &p, float centerMm)
{
    if (!ms.haveRun) {
        return;
    }
    const float feature = centerMm + p.irAheadMm;
    const float offset = (feature - ms.runTargetMm) / p.step.pitchMm;
    const int8_t k = offset >= 0.5f ? 1 : (offset <= -0.5f ? -1 : 0);
    const int8_t mx = static_cast<int8_t>(ms.runToX + k * maze::kDx[maze::dirIndex(ms.runFacing)]);
    const int8_t my = static_cast<int8_t>(ms.runToY + k * maze::kDy[maze::dirIndex(ms.runFacing)]);
    if (maze::mapInArray(mx, my)) {
        maze::mapMarkMarker(ms.map, mx, my);
    }
    ms.haveRun = false;
}

// A white tile with a line: the bridge-out tile only if it was entered from the marker cell. From
// anywhere else the line crosses the robot's path, so it is noted and left.
inline bool onLineZone(Mission &ms)
{
    if (!ms.haveRun) {
        return false;
    }
    ms.haveRun = false;
    if (ms.map.hasMarker && ms.runFromX == ms.map.markerX && ms.runFromY == ms.map.markerY) {
        return true;
    }
    maze::mapMarkLine(ms.map, ms.runToX, ms.runToY);
    return false;
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
                    turnBegin(ms.turn, fb, gridHeading(ms, to), p.hand >= 0 ? 180.0f : -180.0f);
                    ms.facing = to;
                    ms.phase = Phase::TurnAround;
                } else {
                    ms.pending = plan(ms, p);
                    if (ms.pending == maze::Move::None) {
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
                    startStep(ms, p, fb);
                } else if (st == MoveStatus::Timeout) {
                    missionFail(ms, FaultCode::MoveTimeout);
                    return DriveCommand{};
                }
                break;
            }

            // Stepping. The floor tells us about the marker and the line zone.
            float center = 0.0f;
            switch (floorDetectorStep(ms.floor, pc.line.allBlack, fb.distMm, p.floor, center)) {
                case FloorEvent::RunStart:
                    onRunStart(ms);
                    break;
                case FloorEvent::Marker:
                    onMarker(ms, p, center);
                    break;
                case FloorEvent::LineZone:
                    if (!ms.haveRun) {
                        break; // near the start tile
                    }
                    if (onLineZone(ms)) {
                        // Hand over mid-stride: the follower starts from the current speed, and
                        // runs this very tick so the drivetrain never sees a gap.
                        lineFollowBegin(ms.line, fb, true);
                        ms.phase = Phase::LineFollow;
                        lineFollowStep(ms.line, p.line, fb, pc.line, pc.tiltDeg,
                                       frontBlocked(p, pc), p.dtMs, target);
                    }
                    break;
                case FloorEvent::None:
                    break;
            }
            if (ms.phase == Phase::LineFollow) {
                break;
            }
            if (ms.out != Mission::Out::None) {
                if (pc.line.allBlack) {
                    ms.outBlack = true;
                } else if (!ms.outBlack && fb.distMm - ms.outCenterMm >= p.step.pitchMm * 0.5f -
                                                                             p.irAheadMm +
                                                                             p.leaveMarginMm) {
                    // Still white well past the tile's edge.
                    if (ms.out == Mission::Out::FromLine) {
                        // The way out along the line: hand over to the follower.
                        lineFollowBegin(ms.line, fb, true);
                        ms.phase = Phase::LineFollow;
                        lineFollowStep(ms.line, p.line, fb, pc.line, pc.tiltDeg,
                                       frontBlocked(p, pc), p.dtMs, target);
                        break;
                    }
                    maze::mapMarkLine(ms.map, ms.tx, ms.ty); // beyond the start tile: white too
                    ms.out = Mission::Out::None;
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
                if (!ms.decided || ms.pending == maze::Move::None) {
                    missionFail(ms, FaultCode::NoRoute);
                    return DriveCommand{};
                }
                startMove(ms, p, fb);
            }
            break;
        }

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
