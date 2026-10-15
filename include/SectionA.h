// ============================================================================
//  SectionA.h  --  the Section A mission: explore from any tile until the white tile, stop on it.
//  No hardware, no Arduino.h - testable under env:native.
//
//    WaitStart -> Sense -> Turn -> Sense -> Step -> ... -> Done
//                                          any -> Fault
//
//  Sense stands still and reads the cell's three walls; the planner (lib/maze) picks the next
//  tile; Turn faces it; Step drives it, reading the walls of the tile ahead on the way and rolling
//  on through the centre when the next tile is straight on. Done is where the robot has reached the
//  white tile and holds: the hand-over point for what comes after Section A.
//  docs/architecture/modules/section_a.md
// ============================================================================

#pragma once

#include <stdint.h>

#include "DriveCommand.h"
#include "Maze.h"
#include "MotionControl.h"
#include "MotionPrimitives.h"
#include "Perception.h"
#include "RobotSpec.h"
#include "StepCell.h"

enum class SectionAPhase : uint8_t { WaitStart, Sense, Turn, Step, Done, Fault };

enum class SectionAFault : uint8_t {
    None,
    NoImu,        // set by the caller: no gyro, no heading, no turns
    NoRoute,      // nothing left to explore, the rescans used, the white tile not found
    StuckTurn,    // a turn made no progress, retried
    StuckStep,    // a step made no progress
    Stall,        // motors pushed hard and nothing moved
    TooManyStops, // blocked steps over the limit: something keeps stopping the robot
    TimeUp        // set by the caller: the run limit passed before the run ended
};

struct SectionAParams {
    StepCellParams step;
    MotionControlParams control;
    uint16_t senseMs = SECTION_A_SENSE_MS;
    uint8_t frontTries = SECTION_A_FRONT_TRIES;
    uint8_t rescans = SECTION_A_RESCANS;
    uint8_t resumes = SECTION_A_RESUMES;
    uint8_t turnRetries = SECTION_A_TURN_RETRIES;
    uint8_t maxBlocked = SECTION_A_MAX_BLOCKED;
    uint16_t finishWhiteMs = SECTION_A_FINISH_WHITE_MS;
    // The white tile is told by the floor sensor; without it (the array not calibrated) the run
    // explores every reachable tile and stops (Done).
    bool wallsOnly = false;
    // A run that begins on a white tile is finished at once. The competition's start tile is white
    // too and must be left, so this is off by default.
    bool finishAtStart = false;
    uint16_t dtMs = CONTROL_LOOP_PERIOD_MS;
};

// Every parameter at its RobotSpec.h value: what the robot runs with.
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
    return p;
}

struct SectionA {
    SectionAPhase phase = SectionAPhase::WaitStart;
    SectionAFault fault = SectionAFault::None;

    maze::Explorer map;
    float h0 = 0.0f; // the gyro heading at the start; map North is this

    StepCell step;
    Turn turn;
    MotionControl control;

    // The cell being read: votes while standing, and what they said.
    uint8_t votes[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    uint16_t senseMs = 0;
    bool frontHeard = false;

    maze::Dir turnTo = maze::Dir::North;
    uint8_t turnTries = 0;
    uint8_t frontTries = 0;
    uint8_t rescans = 0;
    uint8_t resumes = 0;

    // The step in progress: whether the explorer has been moved into its target cell, how many
    // cells it was moved, and whether the planner has chosen what happens at that centre.
    bool entered = false;
    uint8_t advanced = 0;
    bool decided = false;
    bool resumeNext = false;
    bool afterStop = false;  // standing after a step cut short: decide what stopped it
    bool finishHere = false; // the tile ahead is white: stop on its centre
    uint16_t whiteMs = 0;

    float headingFixDeg = 0.0f;
    uint16_t moves = 0;
    uint16_t turns = 0;
    uint16_t blocked = 0; // steps that ended short of their centre
    uint16_t resumed = 0; // ... carried on to it (no wall explained the stop)
};

inline void sectionAReset(SectionA &ms)
{
    ms = SectionA{};
    maze::explorerInit(ms.map);
    motionControlReset(ms.control);
}

// The caller found a precondition missing before the run.
inline void sectionAFail(SectionA &ms, SectionAFault code)
{
    ms.phase = SectionAPhase::Fault;
    ms.fault = code;
}

// Degrees to subtract from the gyro heading (headingSet(hd, hd.deg - fix)); see StepCell.h.
inline float sectionATakeHeadingFix(SectionA &ms)
{
    const float fix = ms.headingFixDeg;
    ms.headingFixDeg = 0.0f;
    return fix;
}

namespace section_a_detail
{

// The gyro heading when the robot is square to map direction `d`. Map directions run clockwise,
// the gyro counter-clockwise.
inline float gridHeading(const SectionA &ms, maze::Dir d)
{
    return headingWrapDeg(ms.h0 - 90.0f * static_cast<float>(static_cast<uint8_t>(d)));
}

// The sweep that ends exactly on `targetDeg` going the way the quarter turns say (right is
// negative). Turning from where the robot is, not from a nominal 90, keeps the last turn's error
// out of this one.
inline float sweepTo(float fromDeg, float targetDeg, uint8_t quarters)
{
    float d = headingWrapDeg(targetDeg - fromDeg);
    if (quarters == 1 && d >= 0.0f) {
        d -= 360.0f;
    } else if (quarters != 1 && d <= 0.0f) {
        d += 360.0f; // left, or a half turn counter-clockwise
    }
    return d;
}

inline void clearVotes(SectionA &ms)
{
    for (auto &row : ms.votes) {
        for (auto &v : row) {
            v = 0;
        }
    }
    ms.senseMs = 0;
}

inline void observe(SectionA &ms, bool front, bool frontHeard, bool left, bool leftHeard,
                    bool right, bool rightHeard)
{
    maze::explorerObserve(ms.map, front && frontHeard, left && leftHeard, right && rightHeard);
    ms.frontHeard = frontHeard;
}

// Record what the standing votes say.
inline void observeVotes(SectionA &ms)
{
    bool unreliable = false;
    const bool f = step_detail::verdict(ms.votes[0], unreliable);
    const bool l = step_detail::verdict(ms.votes[1], unreliable);
    const bool r = step_detail::verdict(ms.votes[2], unreliable);
    observe(ms, f, step_detail::heard(ms.votes[0]), l, step_detail::heard(ms.votes[1]), r,
            step_detail::heard(ms.votes[2]));
}

inline void observeWalls(SectionA &ms, const CellWalls &w)
{
    observe(ms, w.front, w.frontHeard, w.left, w.leftHeard, w.right, w.rightHeard);
}

// The explorer is `advanced` cells into the step; make that the number the odometer says.
inline void reconcile(SectionA &ms, const StepCellParams &sp, const MotionFeedback &fb)
{
    int cells = stepCellCells(ms.step, sp, fb);
    cells = cells < 0 ? 0 : cells;
    while (ms.advanced < cells && maze::explorerAdvance(ms.map)) {
        ++ms.advanced;
        ++ms.moves;
    }
    while (ms.advanced > cells && ms.advanced > 0) {
        maze::explorerRetreat(ms.map);
        --ms.advanced;
    }
}

inline void beginSense(SectionA &ms)
{
    clearVotes(ms);
    ms.phase = SectionAPhase::Sense;
}

// Choose what happens at this centre with the walls known: face and drive, read again, or end.
// Called when the robot stands still on a centre.
inline void planNext(SectionA &ms, const SectionAParams &p, const MotionFeedback &fb)
{
    using namespace maze;
    for (;;) {
        Dir next = Dir::North;
        if (!explorerPlan(ms.map, next)) {
            if (p.wallsOnly) {
                ms.phase = SectionAPhase::Done; // every reachable tile has been entered
                return;
            }
            if (ms.rescans >= p.rescans) {
                sectionAFail(ms, SectionAFault::NoRoute);
                return;
            }
            ++ms.rescans; // a wall that was only a bad reading can strand the search
            explorerRescan(ms.map);
            beginSense(ms);
            return;
        }
        if (next != ms.map.heading) {
            const uint8_t quarters = static_cast<uint8_t>(
                (static_cast<uint8_t>(next) + 4 - static_cast<uint8_t>(ms.map.heading)) & 3);
            const float target = gridHeading(ms, next);
            ms.turnTo = next;
            ms.turnTries = 0;
            turnBegin(ms.turn, fb, target, sweepTo(fb.headingDeg, target, quarters));
            ms.phase = SectionAPhase::Turn;
            return;
        }
        if (!ms.frontHeard) {
            // A front that is not heard may be a wall in the sensor's dead zone: look again, and
            // after enough tries take it as one.
            if (++ms.frontTries < p.frontTries) {
                beginSense(ms);
                return;
            }
            ms.frontTries = 0;
            maze::explorerObserve(ms.map, true, false, false);
            ms.frontHeard = true;
            continue;
        }
        ms.frontTries = 0;
        stepCellBegin(ms.step, p.step, fb, gridHeading(ms, ms.map.heading));
        ms.entered = false;
        ms.advanced = 0;
        ms.decided = false;
        ms.resumeNext = false;
        ms.finishHere = false;
        ms.phase = SectionAPhase::Step;
        return;
    }
}

// The step's walls are in (the explorer is in the cell ahead): record them and decide whether to
// roll on, or stop on this centre.
inline void onWallsReady(SectionA &ms, const SectionAParams &p)
{
    if (!ms.entered) {
        if (!maze::explorerAdvance(ms.map)) {
            sectionAFail(ms, SectionAFault::NoRoute); // the map says a wall is there
            return;
        }
        ms.entered = true;
        ++ms.advanced;
        ++ms.moves;
    }
    observeWalls(ms, ms.step.walls);
    ms.finishHere = !p.wallsOnly && ms.whiteMs >= p.finishWhiteMs && ms.map.pos != maze::kStartCell;
    maze::Dir next = ms.map.heading;
    if (!ms.finishHere && maze::explorerPlan(ms.map, next) && next == ms.map.heading &&
        ms.frontHeard) {
        stepCellContinue(ms.step, p.step); // the next tile is straight on: no stop
        ms.entered = false;
        return;
    }
    ms.decided = true; // the step stops on this centre by itself
}

} // namespace section_a_detail

// One tick. Returns what the drivetrain should do.
inline DriveCommand sectionAStep(SectionA &ms, const SectionAParams &p, const Perception &pc)
{
    using namespace section_a_detail;
    const MotionFeedback &fb = pc.motion;
    MotionTarget target;
    target.distRefMm = fb.distMm; // default: stand where we are, square to the map direction
    target.headingRefDeg =
        ms.phase == SectionAPhase::WaitStart ? fb.headingDeg : gridHeading(ms, ms.map.heading);
    target.newMove = true;

    // White under the array, for how long.
    const bool white = !p.wallsOnly && pc.line.valid && !pc.line.allBlack;
    if (!white) {
        ms.whiteMs = 0;
    } else if (ms.whiteMs < UINT16_MAX - p.dtMs) {
        ms.whiteMs = static_cast<uint16_t>(ms.whiteMs + p.dtMs);
    }

    DriveCommand cmd;
    switch (ms.phase) {
        case SectionAPhase::WaitStart:
            if (pc.startPressed) {
                ms.h0 = fb.headingDeg;
                beginSense(ms);
            }
            return cmd; // motors off until the run starts

        case SectionAPhase::Sense: {
            step_detail::vote(ms.votes[0], pc.walls.frontValid, pc.walls.front);
            step_detail::vote(ms.votes[1], pc.walls.leftValid, pc.walls.left);
            step_detail::vote(ms.votes[2], pc.walls.rightValid, pc.walls.right);
            ms.senseMs = static_cast<uint16_t>(ms.senseMs + p.dtMs);
            if (ms.senseMs >= p.senseMs && ms.afterStop) {
                ms.afterStop = false;
                bool unreliable = false;
                const bool wallAhead = step_detail::verdict(ms.votes[0], unreliable) ||
                                       !step_detail::heard(ms.votes[0]);
                if (!wallAhead && ms.resumes < p.resumes) {
                    // No wall explains the stop: carry on to the planned centre.
                    ++ms.resumes;
                    ++ms.resumed;
                    ms.resumeNext = true;
                    ms.decided = false;
                    ms.phase = SectionAPhase::Step;
                } else {
                    ms.resumes = 0;
                    reconcile(ms, p.step, fb); // the cell it stopped on, by the odometer
                    observeVotes(ms);
                    planNext(ms, p, fb);
                }
            } else if (ms.senseMs >= p.senseMs) {
                observeVotes(ms);
                if (p.finishAtStart && !p.wallsOnly && ms.moves == 0 && ms.turns == 0 &&
                    ms.whiteMs >= p.finishWhiteMs) {
                    ms.phase = SectionAPhase::Done;
                } else {
                    planNext(ms, p, fb);
                }
            }
            cmd.enable = ms.phase != SectionAPhase::Fault;
            break;
        }

        case SectionAPhase::Turn: {
            cmd.enable = true;
            target.pivot = true;
            const MoveStatus st = turnStep(ms.turn, p.step.plan, fb, p.dtMs, target);
            if (st == MoveStatus::Done) {
                maze::explorerFace(ms.map, ms.turnTo);
                ++ms.turns;
                beginSense(ms); // the new front may be an edge nothing has looked at
            } else if (st == MoveStatus::Stuck) {
                if (++ms.turnTries > p.turnRetries) {
                    sectionAFail(ms, SectionAFault::StuckTurn);
                    return DriveCommand{};
                }
                const float t = gridHeading(ms, ms.turnTo);
                const uint8_t q = static_cast<uint8_t>(
                    (static_cast<uint8_t>(ms.turnTo) + 4 - static_cast<uint8_t>(ms.map.heading)) &
                    3);
                turnBegin(ms.turn, fb, t, sweepTo(fb.headingDeg, t, q));
            }
            break;
        }

        case SectionAPhase::Step: {
            cmd.enable = true;
            if (ms.resumeNext) {
                stepCellResume(ms.step, p.step, fb, gridHeading(ms, ms.map.heading));
                ms.resumeNext = false;
            }
            const MoveStatus st = stepCellStep(ms.step, p.step, fb, pc.walls, p.dtMs, target);
            ms.headingFixDeg += stepCellTakeHeadingFix(ms.step);
            if (ms.step.wallsReady && !ms.decided && !ms.step.blocked) {
                onWallsReady(ms, p);
                if (ms.phase == SectionAPhase::Fault) {
                    return DriveCommand{};
                }
            }
            if (st == MoveStatus::Stuck) {
                sectionAFail(ms, SectionAFault::StuckStep);
                return DriveCommand{};
            }
            if (st == MoveStatus::Done) {
                if (ms.step.blocked) {
                    ++ms.blocked;
                    if (ms.blocked > p.maxBlocked) {
                        sectionAFail(ms, SectionAFault::TooManyStops);
                        return DriveCommand{};
                    }
                    ms.afterStop = true; // stand and look: a wall, or something that has gone
                    beginSense(ms);
                } else if (!ms.decided) {
                    reconcile(ms, p.step, fb); // stopped without a verdict
                    beginSense(ms);
                } else {
                    ms.resumes = 0;
                    if (ms.finishHere) {
                        ms.phase = SectionAPhase::Done;
                    } else {
                        planNext(ms, p, fb);
                    }
                }
            }
            break;
        }

        case SectionAPhase::Done:
            cmd.enable = true; // zero duty with the driver enabled holds the wheels
            break;

        case SectionAPhase::Fault:
            return cmd;
    }

    if (ms.phase == SectionAPhase::Fault) {
        return DriveCommand{};
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
