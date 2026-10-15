// ============================================================================
//  SectionA.h  --  the Section A mission: explore from any tile until the approach marker, then
//  drive out of the maze over the gap and follow the line. No hardware, no Arduino.h - testable
//  under env:native.
//
//    WaitStart -> Sense -> Turn -> Sense -> Step -> ... -> Approach -> Line
//                                          any -> Fault          (Done: see below)
//
//  Sense stands still and reads the cell's three walls; the planner (lib/maze) picks the next
//  tile; Turn faces it; Step drives it, reading the walls of the tile ahead on the way and rolling
//  on through the centre when the next tile is straight on. The marker (white frame, black box,
//  white frame under the array) ends the search: Approach drives on over the strip and the gap (a
//  quarter turn first if the way out is to a side), and the first line view past the gap hands
//  over to the line follower. Done is only reached without the line (`followLine` false: stop on
//  the marker's tile) or without a calibration (`wallsOnly`: everything explored).
//  docs/architecture/modules/section_a.md
// ============================================================================

#pragma once

#include <stdint.h>

#include "DriveCommand.h"
#include "FloorPattern.h"
#include "LineFollow.h"
#include "Maze.h"
#include "MotionControl.h"
#include "MotionPrimitives.h"
#include "Perception.h"
#include "RobotSpec.h"
#include "StepCell.h"

constexpr float MARKER_STILL_MM_S = 5.0f; // below this the robot has stopped
constexpr uint16_t MARKER_STILL_MS = 150; // ... for this long

// Approach drives on over the marker, the strip and the gap before the ramp; Line follows the line.
// They come last so the numbers the log prints for the older phases stay.
enum class SectionAPhase : uint8_t { WaitStart, Sense, Turn, Step, Done, Fault, Approach, Line };

enum class SectionAFault : uint8_t {
    None,
    NoImu,        // set by the caller: no gyro, no heading, no turns
    NoRoute,      // nothing left to explore, the rescans used, the white tile not found
    StuckTurn,    // a turn made no progress, retried
    StuckStep,    // a step made no progress
    Stall,        // motors pushed hard and nothing moved
    TooManyStops, // blocked steps over the limit: something keeps stopping the robot
    TimeUp,       // set by the caller: the run limit passed before the run ended
    NoLine,       // drove on from the marker and found no line past the gap
    LineLost      // the line was gone, and searched for in vain
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
    float stuckAcceptMm = SECTION_A_STUCK_ACCEPT_MM;
    PatternParams pattern;
    // After the marker: drive on at approachSpeedMmS through the gap, hand over to the follower at
    // the first line past it, give up if there is none within noLineMm of the marker's centre.
    // followLine false stops on the marker's tile instead (Done).
    bool followLine = SECTION_A_FOLLOW_LINE;
    float approachSpeedMmS = LINE_SPEED_MM_S;
    float noLineMm = APPROACH_NO_LINE_MM;
    float handoverMinMm = BRIDGE_HANDOVER_MIN_MM;
    LineParams line;
    // The marker is told by the floor sensor; without it (the array not calibrated) the run
    // explores every reachable tile and stops (Done).
    bool wallsOnly = false;
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
    bool frontEverHeard = false; // the front sensor has heard a wall in this run: it works
    // Edges of the cell the robot is in (bit = maze::Dir) that a sensor heard open on the way in.
    // Standing, a silent front is ambiguous (a wall inside the dead zone is silent too); an edge
    // that a side sensor measured on the approach is not.
    uint8_t openHeard = 0;
    // Edges of this cell a side sensor was turned to look at (peeked), and those it found open.
    // A side sensor has no dead zone: it settles a silent front, and costs a quarter turn.
    uint8_t peekTried = 0;
    uint8_t peekedOpen = 0;
    bool peeking = false;
    maze::Dir peekEdge = maze::Dir::North;
    bool sideEverHeard = false; // a side sensor has heard a wall in this run: they work

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
    bool finishHere = false; // the marker has been seen: stop on this tile
    bool brakingAtMarker = false;
    uint16_t markerStillMs = 0; // standing still since braking on the marker
    FloorPattern floor;
    float markerCentreMm = 0.0f; // the black box's middle, on the odometer of the array's line
    Straight approach;
    BridgeHandover bridge;
    bool gapSeen = false;
    bool exiting = false; // turning to the marker tile's open edge
    // Standing on the marker's tile to find the way out: the side readings, a silent one counted
    // as the sensor's range, sampled every other tick.
    bool choosingExit = false;
    uint16_t exitMm[2][16] = {{0}, {0}};
    uint8_t exitN = 0;
    LineFollow line;

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
    ms.frontEverHeard = ms.frontEverHeard || step_detail::heard(ms.votes[0]);
    if (ms.peeking) {
        // The edge was turned to the right-hand sensor (the robot turned left): a wall it heard, or
        // an answer from far, or silence from a sensor that is known to work, all say what is
        // there.
        ms.peeking = false;
        const bool heard = step_detail::heard(ms.votes[2]);
        if (!(heard && r) && (heard || ms.sideEverHeard)) {
            ms.peekedOpen =
                static_cast<uint8_t>(ms.peekedOpen | (1u << static_cast<uint8_t>(ms.peekEdge)));
        }
    }
    ms.sideEverHeard =
        ms.sideEverHeard || step_detail::heard(ms.votes[1]) || step_detail::heard(ms.votes[2]);
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
        const uint8_t headBit = static_cast<uint8_t>(1u << static_cast<uint8_t>(ms.map.heading));
        const bool openOnApproach = ((ms.openHeard | ms.peekedOpen) & headBit) != 0;
        if (!ms.frontHeard && !openOnApproach && !maze::explorerAheadVisited(ms.map)) {
            if (ms.sideEverHeard && (ms.peekTried & headBit) == 0) {
                // Silent: a wall inside the front sensor's dead zone, or nothing for metres (the
                // maze's exit). Turn a quarter left so the right-hand sensor, which has no dead
                // zone, looks at this edge.
                ms.peekTried = static_cast<uint8_t>(ms.peekTried | headBit);
                ms.peekEdge = ms.map.heading;
                ms.peeking = true;
                const Dir left = turnLeft(ms.map.heading);
                const float target = gridHeading(ms, left);
                ms.turnTo = left;
                ms.turnTries = 0;
                turnBegin(ms.turn, fb, target, sweepTo(fb.headingDeg, target, 3));
                ms.phase = SectionAPhase::Turn;
                return;
            }
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
        ms.step.openAhead = openOnApproach || maze::explorerAheadVisited(ms.map);
        ms.entered = false;
        ms.advanced = 0;
        ms.decided = false;
        ms.resumeNext = false;
        ms.phase = SectionAPhase::Step;
        return;
    }
}

// Drive on from here, straight along the grid heading, over the strip and the gap. The pattern
// starts afresh: what the array has seen so far belongs to the marker.
inline void beginApproach(SectionA &ms, const SectionAParams &p, const MotionFeedback &fb)
{
    straightBegin(ms.approach, fb, 1000.0f, p.approachSpeedMmS, 0.0f,
                  gridHeading(ms, ms.map.heading));
    bridgeHandoverReset(ms.bridge);
    floorPatternReset(ms.floor);
    ms.gapSeen = false;
    ms.phase = SectionAPhase::Approach;
}

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

// The marker's tile is a dead end with one open edge, which leads out of the maze. Which way it
// lies, from the walls the map holds for this cell: straight on, or a quarter turn.
inline bool exitIsOpen(const SectionA &ms, maze::Dir d)
{
    return (ms.map.cell[ms.map.pos] & maze::wallBit(d)) == 0;
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
    CellWalls w = ms.step.walls;
    ms.frontEverHeard = ms.frontEverHeard || w.frontHeard;
    if (!w.frontHeard && ms.frontEverHeard) {
        // The window opens a wall's distance ahead of its tile (167-197 mm from the axle, past the
        // sensor's dead zone) and nothing answered: the tile is open to beyond the sensor's range,
        // as at the maze's exit. Standing at a centre this is ambiguous (a wall inside the dead
        // zone is silent too), which is why Sense takes an unheard front for a wall.
        w.front = false;
        w.frontHeard = true;
    }
    observeWalls(ms, w);
    ms.sideEverHeard = ms.sideEverHeard || w.leftHeard || w.rightHeard;
    ms.peekTried = 0;
    ms.peekedOpen = 0;
    {
        const uint8_t h = static_cast<uint8_t>(ms.map.heading);
        ms.openHeard = 0;
        ms.openHeard =
            static_cast<uint8_t>(ms.openHeard | ((w.frontHeard && !w.front ? 1u : 0u) << h));
        ms.openHeard = static_cast<uint8_t>(
            ms.openHeard | ((w.rightHeard && !w.right ? 1u : 0u) << ((h + 1) & 3)));
        ms.openHeard = static_cast<uint8_t>(ms.openHeard |
                                            ((w.leftHeard && !w.left ? 1u : 0u) << ((h + 3) & 3)));
    }
    maze::Dir next = ms.map.heading;
    if (!ms.finishHere && maze::explorerPlan(ms.map, next) && next == ms.map.heading &&
        ms.frontHeard) {
        stepCellContinue(ms.step, p.step); // the next tile is straight on: no stop
        ms.step.openAhead = ms.step.openAhead || (w.frontHeard && !w.front && !ms.finishHere);
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

    // The marker: white, black, white under the array along the path. The event comes as the far
    // frame is crossed, which puts the robot's axle about at the tile's centre.
    // Only while stepping: the pattern is a length along the path, and a pivot at a centre sweeps
    // the array across the frame with the odometer standing still (or creeping by a wheel's slip).
    if (!p.wallsOnly && !ms.finishHere && ms.phase == SectionAPhase::Step) {
        float centre = 0.0f;
        if (floorPatternStep(ms.floor, pc.floor, fb.distMm + IR_ARRAY_FORWARD_OFFSET_MM, p.pattern,
                             centre) == PatternEvent::Marker) {
            ms.markerCentreMm = centre;
            ms.finishHere = true;
            ms.decided = true; // no wall or planner decides this centre any more
            if (p.followLine && exitIsOpen(ms, ms.map.heading)) {
                beginApproach(ms, p, fb); // straight through: no stop
            }
        }
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
            if (ms.choosingExit) {
                if (ms.exitN < 16 && ((ms.senseMs / p.dtMs) & 1u) != 0) {
                    ms.exitMm[0][ms.exitN] = pc.walls.leftValid ? pc.walls.leftMm : US_RANGE_CAP_MM;
                    ms.exitMm[1][ms.exitN] =
                        pc.walls.rightValid ? pc.walls.rightMm : US_RANGE_CAP_MM;
                    ++ms.exitN;
                }
                if (ms.senseMs >= p.senseMs) {
                    // Both sides may be unwalled: the other tile of the maze, and the way out. The
                    // way out is the one that answers from far or not at all: a tile's far wall is
                    // within 2.5 m, the room beyond the exit is not. Medians, for the dropouts.
                    ms.choosingExit = false;
                    const maze::Dir left = maze::turnLeft(ms.map.heading);
                    const maze::Dir right = maze::turnRight(ms.map.heading);
                    const bool leftOpen = exitIsOpen(ms, left);
                    const bool rightOpen = exitIsOpen(ms, right);
                    if (!leftOpen && !rightOpen) {
                        sectionAFail(ms, SectionAFault::NoLine); // no way out of the tile
                        return DriveCommand{};
                    }
                    const uint16_t leftMm = medianOf(ms.exitMm[0], ms.exitN);
                    const uint16_t rightMm = medianOf(ms.exitMm[1], ms.exitN);
                    const bool toLeft = leftOpen && (!rightOpen || leftMm >= rightMm);
                    const maze::Dir to = toLeft ? left : right;
                    const float aim = gridHeading(ms, to);
                    ms.turnTo = to;
                    ms.turnTries = 0;
                    ms.exiting = true;
                    turnBegin(ms.turn, fb, aim, sweepTo(fb.headingDeg, aim, toLeft ? 3 : 1));
                    ms.phase = SectionAPhase::Turn;
                }
                cmd.enable = ms.phase != SectionAPhase::Fault;
                break;
            }
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
                planNext(ms, p, fb);
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
                if (ms.exiting) {
                    beginApproach(ms, p, fb); // facing the open edge: out of the maze
                } else {
                    beginSense(ms); // the new front may be an edge nothing has looked at
                }
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
            MoveStatus st = stepCellStep(ms.step, p.step, fb, pc.walls, p.dtMs, target);
            ms.headingFixDeg += stepCellTakeHeadingFix(ms.step);
            if (ms.finishHere && !p.followLine && !ms.brakingAtMarker) {
                // On the marker's tile: stop as soon as the wheels can, wherever the step was
                // going (rolling on through the centre, or stopping on it).
                reconcile(ms, p.step, fb); // the tile the odometer says it is on
                ms.brakingAtMarker = true;
                straightRetarget(ms.step.straight, fb.distMm, 0.0f, p.step.plan.accelMmS2);
            }
            if (ms.brakingAtMarker) {
                // Where it stops does not matter, only that it has: a wheel that sticks a little
                // short of the braking point is not a stuck step.
                const bool still = ms.step.straight.refSpeedMmS == 0.0f &&
                                   fb.speedMmS < MARKER_STILL_MM_S &&
                                   fb.speedMmS > -MARKER_STILL_MM_S;
                ms.markerStillMs = still ? static_cast<uint16_t>(ms.markerStillMs + p.dtMs) : 0;
                if (ms.markerStillMs >= MARKER_STILL_MS) {
                    ms.phase = SectionAPhase::Done;
                    DriveCommand hold;
                    hold.enable = true; // zero duty with the driver enabled holds the wheels
                    return hold;
                }
            }
            if (ms.step.wallsReady && !ms.decided && !ms.step.blocked && !ms.finishHere) {
                onWallsReady(ms, p);
                if (ms.phase == SectionAPhase::Fault) {
                    return DriveCommand{};
                }
            }
            if (st == MoveStatus::Stuck && !ms.brakingAtMarker) {
                // Wheels that stuck a little short of the stop point, outside the arrival
                // tolerance, are an arrival: the next move starts from where it stands, and the
                // walls put the centre right again.
                float err = ms.step.straight.targetMm - fb.distMm;
                err = err < 0.0f ? -err : err;
                if (err <= p.stuckAcceptMm && fb.speedMmS < MARKER_STILL_MM_S &&
                    fb.speedMmS > -MARKER_STILL_MM_S) {
                    st = MoveStatus::Done;
                }
            }
            if (st == MoveStatus::Stuck && !ms.brakingAtMarker) {
                sectionAFail(ms, SectionAFault::StuckStep);
                return DriveCommand{};
            }
            if (st == MoveStatus::Done && !ms.finishHere && !p.wallsOnly) {
                // Stopped a little short of the centre, with the far frame not yet long enough to
                // be reported: it is still the marker.
                float centre = 0.0f;
                if (floorPatternMarkerPending(ms.floor, p.pattern,
                                              fb.distMm + IR_ARRAY_FORWARD_OFFSET_MM, centre)) {
                    ms.markerCentreMm = centre;
                    ms.finishHere = true;
                    ms.decided = true;
                }
            }
            if (st == MoveStatus::Done) {
                if (ms.finishHere && p.followLine) {
                    // Stopped on the marker's tile. With a wall ahead the way out is to a side:
                    // stand and read both sides before choosing.
                    reconcile(ms, p.step, fb);
                    if (exitIsOpen(ms, ms.map.heading)) {
                        beginApproach(ms, p,
                                      fb); // it stopped short of the pattern's end: straight on
                    } else {
                        ms.choosingExit = true;
                        ms.exitN = 0;
                        beginSense(ms);
                    }
                } else if (ms.finishHere) {
                    ms.phase = SectionAPhase::Done;
                } else if (ms.step.blocked) {
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
                    planNext(ms, p, fb);
                }
            }
            break;
        }

        case SectionAPhase::Approach: {
            cmd.enable = true;
            const float pos = fb.distMm + IR_ARRAY_FORWARD_OFFSET_MM;
            float centre = 0.0f;
            if (floorPatternStep(ms.floor, pc.floor, pos, p.pattern, centre) == PatternEvent::Gap) {
                ms.gapSeen = true;
            }
            if (bridgeHandoverStep(ms.bridge, ms.gapSeen, pc.floor, pc.line, pos,
                                   p.handoverMinMm)) {
                // The line is in view past the gap: the follower takes over, mid-stride.
                lineFollowBegin(ms.line, fb, true);
                ms.phase = SectionAPhase::Line;
                lineFollowStep(ms.line, p.line, fb, pc.line, pc.inclineCounts, p.dtMs, target);
            } else if (pos - ms.markerCentreMm >= p.noLineMm) {
                sectionAFail(ms, SectionAFault::NoLine);
                return DriveCommand{};
            } else {
                straightStep(ms.approach, p.step.plan, fb, 0.0f, p.dtMs, target);
            }
            break;
        }

        case SectionAPhase::Line: {
            cmd.enable = true;
            const LineStatus ls =
                lineFollowStep(ms.line, p.line, fb, pc.line, pc.inclineCounts, p.dtMs, target);
            if (ls == LineStatus::Lost) {
                sectionAFail(ms, SectionAFault::LineLost);
                return DriveCommand{};
            }
            if (ls == LineStatus::Ended) {
                ms.phase = SectionAPhase::Done;
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
