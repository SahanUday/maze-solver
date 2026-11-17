// ============================================================================
//  CellExplorer.h  --  explore a maze cell by cell: stand and read the walls, turn to the next tile
//  the planner picks, drive it. No hardware, no Arduino.h - testable under env:native.
//
//    Sense -> Turn -> Sense -> Step -> ... (Step rolls on through a centre when the tile ahead is
//    the next one)
//
//  Sense stands still and reads the cell's three walls; the planner (lib/maze) picks the next
//  tile; Turn faces it; Step drives it, reading the walls of the tile ahead on the way. The
//  explorer is generic in the maze size (ExplorerT<N>) and knows nothing of what ends the search.
//  Its caller can ask it to stop on the tile it is heading for (cellExplorerStopHere) and says,
//  each tick, whether a stop that was not yet asked for is due (ExploreInput); it tells the caller
//  when it has explored everything reachable, or has stopped where asked.
//  docs/architecture/modules/cell_explorer.md
// ============================================================================

#pragma once

#include <stdint.h>

#include "Maze.h"
#include "core/MathUtil.h"
#include "mission/Fault.h"
#include "mission/StepCell.h"
#include "motion/Heading.h"
#include "motion/MotionPrimitives.h"
#include "motion/MotionTypes.h"
#include "sense/Perception.h"
#include "spec/Mission.h"
#include "spec/Platform.h"

constexpr float STOP_STILL_MM_S = 5.0f; // below this the robot has stopped
constexpr uint16_t STOP_STILL_MS = 150; // ... for this long

enum class ExplorePhase : uint8_t { Sense, Turn, Step };

enum class ExploreStatus : uint8_t {
    Running,
    Explored, // nothing reachable is left unvisited (wallsOnly)
    Stopped,  // standing on the tile a stop was asked for, the map in step with the odometer
    Braked,   // the stop braked at once and the wheels have stopped
    Fault,    // `fault` says why
};

struct CellExplorerParams {
    StepCellParams step;
    uint16_t senseMs = SECTION_A_SENSE_MS;
    uint8_t frontTries = SECTION_A_FRONT_TRIES;
    uint8_t rescans = SECTION_A_RESCANS;
    uint8_t resumes = SECTION_A_RESUMES;
    uint8_t turnRetries = SECTION_A_TURN_RETRIES;
    uint8_t maxBlocked = SECTION_A_MAX_BLOCKED;
    float stuckAcceptMm = SECTION_A_STUCK_ACCEPT_MM;
    // The run reads walls only: the search ends (Explored) when every reachable tile has been
    // entered, instead of being rescanned and failed.
    bool wallsOnly = false;
    uint16_t dtMs = CONTROL_LOOP_PERIOD_MS;
};

// What the caller knows each tick.
struct ExploreInput {
    // A requested stop brakes at once, wherever the step was going, instead of running on to the
    // centre.
    bool brakeOnStop = false;
    // A stop is due but only once the step has ended (the pattern that asks for it was seen too
    // late to ask during the step): taken if the step ends this tick and no stop was asked for.
    bool lateStopOffer = false;
};

template <uint8_t N> struct CellExplorerT {
    ExplorePhase phase = ExplorePhase::Sense;
    MissionFault fault = MissionFault::None;

    maze::ExplorerT<N> map;
    float h0 = 0.0f; // the gyro heading at the start; map North is this

    StepCell step;
    Turn turn;

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
    bool afterStop = false; // standing after a step cut short: decide what stopped it

    // The caller asked for a stop on this tile: no wall or planner decides this centre any more.
    bool stopHere = false;
    bool lateTaken = false; // the stop was taken from the offer: the caller is to be told
    bool braking = false;
    uint16_t stillMs = 0; // standing still since braking

    float headingFixDeg = 0.0f;
    uint16_t moves = 0;
    uint16_t turns = 0;
    uint16_t blocked = 0; // steps that ended short of their centre
    uint16_t resumed = 0; // ... carried on to it (no wall explained the stop)
};

// The Section A map.
using CellExplorer = CellExplorerT<SECTION_A_SIZE_TILES>;

template <uint8_t N> inline void cellExplorerReset(CellExplorerT<N> &ce)
{
    ce = CellExplorerT<N>{};
    maze::explorerInit(ce.map);
}

namespace cell_explorer_detail
{

// The gyro heading when the robot is square to map direction `d`. Map directions run clockwise,
// the gyro counter-clockwise.
inline float gridHeading(float h0, maze::Dir d)
{
    return headingWrapDeg(h0 - 90.0f * static_cast<float>(static_cast<uint8_t>(d)));
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

template <uint8_t N> inline void clearVotes(CellExplorerT<N> &ce)
{
    for (auto &row : ce.votes) {
        for (auto &v : row) {
            v = 0;
        }
    }
    ce.senseMs = 0;
}

template <uint8_t N>
inline void observe(CellExplorerT<N> &ce, bool front, bool frontHeard, bool left, bool leftHeard,
                    bool right, bool rightHeard)
{
    maze::explorerObserve(ce.map, front && frontHeard, left && leftHeard, right && rightHeard);
    ce.frontHeard = frontHeard;
}

// Record what the standing votes say.
template <uint8_t N> inline void observeVotes(CellExplorerT<N> &ce)
{
    bool unreliable = false;
    const bool f = step_detail::verdict(ce.votes[0], unreliable);
    const bool l = step_detail::verdict(ce.votes[1], unreliable);
    const bool r = step_detail::verdict(ce.votes[2], unreliable);
    ce.frontEverHeard = ce.frontEverHeard || step_detail::heard(ce.votes[0]);
    if (ce.peeking) {
        // The edge was turned to the right-hand sensor (the robot turned left): a wall it heard, or
        // an answer from far, or silence from a sensor that is known to work, all say what is
        // there.
        ce.peeking = false;
        const bool heard = step_detail::heard(ce.votes[2]);
        if (!(heard && r) && (heard || ce.sideEverHeard)) {
            ce.peekedOpen =
                static_cast<uint8_t>(ce.peekedOpen | (1u << static_cast<uint8_t>(ce.peekEdge)));
        }
    }
    ce.sideEverHeard =
        ce.sideEverHeard || step_detail::heard(ce.votes[1]) || step_detail::heard(ce.votes[2]);
    observe(ce, f, step_detail::heard(ce.votes[0]), l, step_detail::heard(ce.votes[1]), r,
            step_detail::heard(ce.votes[2]));
}

template <uint8_t N> inline void observeWalls(CellExplorerT<N> &ce, const CellWalls &w)
{
    observe(ce, w.front, w.frontHeard, w.left, w.leftHeard, w.right, w.rightHeard);
}

// The explorer is `advanced` cells into the step; make that the number the odometer says.
template <uint8_t N>
inline void reconcile(CellExplorerT<N> &ce, const StepCellParams &sp, const MotionFeedback &fb)
{
    int cells = stepCellCells(ce.step, sp, fb);
    cells = cells < 0 ? 0 : cells;
    while (ce.advanced < cells && maze::explorerAdvance(ce.map)) {
        ++ce.advanced;
        ++ce.moves;
    }
    while (ce.advanced > cells && ce.advanced > 0) {
        maze::explorerRetreat(ce.map);
        --ce.advanced;
    }
}

template <uint8_t N> inline void beginSense(CellExplorerT<N> &ce)
{
    clearVotes(ce);
    ce.phase = ExplorePhase::Sense;
}

// Start a turn to face map direction `to` from where the robot is, without forgetting the tries.
template <uint8_t N>
inline void startTurn(CellExplorerT<N> &ce, const MotionFeedback &fb, maze::Dir to)
{
    const uint8_t quarters = static_cast<uint8_t>(
        (static_cast<uint8_t>(to) + 4 - static_cast<uint8_t>(ce.map.heading)) & 3);
    const float target = gridHeading(ce.h0, to);
    ce.turnTo = to;
    turnBegin(ce.turn, fb, target, sweepTo(fb.headingDeg, target, quarters));
    ce.phase = ExplorePhase::Turn;
}

// Choose what happens at this centre with the walls known: face and drive, read again, or end.
// Called when the robot stands still on a centre. True: the search is over (`wallsOnly` and every
// reachable tile entered).
template <uint8_t N>
inline bool planNext(CellExplorerT<N> &ce, const CellExplorerParams &p, const MotionFeedback &fb)
{
    using namespace maze;
    for (;;) {
        Dir next = Dir::North;
        if (!explorerPlan(ce.map, next)) {
            if (p.wallsOnly) {
                return true; // every reachable tile has been entered
            }
            if (ce.rescans >= p.rescans) {
                ce.fault = MissionFault::NoRoute;
                return false;
            }
            ++ce.rescans; // a wall that was only a bad reading can strand the search
            explorerRescan(ce.map);
            beginSense(ce);
            return false;
        }
        if (next != ce.map.heading) {
            ce.turnTries = 0;
            startTurn(ce, fb, next);
            return false;
        }
        const uint8_t headBit = static_cast<uint8_t>(1u << static_cast<uint8_t>(ce.map.heading));
        const bool openOnApproach = ((ce.openHeard | ce.peekedOpen) & headBit) != 0;
        if (!ce.frontHeard && !openOnApproach && !maze::explorerAheadVisited(ce.map)) {
            if (ce.sideEverHeard && (ce.peekTried & headBit) == 0) {
                // Silent: a wall inside the front sensor's dead zone, or nothing for metres (the
                // maze's exit). Turn a quarter left so the right-hand sensor, which has no dead
                // zone, looks at this edge.
                ce.peekTried = static_cast<uint8_t>(ce.peekTried | headBit);
                ce.peekEdge = ce.map.heading;
                ce.peeking = true;
                ce.turnTries = 0;
                startTurn(ce, fb, turnLeft(ce.map.heading));
                return false;
            }
            // A front that is not heard may be a wall in the sensor's dead zone: look again, and
            // after enough tries take it as one.
            if (++ce.frontTries < p.frontTries) {
                beginSense(ce);
                return false;
            }
            ce.frontTries = 0;
            maze::explorerObserve(ce.map, true, false, false);
            ce.frontHeard = true;
            continue;
        }
        ce.frontTries = 0;
        stepCellBegin(ce.step, p.step, fb, gridHeading(ce.h0, ce.map.heading));
        ce.step.openAhead = openOnApproach || maze::explorerAheadVisited(ce.map);
        ce.entered = false;
        ce.advanced = 0;
        ce.decided = false;
        ce.resumeNext = false;
        ce.phase = ExplorePhase::Step;
        return false;
    }
}

// The step's walls are in (the explorer is in the cell ahead): record them and decide whether to
// roll on, or stop on this centre. False: the map says a wall is there (NoRoute).
template <uint8_t N> inline bool onWallsReady(CellExplorerT<N> &ce, const CellExplorerParams &p)
{
    if (!ce.entered) {
        if (!maze::explorerAdvance(ce.map)) {
            ce.fault = MissionFault::NoRoute; // the map says a wall is there
            return false;
        }
        ce.entered = true;
        ++ce.advanced;
        ++ce.moves;
    }
    CellWalls w = ce.step.walls;
    ce.frontEverHeard = ce.frontEverHeard || w.frontHeard;
    if (!w.frontHeard && ce.frontEverHeard) {
        // The window opens a wall's distance ahead of its tile (167-197 mm from the axle, past the
        // sensor's dead zone) and nothing answered: the tile is open to beyond the sensor's range,
        // as at the maze's exit. Standing at a centre this is ambiguous (a wall inside the dead
        // zone is silent too), which is why Sense takes an unheard front for a wall.
        w.front = false;
        w.frontHeard = true;
    }
    observeWalls(ce, w);
    ce.sideEverHeard = ce.sideEverHeard || w.leftHeard || w.rightHeard;
    ce.peekTried = 0;
    ce.peekedOpen = 0;
    {
        const uint8_t h = static_cast<uint8_t>(ce.map.heading);
        ce.openHeard = 0;
        ce.openHeard =
            static_cast<uint8_t>(ce.openHeard | ((w.frontHeard && !w.front ? 1u : 0u) << h));
        ce.openHeard = static_cast<uint8_t>(
            ce.openHeard | ((w.rightHeard && !w.right ? 1u : 0u) << ((h + 1) & 3)));
        ce.openHeard = static_cast<uint8_t>(ce.openHeard |
                                            ((w.leftHeard && !w.left ? 1u : 0u) << ((h + 3) & 3)));
    }
    maze::Dir next = ce.map.heading;
    if (!ce.stopHere && maze::explorerPlan(ce.map, next) && next == ce.map.heading &&
        ce.frontHeard) {
        stepCellContinue(ce.step, p.step); // the next tile is straight on: no stop
        ce.step.openAhead = ce.step.openAhead || (w.frontHeard && !w.front && !ce.stopHere);
        ce.entered = false;
        return true;
    }
    ce.decided = true; // the step stops on this centre by itself
    return true;
}

} // namespace cell_explorer_detail

// Start exploring from where the robot stands: map North is its heading now.
template <uint8_t N> inline void cellExplorerStart(CellExplorerT<N> &ce, const MotionFeedback &fb)
{
    ce.h0 = fb.headingDeg;
    cell_explorer_detail::beginSense(ce);
}

// Stop on the tile the step is heading for. No wall or planner decides this centre any more.
template <uint8_t N> inline void cellExplorerStopHere(CellExplorerT<N> &ce)
{
    ce.stopHere = true;
    ce.decided = true;
}

// The edge `d` of the cell the robot is in is not a known wall.
template <uint8_t N> inline bool cellExplorerEdgeOpen(const CellExplorerT<N> &ce, maze::Dir d)
{
    return (ce.map.cell[ce.map.pos] & maze::wallBit(d)) == 0;
}

// The gyro heading when the robot is square to the map's current heading.
template <uint8_t N> inline float cellExplorerHeadingRef(const CellExplorerT<N> &ce)
{
    return cell_explorer_detail::gridHeading(ce.h0, ce.map.heading);
}

// Turn to face map direction `to`, the number of tries starting afresh. Run it with
// cellExplorerTurnStep(), or let cellExplorerStep() run it when the explorer asked for it.
template <uint8_t N>
inline void cellExplorerBeginTurn(CellExplorerT<N> &ce, const MotionFeedback &fb, maze::Dir to)
{
    ce.turnTries = 0;
    cell_explorer_detail::startTurn(ce, fb, to);
}

enum class TurnStatus : uint8_t { Running, Done, Fault };

// One tick of the turn in progress. Done: the map faces the new way. A turn that makes no progress
// is started again `turnRetries` times, then Fault (StuckTurn).
template <uint8_t N>
inline TurnStatus cellExplorerTurnStep(CellExplorerT<N> &ce, const CellExplorerParams &p,
                                       const MotionFeedback &fb, MotionTarget &target)
{
    target.pivot = true;
    const MoveStatus st = turnStep(ce.turn, p.step.plan, fb, p.dtMs, target);
    if (st == MoveStatus::Done) {
        maze::explorerFace(ce.map, ce.turnTo);
        ++ce.turns;
        return TurnStatus::Done;
    }
    if (st == MoveStatus::Stuck) {
        if (++ce.turnTries > p.turnRetries) {
            ce.fault = MissionFault::StuckTurn;
            return TurnStatus::Fault;
        }
        cell_explorer_detail::startTurn(ce, fb, ce.turnTo);
    }
    return TurnStatus::Running;
}

// One tick. Fills `target` for the motion controller (the caller owns it and applies it).
template <uint8_t N>
inline ExploreStatus cellExplorerStep(CellExplorerT<N> &ce, const CellExplorerParams &p,
                                      const ExploreInput &in, const Perception &pc,
                                      MotionTarget &target)
{
    using namespace cell_explorer_detail;
    const MotionFeedback &fb = pc.motion;

    switch (ce.phase) {
        case ExplorePhase::Sense: {
            step_detail::vote(ce.votes[0], pc.walls.frontValid, pc.walls.front);
            step_detail::vote(ce.votes[1], pc.walls.leftValid, pc.walls.left);
            step_detail::vote(ce.votes[2], pc.walls.rightValid, pc.walls.right);
            ce.senseMs = static_cast<uint16_t>(ce.senseMs + p.dtMs);
            if (ce.senseMs >= p.senseMs && ce.afterStop) {
                ce.afterStop = false;
                bool unreliable = false;
                const bool wallAhead = step_detail::verdict(ce.votes[0], unreliable) ||
                                       !step_detail::heard(ce.votes[0]);
                if (!wallAhead && ce.resumes < p.resumes) {
                    // No wall explains the stop: carry on to the planned centre.
                    ++ce.resumes;
                    ++ce.resumed;
                    ce.resumeNext = true;
                    ce.decided = false;
                    ce.phase = ExplorePhase::Step;
                } else {
                    ce.resumes = 0;
                    reconcile(ce, p.step, fb); // the cell it stopped on, by the odometer
                    observeVotes(ce);
                    if (planNext(ce, p, fb)) {
                        return ExploreStatus::Explored;
                    }
                }
            } else if (ce.senseMs >= p.senseMs) {
                observeVotes(ce);
                if (planNext(ce, p, fb)) {
                    return ExploreStatus::Explored;
                }
            }
            break;
        }

        case ExplorePhase::Turn: {
            const TurnStatus ts = cellExplorerTurnStep(ce, p, fb, target);
            if (ts == TurnStatus::Done) {
                beginSense(ce); // the new front may be an edge nothing has looked at
            }
            break;
        }

        case ExplorePhase::Step: {
            if (ce.resumeNext) {
                stepCellResume(ce.step, p.step, fb, cellExplorerHeadingRef(ce));
                ce.resumeNext = false;
            }
            MoveStatus st = stepCellStep(ce.step, p.step, fb, pc.walls, p.dtMs, target);
            ce.headingFixDeg += stepCellTakeHeadingFix(ce.step);
            if (ce.stopHere && in.brakeOnStop && !ce.braking) {
                // On the tile to stop on: stop as soon as the wheels can, wherever the step was
                // going (rolling on through the centre, or stopping on it).
                reconcile(ce, p.step, fb); // the tile the odometer says it is on
                ce.braking = true;
                straightRetarget(ce.step.straight, fb.distMm, 0.0f, p.step.plan.accelMmS2);
            }
            if (ce.braking) {
                // Where it stops does not matter, only that it has: a wheel that sticks a little
                // short of the braking point is not a stuck step.
                const bool still = ce.step.straight.refSpeedMmS == 0.0f &&
                                   fb.speedMmS < STOP_STILL_MM_S && fb.speedMmS > -STOP_STILL_MM_S;
                ce.stillMs = still ? static_cast<uint16_t>(ce.stillMs + p.dtMs) : 0;
                if (ce.stillMs >= STOP_STILL_MS) {
                    return ExploreStatus::Braked;
                }
            }
            if (ce.step.wallsReady && !ce.decided && !ce.step.blocked && !ce.stopHere) {
                if (!onWallsReady(ce, p)) {
                    return ExploreStatus::Fault;
                }
            }
            if (st == MoveStatus::Stuck && !ce.braking) {
                // Wheels that stuck a little short of the stop point, outside the arrival
                // tolerance, are an arrival: the next move starts from where it stands, and the
                // walls put the centre right again.
                const float err = absf(ce.step.straight.targetMm - fb.distMm);
                if (err <= p.stuckAcceptMm && fb.speedMmS < STOP_STILL_MM_S &&
                    fb.speedMmS > -STOP_STILL_MM_S) {
                    st = MoveStatus::Done;
                }
            }
            if (st == MoveStatus::Stuck && !ce.braking) {
                ce.fault = MissionFault::StuckStep;
                return ExploreStatus::Fault;
            }
            if (st == MoveStatus::Done && !ce.stopHere && in.lateStopOffer) {
                // Stopped a little short of the centre, with the far frame not yet long enough to
                // be reported: it is still the stop.
                cellExplorerStopHere(ce);
                ce.lateTaken = true;
            }
            if (st == MoveStatus::Done) {
                if (ce.stopHere) {
                    if (!in.brakeOnStop) {
                        reconcile(ce, p.step, fb);
                    }
                    return ExploreStatus::Stopped;
                }
                if (ce.step.blocked) {
                    ++ce.blocked;
                    if (ce.blocked > p.maxBlocked) {
                        ce.fault = MissionFault::TooManyStops;
                        return ExploreStatus::Fault;
                    }
                    ce.afterStop = true; // stand and look: a wall, or something that has gone
                    beginSense(ce);
                } else if (!ce.decided) {
                    reconcile(ce, p.step, fb); // stopped without a verdict
                    beginSense(ce);
                } else {
                    ce.resumes = 0;
                    if (planNext(ce, p, fb)) {
                        return ExploreStatus::Explored;
                    }
                }
            }
            break;
        }
    }
    return ce.fault != MissionFault::None ? ExploreStatus::Fault : ExploreStatus::Running;
}
