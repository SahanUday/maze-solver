// ============================================================================
//  mission/SectionB.h  --  Section B's maze: explore from the entry tile until the finish is told,
//  then prove the path to it is the shortest. Pure logic, no hardware, no Arduino.h - testable
//  under env:native.
//
//    sectionBEnter()          begin the map on the entry tile (BEntry)
//    sectionBExploreStep()    explore until a stop source says "finish" (BExplore)
//    sectionBProveStep()      look at the edges that could still shorten the path (BProve)
//
//  The explorer is the same as Section A's, on the 17x17 start-relative map (ADR 0014, 0019). The
//  marker is not watched: the entry tile has one on it and it is not a goal. What says "finish" is
//  plugged in (FinishSource.h). The proof is by tile count (lib/maze/Proof.h); estimating time is
//  #76. docs/architecture/modules/section_b.md
// ============================================================================

#pragma once

#include <stdint.h>

#include "Maze.h"
#include "Proof.h"
#include "mission/CellExplorer.h"
#include "mission/Fault.h"
#include "mission/FinishSource.h"
#include "motion/MotionTypes.h"
#include "sense/Perception.h"
#include "spec/fixed/Rules.h"

using SectionBMap = maze::ExplorerT<SECTION_B_SIZE_TILES>;

enum class BStatus : uint8_t {
    Running,
    Done,  // explored and stopped on the finish, or the path is proved (or there was no finish)
    Fault, // `SectionB::fault` says why
};

// Where the proof is: reading the map, driving to the tile of an unchecked edge, or turning to it.
enum class ProveStep : uint8_t { Check, Go, Look };

struct SectionB {
    // The edge the robot came in through, closed on the map (sectionBEnter), and how many rescans
    // of the explorer have been seen: a rescan forgets every wall, so the closed edge is put back.
    bool closed = false;
    maze::Dir closedDir = maze::Dir::South;
    uint8_t rescansSeen = 0;

    bool finishKnown = false;
    SectionBMap::Index finish = 0; // the tile the robot stopped on when the finish was told

    ProveStep prove = ProveStep::Check;
    bool looked = false; // lookTile and lookDir hold the edge the last look was for
    SectionBMap::Index lookTile = 0;
    maze::Dir lookDir = maze::Dir::North;
    uint8_t lookTries = 0; // looks at that edge that recorded nothing
    uint16_t proved = 0;   // the proved path's length in tiles (0 until proved)
    uint16_t looks = 0;    // edges looked at for the proof
    MissionFault fault = MissionFault::None;
};

// Begin the map where the robot stands: North is its heading now. `closeBehind`: it came onto this
// tile through the bridge's opening, the edge behind it, which is no way into the maze. Marked a
// wall, so that the explorer never takes the tiles outside the maze for unvisited tiles to reach
// (the proof would otherwise drive out onto the board to look at an edge there). Without it, a
// robot placed on a tile has no way to tell the maze's edge: the fold of the map (#75) bounds that.
inline void sectionBEnter(SectionB &sb, CellExplorerB &ce, const MotionFeedback &fb,
                          bool closeBehind)
{
    cellExplorerStart(ce, fb);
    sb.closed = closeBehind;
    sb.closedDir = maze::opposite(ce.map.heading);
    sb.rescansSeen = ce.rescans;
    if (closeBehind) {
        maze::markWall(ce.map, ce.map.pos, sb.closedDir);
    }
}

// One tick of the exploring. Done when the finish was told and the robot stands on its tile. With
// no finish source plugged in the search ends when every reachable tile has been entered, and no
// finish is known.
inline BStatus sectionBExploreStep(SectionB &sb, CellExplorerB &ce, const CellExplorerParams &p,
                                   const FinishSource &fs, const Perception &pc,
                                   MotionTarget &target)
{
    CellExplorerParams ep = p;
    ep.wallsOnly = p.wallsOnly || !finishPlugged(fs);

    // Told while stepping: stop on the tile being driven to, as the marker does in Section A. The
    // finish has to be told before the robot commits to the next tile, about 50 mm before the
    // finish tile's centre: told later, it stops one tile further on and `sb.finish` is wrong.
    if (ce.phase == ExplorePhase::Step && !ce.stopHere && finishTold(fs, pc)) {
        cellExplorerStopHere(ce);
    }
    const ExploreInput in;
    const ExploreStatus st = cellExplorerStep(ce, ep, in, pc, target);
    if (ce.rescans != sb.rescansSeen) {
        sb.rescansSeen = ce.rescans; // the walls were forgotten: the way in is a wall still
        if (sb.closed) {
            maze::markWall(ce.map, SectionBMap::kStart, sb.closedDir);
        }
    }
    switch (st) {
        case ExploreStatus::Running:
            return BStatus::Running;
        case ExploreStatus::Stopped:
            sb.finish = ce.map.pos;
            sb.finishKnown = true;
            return BStatus::Done;
        case ExploreStatus::Explored:
        case ExploreStatus::AtTarget:
        case ExploreStatus::Braked:
            return BStatus::Done;
        case ExploreStatus::Fault:
            sb.fault = ce.fault;
            return BStatus::Fault;
    }
    sb.fault = MissionFault::NoRoute;
    return BStatus::Fault;
}

// One tick of the proof. Each round reads the map: proved, or an unchecked edge that could still
// make the path shorter. The robot drives to that edge's tile over checked edges, faces the edge
// and stands to read it. A look that records nothing (no echo either way) is tried `frontTries`
// times, then the edge is taken as a wall, as the explorer takes a silent front.
inline BStatus sectionBProveStep(SectionB &sb, CellExplorerB &ce, const CellExplorerParams &p,
                                 const Perception &pc, MotionTarget &target)
{
    if (!sb.finishKnown) {
        return BStatus::Done; // nothing to prove
    }
    switch (sb.prove) {
        case ProveStep::Check: {
            const maze::Proof pr = maze::explorerProof(ce.map, SectionBMap::kStart, sb.finish);
            if (pr.status == maze::ProofStatus::NoRoute) {
                sb.fault = MissionFault::NoRoute;
                return BStatus::Fault;
            }
            if (pr.status == maze::ProofStatus::Proved) {
                sb.proved = pr.length;
                return BStatus::Done;
            }
            SectionBMap::Index tile = static_cast<SectionBMap::Index>(pr.lookTile);
            maze::Dir dir = pr.lookDir;
            // The proof chose the edge by what is reachable from the entry tile, but the robot drives
            // from where it stands. With no checked way from here to that side of the edge, look at
            // it from its other side; with none to either, end the proof unproved.
            maze::Dir first;
            if (tile != ce.map.pos && !maze::explorerRoute(ce.map, ce.map.pos, tile, first)) {
                SectionBMap::Index other = 0;
                if (!SectionBMap::neighbour(tile, dir, other) ||
                    (other != ce.map.pos &&
                     !maze::explorerRoute(ce.map, ce.map.pos, other, first))) {
                    return BStatus::Done;
                }
                tile = other;
                dir = maze::opposite(dir);
            }
            if (sb.looked && tile == sb.lookTile && dir == sb.lookDir) {
                if (++sb.lookTries >= p.frontTries) {
                    maze::markWall(ce.map, tile, dir); // nothing answers: a wall
                    sb.looked = false;
                    sb.lookTries = 0;
                    return BStatus::Running;
                }
            } else {
                sb.lookTries = 0;
            }
            sb.looked = true;
            sb.lookTile = tile;
            sb.lookDir = dir;
            ++sb.looks;
            cellExplorerClearStop(ce);
            cellExplorerDriveTo(ce, tile);
            sb.prove = ProveStep::Go;
            return BStatus::Running;
        }
        case ProveStep::Go:
        case ProveStep::Look: {
            const ExploreInput in;
            switch (cellExplorerStep(ce, p, in, pc, target)) {
                case ExploreStatus::Running:
                    return BStatus::Running;
                case ExploreStatus::AtTarget:
                    break;
                case ExploreStatus::Fault:
                    if (ce.fault == MissionFault::TargetUnreachable) {
                        ce.fault = MissionFault::None; // a wall seen on the way: choose again
                        sb.prove = ProveStep::Check;
                        return BStatus::Running;
                    }
                    sb.fault = ce.fault;
                    return BStatus::Fault;
                case ExploreStatus::Explored:
                case ExploreStatus::Stopped:
                case ExploreStatus::Braked:
                    sb.fault = MissionFault::NoRoute; // not reachable in the target mode
                    return BStatus::Fault;
            }
            if (sb.prove == ProveStep::Go && ce.map.heading != sb.lookDir &&
                (ce.map.cell[ce.map.pos] & maze::checkedBit(sb.lookDir)) == 0) {
                cellExplorerBeginTurn(ce, pc.motion, sb.lookDir); // then Sense reads the edge
                sb.prove = ProveStep::Look;
            } else {
                sb.prove = ProveStep::Check;
            }
            return BStatus::Running;
        }
    }
    sb.fault = MissionFault::NoRoute;
    return BStatus::Fault;
}
