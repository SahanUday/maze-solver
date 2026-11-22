// ============================================================================
//  mission/Parts.h  --  the parts of a run. Pure logic, no hardware.
//
//  A part is a stretch of the course with one kind of logic. The robot tells it has left one by an
//  event it senses (the marker told, the line in view past the gap), never by a clock or a
//  position. In course order (`enum Part`, spec/tuning/Run.h):
//
//    AExplore  Section A's maze: explore cell by cell until the approach marker is told
//    AProve    Section A again: explore the rest, prove the path, return to the marker tile
//    AExit     the marker tile and the way out of it, over the gap to the line
//    Line      the line zone: bridge, ramp and bends, by the IR line view
//    BEntry    Section B's marker tile: off the line, onto the maze floor, a new map origin
//    BExplore  Section B's maze: explore until the finish is told
//    BProve    after the finish: prove the shortest path, save the maps
//
//  docs/architecture/decisions/0016-cut-the-run-into-parts.md
// ============================================================================

#pragma once

#include <stdint.h>

#include "spec/tuning/Run.h" // enum Part, PART_COUNT

// In this firmware? A run skips the parts that are not.
constexpr bool partBuilt(Part p)
{
    switch (p) {
        case Part::AExplore:
        case Part::AExit:
        case Part::Line:
            return true;
        case Part::AProve:
        case Part::BEntry:
        case Part::BExplore:
        case Part::BProve:
            return false;
    }
    return false;
}

// Can a run begin here, nothing handed over? AProve, AExit and BProve carry on a map.
constexpr bool partStartsAlone(Part p)
{
    switch (p) {
        case Part::AExplore:
        case Part::Line:
        case Part::BEntry:
        case Part::BExplore:
            return true;
        case Part::AProve:
        case Part::AExit:
        case Part::BProve:
            return false;
    }
    return false;
}

// Does the part work in the explorer's map frame (North = the heading at the start)?
constexpr bool partUsesMap(Part p)
{
    switch (p) {
        case Part::AExplore:
        case Part::AProve:
        case Part::AExit:
        case Part::BExplore:
        case Part::BProve:
            return true;
        case Part::Line:
        case Part::BEntry:
            return false;
    }
    return false;
}

// The parts to run, `first` to `last`. The whole course by default.
struct PartRange {
    Part first = Part::AExplore;
    Part last = Part::BProve;
};

// The last built part at or before `cap`.
inline bool partLastBuilt(Part cap, Part &out)
{
    for (uint8_t i = static_cast<uint8_t>(cap) + 1; i > 0; --i) {
        const Part p = static_cast<Part>(i - 1);
        if (partBuilt(p)) {
            out = p;
            return true;
        }
    }
    return false;
}

// The built part after `current`, if it is not past `last`.
inline bool partAfter(Part current, Part last, Part &next)
{
    for (uint8_t i = static_cast<uint8_t>(current) + 1; i <= static_cast<uint8_t>(last); ++i) {
        if (partBuilt(static_cast<Part>(i))) {
            next = static_cast<Part>(i);
            return true;
        }
    }
    return false;
}
