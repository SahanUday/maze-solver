// ============================================================================
//  spec/tuning/Run.h  --  the parts a run covers and the status line.
// ============================================================================

#pragma once

#include <stdint.h>

// ---- The parts of a run ----------------------------------------- [TUNED] --
// The run is cut into these, in course order (mission/Parts.h says what each one is and which can
// be built, started alone or refused).
enum class Part : uint8_t { AExplore, AProve, AExit, Line, BEntry, BExplore, BProve };
constexpr uint8_t PART_COUNT = 7;

// Which parts this build runs, from FIRST to LAST. The competition runs the whole course. To test
// on the robot set a shorter stretch, flash, and put the robot where the first part expects it.
// A selection that cannot run is refused before the robot moves (MissionFault::BadConfig).
//
//   FIRST      LAST       what runs                                  put the robot
//   AExplore   BProve     the whole course (the competition): today  on the Section A start tile
//                         AExplore, AExit, Line, and it stays in Line
//                         (no end rule yet, #60, so the Section B
//                         parts are not reached; AProve is skipped)
//   AExplore   Line       Section A, out to the line, then the line  on the Section A start tile
//   AExplore   AExit      Section A and out to the line, then stop   on the Section A start tile
//   AExplore   AExplore   explore until the marker, then stop        on the Section A start tile
//                         (the stop-and-brake is not covered by host tests)
//   Line       Line       follow the line (no end rule yet, #60)     on the line, past the gap
//   BEntry     BProve     Section B alone: explore from the entry    on the entry tile, facing in
//                         tile, then prove the path (host-tested     (the bridge's opening behind)
//                         only; no finish detector yet, #69)
//
//   Refused (BadConfig): FIRST = AProve (not built), AExit or BProve (they carry on a map and
//   cannot start alone); LAST before FIRST does not compile.
//   Without a line calibration the run is walls-only and ends after AExplore whatever LAST says;
//   Line alone is then refused. Parts and what they need: mission/Parts.h. Until the finish
//   detector exists Section B's exploring has no finish to find and ends when every reachable tile
//   is entered.
constexpr Part RUN_PART_FIRST = Part::AExplore;
constexpr Part RUN_PART_LAST = Part::BProve;
static_assert(static_cast<uint8_t>(RUN_PART_FIRST) <= static_cast<uint8_t>(RUN_PART_LAST),
              "RUN_PART_LAST comes before RUN_PART_FIRST");

// ---- Status line ------------------------------------------------ [TUNED] --
// A status line over serial this often while the run is going (phase, odometer, speed, heading,
// the three ultrasonics, the IR floor class and raw counts, the drive duty). 0 = off. At 250000
// baud a line is ~4 ms of the 10 ms tick.
constexpr uint16_t RUN_LOG_EVERY_MS = 200;
