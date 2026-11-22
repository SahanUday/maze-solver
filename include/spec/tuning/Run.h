// ============================================================================
//  spec/tuning/Run.h  --  the parts a run covers, run start, status line.
//  Layout, tags and how to update: spec/README.md.
// ============================================================================

#pragma once

#include <stdint.h>

// ---- The parts of a run ------------------------------------------------ [TUNED] --
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
//                         it is AExplore, AExit, Line, because the
//                         Section B parts are not built and are skipped
//   AExplore   Line       Section A, out to the line, then the line  on the Section A start tile
//   AExplore   AExit      Section A and out to the line, then stop   on the Section A start tile
//   AExplore   AExplore   explore until the marker, then stop        on the Section A start tile
//                         (the stop-and-brake is not covered by host tests)
//   Line       Line       follow the line (no end rule yet, #60)     on the line, past the gap
//   Line       BProve     the same today; later the line, Section B  on the line, past the gap
//
//   Refused (BadConfig): FIRST = AProve, AExit or BProve (they carry on a map and cannot start
//   alone), FIRST = BEntry or BExplore (not built yet), LAST before FIRST (does not compile).
//   Without a line calibration the run is walls-only and ends after AExplore whatever LAST says;
//   Line alone is then refused. Parts and what they need: mission/Parts.h.
constexpr Part RUN_PART_FIRST = Part::AExplore;
constexpr Part RUN_PART_LAST = Part::BProve;
static_assert(static_cast<uint8_t>(RUN_PART_FIRST) <= static_cast<uint8_t>(RUN_PART_LAST),
              "RUN_PART_LAST comes before RUN_PART_FIRST");

// ---- Run start and status line ---------------------------------------- [TUNED] --
// No start button is fitted: the run begins by itself, RUN_START_DELAY_MS after the loop starts
// (the gyro has been zeroed by then, with the robot held still), so the power switch is the start.
// With a button fitted, set this true and the run waits for a press.
constexpr bool RUN_START_BUTTON_FITTED = false;
constexpr uint16_t RUN_START_DELAY_MS = 1000; // time to take the hands off after the zeroing
// The start button must read pressed this many ticks in a row (a bounce is shorter).
constexpr uint8_t RUN_START_DEBOUNCE_TICKS = 5;
// A status line over serial this often while the run is going (phase, odometer, speed, heading,
// the three ultrasonics, the IR floor class and raw counts, the drive duty). 0 = off. At 250000
// baud a line is ~4 ms of the 10 ms tick.
constexpr uint16_t RUN_LOG_EVERY_MS = 200;
