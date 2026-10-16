// ============================================================================
//  spec/Mission.h  --  Section A, bridge hand-over, run start and the status line.
//
//  `<<TBD HARDWARE>>` / `<<TBD CALIBRATION>>` = not yet measured/tuned.
//  See scripts/list-tbds.sh.
// ============================================================================

#pragma once

#include <stdint.h>

// ---- Section A mission --------------------------------------- [FROM TESTS] --
// How long the robot stands and reads a cell's walls before it decides (a few refreshes of each
// sensor: 4 at 60 ms), and how often a doubtful reading is tried again before it is acted on.
constexpr uint16_t SECTION_A_SENSE_MS = 250;
constexpr uint8_t SECTION_A_FRONT_TRIES = 4; // senses of an unheard front: then it is a wall
constexpr uint8_t SECTION_A_RESCANS = 3;     // map forgotten and explored again (Maze.h)
constexpr uint8_t SECTION_A_RESUMES = 3;     // stops no wall explains, carried on to the centre
constexpr uint8_t SECTION_A_TURN_RETRIES = 3;
constexpr uint8_t SECTION_A_MAX_BLOCKED = 20; // blocked steps in a run before it gives up
// After the marker the mission drives on (SECTION_A_FOLLOW_LINE) and hands over to the line
// follower; false stops on the marker's tile instead, for a first run that does not leave the
// maze. The hand-over is the first line view BRIDGE_HANDOVER_MIN_MM past the end of the gap (the
// ~8 mm dark patch right after it must not count); no line within APPROACH_NO_LINE_MM of the
// marker's centre is `NoLine` (marker centre to the ramp's line is ~300 mm).
constexpr bool SECTION_A_FOLLOW_LINE = true;
constexpr float BRIDGE_HANDOVER_MIN_MM = 4.0f;
// Without a seen gap, hand over at the first line view this far past the marker's box centre (the
// gap ends ~182 mm and the dark patch ~190 mm from the tile's centre; the line is on the board
// from there). Measured on the robot 2026-10-15: the gap read 915-1010, not air.
constexpr float BRIDGE_HANDOVER_NO_GAP_MM = 230.0f;
constexpr float APPROACH_NO_LINE_MM = 400.0f;
// A step that makes no progress (a stiction stop short of its target) this close to the target,
// standing still, is taken as arrived instead of a fault.
constexpr float SECTION_A_STUCK_ACCEPT_MM = 20.0f;
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
