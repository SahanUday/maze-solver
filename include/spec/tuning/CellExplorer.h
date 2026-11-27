// ============================================================================
//  spec/tuning/CellExplorer.h  --  how the explorer senses a cell and how often it retries
//  (mission/CellExplorer.h).
// ============================================================================

#pragma once

#include <stdint.h>

// ---- Section A mission ------------------------------------- [HOST MODEL] --
// How long the robot stands and reads a cell's walls before it decides (a few refreshes of each
// sensor: 4 at 60 ms), and how often a doubtful reading is tried again before it is acted on.
constexpr uint16_t SECTION_A_SENSE_MS = 250;
constexpr uint8_t SECTION_A_FRONT_TRIES = 4; // senses of an unheard front: then it is a wall
constexpr uint8_t SECTION_A_RESCANS = 3;     // map forgotten and explored again (Maze.h)
constexpr uint8_t SECTION_A_RESUMES = 3;     // stops no wall explains, carried on to the centre
constexpr uint8_t SECTION_A_TURN_RETRIES = 3;
constexpr uint8_t SECTION_A_MAX_BLOCKED = 20; // blocked steps in a run before it gives up
// A step that makes no progress (a stiction stop short of its target) this close to the target,
// standing still, is taken as arrived instead of a fault.
constexpr float SECTION_A_STUCK_ACCEPT_MM = 20.0f;

// ---- Dead ends -------------------------------------------------- [DESIGN] --
// Where the way on from a tile the robot drove into is straight back (a dead end, or a tile whose
// other ways are all explored), reverse one tile instead of turning round: a half turn swings the
// tail past the walls, and it hit one on the practice maze. false = turn round.
// docs/architecture/decisions/0025-back-out-of-a-dead-end.md
constexpr bool SECTION_A_BACK_OUT = true;
