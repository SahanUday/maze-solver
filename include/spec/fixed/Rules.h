// ============================================================================
//  spec/fixed/Rules.h  --  competition rules and project budgets.
//  Layout, tags and the TBD legend: docs/architecture/ARCHITECTURE.md ("Spec layout").
//  Sizes of the arena as built (tiles, walls): measurements/Arena.h.
// ============================================================================

#pragma once

#include <stdint.h>

// ---- Section sizes -------------------------------------------------- [RULES] --
constexpr uint8_t SECTION_A_SIZE_TILES = 4; // 4x4
constexpr uint8_t SECTION_B_SIZE_TILES = 9; // 9x9

// ---- Competition timing --------------------------------------------- [RULES] --
constexpr uint32_t RUN_LIMIT_MS = 8UL * 60UL * 1000UL;         // 8 minutes per trial
constexpr uint32_t ARENA_TIME_LIMIT_MS = 30UL * 60UL * 1000UL; // total arena access
constexpr uint8_t TRIAL_COUNT = 3;                             // best-of-3

// ---- Power / memory budget ---------------------------------- [RULES, DESIGN] --
constexpr uint16_t BATTERY_MAX_MV = 15000;   // charged pack must stay under this
constexpr uint16_t SRAM_BUDGET_BYTES = 8192; // enforced by scripts/check-ram-budget.sh
