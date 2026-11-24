// ============================================================================
//  spec/tuning/Panel.h  --  the LED blink timings of the operator panel (include/panel/).
// ============================================================================

#pragma once

#include <stdint.h>

// ---- LED patterns ------------------------------------------------ [DESIGN] --
// Proposals, to be tuned by eye under the arena lighting (#64, #54).
// HEARTBEAT: on for this long, then off for the same.
constexpr uint16_t PANEL_HEARTBEAT_HALF_PERIOD_MS = 500;
// FAST BLINK: on for this long, then off for the same.
constexpr uint16_t PANEL_FAST_BLINK_HALF_PERIOD_MS = 100;
// CODE N: N blinks of ON then OFF, then a pause, repeated. N is the mode code (1 explore ...).
constexpr uint16_t PANEL_CODE_ON_MS = 150;
constexpr uint16_t PANEL_CODE_OFF_MS = 250;
constexpr uint16_t PANEL_CODE_PAUSE_MS = 1200;
