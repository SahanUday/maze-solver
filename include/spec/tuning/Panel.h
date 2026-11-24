// ============================================================================
//  spec/tuning/Panel.h  --  the LED blink timings of the operator panel (include/panel/).
// ============================================================================

#pragma once

#include <stdint.h>

// ---- LED patterns ------------------------------------------------ [DESIGN] --
// Proposals, to tune by eye in the arena (#64).
constexpr uint16_t PANEL_HEARTBEAT_HALF_PERIOD_MS = 500;  // on, then off, this long each
constexpr uint16_t PANEL_FAST_BLINK_HALF_PERIOD_MS = 100; // the same, fast
// CODE N: N blinks (ON, OFF), then a pause.
constexpr uint16_t PANEL_CODE_ON_MS = 150;
constexpr uint16_t PANEL_CODE_OFF_MS = 250;
constexpr uint16_t PANEL_CODE_PAUSE_MS = 1200;
