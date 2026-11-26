// ============================================================================
//  spec/tuning/Panel.h  --  the start timings and the LED blink timings of the operator panel.
// ============================================================================

#pragma once

#include <stdint.h>

// ---- Start ------------------------------------------------------- [TUNED] --
// Time to take the hands off: after an accepted press, or after power-up with no button wired.
constexpr uint16_t PANEL_START_DELAY_MS = 1000;
// The button must read pressed this many ticks in a row (a bounce is shorter).
constexpr uint8_t PANEL_START_DEBOUNCE_TICKS = 5;

// ---- LED patterns ------------------------------------------------ [DESIGN] --
// Proposals, to tune by eye in the arena (#64).
constexpr uint16_t PANEL_HEARTBEAT_HALF_PERIOD_MS = 500;  // on, then off, this long each
constexpr uint16_t PANEL_FAST_BLINK_HALF_PERIOD_MS = 100; // the same, fast
// CODE N: N blinks (ON, OFF), then a pause.
constexpr uint16_t PANEL_CODE_ON_MS = 150;
constexpr uint16_t PANEL_CODE_OFF_MS = 250;
constexpr uint16_t PANEL_CODE_PAUSE_MS = 1200;
