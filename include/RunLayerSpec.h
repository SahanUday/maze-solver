// ============================================================================
//  RunLayerSpec.h  --  start-up and self-test constants shared by every run
//  (RunLayer.h). A run's own limits are its RunLimits, not constants here.
//  No pins (see RobotConfig.h), no Arduino.h (needed for env:native).
//  Behaviour: docs/architecture/modules/run_layer.md
// ============================================================================

#pragma once

#include <stdint.h>

// ---- Start ------------------------------------------------- [FROM DESIGN] --
// Is a START button wired to PIN_BTN_START? Without one the run begins by itself after
// RUN_AUTOSTART_MS (time to put the robot down in the maze) and there is no button e-stop.
// <<TBD HARDWARE>> - set true once the button is fitted.
constexpr bool RUN_START_BUTTON_FITTED = false;
constexpr uint32_t RUN_AUTOSTART_MS = 8000;
constexpr uint16_t RUN_COUNTDOWN_MS = 3000; // hands off after the button press
constexpr uint8_t RUN_BUTTON_DEBOUNCE_TICKS = 3;

// ---- Self-test --------------------------------------------- [FROM DESIGN] --
// Both wheels forward, each encoder must count up by at least this.
constexpr uint16_t RUN_SELFTEST_MS = 500;
constexpr int32_t RUN_SELFTEST_MIN_COUNTS = 20;
