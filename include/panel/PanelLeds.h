// ============================================================================
//  panel/PanelLeds.h  --  what the two LEDs show. Pure logic, no hardware, no Arduino.h - testable
//  under env:native. docs/architecture/modules/panel.md
//
//  A pure function of the run's state and the time, so there is nothing to keep between ticks.
//  The panel does not include the mission: main.cpp maps the mission's stage to PanelRun.
//
//    state                               status LED (green)      error LED (red)
//    waiting, mode built                 CODE N                  off (HEARTBEAT if no calibration)
//    waiting, mode refused or not built  off                     FAST BLINK
//    running, proving or saving          HEARTBEAT               off
//    ended normally                      SOLID                   off
//    ended badly                         SOLID                   SOLID
//
//  No heartbeat means the robot has ended, which is the signal to take it.
// ============================================================================

#pragma once

#include <stdint.h>

#include "panel/PanelButtons.h"
#include "spec/tuning/Panel.h"

enum class PanelRun : uint8_t { Waiting, Running, EndedOk, EndedBad };

struct PanelView {
    PanelRun run = PanelRun::Waiting;
    PanelMode mode = PanelMode::Explore;
    bool noCalibration = false; // no usable IR calibration: a walls-only run
};

struct PanelLeds {
    bool status = false;
    bool error = false;
};

// On for the first half period, off for the second, repeating.
constexpr bool panelBlink(uint32_t nowMs, uint16_t halfPeriodMs)
{
    return (nowMs / halfPeriodMs) % 2 == 0;
}

// `n` blinks, then a pause, repeating. n = 0 is never on.
constexpr bool panelCode(uint32_t nowMs, uint8_t n)
{
    const uint32_t blinkMs = PANEL_CODE_ON_MS + PANEL_CODE_OFF_MS;
    const uint32_t cycleMs = n * blinkMs + PANEL_CODE_PAUSE_MS;
    const uint32_t t = nowMs % cycleMs;
    return t < n * blinkMs && t % blinkMs < PANEL_CODE_ON_MS;
}

constexpr PanelLeds panelLeds(const PanelView &view, uint32_t nowMs)
{
    PanelLeds out;
    switch (view.run) {
        case PanelRun::Waiting:
            if (!panelModeBuilt(view.mode)) {
                out.error = panelBlink(nowMs, PANEL_FAST_BLINK_HALF_PERIOD_MS);
                return out;
            }
            out.status = panelCode(nowMs, panelModeCode(view.mode));
            out.error = view.noCalibration && panelBlink(nowMs, PANEL_HEARTBEAT_HALF_PERIOD_MS);
            return out;
        case PanelRun::Running:
            out.status = panelBlink(nowMs, PANEL_HEARTBEAT_HALF_PERIOD_MS);
            return out;
        case PanelRun::EndedOk:
            out.status = true;
            return out;
        case PanelRun::EndedBad:
            out.status = true;
            out.error = true;
            return out;
    }
    return out;
}
