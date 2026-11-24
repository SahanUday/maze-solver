// ============================================================================
//  panel/PanelStart.h  --  when the operator lets the run begin: a debounced press of the start
//  button, or by itself after power-up while no button is wired. Pure logic, no Arduino.h.
//  The rules: docs/architecture/modules/panel.md
// ============================================================================

#pragma once

#include <stdint.h>

#include "panel/PanelMode.h"
#include "spec/tuning/Panel.h"

struct PanelStart {
    bool buttonWired = false;
    bool released = false; // the button has been seen up since boot
    uint8_t pressedTicks = 0;
    bool counting = false; // accepted, waiting for the hands to come off
    uint32_t acceptedAtMs = 0;
    bool go = false; // latched: the run may begin
};

inline void panelStartInit(PanelStart &ps, bool buttonWired)
{
    ps = PanelStart{};
    ps.buttonWired = buttonWired;
}

// One tick. `mode` is the DIP setting now and `buttonDown` the raw button. A setting that is not
// built is never accepted; once accepted, `go` follows PANEL_START_DELAY_MS later whatever the DIP
// or the button do.
inline void panelStartTick(PanelStart &ps, uint32_t nowMs, bool buttonDown, PanelMode mode)
{
    if (ps.go) {
        return;
    }
    if (ps.counting) {
        if (nowMs - ps.acceptedAtMs >= PANEL_START_DELAY_MS) {
            ps.go = true;
        }
        return;
    }

    bool accept = false;
    if (!ps.buttonWired) {
        accept = panelModeBuilt(mode); // no button: the first tick with a setting that can run
    } else if (!buttonDown) {
        ps.released = true;
        ps.pressedTicks = 0;
    } else if (!ps.released) {
        ps.pressedTicks = 0; // held since boot: wait for a release
    } else if (!panelModeBuilt(mode)) {
        ps.released = false; // a press on a refused setting is used up
        ps.pressedTicks = 0;
    } else {
        if (ps.pressedTicks < PANEL_START_DEBOUNCE_TICKS) {
            ++ps.pressedTicks;
        }
        accept = ps.pressedTicks >= PANEL_START_DEBOUNCE_TICKS; // on the press, not the release
    }

    if (accept) {
        ps.counting = true;
        ps.acceptedAtMs = nowMs;
    }
}
