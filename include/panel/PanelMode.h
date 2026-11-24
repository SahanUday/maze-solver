// ============================================================================
//  panel/PanelMode.h  --  what the DIP bank means. Pure logic, no Arduino.h.
//  The settings table: docs/architecture/modules/panel.md
// ============================================================================

#pragma once

#include <stdint.h>

constexpr uint8_t PANEL_DIP_RUN = 1u << 0;      // DIP1: OFF explore, ON speed run
constexpr uint8_t PANEL_DIP_FAST = 1u << 1;     // DIP2: OFF safe, ON fast (only with RUN)
constexpr uint8_t PANEL_DIP_CAL = 1u << 2;      // DIP3: ON = IR calibration, alone
constexpr uint8_t PANEL_DIP_RESERVED = 1u << 3; // DIP4: must be OFF
constexpr uint8_t PANEL_DIP_COUNT = 4;

enum class PanelMode : uint8_t { Explore, SpeedSafe, SpeedFast, Calibrate, Refused };

// The mode a setting asks for; all off is explore. Bits above the fourth are ignored.
constexpr PanelMode panelDecode(uint8_t dip)
{
    dip &= static_cast<uint8_t>((1u << PANEL_DIP_COUNT) - 1u);
    if ((dip & PANEL_DIP_RESERVED) != 0) {
        return PanelMode::Refused;
    }
    if ((dip & PANEL_DIP_CAL) != 0) {
        return dip == PANEL_DIP_CAL ? PanelMode::Calibrate : PanelMode::Refused; // CAL stands alone
    }
    if ((dip & PANEL_DIP_RUN) == 0) {
        return PanelMode::Explore;
    }
    return (dip & PANEL_DIP_FAST) != 0 ? PanelMode::SpeedFast : PanelMode::SpeedSafe;
}

// Is the mode built? One that is not shows as refused. #66 and #67 set theirs true.
constexpr bool panelModeBuilt(PanelMode mode)
{
    switch (mode) {
        case PanelMode::Explore:
            return true;
        case PanelMode::SpeedSafe:
        case PanelMode::SpeedFast:
        case PanelMode::Calibrate:
        case PanelMode::Refused:
            return false;
    }
    return false;
}

// Blinks that show the mode while waiting: 1 explore, 2 safe, 3 fast, 4 calibrate; 0 refused.
constexpr uint8_t panelModeCode(PanelMode mode)
{
    return mode == PanelMode::Refused ? 0 : static_cast<uint8_t>(static_cast<uint8_t>(mode) + 1);
}
