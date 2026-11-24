// ============================================================================
//  panel/PanelControls.h  --  what the DIP bank means. Pure logic, no hardware, no Arduino.h -
//  testable under env:native. docs/architecture/modules/panel.md
//
//  The bank has four positions in PIN_DIP order, one bit each, a set bit = the switch is ON:
//
//    RUN  FAST  CAL  RES   mode
//    off  any   off  off   explore (trial 1); FAST is ignored
//    on   off   off  off   speed run, safe (trial 2)
//    on   on    off  off   speed run, fast (trial 3)
//    off  off   on   off   IR calibration
//    anything else         refused
//
//  All off is explore, the safest default; a cut wire reads OFF, which falls the same way.
// ============================================================================

#pragma once

#include <stdint.h>

constexpr uint8_t PANEL_DIP_RUN = 1u << 0;      // DIP1: OFF explore, ON speed run
constexpr uint8_t PANEL_DIP_FAST = 1u << 1;     // DIP2: OFF safe, ON fast (only with RUN)
constexpr uint8_t PANEL_DIP_CAL = 1u << 2;      // DIP3: ON = IR calibration, alone
constexpr uint8_t PANEL_DIP_RESERVED = 1u << 3; // DIP4: must be OFF
constexpr uint8_t PANEL_DIP_COUNT = 4;

enum class PanelMode : uint8_t { Explore, SpeedSafe, SpeedFast, Calibrate, Refused };

// The mode a DIP bank setting asks for. Bits above the fourth are not switches and are ignored.
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

// Is the mode in this firmware? A mode that is not built is shown as refused. Speed run (#66) and
// calibration (#67) set theirs true when they are built.
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

// The number of blinks that shows the mode while waiting: 1 explore, 2 speed safe, 3 speed fast,
// 4 calibrate. 0 for a refused setting.
constexpr uint8_t panelModeCode(PanelMode mode)
{
    return mode == PanelMode::Refused ? 0 : static_cast<uint8_t>(static_cast<uint8_t>(mode) + 1);
}
