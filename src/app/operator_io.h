// ============================================================================
//  operator_io.h  --  the panel's pins: the start button, the DIP bank and the two LEDs. What they
//  mean and show is include/panel/. Non-critical glue on the Arduino framework (decisions/0001).
// ============================================================================

#pragma once

#include <stdint.h>

#include "panel/PanelLeds.h"

// Pin modes: LEDs out; the DIP bank, and the button if wired (spec/pins/Wired.h), in with pull-ups.
void operatorIoInit();

// The DIP bank: bit i is set while PIN_DIP[i] is ON. An unwired bank reads 0.
uint8_t operatorIoDip();

// The raw start button; always false while it is not wired.
bool operatorIoStartButton();

// Once per tick: show `view` on the LEDs.
void operatorIoTick(uint32_t nowMs, const PanelView &view);
