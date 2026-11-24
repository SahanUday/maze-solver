// ============================================================================
//  operator_io.h  --  what the operator touches and sees, on the pins: the start button, the DIP
//  bank and the two LEDs. What the DIP means and what the LEDs show is the panel
//  (include/panel/), pure logic; this file only reads the pins and writes the LEDs.
//  Non-critical glue, so it uses the Arduino framework rather than the registers a HAL driver
//  would (docs/architecture/decisions/0001).
// ============================================================================

#pragma once

#include <stdint.h>

#include "panel/PanelLeds.h"

// Pin modes: LEDs out, DIP bank in with pull-ups, and the start button in with a pull-up where
// one is wired (spec/pins/Wired.h).
void operatorIoInit();

// The DIP bank: bit i is set while position i (PIN_DIP[i]) is ON. Read it while waiting for the
// start, not during the run. An unwired bank reads 0.
uint8_t operatorIoDip();

// The raw start button: true while it is held. Always false without a button wired.
bool operatorIoStartButton();

// Once per tick: the LEDs show `view` (panel/PanelLeds.h).
void operatorIoTick(uint32_t nowMs, const PanelView &view);
