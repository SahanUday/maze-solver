// ============================================================================
//  operator_io.h  --  what the operator touches and sees: the start button and the two LEDs.
//  Non-critical glue, so it uses the Arduino framework rather than the registers a HAL driver
//  would (docs/architecture/decisions/0001).
// ============================================================================

#pragma once

#include <stdint.h>

// Pin modes for the LEDs and, where one is fitted, the start button.
void operatorIoInit();

// The raw start button: true while it is held. Always false without a button fitted.
bool operatorIoStartButton();

// Once per tick: the heartbeat LED toggles every 500 ms, and the error LED follows `fault`.
void operatorIoTick(uint32_t nowMs, bool fault);
