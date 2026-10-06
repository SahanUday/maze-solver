// ============================================================================
//  line_sensors.h  --  8-channel analog IR reflectance array driver.
//  docs/architecture/modules/line_sensors.md
// ============================================================================

#pragma once

#include <stdint.h>

#include "RobotSpec.h"

// Takes over the ADC and PORTF, turns the emitters on and waits for them to settle. Call once.
void lineSensorsInit();

// One 10-bit ADC conversion per channel, index 0 = A0 = module D1. Lower = more reflection.
// Blocks for ~240 us (measured).
void lineSensorsRead(uint16_t (&counts)[IR_CHANNEL_COUNT]);

// Switches the IR emitter bank and, if that changed it, waits IR_EMITTER_SETTLE_US so the
// next lineSensorsRead() is valid. Call after lineSensorsInit().
void lineSensorsSetEmitters(bool on);
