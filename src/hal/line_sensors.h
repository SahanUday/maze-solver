// ============================================================================
//  line_sensors.h  --  8-channel digital IR array driver.
//  docs/architecture/modules/line_sensors.md
// ============================================================================

#pragma once

#include <stdint.h>

namespace hal
{
namespace line_sensors
{

// Configures PORTF and turns the emitters on. Call once.
void begin();

// Debounced raw levels. Bit 0 = A0 = module D1.
uint8_t read();

// Switches the IR emitter bank.
void setEmitters(bool on);

} // namespace line_sensors
} // namespace hal
