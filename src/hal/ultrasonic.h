// ============================================================================
//  ultrasonic.h  --  3x HC-SR04 wall ranging, non-blocking.
//  docs/architecture/modules/ultrasonic.md
// ============================================================================

#pragma once

#include <stdint.h>

namespace hal
{
namespace ultrasonic
{

enum Sensor : uint8_t { kFront = 0, kLeft = 1, kRight = 2 };

// Configures trigger pins, echo pins and Timer1. Call once.
void begin();

// Advances the round-robin. Call once per tick. Never blocks.
void update();

// Last distance for one sensor. Check validMask() first.
uint16_t distanceMm(Sensor s);

// Bit per sensor, same order as the enum.
uint8_t validMask();

} // namespace ultrasonic
} // namespace hal
