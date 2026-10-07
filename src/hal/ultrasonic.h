// ============================================================================
//  ultrasonic.h  --  3x HC-SR04 wall ranging, non-blocking.
//  docs/architecture/modules/ultrasonic.md
// ============================================================================

#pragma once

#include <stdint.h>

enum class Ultrasonic : uint8_t { Front = 0, Left = 1, Right = 2 };

// Trigger pins, echo pins and the echo timer. Call once.
void ultrasonicInit();

// Advances the round-robin. Call once per tick. Never blocks.
void ultrasonicUpdate();

// Last distance for one sensor. Check ultrasonicValidMask() first.
uint16_t ultrasonicDistanceMm(Ultrasonic sensor);

// Bit per sensor, same order as the enum.
uint8_t ultrasonicValidMask();
