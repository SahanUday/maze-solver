// ============================================================================
//  operator_io.h  --  the start button and the DIP switches. Non-critical glue,
//  so the Arduino framework is fine here (docs/architecture/decisions/0001).
//
//  DIP switches (on = LOW):
//    bit 0  hand: off = prefer left turns, on = right
//    bit 3  line calibration mode: slide the robot over white and black, press START
// ============================================================================

#pragma once

#include <stdint.h>

constexpr uint8_t DIP_HAND_RIGHT = 0x01;
constexpr uint8_t DIP_LINE_CAL = 0x08;

// Inputs with pull-ups, the error LED as an output.
void operatorInit();

// START is pressed right now.
bool operatorStartPressed();

// One bit per switch, 1 = on.
uint8_t operatorReadDip();
