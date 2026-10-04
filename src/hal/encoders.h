// ============================================================================
//  encoders.h  --  JGA25-370 quadrature encoder HAL (register-level, 4x).
// ============================================================================

#pragma once

#include <stdint.h>

// Pins -> input + pull-up, edge interrupts on, counts zeroed.
void encodersInit();

// Atomic snapshot of both signed counts. "+" is wheel-forward per ENC_*_INVERT.
void encodersRead(int32_t &left, int32_t &right);
