// ============================================================================
//  motors.h  --  two BTS7960 (HW-039 / IBT_2) drivers, register-level PWM.
// ============================================================================

#pragma once

#include <stdint.h>

enum class Motor : uint8_t { Left, Right };

// All pins low, EN low (coast), PWM timers running at duty 0. Does NOT enable.
void motorsInit();

// EN pins. false = both half-bridges off (coast) and both commands zeroed, so a
// later enable starts from rest.
void motorsEnable(bool enabled);

// Signed duty in timer counts, clamped to +-MOTOR_PWM_TOP. > 0 is forward
// (after MOTOR_*_INVERT). 0 with EN high holds both inputs low.
void motorSet(Motor motor, int16_t command);
