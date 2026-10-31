// ============================================================================
//  spec/Platform.h  --  clocks and the control-loop period every module shares.
//  No pins (see config/RobotConfig.h), no Arduino.h (needed for env:native).
//
//  `<<TBD HARDWARE>>` / `<<TBD CALIBRATION>>` = not yet measured/tuned.
//  See scripts/list-tbds.sh.
// ============================================================================

#pragma once

#include <stdint.h>

// ---- Control loop timing ---------------------------------------------- --
constexpr uint16_t CONTROL_LOOP_PERIOD_MS = 10; // 100 Hz, keeps PID dt fixed

// ---- CPU clock ----------------------------------------------------------- --
constexpr uint32_t CPU_HZ = 16000000UL;

// ---- Debug serial ------------------------------------------------------ --
constexpr uint32_t DEBUG_SERIAL_BAUD = 250000; // exact divisor of 16 MHz
