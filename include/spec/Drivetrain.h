// ============================================================================
//  spec/Drivetrain.h  --  motor PWM, wheel and encoder geometry, polarity, minimum duty.
//  No pins (see config/RobotConfig.h), no Arduino.h (needed for env:native).
//
//  `<<TBD HARDWARE>>` / `<<TBD CALIBRATION>>` = not yet measured/tuned.
//  See scripts/list-tbds.sh.
// ============================================================================

#pragma once

#include <stdint.h>
#include "spec/Platform.h"

// ---- Motor PWM -------------------------------------------- [FROM DESIGN] --
// Fast PWM, ICRn as TOP, clk/1. BTS7960 tolerates up to ~25 kHz; 20 kHz is
// above audible range. Duty is expressed in timer counts, 0..MOTOR_PWM_TOP.

constexpr uint32_t MOTOR_PWM_FREQ_HZ = 20000;
static_assert(CPU_HZ % MOTOR_PWM_FREQ_HZ == 0, "PWM frequency must divide CPU_HZ exactly");
constexpr uint16_t MOTOR_PWM_TOP = CPU_HZ / MOTOR_PWM_FREQ_HZ - 1; // 799

// ---- Drivetrain polarity ------------------------------------ [MEASURED] --
// The motors are mounted mirrored, so one side needs its direction flipped.
// Set on the bench: command a positive speed, the robot must roll forward and
// the encoder count must rise.
constexpr bool MOTOR_L_INVERT = true;
constexpr bool MOTOR_R_INVERT = false;
constexpr bool ENC_L_INVERT = false;
constexpr bool ENC_R_INVERT = false;
// Measured 2026-10-13 on the compact chassis, wheels free, duty 300 for 800 ms: each wheel rolled
// forward and its own encoder rose (left +1155, right +1154; both together +1173 / +1154).

// ---- Wheel / drivetrain geometry --------------------------- [MEASURED] --
constexpr uint16_t WHEEL_DIAMETER_MM = 65; // ruler, rolled circumference / pi
// Effective track: a 109 deg gyro pivot against the encoder difference. Tyre-centre spacing is
// ~155 mm; the rest is tyre scrub.
constexpr uint16_t TRACK_WIDTH_MM = 170;
constexpr uint16_t ENCODER_COUNTS_PER_REV = 898; // 10 hand turns: left 8978, right 8971
constexpr float MM_PER_ENCODER_COUNT =
    3.14159265f * WHEEL_DIAMETER_MM / ENCODER_COUNTS_PER_REV; // ~0.2274

// ---- Motor limits ---------------------------------------------- [MEASURED] --
// Start-from-rest duty (0..MOTOR_PWM_TOP), forward, on the floor, from single 150 ms pulses.
// A wheel already turning keeps going down to ~100-120. Measure at MOTOR_PWM_FREQ_HZ.
constexpr uint16_t MOTOR_MIN_PWM_L = 160; // the lowest duty tried
constexpr uint16_t MOTOR_MIN_PWM_R = 250;

// What the controller lifts non-zero commands to; 0 = no lift. The start values above make a
// turn chatter around its target.
constexpr uint16_t CONTROL_MIN_PWM_L = 0;
constexpr uint16_t CONTROL_MIN_PWM_R = 0;
