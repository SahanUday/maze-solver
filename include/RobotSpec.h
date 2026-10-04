// ============================================================================
//  RobotSpec.h  --  geometry, timing, motion and control constants.
//  No pins (see RobotConfig.h), no Arduino.h (needed for env:native).
//
//  `<<TBD HARDWARE>>` / `<<TBD CALIBRATION>>` = not yet measured/tuned.
//  See scripts/list-tbds.sh.
// ============================================================================

#pragma once

#include <stdint.h>

// ---- Competition geometry --------------------------------- [FROM SPEC] --
constexpr uint16_t TILE_PITCH_MM = 250; // centre-to-centre tile spacing
constexpr uint16_t WALL_HEIGHT_MM = 100;
constexpr uint16_t USABLE_CORRIDOR_MM = 235;  // tile pitch minus wall thickness
constexpr uint8_t SECTION_A_SIZE_TILES = 4;   // 4x4
constexpr uint8_t SECTION_B_SIZE_TILES = 9;   // 9x9
constexpr uint16_t BRIDGE_LINE_WIDTH_MM = 30; // 3 cm black line

// ---- Competition timing ------------------------------------ [FROM SPEC] --
constexpr uint32_t RUN_LIMIT_MS = 8UL * 60UL * 1000UL;         // 8 minutes per trial
constexpr uint32_t ARENA_TIME_LIMIT_MS = 30UL * 60UL * 1000UL; // total arena access
constexpr uint8_t TRIAL_COUNT = 3;                             // best-of-3

// ---- Power / memory budget ---------------------------------- [FROM SPEC] --
constexpr uint16_t BATTERY_MAX_MV = 15000;   // charged pack must stay under this
constexpr uint16_t SRAM_BUDGET_BYTES = 8192; // enforced by scripts/check-ram-budget.sh

// ---- Control loop timing ---------------------------------------------- --
constexpr uint16_t CONTROL_LOOP_PERIOD_MS = 10; // 100 Hz, keeps PID dt fixed

// ---- Motor PWM -------------------------------------------- [FROM DESIGN] --
// Fast PWM, ICRn as TOP, clk/1. BTS7960 tolerates up to ~25 kHz; 20 kHz is
// above audible range. Duty is expressed in timer counts, 0..MOTOR_PWM_TOP.
constexpr uint32_t CPU_HZ = 16000000UL;
constexpr uint32_t MOTOR_PWM_FREQ_HZ = 20000;
static_assert(CPU_HZ % MOTOR_PWM_FREQ_HZ == 0, "PWM frequency must divide CPU_HZ exactly");
constexpr uint16_t MOTOR_PWM_TOP = CPU_HZ / MOTOR_PWM_FREQ_HZ - 1; // 799

// ---- Drivetrain polarity ------------------------------ <<TBD HARDWARE>> --
// The motors are mounted mirrored, so one side needs its direction flipped.
// Set on the bench: command a positive speed, the robot must roll forward and
// the encoder count must rise.
constexpr bool MOTOR_L_INVERT = false;
constexpr bool MOTOR_R_INVERT = false;
constexpr bool ENC_L_INVERT = false;
constexpr bool ENC_R_INVERT = false;

// ---- Debug serial ------------------------------------------------------ --
constexpr uint32_t DEBUG_SERIAL_BAUD = 250000; // exact divisor of 16 MHz

// ---- Wheel / drivetrain geometry --------------------- <<TBD HARDWARE>> --
constexpr uint16_t WHEEL_DIAMETER_MM = 0;      // rolled circumference / pi
constexpr uint16_t TRACK_WIDTH_MM = 0;         // measured, not nominal
constexpr uint16_t ENCODER_COUNTS_PER_REV = 0; // one full wheel turn
constexpr float MM_PER_ENCODER_COUNT = 0.0f;   // derived from the two rows above

// ---- Motor limits ------------------------------------ <<TBD HARDWARE>> --
constexpr uint16_t MOTOR_MIN_PWM_L = 0; // duty counts (0..MOTOR_PWM_TOP) that start the wheel
constexpr uint16_t MOTOR_MIN_PWM_R = 0; // measure at MOTOR_PWM_FREQ_HZ, not Arduino's default

// ---- IR array ---------------------------------------------------------- --
// Digital (comparator) array. See decisions/0002-*.md.
constexpr uint8_t IR_CHANNEL_COUNT = 8;

// Majority-vote debounce, within one tick. Count must be odd.
constexpr uint8_t IR_DEBOUNCE_SAMPLES = 3;
constexpr uint16_t IR_DEBOUNCE_SPACING_US = 100;

// ---- IR array mounting ------------------------------- <<TBD HARDWARE>> --
// No trimpots on this board; height is the only adjustment.
constexpr uint8_t IR_RIDE_HEIGHT_MM = 0;

// ---- IR array polarity ------------------------------------------------- --
// Measured with emitters on: white reads LOW, so black reads HIGH.
// Not applied by the driver; irRaw holds levels as read.
constexpr bool IR_BLACK_IS_HIGH = true;

// ---- Ultrasonic sensing ------------------------------- <<TBD HARDWARE>> --
constexpr uint16_t US_MIN_RANGE_MM = 0; // closest reliable reading
constexpr uint16_t US_MAX_RANGE_MM = 0;

// ---- Motion PID gains -------------------------------- <<TBD CALIBRATION>> --
constexpr float KP_DISTANCE = 0.0f;
constexpr float KI_DISTANCE = 0.0f;
constexpr float KD_DISTANCE = 0.0f;
constexpr float KP_TURN = 0.0f;
constexpr float KI_TURN = 0.0f;
constexpr float KD_TURN = 0.0f;
