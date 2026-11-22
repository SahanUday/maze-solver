// ============================================================================
//  spec/fixed/Hardware.h  --  clock, loop period, motor PWM and the chip and module settings.
//  Layout, tags and how to update: spec/README.md.
// ============================================================================

#pragma once

#include <stdint.h>

// ---- Clock, loop and serial -------------------------------------------- [DESIGN] --
constexpr uint32_t CPU_HZ = 16000000UL;
constexpr uint16_t CONTROL_LOOP_PERIOD_MS = 10; // 100 Hz, keeps PID dt fixed
constexpr uint32_t DEBUG_SERIAL_BAUD = 250000;  // exact divisor of 16 MHz

// ---- Motor PWM -------------------------------------------------------- [DESIGN] --
// Fast PWM, ICRn as TOP, clk/1. BTS7960 tolerates up to ~25 kHz; 20 kHz is
// above audible range. Duty is expressed in timer counts, 0..MOTOR_PWM_TOP.
constexpr uint32_t MOTOR_PWM_FREQ_HZ = 20000;
static_assert(CPU_HZ % MOTOR_PWM_FREQ_HZ == 0, "PWM frequency must divide CPU_HZ exactly");
constexpr uint16_t MOTOR_PWM_TOP = CPU_HZ / MOTOR_PWM_FREQ_HZ - 1; // 799

// ---- IMU (MPU-6050) --------------------------------------------------- [DESIGN] --
// Register values and the reasoning: decisions/0008-mpu6050-heading-and-tilt.md.
constexpr uint8_t IMU_I2C_ADDRESS = 0x68;      // ADO left floating
constexpr uint32_t IMU_I2C_CLOCK_HZ = 400000;  // the MPU-6050's fast-mode limit
constexpr uint8_t IMU_STARTUP_DELAY_MS = 30;   // gyro start-up after clearing SLEEP
constexpr float GYRO_COUNTS_PER_DPS = 65.5f;   // +-500 deg/s full scale
constexpr uint16_t ACCEL_COUNTS_PER_G = 16384; // +-2 g full scale
// Wire has no timeout by default: a bus held low (an unpowered or unplugged gyro can do it) would
// hang the control loop with the motors still driving. A transfer takes < 1 ms at 400 kHz.
constexpr uint32_t IMU_I2C_TIMEOUT_US = 3000;

// GYRO_COUNTS_PER_DPS was checked on the compact chassis, 2026-10-14, by turning the robot by hand
// through five closed laps each way and integrating the raw counts: 1.001-1.002 of nominal
// (+-0.002, from where the hand stopped), so it stays at 65.5. The accelerometer reads about 4.7%
// high on this unit: see IMU_ACCEL_ONE_G_COUNTS. Numbers: docs/calibration/.

// ---- IR array and ADC ------------------------------------------------- [DESIGN] --
// Analog reflectance array read through the ADC. See decisions/0005-*.md.
constexpr uint8_t IR_CHANNEL_COUNT = 8;

// ADC clock = CPU_HZ / IR_ADC_PRESCALER; the driver caps it at 1 MHz.
constexpr uint8_t IR_ADC_PRESCALER = 32;

// Wait after the emitter enable switches before a reading is valid.
constexpr uint16_t IR_EMITTER_SETTLE_US = 1000;

// 10-bit ADC.
constexpr uint16_t ADC_FULL_SCALE_COUNTS = 1023;
constexpr uint16_t ADC_NOISE_COUNTS = 1; // +-1 count, one sigma

// ---- Ultrasonic sensing ----------------------------------------------- [DESIGN] --
// HC-SR04 x3, round-robin. See decisions/0007-*.md. Per-sensor offsets: measurements/Robot.h.
constexpr uint8_t US_SENSOR_COUNT = 3;

// One sensor per slot; 2 ticks, so slots land on tick boundaries.
constexpr uint8_t US_SLOT_MS = 20;

// Echo timer (Timer5) prescaler 8 at 16MHz gives 0.5us per tick.
constexpr uint8_t US_TIMER_PRESCALER = 8;

// Speed of sound at 20C. Temperature is not compensated.
constexpr uint16_t SPEED_OF_SOUND_M_S = 343;

// Datasheet capability. <<TBD HARDWARE>> - confirm on the bench.
constexpr uint16_t US_MIN_RANGE_MM = 20;
constexpr uint16_t US_MAX_RANGE_MM = 4000;

// Driver stops listening here, before the module's own ~71ms no-echo timeout (bench).
// Exceeds the 2250mm longest sightline a 9x9 arena can present.
constexpr uint16_t US_RANGE_CAP_MM = 2500;

// Echo deadline in 0.5us ticks: US_RANGE_CAP_MM * 4000 / 343.
constexpr uint16_t US_ECHO_TIMEOUT_TICKS =
    static_cast<uint16_t>((static_cast<uint32_t>(US_RANGE_CAP_MM) * 4000u) / SPEED_OF_SOUND_M_S);
