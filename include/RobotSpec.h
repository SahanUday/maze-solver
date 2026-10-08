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
// Analog reflectance array read through the ADC. See decisions/0005-*.md.
constexpr uint8_t IR_CHANNEL_COUNT = 8;

// ADC clock = CPU_HZ / IR_ADC_PRESCALER; the driver caps it at 1 MHz.
constexpr uint8_t IR_ADC_PRESCALER = 32;

// Wait after the emitter enable switches before a reading is valid.
constexpr uint16_t IR_EMITTER_SETTLE_US = 1000;

// ---- IMU (MPU-6050) ---------------------------------------- [FROM DESIGN] --
// Register values and the reasoning: decisions/0007-mpu6050-heading-and-tilt.md.
constexpr uint8_t IMU_I2C_ADDRESS = 0x68;      // ADO left floating
constexpr uint32_t IMU_I2C_CLOCK_HZ = 400000;  // the MPU-6050's fast-mode limit
constexpr uint8_t IMU_STARTUP_DELAY_MS = 30;   // gyro start-up after clearing SLEEP
constexpr float GYRO_COUNTS_PER_DPS = 65.5f;   // +-500 deg/s full scale
constexpr uint16_t ACCEL_COUNTS_PER_G = 16384; // +-2 g full scale
constexpr uint8_t IMU_GYRO_AXIS_YAW = 2;       // board mounted flat, so yaw is Z

// Gyro bias: averaged at boot with the robot held still. Integrating an
// uncorrected bias of 2 deg/s over RUN_LIMIT_MS would accumulate ~960 deg.
constexpr uint16_t GYRO_BIAS_SAMPLES = 1000;
constexpr uint16_t GYRO_BIAS_MIN_SAMPLES = 500; // fewer usable reads = not trustworthy

// ---- Gyro polarity ----------------------------------- <<TBD HARDWARE>> --
// Set on the bench: rotate the robot counter-clockwise, headingDeg must rise.
constexpr bool GYRO_YAW_INVERT = false;

// ---- Line detection ---------------------------------- <<TBD CALIBRATION>> --
// 10-bit ADC. A channel whose white-to-black range is below the minimum is unusable (noise is
// +-1 count).
constexpr uint16_t ADC_FULL_SCALE_COUNTS = 1023;
constexpr uint16_t LINE_CAL_MIN_SPAN_COUNTS = 10;

// Normalized 0 (white) .. 255 (black): a channel turns black above ON and white below OFF.
constexpr uint8_t LINE_MASK_ON_PCT = 60;
constexpr uint8_t LINE_MASK_OFF_PCT = 40;

// ---- IR array mounting ------------------------------- <<TBD HARDWARE>> --
// Sensor face to floor at the final mount; the signal falls steeply with height.
constexpr float IR_RIDE_HEIGHT_MM = 0.0f;

// ---- Ultrasonic sensing ------------------------------------------------ --
// HC-SR04 x3, round-robin. See decisions/0007-*.md.
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

// ---- Ultrasonic offsets --------------------------- <<TBD CALIBRATION>> --
// Added to the raw face-to-target reading, per sensor.
// Negative = sensor sits ahead of its reference point.
// Each sensor may use a different reference point.
constexpr int16_t US_OFFSET_FRONT_MM = 0;
constexpr int16_t US_OFFSET_LEFT_MM = 0;
constexpr int16_t US_OFFSET_RIGHT_MM = 0;

// Echo deadline in 0.5us ticks: US_RANGE_CAP_MM * 4000 / 343.
constexpr uint16_t US_ECHO_TIMEOUT_TICKS =
    static_cast<uint16_t>((static_cast<uint32_t>(US_RANGE_CAP_MM) * 4000u) / SPEED_OF_SOUND_M_S);

// ---- Motion PID gains -------------------------------- <<TBD CALIBRATION>> --
constexpr float KP_DISTANCE = 0.0f;
constexpr float KI_DISTANCE = 0.0f;
constexpr float KD_DISTANCE = 0.0f;
constexpr float KP_TURN = 0.0f;
constexpr float KI_TURN = 0.0f;
constexpr float KD_TURN = 0.0f;
