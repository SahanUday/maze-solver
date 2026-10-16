// ============================================================================
//  spec/LineSensor.h  --  IR array, ADC settings, line calibration limits and mounting.
//
//  `<<TBD HARDWARE>>` / `<<TBD CALIBRATION>>` = not yet measured/tuned.
//  See scripts/list-tbds.sh.
// ============================================================================

#pragma once

#include <stdint.h>

// ---- IR array ---------------------------------------------------------- --
// Analog reflectance array read through the ADC. See decisions/0005-*.md.
constexpr uint8_t IR_CHANNEL_COUNT = 8;

// ADC clock = CPU_HZ / IR_ADC_PRESCALER; the driver caps it at 1 MHz.
constexpr uint8_t IR_ADC_PRESCALER = 32;

// Wait after the emitter enable switches before a reading is valid.
constexpr uint16_t IR_EMITTER_SETTLE_US = 1000;

// ---- Line detection ---------------------------------- <<TBD CALIBRATION>> --
// 10-bit ADC. A channel whose white-to-black range is below the minimum is unusable.
constexpr uint16_t ADC_FULL_SCALE_COUNTS = 1023;
constexpr uint16_t ADC_NOISE_COUNTS = 1; // +-1 count, one sigma
constexpr uint16_t LINE_CAL_MIN_SPAN_COUNTS = 30;

// Normalized 0 (white) .. 255 (black): a channel turns black above ON and white below OFF.
constexpr uint8_t LINE_MASK_ON_PCT = 60;
constexpr uint8_t LINE_MASK_OFF_PCT = 40;
// The hysteresis band of the narrowest accepted channel must be at least 6 sigma wide, or its mask
// flickers where the line edge passes: host model, reading at the middle of the range, 1000 ticks
// of 8 channels: a span of 20 flipped 604 times, 25 151 times, 29 twice, 36 never.
static_assert(static_cast<uint32_t>(LINE_CAL_MIN_SPAN_COUNTS) *
                      (LINE_MASK_ON_PCT - LINE_MASK_OFF_PCT) / 100 >=
                  6u * ADC_NOISE_COUNTS,
              "LINE_CAL_MIN_SPAN_COUNTS is too small for the hysteresis band against ADC noise");

// ---- IR array mounting ---------------------------------------- [MEASURED] --
// Fixed by the chassis. Measured on the robot (PR #28, 2026-10-14/15).
// Sensor face to floor: the signal falls steeply with height.
constexpr float IR_RIDE_HEIGHT_MM = 7.0f;
constexpr float IR_SENSOR_PITCH_MM = 8.0f;          // channel to channel
constexpr float IR_ARRAY_FORWARD_OFFSET_MM = 86.0f; // wheel axle to the array's sensing line
// Channel 0 is D1 (pin A0), and D1 is the robot's RIGHT end: a black card under only the right end
// saturated channels 0 and 1, under only the left end channels 6 and 7.
constexpr bool IR_D1_ON_RIGHT = true;

// ---- Stored calibration check ------------------------------------------ --
// A stored calibration is only believed if it looks like this array on this floor: the whites read
// at least this high (lower is an array without its emitter supply) and differ from each other by
// at least IR_CAL_WHITE_SPREAD_COUNTS (eight identical whites is the same fault).
constexpr uint16_t IR_CAL_WHITE_MIN_COUNTS = 400;
constexpr uint16_t IR_CAL_WHITE_SPREAD_COUNTS = 15;
