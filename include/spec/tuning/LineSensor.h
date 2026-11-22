// ============================================================================
//  spec/tuning/LineSensor.h  --  line calibration limits and the black / white thresholds.
//  Layout, tags and the TBD legend: docs/architecture/ARCHITECTURE.md ("Spec layout").
//  The array's channel count and ADC: fixed/Hardware.h; its mounting: measurements/Robot.h.
// ============================================================================

#pragma once

#include <stdint.h>
#include "spec/fixed/Hardware.h"

// ---- Line detection ---------------------------------- <<TBD CALIBRATION>> --
// A channel whose white-to-black range is below the minimum is unusable.
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

// ---- Stored calibration check ------------------------------------- [DESIGN] --
// A stored calibration is only believed if it looks like this array on this floor: the whites read
// at least this high (lower is an array without its emitter supply) and differ from each other by
// at least IR_CAL_WHITE_SPREAD_COUNTS (eight identical whites is the same fault).
constexpr uint16_t IR_CAL_WHITE_MIN_COUNTS = 400;
constexpr uint16_t IR_CAL_WHITE_SPREAD_COUNTS = 15;
