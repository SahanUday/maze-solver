// ============================================================================
//  LineSense.h  --  per-channel calibration and black/white mask for the IR
//  array counts. No hardware, no Arduino.h - testable under env:native.
//
//  Counts fall as reflection rises, so white is the LOW end of a channel's
//  range and black the HIGH end. Normalized values run 0 (white) .. 255 (black).
//  docs/architecture/modules/line_sense.md
// ============================================================================

#pragma once

#include <stdint.h>

#include "RobotSpec.h"

struct LineCalibration {
    uint16_t white[IR_CHANNEL_COUNT]; // lowest count seen
    uint16_t black[IR_CHANNEL_COUNT]; // highest count seen
    uint32_t scale[IR_CHANNEL_COUNT]; // (255 << 16) / range; 0 = channel unusable
};

// Start a run: nothing seen yet.
inline void lineCalibrationReset(LineCalibration &cal)
{
    for (uint8_t ch = 0; ch < IR_CHANNEL_COUNT; ++ch) {
        cal.white[ch] = ADC_FULL_SCALE_COUNTS;
        cal.black[ch] = 0;
        cal.scale[ch] = 0;
    }
}

// Widen each channel's range with one sweep. Call it for every sweep while the robot passes
// over both white and black.
inline void lineCalibrationAccumulate(LineCalibration &cal,
                                      const uint16_t (&counts)[IR_CHANNEL_COUNT])
{
    for (uint8_t ch = 0; ch < IR_CHANNEL_COUNT; ++ch) {
        if (counts[ch] < cal.white[ch]) {
            cal.white[ch] = counts[ch];
        }
        if (counts[ch] > cal.black[ch]) {
            cal.black[ch] = counts[ch];
        }
    }
}

// End the run. This is the only division; lineNormalize() multiplies. Returns the usable
// channels, bit ch set = usable. A range under LINE_CAL_MIN_SPAN_COUNTS is not.
inline uint8_t lineCalibrationFinish(LineCalibration &cal)
{
    uint8_t usable = 0;
    for (uint8_t ch = 0; ch < IR_CHANNEL_COUNT; ++ch) {
        const uint16_t range = cal.black[ch] > cal.white[ch]
                                   ? static_cast<uint16_t>(cal.black[ch] - cal.white[ch])
                                   : static_cast<uint16_t>(0);
        if (range >= LINE_CAL_MIN_SPAN_COUNTS) {
            cal.scale[ch] = (255UL << 16) / range;
            usable = static_cast<uint8_t>(usable | (1u << ch));
        } else {
            cal.scale[ch] = 0;
        }
    }
    return usable;
}

// counts -> 0 (white) .. 255 (black), clamped to the calibrated range. An unusable channel
// reads 0.
inline void lineNormalize(const LineCalibration &cal, const uint16_t (&counts)[IR_CHANNEL_COUNT],
                          uint8_t (&norm)[IR_CHANNEL_COUNT])
{
    for (uint8_t ch = 0; ch < IR_CHANNEL_COUNT; ++ch) {
        if (cal.scale[ch] == 0 || counts[ch] <= cal.white[ch]) {
            norm[ch] = 0;
            continue;
        }
        const uint32_t range = cal.black[ch] - cal.white[ch];
        uint32_t delta = counts[ch] - cal.white[ch];
        if (delta > range) {
            delta = range;
        }
        norm[ch] = static_cast<uint8_t>((delta * cal.scale[ch] + (1UL << 15)) >> 16);
    }
}

// Mask bit ch = 1 for black. Between OFF and ON a channel keeps its previous state, so noise
// near the threshold does not make it flicker.
inline uint8_t lineMaskUpdate(const uint8_t (&norm)[IR_CHANNEL_COUNT], uint8_t previous)
{
    constexpr uint8_t on = (255u * LINE_MASK_ON_PCT) / 100;
    constexpr uint8_t off = (255u * LINE_MASK_OFF_PCT) / 100;
    static_assert(off < on, "LINE_MASK_OFF_PCT must be below LINE_MASK_ON_PCT");

    uint8_t mask = previous;
    for (uint8_t ch = 0; ch < IR_CHANNEL_COUNT; ++ch) {
        if (norm[ch] >= on) {
            mask = static_cast<uint8_t>(mask | (1u << ch));
        } else if (norm[ch] <= off) {
            mask = static_cast<uint8_t>(mask & ~(1u << ch));
        }
    }
    return mask;
}
