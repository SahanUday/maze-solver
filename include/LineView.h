// ============================================================================
//  LineView.h  --  one sweep of the IR array to where the line is. No hardware, no Arduino.h -
//  testable under env:native.
//
//  The maze floor is black, so "all black" is the normal state; on bridge tiles the line is the
//  black part.
//  docs/architecture/modules/line_view.md
// ============================================================================

#pragma once

#include <stdint.h>

#include "RobotSpec.h"

struct LineView {
    bool valid = false;    // at least one usable channel; otherwise the three below are all false
    bool allBlack = false; // every usable channel black: bare floor, or the array lifted
    bool allWhite = false; // no usable channel black: off the line, or a white tile
    bool onLine = false;   // some black, some white
    // Centroid of the black channels, mm from the array centre, positive to the LEFT (CCW, like
    // heading). Meaningless when allWhite.
    float errMm = 0.0f;
};

// `usable` = bit per channel the calibration accepted; `mask` = bit per black channel.
inline LineView lineView(const uint8_t (&norm)[IR_CHANNEL_COUNT], uint8_t mask, uint8_t usable,
                         float pitchMm)
{
    LineView v;
    const uint8_t black = static_cast<uint8_t>(mask & usable);
    v.valid = usable != 0;
    v.allBlack = v.valid && black == usable;
    v.allWhite = v.valid && black == 0;
    v.onLine = v.valid && !v.allBlack && !v.allWhite;

    float weight = 0.0f;
    float moment = 0.0f;
    for (uint8_t ch = 0; ch < IR_CHANNEL_COUNT; ++ch) {
        if ((black & (1u << ch)) == 0) {
            continue;
        }
        // Channel 0 is the leftmost (D1), so it sits at the most positive position.
        const float pos =
            (static_cast<float>(IR_CHANNEL_COUNT - 1) * 0.5f - static_cast<float>(ch)) * pitchMm;
        const float w = static_cast<float>(norm[ch]);
        weight += w;
        moment += w * pos;
    }
    v.errMm = weight > 0.0f ? moment / weight : 0.0f;
    return v;
}
