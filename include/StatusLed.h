// ============================================================================
//  StatusLed.h  --  what the on-board LED shows during a run, for when no
//  serial monitor is attached. No hardware, no Arduino.h - testable under
//  env:native.
//
//    WaitStart / Countdown   slow blink, 1 Hz
//    SelfTest .. Sweep       solid on (the robot is driving)
//    Stopped                 N slow flashes, a pause, repeat; N = why it stopped
//    Fault                   N fast flashes, a pause, repeat; N = what is wrong
//
//  N is statusBlinkCount(): the reason's number, with its meaning listed in
//  docs/architecture/modules/run_layer.md.
// ============================================================================

#pragma once

#include <stdint.h>

#include "RunLayer.h"

constexpr uint16_t STATUS_WAIT_HALF_PERIOD_MS = 500;
constexpr uint16_t STATUS_STOPPED_HALF_PERIOD_MS = 300;
constexpr uint16_t STATUS_FAULT_HALF_PERIOD_MS = 100;
constexpr uint16_t STATUS_PAUSE_MS = 1500;

// Flashes per cycle for a finished run; 0 for any phase that is not finished.
inline uint8_t statusBlinkCount(RunPhase phase, RunReason reason)
{
    const uint8_t code = static_cast<uint8_t>(reason);
    if (phase == RunPhase::Stopped) {
        return code;
    }
    if (phase == RunPhase::Fault) {
        return static_cast<uint8_t>(code - static_cast<uint8_t>(RunReason::SelfTestReversed) + 1);
    }
    return 0;
}

inline bool statusLedOn(RunPhase phase, RunReason reason, uint32_t nowMs)
{
    switch (phase) {
        case RunPhase::WaitStart:
        case RunPhase::Countdown:
            return (nowMs % (2UL * STATUS_WAIT_HALF_PERIOD_MS)) < STATUS_WAIT_HALF_PERIOD_MS;
        case RunPhase::SelfTest:
        case RunPhase::Cruise:
        case RunPhase::Settle:
        case RunPhase::Turn:
        case RunPhase::Sweep:
            return true;
        case RunPhase::Stopped:
        case RunPhase::Fault:
            break;
    }
    const uint16_t half =
        phase == RunPhase::Fault ? STATUS_FAULT_HALF_PERIOD_MS : STATUS_STOPPED_HALF_PERIOD_MS;
    const uint32_t flashes = static_cast<uint32_t>(statusBlinkCount(phase, reason)) * 2UL * half;
    const uint32_t t = nowMs % (flashes + STATUS_PAUSE_MS);
    return t < flashes && (t % (2UL * half)) < half;
}
