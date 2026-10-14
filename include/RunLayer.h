// ============================================================================
//  RunLayer.h  --  what every test run goes through: wait for the button, count
//  down, check the wheels, then ask the run what it wants and apply it under the
//  run's limits (cap, ramp, time limit). Stops on the button or on a fault.
//  No hardware, no Arduino.h - testable under env:native.
//  docs/architecture/modules/run_layer.md
//
//  A run (SafeRun.h, ...) is the behaviour on top. It provides four functions,
//  found by argument type:
//      RunLimits runLimits(const R &run);
//      RunReason runReady(const R &run, const RobotState &s);  // None, or why not
//      RunWant   runBegin(R &run, const RobotState &s);        // self-test passed
//      RunWant   runStep(R &run, const RobotState &s);         // once per tick
// ============================================================================

#pragma once

#include <stdint.h>

#include "RobotSpec.h"
#include "RobotState.h"
#include "RunLayerSpec.h"

// Bits of RobotState::usValid.
constexpr uint8_t RUN_US_FRONT = 0x1;
constexpr uint8_t RUN_US_LEFT = 0x2;
constexpr uint8_t RUN_US_RIGHT = 0x4;

// What the robot is doing. WaitStart .. SelfTest, Stopped and Fault belong to the layer; the rest
// are reported by a run, so a run that needs another one adds it at the end (the log and the
// status LED read this enum, and the log keeps its number).
enum class RunPhase : uint8_t {
    WaitStart,
    Countdown,
    SelfTest,
    Cruise,
    Settle,
    Turn,
    Stopped,
    Fault,
    Sweep
};

// Why the run ended. Stopped: an expected end. Fault: something is wrong, and every fault comes
// after SelfTestReversed, so a new stop goes before it. The log keeps this in a nibble, so there
// is room for 16.
enum class RunReason : uint8_t {
    None,
    EStop,        // button pressed during the run
    TimeLimit,    // the run's timeLimitMs elapsed
    FrontWall,    // wall ahead and no way to turn (no gyro, or still in the self-test)
    NoClearSide,  // wall ahead, and a side is silent so a turn would be a guess
    TooManyTurns, // too many turns with no straight run between them
    Complete,     // the run finished what it set out to do
    SelfTestReversed,
    SelfTestNoMotion,
    Stall,
    FrontBlind,
    TurnTimeout,
};

inline bool runReasonIsFault(RunReason reason)
{
    return reason >= RunReason::SelfTestReversed;
}

// The envelope a run declares. The layer enforces it on every wheel command.
struct RunLimits {
    int16_t maxDuty = 0;      // cap on any wheel command; never above MOTOR_PWM_TOP
    int16_t rampPerTick = 0;  // duty change per tick
    uint32_t timeLimitMs = 0; // the run stops itself this long after the countdown
    int16_t selfTestDuty = 0; // both wheels forward during the self-test
    uint16_t frontStopMm = 0; // the self-test stops for a heard wall this close; 0 = ignore
};

// What a run asks for on one tick.
struct RunWant {
    int16_t left = 0; // signed duty, + = forward; the layer caps and ramps it
    int16_t right = 0;
    bool hardStop = false;             // zero the wheels now instead of ramping down
    RunPhase phase = RunPhase::Cruise; // for the log and the status LED
    RunReason end = RunReason::None;   // anything else: the run is over, and why
};

struct RunOutput {
    bool enable = false; // BTS7960 EN pins
    int16_t left = 0;    // signed duty, + = forward
    int16_t right = 0;
};

struct RunLayer {
    RunPhase phase = RunPhase::WaitStart;
    RunReason reason = RunReason::None;
    bool buttonFitted = true;

    // Debounced button. Starts "down" so a button held through boot cannot start a run.
    bool buttonDown = true;
    uint8_t buttonTicks = 0;

    uint32_t runStartMs = 0;
    uint32_t phaseStartMs = 0;
    int32_t phaseEncL = 0;
    int32_t phaseEncR = 0;

    int16_t outL = 0; // what is being sent, after the ramp
    int16_t outR = 0;
};

// `buttonFitted` false: the run starts by itself after RUN_AUTOSTART_MS and the button input is
// ignored.
inline void runLayerInit(RunLayer &layer, bool buttonFitted)
{
    layer = RunLayer{};
    layer.buttonFitted = buttonFitted;
}

// Clamps `v` to +-`limit`.
template <typename T> inline T runClamp(T v, T limit)
{
    return v > limit ? limit : (v < -limit ? -limit : v);
}

// Moves `cur` toward `target` by at most `step`.
inline int16_t runSlew(int16_t cur, int16_t target, int16_t step)
{
    if (target > cur + step) {
        return static_cast<int16_t>(cur + step);
    }
    if (target < cur - step) {
        return static_cast<int16_t>(cur - step);
    }
    return target;
}

// True on the tick the debounced button goes from released to pressed.
inline bool runButtonEdge(RunLayer &layer, bool pressed)
{
    if (pressed == layer.buttonDown) {
        layer.buttonTicks = 0;
        return false;
    }
    if (++layer.buttonTicks < RUN_BUTTON_DEBOUNCE_TICKS) {
        return false;
    }
    layer.buttonDown = pressed;
    layer.buttonTicks = 0;
    return pressed;
}

inline void runEnter(RunLayer &layer, RunPhase phase, const RobotState &s)
{
    layer.phase = phase;
    layer.phaseStartMs = s.timestampMs;
    layer.phaseEncL = s.encoderCountL;
    layer.phaseEncR = s.encoderCountR;
}

// Motors off at once, no ramp. Also the only way out of a driving phase.
inline void runHalt(RunLayer &layer, RunPhase phase, RunReason reason, const RobotState &s)
{
    runEnter(layer, phase, s);
    layer.reason = reason;
    layer.outL = 0;
    layer.outR = 0;
}

inline bool runLayerEnded(const RunLayer &layer)
{
    return layer.phase == RunPhase::Stopped || layer.phase == RunPhase::Fault;
}

// What the layer needs from the run next on this tick.
enum class RunCall : uint8_t { None, Begin, Step };

// Applies what the run asked for under its limits. An `end` reason halts the run.
inline RunOutput runLayerApply(RunLayer &layer, const RunLimits &limits, const RunWant &want,
                               const RobotState &s)
{
    if (want.end != RunReason::None) {
        runHalt(layer, runReasonIsFault(want.end) ? RunPhase::Fault : RunPhase::Stopped, want.end,
                s);
        return RunOutput{};
    }

    layer.phase = want.phase;
    if (want.hardStop) {
        layer.outL = 0;
        layer.outR = 0;
    }
    const int16_t cap = limits.maxDuty > static_cast<int16_t>(MOTOR_PWM_TOP)
                            ? static_cast<int16_t>(MOTOR_PWM_TOP)
                            : limits.maxDuty;
    layer.outL = runSlew(layer.outL, runClamp(want.left, cap), limits.rampPerTick);
    layer.outR = runSlew(layer.outR, runClamp(want.right, cap), limits.rampPerTick);

    RunOutput out;
    out.enable = true;
    out.left = layer.outL;
    out.right = layer.outR;
    return out;
}

// The part of a tick that does not depend on which run it is: the button, the countdown, the
// self-test, the time limit. Returns what it needs from the run; for None, `out` is the tick's
// answer. `notReady` is the run's runReady(), only looked at when the countdown ends.
inline RunCall runLayerBefore(RunLayer &layer, const RunLimits &limits, const RobotState &s,
                              RunReason notReady, RunOutput &out)
{
    const uint32_t now = s.timestampMs;
    const bool buttonEdge = layer.buttonFitted && runButtonEdge(layer, s.startButtonPressed);
    const bool frontHeard = (s.usValid & RUN_US_FRONT) != 0;
    out = RunOutput{};

    if (layer.phase == RunPhase::WaitStart) {
        if (buttonEdge || !layer.buttonFitted) {
            runEnter(layer, RunPhase::Countdown, s);
        }
        return RunCall::None;
    }
    if (runLayerEnded(layer)) {
        return RunCall::None;
    }

    // From here the run is live: anything below can end it, and ending it zeroes the output.
    if (buttonEdge) {
        runHalt(layer, RunPhase::Stopped, RunReason::EStop, s);
        return RunCall::None;
    }
    if (layer.phase == RunPhase::Countdown) {
        const uint32_t countdownMs = layer.buttonFitted ? RUN_COUNTDOWN_MS : RUN_AUTOSTART_MS;
        if (now - layer.phaseStartMs >= countdownMs) {
            if (notReady != RunReason::None) {
                runHalt(layer, RunPhase::Fault, notReady, s);
            } else {
                layer.runStartMs = now;
                runEnter(layer, RunPhase::SelfTest, s);
            }
        }
        return RunCall::None; // the countdown never drives
    }
    if (now - layer.runStartMs >= limits.timeLimitMs) {
        runHalt(layer, RunPhase::Stopped, RunReason::TimeLimit, s);
        return RunCall::None;
    }
    if (layer.phase != RunPhase::SelfTest) {
        return RunCall::Step;
    }

    const bool frontBlocked =
        limits.frontStopMm != 0 && frontHeard && s.distFrontMm <= limits.frontStopMm;
    if (frontBlocked) {
        runHalt(layer, RunPhase::Stopped, RunReason::FrontWall, s);
        return RunCall::None;
    }
    if (now - layer.phaseStartMs >= RUN_SELFTEST_MS) {
        const int32_t dL = s.encoderCountL - layer.phaseEncL;
        const int32_t dR = s.encoderCountR - layer.phaseEncR;
        if (dL <= -RUN_SELFTEST_MIN_COUNTS || dR <= -RUN_SELFTEST_MIN_COUNTS) {
            runHalt(layer, RunPhase::Fault, RunReason::SelfTestReversed, s);
            return RunCall::None;
        }
        if (dL < RUN_SELFTEST_MIN_COUNTS || dR < RUN_SELFTEST_MIN_COUNTS) {
            runHalt(layer, RunPhase::Fault, RunReason::SelfTestNoMotion, s);
            return RunCall::None;
        }
        return RunCall::Begin;
    }
    RunWant want;
    want.left = limits.selfTestDuty;
    want.right = limits.selfTestDuty;
    want.phase = RunPhase::SelfTest;
    out = runLayerApply(layer, limits, want, s);
    return RunCall::None;
}

// One call per control tick, after the sensors are read. Returns what the motors should do.
// Only the three calls into the run are per run type; the rest is runLayerBefore/Apply.
template <typename R> inline RunOutput runLayerStep(RunLayer &layer, R &run, const RobotState &s)
{
    const RunLimits limits = runLimits(run);
    const RunReason notReady =
        layer.phase == RunPhase::Countdown ? runReady(run, s) : RunReason::None;
    RunOutput out;
    switch (runLayerBefore(layer, limits, s, notReady, out)) {
        case RunCall::Begin:
            return runLayerApply(layer, limits, runBegin(run, s), s);
        case RunCall::Step:
            return runLayerApply(layer, limits, runStep(run, s), s);
        case RunCall::None:
            break;
    }
    return out;
}
