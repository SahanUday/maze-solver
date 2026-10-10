// ============================================================================
//  SweepRun.h  --  a run for RunLayer that measures wheel speed against duty:
//  one wheel at a time, forward then reverse, climbing a duty ladder and coming
//  back, each rung held long enough to settle and measured over its last part.
//  No hardware, no Arduino.h - testable under env:native.
//  docs/architecture/modules/sweep_run.md
// ============================================================================

#pragma once

#include <stdint.h>

#include "RobotState.h"
#include "RunLayer.h"
#include "SweepRunSpec.h"

// One measured rung.
struct SweepRow {
    bool right = false;   // which wheel
    bool reverse = false; // which direction it was driven
    bool up = true;       // on the way up the ladder, not back down
    int16_t duty = 0;     // magnitude
    int32_t counts = 0;   // signed encoder counts over the measuring window
    uint16_t dtMs = 0;    // length of that window
};

struct SweepRun {
    uint8_t combo = 0; // wheel and direction: 0 left fwd, 1 left rev, 2 right fwd, 3 right rev
    uint8_t step = 0;  // 0 .. 2 * SWEEP_STEPS_PER_RAMP - 1 within a combo
    bool settling = true;
    bool windowOpen = false;
    bool done = false;
    uint32_t phaseStartMs = 0;
    uint32_t windowStartMs = 0;
    int32_t windowStartCounts = 0;
    SweepRow row;
    bool rowReady = false;
};

inline void sweepRunInit(SweepRun &run)
{
    run = SweepRun{};
}

// Hands over the row measured on the last tick, once.
inline bool sweepRunTakeRow(SweepRun &run, SweepRow &out)
{
    if (!run.rowReady) {
        return false;
    }
    out = run.row;
    run.rowReady = false;
    return true;
}

// Duty magnitude of a rung: up the ladder, then back down from the top.
inline int16_t sweepDutyAt(uint8_t step)
{
    if (step < SWEEP_STEPS_PER_RAMP) {
        return static_cast<int16_t>(SWEEP_DUTY_START + step * SWEEP_DUTY_STEP);
    }
    return static_cast<int16_t>(SWEEP_DUTY_MAX - (step - SWEEP_STEPS_PER_RAMP) * SWEEP_DUTY_STEP);
}

inline RunWant sweepStopped()
{
    RunWant want;
    want.hardStop = true;
    want.phase = RunPhase::Sweep;
    return want;
}

// ---- The four functions RunLayer asks of a run ------------------------------

inline RunLimits runLimits(const SweepRun &)
{
    RunLimits limits;
    limits.maxDuty = SWEEP_DUTY_MAX;
    limits.rampPerTick = SWEEP_DUTY_STEP; // a rung is reached in one tick
    limits.timeLimitMs = SWEEP_TIME_LIMIT_MS;
    limits.selfTestDuty = SWEEP_SELFTEST_DUTY;
    limits.frontStopMm = 0; // no walls are needed, and none are expected
    return limits;
}

inline RunReason runReady(const SweepRun &, const RobotState &)
{
    return RunReason::None;
}

inline RunWant runBegin(SweepRun &run, const RobotState &s)
{
    run.settling = true;
    run.phaseStartMs = s.timestampMs;
    return sweepStopped();
}

inline RunWant runStep(SweepRun &run, const RobotState &s)
{
    if (run.done) {
        RunWant want;
        want.end = RunReason::Complete;
        return want;
    }
    const uint32_t now = s.timestampMs;

    if (run.settling) {
        if (now - run.phaseStartMs >= SWEEP_SETTLE_MS) {
            run.settling = false;
            run.step = 0;
            run.windowOpen = false;
            run.phaseStartMs = now;
        }
        return sweepStopped();
    }

    const bool right = run.combo >= 2;
    const bool reverse = (run.combo & 1) != 0;
    const int16_t duty = sweepDutyAt(run.step);
    const int32_t counts = right ? s.encoderCountR : s.encoderCountL;
    const uint32_t elapsed = now - run.phaseStartMs;

    if (!run.windowOpen && elapsed >= SWEEP_STEP_MS - SWEEP_MEASURE_MS) {
        run.windowOpen = true;
        run.windowStartMs = now;
        run.windowStartCounts = counts;
    }

    if (run.windowOpen && elapsed >= SWEEP_STEP_MS) {
        run.row.right = right;
        run.row.reverse = reverse;
        run.row.up = run.step < SWEEP_STEPS_PER_RAMP;
        run.row.duty = duty;
        run.row.counts = counts - run.windowStartCounts;
        run.row.dtMs = static_cast<uint16_t>(now - run.windowStartMs);
        run.rowReady = true;

        run.windowOpen = false;
        run.phaseStartMs = now;
        if (++run.step >= 2 * SWEEP_STEPS_PER_RAMP) {
            run.step = 0;
            run.settling = true;
            if (++run.combo >= SWEEP_COMBOS) {
                run.done = true;
            }
        }
    }

    if (run.settling || run.done) {
        return sweepStopped(); // that was the last rung of the combo
    }

    RunWant want;
    want.phase = RunPhase::Sweep;
    const int16_t signedDuty = reverse ? static_cast<int16_t>(-duty) : duty;
    if (right) {
        want.right = signedDuty;
    } else {
        want.left = signedDuty;
    }
    return want;
}
