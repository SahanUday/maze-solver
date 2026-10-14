// ============================================================================
//  SafeRun.h  --  a run for RunLayer: drive between the walls at low duty, turn
//  at a front wall, and end on anything unexpected. No hardware, no Arduino.h -
//  testable under env:native.
//  docs/architecture/modules/safe_run.md
// ============================================================================

#pragma once

#include <stdint.h>

#include "Heading.h"
#include "RobotSpec.h"
#include "RobotState.h"
#include "RunLayer.h"
#include "SafeRunSpec.h"

struct SafeRun {
    RunPhase phase = RunPhase::Settle; // Settle, Cruise or Turn
    RunPhase next = RunPhase::Cruise;  // where Settle goes
    bool canTurn = false;

    uint32_t phaseStartMs = 0;
    uint32_t windowStartMs = 0;
    uint32_t lastFrontOkMs = 0;
    int32_t windowEncL = 0;
    int32_t windowEncR = 0;

    float turnStartDeg = 0.0f;
    float turnTargetDeg = 0.0f;
    int8_t turnDir = 1; // +1 = counter-clockwise (left), -1 = clockwise
    uint8_t turnsInRow = 0;

    // Heading hold. The target is where a straight stretch is trying to point; after a turn it
    // moves by the ideal angle, not the measured one, so an overshoot is corrected instead of
    // followed.
    float targetDeg = 0.0f;
    bool targetSet = false;
    float learnDeg = 0.0f; // corridor direction learned from the walls, on top of the target
    // Integral term, in duty. Kept across corners: the wheel mismatch it absorbs does not turn.
    float headingI = 0.0f;
};

// `canTurn`: the gyro answered and its bias is ready. Without it the run stops at the first
// front wall instead of turning.
inline void safeRunInit(SafeRun &run, bool canTurn)
{
    run = SafeRun{};
    run.canTurn = canTurn;
}

inline int32_t safeRunAbs(int32_t v)
{
    return v < 0 ? -v : v;
}

inline void safeRunResetWindow(SafeRun &run, const RobotState &s)
{
    run.windowStartMs = s.timestampMs;
    run.windowEncL = s.encoderCountL;
    run.windowEncR = s.encoderCountR;
}

inline void safeRunEnter(SafeRun &run, RunPhase phase, const RobotState &s)
{
    run.phase = phase;
    run.phaseStartMs = s.timestampMs;
    safeRunResetWindow(run, s);
}

// Both wheels to zero between moves. A hard stop, not a ramp: the wheels coast on from a pivot,
// so ramping would overshoot it.
inline RunWant safeRunSettle(SafeRun &run, RunPhase next, const RobotState &s)
{
    run.next = next;
    safeRunEnter(run, RunPhase::Settle, s);
    RunWant want;
    want.hardStop = true;
    want.phase = RunPhase::Settle;
    return want;
}

inline RunWant safeRunEnd(RunReason reason)
{
    RunWant want;
    want.end = reason;
    return want;
}

// True if either wheel failed to move SAFE_STALL_MIN_COUNTS in the last window. Rolls the
// window over once it is full.
inline bool safeRunStalled(SafeRun &run, const RobotState &s)
{
    if (s.timestampMs - run.windowStartMs < SAFE_STALL_WINDOW_MS) {
        return false;
    }
    const bool stalled = safeRunAbs(s.encoderCountL - run.windowEncL) < SAFE_STALL_MIN_COUNTS ||
                         safeRunAbs(s.encoderCountR - run.windowEncR) < SAFE_STALL_MIN_COUNTS;
    safeRunResetWindow(run, s);
    return stalled;
}

// Wall ahead: pick a turn, or end if there is no safe one. Prefers left when both sides are open.
// A side that is silent is unknown, not open, so it never gets turned into.
inline RunWant safeRunPlanTurn(SafeRun &run, const RobotState &s)
{
    const bool leftValid = (s.usValid & RUN_US_LEFT) != 0;
    const bool rightValid = (s.usValid & RUN_US_RIGHT) != 0;
    const bool leftOpen = leftValid && s.distLeftMm >= SAFE_SIDE_OPEN_MM;
    const bool rightOpen = rightValid && s.distRightMm >= SAFE_SIDE_OPEN_MM;
    const bool leftWall = leftValid && s.distLeftMm < SAFE_SIDE_OPEN_MM;
    const bool rightWall = rightValid && s.distRightMm < SAFE_SIDE_OPEN_MM;

    if (!run.canTurn) {
        return safeRunEnd(RunReason::FrontWall);
    }
    if (run.turnsInRow >= SAFE_MAX_TURNS_IN_ROW) {
        return safeRunEnd(RunReason::TooManyTurns);
    }
    if (leftOpen || rightOpen) {
        run.turnDir = leftOpen ? 1 : -1;
        run.turnTargetDeg = 90.0f;
    } else if (leftWall && rightWall) {
        run.turnDir = -1; // dead end
        run.turnTargetDeg = 180.0f;
    } else {
        return safeRunEnd(RunReason::NoClearSide);
    }
    ++run.turnsInRow;
    return safeRunSettle(run, RunPhase::Turn, s);
}

inline bool safeRunBothWallsClose(const RobotState &s)
{
    const uint8_t both = RUN_US_LEFT | RUN_US_RIGHT;
    return (s.usValid & both) == both && s.distLeftMm < SAFE_SIDE_OPEN_MM &&
           s.distRightMm < SAFE_SIDE_OPEN_MM;
}

// Steering for a straight stretch when the gyro works, in duty; positive = steer left. The inner
// loop is a PI on heading; the outer loop moves its target a few degrees away from the nearer wall
// and slowly learns the corridor's direction, so a start placed at an angle straightens out.
inline int16_t safeRunHeadingSteer(SafeRun &run, const RobotState &s)
{
    constexpr float dt = CONTROL_LOOP_PERIOD_MS / 1000.0f;
    float tilt = 0.0f;
    if (safeRunBothWallsClose(s)) {
        const float e = static_cast<float>(s.distLeftMm) -
                        static_cast<float>(s.distRightMm); // + = nearer right
        tilt = runClamp(SAFE_WALL_TILT_DEG_PER_MM * e, SAFE_WALL_TILT_MAX_DEG);
        run.learnDeg =
            runClamp(run.learnDeg + SAFE_WALL_LEARN_DEG_PER_MM_S * e * dt, SAFE_WALL_LEARN_MAX_DEG);
    }
    const float error = headingWrapDeg(run.targetDeg + run.learnDeg + tilt - s.headingDeg);
    run.headingI = runClamp(run.headingI + SAFE_HEADING_KI * error * dt,
                            static_cast<float>(SAFE_HEADING_I_MAX));
    return static_cast<int16_t>(
        runClamp(SAFE_HEADING_KP * error + run.headingI, static_cast<float>(SAFE_STEER_TOTAL_MAX)));
}

// Steering correction for the two-wall case, in duty. Positive = steer left (the left wheel
// slows, the right one speeds up). Zero unless both side walls are close and heard.
inline int16_t safeRunSteer(const RobotState &s)
{
    if (!safeRunBothWallsClose(s)) {
        return 0;
    }
    const int16_t error =
        static_cast<int16_t>(s.distLeftMm - s.distRightMm); // + = nearer the right wall
    return runClamp(static_cast<int16_t>(error * SAFE_STEER_COUNTS_PER_MM), SAFE_STEER_MAX);
}

// ---- The four functions RunLayer asks of a run ------------------------------

inline RunLimits runLimits(const SafeRun &)
{
    RunLimits limits;
    limits.maxDuty = SAFE_MAX_DUTY;
    limits.rampPerTick = SAFE_RAMP_PER_TICK;
    limits.timeLimitMs = SAFE_RUN_LIMIT_MS;
    limits.selfTestDuty = SAFE_CRUISE_DUTY;
    limits.frontStopMm = SAFE_FRONT_STOP_MM;
    return limits;
}

// No front eyes means no way to stop at a wall: do not start.
inline RunReason runReady(const SafeRun &, const RobotState &s)
{
    return (s.usValid & RUN_US_FRONT) != 0 ? RunReason::None : RunReason::FrontBlind;
}

// The wheels turn: settle, then cruise.
inline RunWant runBegin(SafeRun &run, const RobotState &s)
{
    return safeRunSettle(run, RunPhase::Cruise, s);
}

inline RunWant runStep(SafeRun &run, const RobotState &s)
{
    const uint32_t now = s.timestampMs;
    const bool frontOk = (s.usValid & RUN_US_FRONT) != 0;
    const bool frontBlocked = frontOk && s.distFrontMm <= SAFE_FRONT_STOP_MM;
    RunWant want;
    want.phase = run.phase;

    switch (run.phase) {
        case RunPhase::Cruise:
            if (frontOk) {
                run.lastFrontOkMs = now;
            } else if (now - run.lastFrontOkMs > SAFE_FRONT_BLIND_MS) {
                return safeRunEnd(RunReason::FrontBlind);
            }
            if (now - run.phaseStartMs >= SAFE_TURN_RESET_MS) {
                run.turnsInRow = 0;
            }
            if (frontBlocked) {
                return safeRunPlanTurn(run, s);
            }
            if (safeRunStalled(run, s)) {
                return safeRunEnd(RunReason::Stall);
            }
            {
                const int16_t steer = run.canTurn ? safeRunHeadingSteer(run, s) : safeRunSteer(s);
                want.left = static_cast<int16_t>(SAFE_CRUISE_DUTY - steer);
                want.right = static_cast<int16_t>(SAFE_CRUISE_DUTY + steer);
            }
            break;

        case RunPhase::Settle:
            if (now - run.phaseStartMs >= SAFE_SETTLE_MS) {
                const RunPhase to = run.next;
                safeRunEnter(run, to, s);
                want.phase = to;
                if (to == RunPhase::Turn) {
                    run.turnStartDeg = s.headingDeg;
                } else {
                    run.lastFrontOkMs = now;
                    if (!run.targetSet) {
                        run.targetDeg =
                            s.headingDeg; // the first stretch holds the way it was placed
                        run.targetSet = true;
                    }
                }
            }
            break;

        case RunPhase::Turn: {
            const float turned = headingWrapDeg(s.headingDeg - run.turnStartDeg);
            const float turnedAbs = turned < 0.0f ? -turned : turned;
            if (turnedAbs >= run.turnTargetDeg - SAFE_TURN_EARLY_DEG) {
                run.targetDeg = headingWrapDeg(run.targetDeg + run.turnDir * run.turnTargetDeg);
                run.learnDeg = 0.0f;
                return safeRunSettle(run, RunPhase::Cruise, s);
            }
            if (now - run.phaseStartMs > SAFE_TURN_TIMEOUT_MS ||
                (now - run.phaseStartMs > SAFE_TURN_PROGRESS_MS &&
                 turnedAbs < SAFE_TURN_PROGRESS_DEG)) {
                return safeRunEnd(RunReason::TurnTimeout);
            }
            if (safeRunStalled(run, s)) {
                return safeRunEnd(RunReason::Stall);
            }
            want.left = static_cast<int16_t>(-run.turnDir * SAFE_TURN_DUTY);
            want.right = static_cast<int16_t>(run.turnDir * SAFE_TURN_DUTY);
            break;
        }

        default:
            break; // the layer's phases never reach a run
    }
    return want;
}
