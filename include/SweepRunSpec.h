// ============================================================================
//  SweepRunSpec.h  --  the duty schedule of the sweep run (SweepRun.h).
//  No pins (see RobotConfig.h), no Arduino.h (needed for env:native).
//  Behaviour: docs/architecture/modules/sweep_run.md
//
//  Duties are timer counts, 0..MOTOR_PWM_TOP.
// ============================================================================

#pragma once

#include <stdint.h>

#include "RobotSpec.h"

// ---- Duty ladder ------------------------------------------- [FROM DESIGN] --
// Each wheel and direction climbs from START to MAX and comes back, one STEP at a time.
constexpr int16_t SWEEP_DUTY_START = 40;
constexpr int16_t SWEEP_DUTY_STEP = 20;
constexpr int16_t SWEEP_DUTY_MAX = 760;

// Both wheels forward at this before the sweep, so a wrong motor or encoder sign shows up as a
// fault, not as a curve. Above MOTOR_MIN_PWM_R.
constexpr int16_t SWEEP_SELFTEST_DUTY = 300;

// ---- Timing ------------------------------------------------ [FROM DESIGN] --
constexpr uint16_t SWEEP_STEP_MS = 400;    // each duty is held this long
constexpr uint16_t SWEEP_MEASURE_MS = 200; // the speed comes from the last of it
constexpr uint16_t SWEEP_SETTLE_MS = 1000; // stopped before each wheel and direction

// ---- Derived ----------------------------------------------------------------
constexpr uint8_t SWEEP_STEPS_PER_RAMP = (SWEEP_DUTY_MAX - SWEEP_DUTY_START) / SWEEP_DUTY_STEP + 1;
constexpr uint8_t SWEEP_COMBOS = 4; // left forward, left reverse, right forward, right reverse
constexpr uint32_t SWEEP_RUN_MS = static_cast<uint32_t>(SWEEP_COMBOS) *
                                  (SWEEP_SETTLE_MS + 2UL * SWEEP_STEPS_PER_RAMP * SWEEP_STEP_MS);
constexpr uint32_t SWEEP_TIME_LIMIT_MS = SWEEP_RUN_MS + 10UL * 1000UL; // margin, not a target

static_assert((SWEEP_DUTY_MAX - SWEEP_DUTY_START) % SWEEP_DUTY_STEP == 0,
              "the ladder must land on SWEEP_DUTY_MAX");
static_assert(SWEEP_DUTY_MAX <= MOTOR_PWM_TOP, "the top of the ladder must fit the PWM range");
static_assert(SWEEP_SELFTEST_DUTY <= SWEEP_DUTY_MAX, "the self-test must fit under the cap");
static_assert(SWEEP_MEASURE_MS <= SWEEP_STEP_MS, "the measuring window must fit in a step");
