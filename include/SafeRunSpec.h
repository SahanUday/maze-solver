// ============================================================================
//  SafeRunSpec.h  --  limits and gains for the safe run (SafeRun.h). Start-up and
//  self-test constants shared with other runs are in RunLayerSpec.h.
//  No pins (see RobotConfig.h), no Arduino.h (needed for env:native).
//  Behaviour: docs/architecture/modules/safe_run.md
//
//  Deliberately timid: the run drives open loop at ~35% duty and stops on any
//  surprise. Duties are timer counts, 0..MOTOR_PWM_TOP.
// ============================================================================

#pragma once

#include <stdint.h>

#include "RobotSpec.h"

// ---- Duty -------------------------------------------------- [FROM DESIGN] --
constexpr int16_t SAFE_CRUISE_DUTY = 280; // ~35% of MOTOR_PWM_TOP; above MOTOR_MIN_PWM_R
constexpr int16_t SAFE_TURN_DUTY = 320; // each wheel, opposite directions; the right wheel needs it
constexpr int16_t SAFE_MAX_DUTY = 420;  // hard cap on any wheel command
constexpr int16_t SAFE_RAMP_PER_TICK = 8; // duty change per tick: 0 -> cruise in 300 ms

// ---- Time limit and settle --------------------------------- [FROM DESIGN] --
constexpr uint32_t SAFE_RUN_LIMIT_MS = 60UL * 1000UL; // the run stops itself after a minute
constexpr uint16_t SAFE_SETTLE_MS = 400; // stopped between phases, so no reversal at speed

// ---- Stall ------------------------------------------------- [FROM DESIGN] --
// Each encoder must move this much in every window while driving.
constexpr uint16_t SAFE_STALL_WINDOW_MS = 500;
constexpr int32_t SAFE_STALL_MIN_COUNTS = 10;

// ---- Walls ------------------------------------------------- [FROM DESIGN] --
// Face-to-wall distances; the ultrasonic offsets are not calibrated yet. A hard stop ends ~10 mm
// short of SAFE_FRONT_STOP_MM. The robot pivots about its axle with ~3 cm to each side wall, so it
// has to stop close to the front wall for the tail to clear. <<TBD HARDWARE>> - depends on the
// chassis outline.
constexpr uint16_t SAFE_FRONT_STOP_MM = 70;
constexpr uint16_t SAFE_FRONT_BLIND_MS = 300; // no front echo for this long while moving = stop

// A side farther than this (or silent) is not steered by.
constexpr uint16_t SAFE_SIDE_OPEN_MM = 200;
constexpr int16_t SAFE_STEER_COUNTS_PER_MM = 1;
constexpr int16_t SAFE_STEER_MAX = 40; // wall steering when there is no gyro

// ---- Heading hold (gyro) ----------------------------------- [FROM DESIGN] --
// The wheels do not match under load (docs/architecture/modules/motors.md), which a P loop alone
// would answer with a steady heading error; the integral term absorbs it.
constexpr float SAFE_HEADING_KP = 6.0f;       // duty per degree of heading error
constexpr float SAFE_HEADING_KI = 15.0f;      // duty per degree-second
constexpr int16_t SAFE_HEADING_I_MAX = 80;    // duty
constexpr int16_t SAFE_STEER_TOTAL_MAX = 100; // duty, P + I

// Wall alignment: left-minus-right distance tilts the target heading, and a slow integral learns
// the corridor direction (a start that was placed a few degrees off).
constexpr float SAFE_WALL_TILT_DEG_PER_MM = 0.15f;
constexpr float SAFE_WALL_TILT_MAX_DEG = 8.0f;
constexpr float SAFE_WALL_LEARN_DEG_PER_MM_S = 0.1f;
constexpr float SAFE_WALL_LEARN_MAX_DEG = 12.0f;

// ---- Turns ------------------------------------------------- [FROM DESIGN] --
// A turn ends on the gyro angle (sign-independent), or on the timeout.
constexpr uint16_t SAFE_TURN_TIMEOUT_MS = 3000;

// A pivot that is not at least this far round after this long is jammed against a wall: stop
// pushing.
constexpr uint16_t SAFE_TURN_PROGRESS_MS = 1000;
constexpr float SAFE_TURN_PROGRESS_DEG = 30.0f;

// Stop this far short of the target; the wheels coast on (~170 deg/s).
constexpr float SAFE_TURN_EARLY_DEG = 8.0f;
constexpr uint8_t SAFE_MAX_TURNS_IN_ROW = 3;
constexpr uint16_t SAFE_TURN_RESET_MS = 1000; // this long straight clears the in-a-row count

static_assert(SAFE_MAX_DUTY <= MOTOR_PWM_TOP, "the cap must fit the PWM range");
static_assert(SAFE_CRUISE_DUTY + SAFE_STEER_TOTAL_MAX <= SAFE_MAX_DUTY,
              "steering must not hit the cap");
static_assert(SAFE_CRUISE_DUTY + SAFE_STEER_MAX <= SAFE_MAX_DUTY,
              "wall steering must not hit the cap");
static_assert(SAFE_TURN_DUTY <= SAFE_MAX_DUTY, "turn duty must fit under the cap");
