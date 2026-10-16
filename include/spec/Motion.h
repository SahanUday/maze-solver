// ============================================================================
//  spec/Motion.h  --  distance and turn PID, speed limits, feedforward, move tolerances, stall.
//
//  `<<TBD HARDWARE>>` / `<<TBD CALIBRATION>>` = not yet measured/tuned.
//  See scripts/list-tbds.sh.
// ============================================================================

#pragma once

#include <stdint.h>

// ---- Motion PID gains ---------------------------------------- [MEASURED] --
// Tuned on the floor, 2026-10-14. Duty counts per mm and per deg; KD_TURN per deg/s.
constexpr float KP_DISTANCE = 4.0f;
constexpr float KI_DISTANCE = 2.0f;
constexpr float KD_DISTANCE = 0.0f;
constexpr float KP_TURN = 4.0f;
constexpr float KI_TURN = 60.0f; // pivoting: must break the wheels loose (they stick below ~160)
constexpr float KI_TURN_HOLD = 8.0f; // driving: a turn-sized integral rings on a straight
constexpr float KD_TURN = 0.5f;

// ---- Motion limits and feedforward --------------------------- [MEASURED] --
// Feedforward fit from logged step responses at stepped duty; accelerations are the fastest
// slip-free ramps.
constexpr float SEARCH_SPEED_MM_S = 150.0f; // cruise speed while exploring
// On the line the speed is set by feedforward alone (there is no distance error to push on), and
// it comes out about 20% over the setting on the floor: 130 gave 120-215 mm/s, 170 gave 177-215.
// 130 with no slowdown took 90-degree bends and an arc; 170 lost the line at the first 90-degree
// bend. Below ~100 mm/s the wheels do not start. (PR #28, 2026-10-15)
constexpr float LINE_SPEED_MM_S = 130.0f;         // cruise speed on the line
constexpr float LINE_ACCEL_MM_S2 = 400.0f;        // its acceleration
constexpr float ACCEL_MM_S2 = 500.0f;             // linear acceleration and deceleration
constexpr float TURN_RATE_MAX_DPS = 120.0f;       // in-place turn peak rate
constexpr float TURN_ACCEL_DPS2 = 400.0f;         // in-place turn ramp
constexpr float FEEDFORWARD_PWM_PER_MM_S = 1.15f; // duty counts per mm/s of wheel speed
constexpr float FEEDFORWARD_PWM_PER_DPS = 1.3f;   // duty counts per deg/s of yaw rate
// Friction a rolling wheel must beat, added along the speed reference while it is non-zero. The
// wheels keep going down to duty ~100-120 on the stand but need 160 / 250 to start, so a move
// that ends short of its target stands still and the distance integral (2 duty per mm-second)
// needs seconds to start it again: the host model with those figures stopped 8-12 mm short and
// ended the run in 8 to 46 of 60 starts. Any value from 60 up finished all 60 in the model for
// rolling friction from 60 to 150 duty; larger moves the stop further past the target when the
// wheels roll more freely than assumed. [FROM TESTS] Not measured: log open-loop duty against
// speed with tools/probe `o` and set it to the intercept.
constexpr float FEEDFORWARD_FRICTION_DUTY = 60.0f;
// On the floor the right wheel runs ~20% slower than the left at equal duty (1134 vs 1417
// encoder counts at duty 340); on a stand they match. Multiplies the whole right command.
constexpr float FEEDFORWARD_RIGHT_SCALE = 1.2f;

// ---- Moves ---------------------------------------------------- [MEASURED] --
// 40 absolute 90/180 deg turns, both ways: all within 2 deg of target in 1.3-2.7 s.
// STEP_TOLERANCE_MM: the front sensor jitters +-4 mm and a centre needs ~+-10 mm.
constexpr float STEP_TOLERANCE_MM = 8.0f;  // a stop this close to the target counts as arrived
constexpr float TURN_TOLERANCE_DEG = 2.0f; // a turn this close to the target counts as done
constexpr uint16_t TURN_SETTLE_MS = 150;   // ...for this long

// ---- Stuck detection ---------------------------------- [FROM TESTS] --
// A move is stuck when its error sets no new low (by the minimum step) for PATIENCE_MS of time
// outside tolerance + one step. Move length and speed never trip it. PATIENCE_MS must outlast the
// wheels' break-away at a small residual error (0.8-1.4 s in the host model). Not yet checked on
// the robot: log the longest idle stretch over the 40 turns and some starts from rest, and keep
// PATIENCE_MS well above it.
constexpr float PROGRESS_MIN_MM = 2.0f;
constexpr float PROGRESS_MIN_DEG = 1.0f;
constexpr uint16_t PATIENCE_MS = 2000; // 0 = off

// ---- Stall protection --------------------------------- [FROM TESTS] --
// Duty >= STALL_DUTY with no motion for STALL_TIME_MS cuts the motors. Tuned moves peak at ~280
// while moving and sit below ~80 when stuck at a target. Not yet exercised by a real stall.
constexpr uint16_t STALL_DUTY = 250; // 0 = disabled
constexpr uint16_t STALL_TIME_MS = 600;
constexpr float STALL_MIN_SPEED_MM_S = 5.0f;
constexpr float STALL_MIN_YAW_DPS = 5.0f;
