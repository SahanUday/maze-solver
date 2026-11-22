// ============================================================================
//  spec/tuning/LineFollow.h  --  line-following gains, speed, search and slowdown.
//  Layout, tags and the TBD legend: docs/architecture/ARCHITECTURE.md ("Spec layout").
//  The line's width: BRIDGE_OUT_LINE_WIDTH_MM / BRIDGE_IN_LINE_WIDTH_MM in measurements/Arena.h.
// ============================================================================

#pragma once

#include <stdint.h>

// ---- Line following ------------------------------------------------- [TUNED] --
// Line following (PR #28, 2026-10-15, tools/probe/motion.cpp `L`, a 24 mm black line on a white
// floor, 50 cm runs from 11 mm off-centre, 170 mm/s). The follower steers by adding a heading
// offset to the CURRENT heading, so with proportional gain alone the error behaves like an
// integrator and oscillates: KP_LINE 0.3, no D: +-20 mm swings that did not die away; 0.12, no D:
// worse (overshoot to +24 mm, the edge of the array), because a lower gain is less damped. Adding
// D settles it: KP 0.3 + KD 0.15 reached zero in 90-140 mm and held +-5 mm (sd 2.5-3.3 mm) from
// either side.
constexpr float KP_LINE = 0.3f;
constexpr float KD_LINE = 0.15f;
constexpr float LINE_STEER_MAX_DEG = 15.0f;   // clamp on the heading offset the line may command
constexpr uint8_t LINE_SLOWDOWN_PCT = 100;    // speed kept at the largest line error (0..100)
constexpr float LINE_SLOWDOWN_ERR_MM = 20.0f; // line error at which that slowdown is reached
// On the line the speed is set by feedforward alone (there is no distance error to push on), and
// it comes out about 20% over the setting on the floor: 130 gave 120-215 mm/s, 170 gave 177-215.
// 130 with no slowdown took 90-degree bends and an arc; 170 lost the line at the first 90-degree
// bend. Below ~100 mm/s the wheels do not start. (PR #28, 2026-10-15)
constexpr float LINE_SPEED_MM_S = 130.0f;  // cruise speed on the line
constexpr float LINE_ACCEL_MM_S2 = 400.0f; // its acceleration
// A lost line is searched for by turning toward the side it was last seen on, in place at
// LINE_SEARCH_SPEED_MM_S 0 (recovered the robot at the 90-degree bends of the flat test track),
// and given up after LINE_LOST_TIMEOUT_MS: the motors are cut (`LineLost`). LINE_END_LOST_MM > 0
// would end the run when the line has been gone that far; 0 = off: the robot is powered off at the
// end of the line (or, if it is not, it searches for 3 s and stops).
constexpr float LINE_SEARCH_RATE_DPS = 60.0f;
constexpr float LINE_SEARCH_SPEED_MM_S = 0.0f;
constexpr uint16_t LINE_LOST_TIMEOUT_MS = 3000;
constexpr float LINE_END_LOST_MM = 0.0f;
