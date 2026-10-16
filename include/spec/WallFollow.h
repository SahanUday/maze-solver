// ============================================================================
//  spec/WallFollow.h  --  wall presence and centring thresholds, heading trim, front-echo
//  filtering.
//
//  `<<TBD HARDWARE>>` / `<<TBD CALIBRATION>>` = not yet measured/tuned.
//  See scripts/list-tbds.sh.
// ============================================================================

#pragma once

#include <stdint.h>
#include "spec/Motion.h"

// ---- Wall centring gains ------------------------------------- [PARTIAL] --
// Wall centring: a lateral error (mm) to a heading offset (deg). KP_WALL tuned on the floor,
// 2026-10-14; KI_WALL and the bias limits from the host model with the measured sensor figures.
constexpr float KP_WALL = 0.1f;       // deg per mm; 0.2 and up swings the heading at KI_TURN 60
constexpr float KI_WALL = 0.10f;      // deg per mm per second; learns the heading reference error
constexpr float STEER_MAX_DEG = 8.0f; // clamp on the whole offset

// The corridor's direction is measured over this much travel along one wall (a fitted line, about
// 1 degree of noise at 3 mm sensor noise); the heading frame's error against it is taken out of
// the gyro heading by this share per stretch (stepCellTakeHeadingFix), where the bias integral
// alone relearns it after every turn.
constexpr float WALL_SNAP_MM = 150.0f;
constexpr float WALL_SNAP_GAIN = 0.5f;
constexpr float WALL_SNAP_MIN_DEG = 0.3f;
// What the integral may learn: a 2 deg turn tolerance plus ~4 deg of gyro drift over a run. It is
// dropped when the heading moves this far from where it was learned (a turn).
constexpr float WALL_BIAS_MAX_DEG = 6.0f;
constexpr float WALL_BIAS_RESET_DEG = 20.0f;
// With no wall to confirm it the bias is held this long (a gap of a cell or two), then faded out:
// a bias learned from a bad reading must not steer the robot for ever.
constexpr uint16_t WALL_BIAS_HOLD_MS = 2000;
constexpr float WALL_BIAS_FADE_DEG_S = 4.0f;

// ---- Wall sensing ------------------------------------------- [MEASURED] --
// Readings are the offset-corrected values in RobotState: the front from the axle, the sides from
// the centreline. A wall reads 117 +-30 mm at the side and a gap 396 mm or more, never "no echo".
// The front reads true to ~3 mm up to 300 mm and up to 6% short beyond; the wall that ends the
// cell is 167-197 mm ahead of the axle at the sample point and the next one 350+ mm. Corridor
// 231-235 mm (left + right of a centred robot).
constexpr uint16_t WALL_PRESENT_SIDE_MM = 250;  // side reading below this = wall
constexpr uint16_t WALL_PRESENT_FRONT_MM = 260; // front reading below this = wall (at the sample)
constexpr uint16_t WALL_CENTER_DIST_MM = 117;   // side reading when centred; one wall only
// Both walls: |left + right - 2 * centre| above this means one reading is wrong (a corridor
// measured 231-235 mm; two readings at 3 mm noise). No centring then; the gyro holds the heading.
constexpr uint16_t WALL_PAIR_TOL_MM = 30;
// A side sensor whose reading does not change at all for this long is frozen, not a flat wall
// (the real reading scatters by ~3 mm), so its valid bit is dropped.
constexpr uint16_t WALL_FROZEN_MS = 400;
constexpr uint16_t FRONT_STOP_MM = 117; // front reading when stopped at a cell centre
// Braking from SEARCH_SPEED_MM_S at ACCEL_MM_S2 takes v^2 / 2a = 22 mm; at 150 mm/s a sensor
// updates every 60 ms (9 mm), so 30 mm holds about 3 echoes.
constexpr uint16_t WALL_SAMPLE_LEAD_MM = 50;   // walls are sampled this far before the centre
constexpr uint16_t WALL_SAMPLE_WINDOW_MM = 30; // ...over this much travel
static_assert(WALL_SAMPLE_LEAD_MM >= SEARCH_SPEED_MM_S * SEARCH_SPEED_MM_S / (2.0f * ACCEL_MM_S2),
              "the decision point must be at least a braking distance from the centre");
// A front echo moves the stop only if it puts the wall within this of the echo before (two echoes
// jitter +-4 mm each, plus a tick of travel). Not yet checked against logged stray echoes.
constexpr float FRONT_AGREE_MM = 15.0f;
// Weight of each accepted front echo in the stop; the front alternated 110/119 mm at a dead end.
constexpr float FRONT_ANCHOR_FILTER = 0.3f;
// The front sensor returns nothing below ~30 mm raw, 114 mm from the axle, and a stop lands 109 to
// 126 mm: a move that begins in that zone hears no echo. A first echo is waited for this long (5
// refresh periods, three misses at the 120 ms a missed slot costs), then the wall is taken to be in
// the dead zone. 0 = never wait.
constexpr uint16_t FRONT_BLIND_MS = 300;
// A front reading that does not change over this much travel is frozen: approaching a wall it
// changes by the distance covered. Not applied at rest, where it repeats (sd 0.0 at 300 mm).
constexpr float FRONT_FROZEN_MM = 60.0f;
