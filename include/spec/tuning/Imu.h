// ============================================================================
//  spec/tuning/Imu.h  --  gyro bias capture, incline level capture and the ramp threshold.
//  The chip's settings: fixed/Hardware.h; its orientation and level: measurements/Robot.h.
// ============================================================================

#pragma once

#include <stdint.h>
#include "spec/measurements/Robot.h"

// ---- Gyro bias capture ------------------------------------------ [TUNED] --
// Averaged at boot with the robot held still. Integrating an
// uncorrected bias of 2 deg/s over RUN_LIMIT_MS would accumulate ~960 deg.
constexpr uint16_t GYRO_BIAS_SAMPLES = 1000;
constexpr uint16_t GYRO_BIAS_MIN_SAMPLES = 500; // fewer usable reads = not trustworthy
// A capture whose readings scatter more than this (standard deviation, raw counts) is thrown away
// and repeated: a wrong zero is silent and heading integrates it without bound. Measured on the
// compact chassis, 2026-10-13/14: a still gyro scatters 1.9-2.9 counts in every still capture and
// still stretch logged; the first 1.5 s of a log taken while the robot was being turned scattered
// 334.
constexpr float GYRO_BIAS_MAX_SPREAD_COUNTS = 8.0f;
constexpr uint8_t GYRO_BIAS_MAX_ATTEMPTS = 10; // each is ~1.4 s at boot
static_assert(GYRO_BIAS_MAX_ATTEMPTS >= 1, "at least one capture attempt");
// Drift left after the boot capture, robot still, 100 Hz, compact chassis, 2026-10-14: +2.40 deg in
// 10 min with the chip already warm and +4.81 deg just after power-on (worst 8 minutes: 2.35 and
// 4.83). The bias moved up to 0.5 counts (warm) and 0.5-0.8 counts (just powered), 0.008-0.012
// deg/s, with the chip only 0.6-1.3 C warmer. A warm-up wait would not have helped, and capturing
// again at START made the 8-minute error smaller in 2 of 4 cases and larger in the other 2, so the
// boot capture stays as it is. Numbers: docs/calibration/.

// ---- Incline level capture --------------------------------- [HOST MODEL] --
// Tilt.h averages the forward-axis reading at boot. A still capture scattered 16-25 counts (std)
// on the bench, a range of about 150 over 1000 samples; one taken while the robot is carried or
// tilted by hand spreads far wider. Not yet checked with the motors running.
constexpr uint16_t INCLINE_LEVEL_MIN_SAMPLES = 50;
constexpr uint16_t INCLINE_LEVEL_MAX_RANGE_COUNTS = 600; // about 2 deg; 4x a still capture

// ---- Ramp incline ------------------------------------------ [HOST MODEL] --
// On the ramp the array may be lifted and read white floor as dark (or dark as white): an all-white
// view is ignored above this nose-up slope. 6 deg: flat floor reads ~0.8 deg and a hard start or
// stop fakes a few; the slope itself reads ~12. In accel counts (Tilt.h).
constexpr int16_t RAMP_INCLINE_COUNTS =
    static_cast<int16_t>(static_cast<int32_t>(IMU_ACCEL_ONE_G_COUNTS) * 1045 / 10000);
