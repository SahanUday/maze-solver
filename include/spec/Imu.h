// ============================================================================
//  spec/Imu.h  --  MPU-6050 settings, gyro bias capture, accel orientation and level, incline.
//  No pins (see config/RobotConfig.h), no Arduino.h (needed for env:native).
//
//  `<<TBD HARDWARE>>` / `<<TBD CALIBRATION>>` = not yet measured/tuned.
//  See scripts/list-tbds.sh.
// ============================================================================

#pragma once

#include <stdint.h>

// ---- IMU (MPU-6050) ---------------------------------------- [FROM DESIGN] --
// Register values and the reasoning: decisions/0008-mpu6050-heading-and-tilt.md.
constexpr uint8_t IMU_I2C_ADDRESS = 0x68;      // ADO left floating
constexpr uint32_t IMU_I2C_CLOCK_HZ = 400000;  // the MPU-6050's fast-mode limit
constexpr uint8_t IMU_STARTUP_DELAY_MS = 30;   // gyro start-up after clearing SLEEP
constexpr float GYRO_COUNTS_PER_DPS = 65.5f;   // +-500 deg/s full scale
constexpr uint16_t ACCEL_COUNTS_PER_G = 16384; // +-2 g full scale
constexpr uint8_t IMU_GYRO_AXIS_YAW = 2;       // board mounted flat, so yaw is Z
// Wire has no timeout by default: a bus held low (an unpowered or unplugged gyro can do it) would
// hang the control loop with the motors still driving. A transfer takes < 1 ms at 400 kHz.
constexpr uint32_t IMU_I2C_TIMEOUT_US = 3000;

// GYRO_COUNTS_PER_DPS was checked on the compact chassis, 2026-10-14, by turning the robot by hand
// through five closed laps each way and integrating the raw counts: 1.001-1.002 of nominal
// (+-0.002, from where the hand stopped), so it stays at 65.5. The accelerometer reads about 4.7%
// high on this unit: see IMU_ACCEL_ONE_G_COUNTS. Numbers: docs/calibration/.

// Gyro bias: averaged at boot with the robot held still. Integrating an
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

// ---- Gyro polarity ----------------------------------------- [MEASURED] --
// Set on the bench: rotate the robot counter-clockwise, headingDeg must rise.
constexpr bool GYRO_YAW_INVERT = false;
// Measured 2026-10-13: a hand turn of about 90 degrees counter-clockwise took headingDeg from 0 to
// +89.4 with this false. Gyro Z carried the turn (peak 35 deg/s; X and Y peaked at 12 and 3).

// ---- Accel orientation ------------------------------------- [MEASURED] --
// Which accel axis lies along the robot, and which sign it reads when the nose rises; the bridge
// incline test reads it (decisions/0008). Measured 2026-10-13 by lifting the front end about
// 26 degrees by hand: Y went from +165 to -7250 counts, against -1790 on X (the sideways lean of
// a hand-held tilt) and -1910 on Z. +Y therefore points toward the rear.
constexpr uint8_t IMU_ACCEL_AXIS_FORWARD = 1;     // index into ImuRaw::accel (0 X, 1 Y, 2 Z)
constexpr bool IMU_ACCEL_NOSE_UP_NEGATIVE = true; // that axis reads negative when the nose is up
// Level reading, 2026-10-14: robot still on a bench, 30 s in each of two orientations 180 degrees
// apart, so a sloping surface cancels. |g| was 17152 counts both times (1.047 x ACCEL_COUNTS_PER_G:
// use this, not 16384, in any angle or threshold formula). The part of X and Y that stayed with the
// robot was X +57 and Y +245 counts: sensor offset and mount tilt together (about 0.8 degrees,
// not separable with two positions). It is a bench value: the same robot read Y between +32 and
// +457 on the table in the two orientations, so a bridge detector should take its level reference
// from the surface the robot starts on and use this only as a sanity check.
constexpr uint16_t IMU_ACCEL_ONE_G_COUNTS = 17152;
constexpr int16_t IMU_ACCEL_FORWARD_LEVEL_COUNTS = 245; // forward-axis (Y) reading when level
// 32-bit on purpose: int is 16 bits on the Mega, and 16384 * 11 does not fit.
static_assert(static_cast<uint32_t>(IMU_ACCEL_ONE_G_COUNTS) * 10 >
                      static_cast<uint32_t>(ACCEL_COUNTS_PER_G) * 9 &&
                  static_cast<uint32_t>(IMU_ACCEL_ONE_G_COUNTS) * 10 <
                      static_cast<uint32_t>(ACCEL_COUNTS_PER_G) * 11,
              "measured 1 g should be within 10% of the nominal scale");

// ---- Incline level capture ---------------------------------- [FROM TESTS] --
// Tilt.h averages the forward-axis reading at boot. A still capture scattered 16-25 counts (std)
// on the bench, a range of about 150 over 1000 samples; one taken while the robot is carried or
// tilted by hand spreads far wider. Not yet checked with the motors running.
constexpr uint16_t INCLINE_LEVEL_MIN_SAMPLES = 50;
constexpr uint16_t INCLINE_LEVEL_MAX_RANGE_COUNTS = 600; // about 2 deg; 4x a still capture

// ---- Ramp incline ------------------------------------------- [FROM TESTS] --
// On the ramp the array may be lifted and read white floor as dark (or dark as white): an all-white
// view is ignored above this nose-up slope. 6 deg: flat floor reads ~0.8 deg and a hard start or
// stop fakes a few; the slope itself reads ~12. In accel counts (Tilt.h).
constexpr int16_t RAMP_INCLINE_COUNTS =
    static_cast<int16_t>(static_cast<int32_t>(IMU_ACCEL_ONE_G_COUNTS) * 1045 / 10000);
