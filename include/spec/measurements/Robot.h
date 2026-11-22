// ============================================================================
//  spec/measurements/Robot.h  --  the robot as built: wheels, sensor mounting and offsets,
//  polarity, IMU orientation. Re-measure after a chassis change. Layout, tags and the TBD legend:
//  docs/architecture/ARCHITECTURE.md ("Spec layout").
// ============================================================================

#pragma once

#include <stdint.h>
#include "spec/fixed/Hardware.h"

// ---- Drivetrain polarity ------------------------------------ [MEASURED] --
// The motors are mounted mirrored, so one side needs its direction flipped.
// Set on the bench: command a positive speed, the robot must roll forward and
// the encoder count must rise.
constexpr bool MOTOR_L_INVERT = true;
constexpr bool MOTOR_R_INVERT = false;
constexpr bool ENC_L_INVERT = false;
constexpr bool ENC_R_INVERT = false;
// Measured 2026-10-13 on the compact chassis, wheels free, duty 300 for 800 ms: each wheel rolled
// forward and its own encoder rose (left +1155, right +1154; both together +1173 / +1154).

// ---- Wheel / drivetrain geometry --------------------------- [MEASURED] --
constexpr uint16_t WHEEL_DIAMETER_MM = 65; // ruler, rolled circumference / pi
// Effective track: a 109 deg gyro pivot against the encoder difference. Tyre-centre spacing is
// ~155 mm; the rest is tyre scrub.
constexpr uint16_t TRACK_WIDTH_MM = 170;
constexpr uint16_t ENCODER_COUNTS_PER_REV = 898; // 10 hand turns: left 8978, right 8971
constexpr float MM_PER_ENCODER_COUNT =
    3.14159265f * WHEEL_DIAMETER_MM / ENCODER_COUNTS_PER_REV; // ~0.2274

// ---- Motor start duty -------------------------------------------- [MEASURED] --
// Start-from-rest duty (0..MOTOR_PWM_TOP), forward, on the floor, from single 150 ms pulses.
// A wheel already turning keeps going down to ~100-120. Measure at MOTOR_PWM_FREQ_HZ.
// What the controller lifts commands to: CONTROL_MIN_PWM_* in tuning/Motion.h.
constexpr uint16_t MOTOR_MIN_PWM_L = 160; // the lowest duty tried
constexpr uint16_t MOTOR_MIN_PWM_R = 250;

// ---- IR array mounting ---------------------------------------- [MEASURED] --
// Fixed by the chassis. Measured on the robot (PR #28, 2026-10-14/15).
// Sensor face to floor: the signal falls steeply with height.
constexpr float IR_RIDE_HEIGHT_MM = 7.0f;
constexpr float IR_SENSOR_PITCH_MM = 8.0f;          // channel to channel
constexpr float IR_ARRAY_FORWARD_OFFSET_MM = 86.0f; // wheel axle to the array's sensing line
// Channel 0 is D1 (pin A0), and D1 is the robot's RIGHT end: a black card under only the right end
// saturated channels 0 and 1, under only the left end channels 6 and 7.
constexpr bool IR_D1_ON_RIGHT = true;

// ---- Ultrasonic offsets --------------------------- <<TBD CALIBRATION>> --
// Added to the raw face-to-target reading, per sensor.
// offset = true distance from the reference point to the wall - raw reading. A sensor that sits
// ahead of its reference point therefore needs a positive offset.
// Each sensor may use a different reference point: front = the wheel-axle line, left and right =
// the robot's centreline.
// Front: measured 2026-10-14, flat wall square to the robot, 5 s of readings per position, robot
// still. Axle-to-wall 150 / 200 / 250 / 300 mm read 63.8 / 116.8 / 166.3 / 216.6 mm, so the offset
// is +86 / +83 / +84 / +83 mm: the sensor face is about 84 mm ahead of the axle. (A first 200 mm
// capture read 110.9; the robot was not in place, and a retake gave 116.8.)
constexpr int16_t US_OFFSET_FRONT_MM = 84;
// Left: measured 2026-10-14 the same way, from the robot's centreline. Centreline-to-wall
// 100 / 150 / 200 / 250 / 296 mm read 61.5 / 115.4 / 170.6 / 220.8 / 255.5 mm, so the offset is
// +38.5 / +35 / +29 / +29 / +40.5 mm. It scatters by more than the sensor's own noise (about 3
// mm), so hand placement of the wall limits it; +35 is the middle of the spread.
constexpr int16_t US_OFFSET_LEFT_MM = 35;
// Right: measured 2026-10-14 the same way. Centreline-to-wall 100 / 150 / 200 / 250 mm read 51.2 /
// 104.9 / 153.8 / 202.1 mm, so the offset is +48.8 / +45.1 / +46.2 / +47.9 mm (a first 250 mm
// capture read 197.3; the wall had slipped, and a retake gave 202.1).
constexpr int16_t US_OFFSET_RIGHT_MM = 47;

// ---- IMU mounting and polarity ----------------------------- [MEASURED] --
constexpr uint8_t IMU_GYRO_AXIS_YAW = 2; // board mounted flat, so yaw is Z
// Set on the bench: rotate the robot counter-clockwise, headingDeg must rise.
constexpr bool GYRO_YAW_INVERT = false;
// Measured 2026-10-13: a hand turn of about 90 degrees counter-clockwise took headingDeg from 0 to
// +89.4 with this false. Gyro Z carried the turn (peak 35 deg/s; X and Y peaked at 12 and 3).

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
