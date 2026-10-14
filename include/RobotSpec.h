// ============================================================================
//  RobotSpec.h  --  geometry, timing, motion and control constants.
//  No pins (see RobotConfig.h), no Arduino.h (needed for env:native).
//
//  `<<TBD HARDWARE>>` / `<<TBD CALIBRATION>>` = not yet measured/tuned.
//  See scripts/list-tbds.sh.
// ============================================================================

#pragma once

#include <stdint.h>

// ---- Competition geometry --------------------------------- [FROM SPEC] --
constexpr uint16_t TILE_PITCH_MM = 250; // centre-to-centre tile spacing
constexpr uint16_t WALL_HEIGHT_MM = 100;
constexpr uint16_t USABLE_CORRIDOR_MM = 235;  // tile pitch minus wall thickness
constexpr uint8_t SECTION_A_SIZE_TILES = 4;   // 4x4
constexpr uint8_t SECTION_B_SIZE_TILES = 9;   // 9x9
constexpr uint16_t BRIDGE_LINE_WIDTH_MM = 30; // 3 cm black line

// ---- Competition timing ------------------------------------ [FROM SPEC] --
constexpr uint32_t RUN_LIMIT_MS = 8UL * 60UL * 1000UL;         // 8 minutes per trial
constexpr uint32_t ARENA_TIME_LIMIT_MS = 30UL * 60UL * 1000UL; // total arena access
constexpr uint8_t TRIAL_COUNT = 3;                             // best-of-3

// ---- Power / memory budget ---------------------------------- [FROM SPEC] --
constexpr uint16_t BATTERY_MAX_MV = 15000;   // charged pack must stay under this
constexpr uint16_t SRAM_BUDGET_BYTES = 8192; // enforced by scripts/check-ram-budget.sh

// ---- Control loop timing ---------------------------------------------- --
constexpr uint16_t CONTROL_LOOP_PERIOD_MS = 10; // 100 Hz, keeps PID dt fixed

// ---- Motor PWM -------------------------------------------- [FROM DESIGN] --
// Fast PWM, ICRn as TOP, clk/1. BTS7960 tolerates up to ~25 kHz; 20 kHz is
// above audible range. Duty is expressed in timer counts, 0..MOTOR_PWM_TOP.
constexpr uint32_t CPU_HZ = 16000000UL;
constexpr uint32_t MOTOR_PWM_FREQ_HZ = 20000;
static_assert(CPU_HZ % MOTOR_PWM_FREQ_HZ == 0, "PWM frequency must divide CPU_HZ exactly");
constexpr uint16_t MOTOR_PWM_TOP = CPU_HZ / MOTOR_PWM_FREQ_HZ - 1; // 799

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

// ---- Debug serial ------------------------------------------------------ --
constexpr uint32_t DEBUG_SERIAL_BAUD = 250000; // exact divisor of 16 MHz

// ---- Wheel / drivetrain geometry --------------------------- [MEASURED] --
constexpr uint16_t WHEEL_DIAMETER_MM = 65; // ruler, rolled circumference / pi
// Effective track: a 109 deg gyro pivot against the encoder difference. Tyre-centre spacing is
// ~155 mm; the rest is tyre scrub.
constexpr uint16_t TRACK_WIDTH_MM = 170;
constexpr uint16_t ENCODER_COUNTS_PER_REV = 898; // 10 hand turns: left 8978, right 8971
constexpr float MM_PER_ENCODER_COUNT =
    3.14159265f * WHEEL_DIAMETER_MM / ENCODER_COUNTS_PER_REV; // ~0.2274

// ---- Motor limits ---------------------------------------------- [MEASURED] --
// Start-from-rest duty (0..MOTOR_PWM_TOP), forward, on the floor, from single 150 ms pulses.
// A wheel already turning keeps going down to ~100-120. Measure at MOTOR_PWM_FREQ_HZ.
constexpr uint16_t MOTOR_MIN_PWM_L = 160; // the lowest duty tried
constexpr uint16_t MOTOR_MIN_PWM_R = 250;

// What the controller lifts non-zero commands to; 0 = no lift. The start values above make a
// turn chatter around its target.
constexpr uint16_t CONTROL_MIN_PWM_L = 0;
constexpr uint16_t CONTROL_MIN_PWM_R = 0;

// ---- IR array ---------------------------------------------------------- --
// Analog reflectance array read through the ADC. See decisions/0005-*.md.
constexpr uint8_t IR_CHANNEL_COUNT = 8;

// ADC clock = CPU_HZ / IR_ADC_PRESCALER; the driver caps it at 1 MHz.
constexpr uint8_t IR_ADC_PRESCALER = 32;

// Wait after the emitter enable switches before a reading is valid.
constexpr uint16_t IR_EMITTER_SETTLE_US = 1000;

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

// ---- Line detection ---------------------------------- <<TBD CALIBRATION>> --
// 10-bit ADC. A channel whose white-to-black range is below the minimum is unusable.
constexpr uint16_t ADC_FULL_SCALE_COUNTS = 1023;
constexpr uint16_t ADC_NOISE_COUNTS = 1; // +-1 count, one sigma
constexpr uint16_t LINE_CAL_MIN_SPAN_COUNTS = 30;

// Normalized 0 (white) .. 255 (black): a channel turns black above ON and white below OFF.
constexpr uint8_t LINE_MASK_ON_PCT = 60;
constexpr uint8_t LINE_MASK_OFF_PCT = 40;
// The hysteresis band of the narrowest accepted channel must be at least 6 sigma wide, or its mask
// flickers where the line edge passes: host model, reading at the middle of the range, 1000 ticks
// of 8 channels: a span of 20 flipped 604 times, 25 151 times, 29 twice, 36 never.
static_assert(static_cast<uint32_t>(LINE_CAL_MIN_SPAN_COUNTS) *
                      (LINE_MASK_ON_PCT - LINE_MASK_OFF_PCT) / 100 >=
                  6u * ADC_NOISE_COUNTS,
              "LINE_CAL_MIN_SPAN_COUNTS is too small for the hysteresis band against ADC noise");

// ---- IR array mounting ------------------------------- <<TBD HARDWARE>> --
// Sensor face to floor at the final mount; the signal falls steeply with height.
constexpr float IR_RIDE_HEIGHT_MM = 0.0f;
constexpr float IR_SENSOR_PITCH_MM = 0.0f; // channel to channel; 0 = LineView::errMm reads 0

// ---- Ultrasonic sensing ------------------------------------------------ --
// HC-SR04 x3, round-robin. See decisions/0007-*.md.
constexpr uint8_t US_SENSOR_COUNT = 3;

// One sensor per slot; 2 ticks, so slots land on tick boundaries.
constexpr uint8_t US_SLOT_MS = 20;

// Echo timer (Timer5) prescaler 8 at 16MHz gives 0.5us per tick.
constexpr uint8_t US_TIMER_PRESCALER = 8;

// Speed of sound at 20C. Temperature is not compensated.
constexpr uint16_t SPEED_OF_SOUND_M_S = 343;

// Datasheet capability. <<TBD HARDWARE>> - confirm on the bench.
constexpr uint16_t US_MIN_RANGE_MM = 20;
constexpr uint16_t US_MAX_RANGE_MM = 4000;

// Driver stops listening here, before the module's own ~71ms no-echo timeout (bench).
// Exceeds the 2250mm longest sightline a 9x9 arena can present.
constexpr uint16_t US_RANGE_CAP_MM = 2500;

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

// Echo deadline in 0.5us ticks: US_RANGE_CAP_MM * 4000 / 343.
constexpr uint16_t US_ECHO_TIMEOUT_TICKS =
    static_cast<uint16_t>((static_cast<uint32_t>(US_RANGE_CAP_MM) * 4000u) / SPEED_OF_SOUND_M_S);

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
constexpr float SEARCH_SPEED_MM_S = 150.0f;       // cruise speed while exploring
constexpr float LINE_SPEED_MM_S = 0.0f;           // cruise speed on the line; not set
constexpr float ACCEL_MM_S2 = 500.0f;             // linear acceleration and deceleration
constexpr float TURN_RATE_MAX_DPS = 120.0f;       // in-place turn peak rate
constexpr float TURN_ACCEL_DPS2 = 400.0f;         // in-place turn ramp
constexpr float FEEDFORWARD_PWM_PER_MM_S = 1.15f; // duty counts per mm/s of wheel speed
constexpr float FEEDFORWARD_PWM_PER_DPS = 1.3f;   // duty counts per deg/s of yaw rate
// On the floor the right wheel runs ~20% slower than the left at equal duty (1134 vs 1417
// encoder counts at duty 340); on a stand they match. Multiplies the whole right command.
constexpr float FEEDFORWARD_RIGHT_SCALE = 1.2f;

// ---- Steering gains ----------------------------------------- [PARTIAL] --
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
