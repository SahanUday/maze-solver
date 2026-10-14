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
constexpr uint16_t WHEEL_DIAMETER_MM = 65; // measured with a ruler, rolled circumference / pi
// Effective track, from a recorded 109 degree pivot (gyro) against the encoder difference: 170 mm.
// The two motors sit end to end between the tyres and the axle is 180 mm over the tyres, so tyre
// centre to tyre centre is ~155 mm geometrically; the extra is tyre scrub. Re-measure geometrically
// on the new chassis.
constexpr uint16_t TRACK_WIDTH_MM = 170;
constexpr uint16_t ENCODER_COUNTS_PER_REV = 898; // 10 hand turns: left 8978, right 8971 counts
constexpr float MM_PER_ENCODER_COUNT =
    3.14159265f * WHEEL_DIAMETER_MM / ENCODER_COUNTS_PER_REV; // ~0.2274

// ---- Motor limits ---------------------------------------------- [MEASURED] --
constexpr uint16_t MOTOR_MIN_PWM_L = 160; // duty counts (0..MOTOR_PWM_TOP) that start the wheel
constexpr uint16_t MOTOR_MIN_PWM_R = 250; // measure at MOTOR_PWM_FREQ_HZ, not Arduino's default
// Both are forward, on the floor, from single 150 ms pulses starting at rest. The left moved at
// 160, the lowest tried. A wheel already turning on the stand keeps going down to duty ~100-120, so
// these are start-from-rest values under load, not the floor of the speed curve
// (docs/calibration/).

// What the motion controller adds to every non-zero duty command (MotionControl.h toDuty). The
// start-from-rest values above are not it: lifting every command to 160 / 250 made the turn
// controller chatter between +150 and -200 duty around its target (tried 2026-10-14).
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

// ---- Line detection ---------------------------------- <<TBD CALIBRATION>> --
// 10-bit ADC. A channel whose white-to-black range is below the minimum is unusable (noise is
// +-1 count).
constexpr uint16_t ADC_FULL_SCALE_COUNTS = 1023;
constexpr uint16_t LINE_CAL_MIN_SPAN_COUNTS = 10;

// Normalized 0 (white) .. 255 (black): a channel turns black above ON and white below OFF.
constexpr uint8_t LINE_MASK_ON_PCT = 60;
constexpr uint8_t LINE_MASK_OFF_PCT = 40;

// ---- IR array mounting ------------------------------- <<TBD HARDWARE>> --
// Sensor face to floor at the final mount; the signal falls steeply with height.
constexpr float IR_RIDE_HEIGHT_MM = 0.0f;

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

// ---- Ultrasonic offsets --------------------------------------- [MEASURED] --
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
// Tuned 2026-10-14 on the floor with tools/probe/motion.cpp: 300 and 400 mm straights arrive within
// 1.5 mm of the target with the heading inside 2 deg of its reference; 90 deg turns arrive in
// ~1.3 s without overshoot. Distance gains are duty counts per mm; turn gains per deg (KD_TURN per
// deg/s). The integral terms remove the constant left/right difference: at equal duty the right
// wheel runs ~20% slower on the floor than the left, though the two matched on a stand.
constexpr float KP_DISTANCE = 4.0f;
constexpr float KI_DISTANCE = 2.0f;
constexpr float KD_DISTANCE = 0.0f;
constexpr float KP_TURN = 4.0f;
// KI_TURN: the wheels stick below duty ~160, so near the end of a turn the integral is the only
// thing that gets them moving. At 8 it crept up by 9 duty per 200 ms and a turn that stopped 7 deg
// short timed out; at 60, 16 of 16 turns (90 and 180 deg, both ways) arrived in under 2.4 s, the
// 90s to within 0.5 deg. The straights held the heading within ~3 deg with it.
constexpr float KI_TURN = 60.0f;
// While driving straight the same integral rang (the zig-zag); a separate, much lower one holds the
// heading. Tuned in tools/probe/motion.cpp.
constexpr float KI_TURN_HOLD = 8.0f;
constexpr float KD_TURN = 0.5f;

// ---- Run options ------------------------------------------ [FROM DESIGN] --
// What the run depends on. Each switch keeps the full behaviour available; the defaults below let
// the robot run in a walls-only maze with no IR calibration, no START button and no DIP switches.
// Floor sensing: the IR array finds the approach marker and the bridge line. Off = the robot
// explores every cell and stops (Phase::Done); the line calibration is not required.
constexpr bool RUN_USE_FLOOR_SENSING = false;
// START button on PIN_BTN_START. Off = the run starts RUN_AUTOSTART_MS after boot finishes.
constexpr bool RUN_USE_START_BUTTON = false;
constexpr uint32_t RUN_AUTOSTART_MS = 5000; // time to put the robot down at the start cell
// DIP switches on PIN_DIP. Off = no line-calibration mode and the hand below is used.
constexpr bool RUN_USE_DIP_SWITCHES = false;
constexpr int8_t RUN_HAND = 1; // +1 prefer left turns on a tie, -1 right

// ---- Navigation geometry ----------------------------- <<TBD HARDWARE>> --
// Measured along the robot's forward axis / across it, from the mount drawing or a ruler.
constexpr float IR_ARRAY_FORWARD_OFFSET_MM = 0.0f; // wheel axle to the IR array's sensing line
constexpr float IR_SENSOR_PITCH_MM = 0.0f;         // centre to centre of adjacent IR channels

// ---- Motion limits and feedforward --------------------------- [MEASURED] --
// Fit from logged step responses at stepped duty: speed against duty gives the feedforward,
// the fastest slip-free ramp gives the accelerations.
constexpr float SEARCH_SPEED_MM_S = 150.0f;       // cruise speed while exploring
constexpr float LINE_SPEED_MM_S = 0.0f;           // cruise speed on the line
constexpr float ACCEL_MM_S2 = 500.0f;             // linear acceleration and deceleration
constexpr float TURN_RATE_MAX_DPS = 120.0f;       // in-place turn peak rate
constexpr float TURN_ACCEL_DPS2 = 400.0f;         // in-place turn ramp
constexpr float FEEDFORWARD_PWM_PER_MM_S = 1.15f; // duty counts per mm/s of wheel speed
constexpr float FEEDFORWARD_PWM_PER_DPS = 1.3f;   // duty counts per deg/s of yaw rate
// The right wheel runs ~20% slower than the left at the same duty on the floor (left 1417 vs right
// 1134 encoder counts at duty 340, 2026-10-14), though the two matched on a stand. Its command is
// scaled by this; 1.0 = no correction. Straight-line test, 240 mm, hold integral 8: wheel-duty
// difference 23 at 1.0, 10 at 1.2, heading held within 0.9 deg either way; the 12 turns after it
// all arrived (90 deg within 3 deg, 180 deg within 4.4 deg).
constexpr float FEEDFORWARD_RIGHT_SCALE = 1.2f;

// ---- Steering gains ----------------------------------------- [PARTIAL] --
// Wall centring turns a lateral error (mm) into a heading offset (deg); line following does the
// same from the line centroid. Tune P first, add D if it oscillates.
// Wall (2026-10-14, tools/probe/motion.cpp). Measured twice. With KI_TURN 8, KP_WALL 0.3-0.7 looked
// stable over 450 mm. With KI_TURN 60 (needed to finish turns) and the ultrasonics' 60 ms refresh,
// 0.2 and 0.4 swung the heading by 17-19 deg peak to peak (the zig-zag, and a wall hit in the
// maze); 0.1 held it within 1.6 deg of its reference and the robot within about +-10 mm of the
// centre of the corridor. KD_WALL is not used.
constexpr float KP_WALL = 0.1f;
constexpr float KD_WALL = 0.0f;
constexpr float KP_LINE = 0.0f;
constexpr float KD_LINE = 0.0f;
constexpr float STEER_MAX_DEG = 8.0f; // clamp on the offset either loop may command

// ---- Wall sensing ------------------------------------------- [MEASURED] --
// Distances are the offset-corrected readings in RobotState, so the reference point of each is
// whatever US_OFFSET_* made it: the axle for the front, the centreline for the sides.
// Side: a wall reads 117 +-30 mm; through a gap in the wall it read 396-418 mm (a few echoes
// 763-784), never "no echo". 250 sits between them. Front: the wall that ends the cell is 117 mm
// ahead of the axle at the centre, 167-197 mm at the sample point, and the next wall is 350+ mm;
// the reading is good to ~3 mm up to 300 mm and falls up to 6% short beyond that (the beam is wider
// than the corridor), so 260 is well inside the accurate range. The corridor measured 231-235 mm
// (left + right readings of a centred robot).
constexpr uint16_t WALL_PRESENT_SIDE_MM = 250;  // side reading below this = wall
constexpr uint16_t WALL_PRESENT_FRONT_MM = 260; // front reading below this = wall (at the sample)
constexpr uint16_t WALL_CENTER_DIST_MM = 117; // side reading when centred, used with one wall only
constexpr uint16_t FRONT_STOP_MM = 117;       // front reading when stopped at a cell centre
// Braking from SEARCH_SPEED_MM_S at ACCEL_MM_S2 takes v^2 / 2a = 22 mm, so the decision point is
// 50 mm out; at 150 mm/s each sensor updates every 60 ms (9 mm), so 30 mm holds about 3 echoes.
constexpr uint16_t WALL_SAMPLE_LEAD_MM = 50;   // walls are sampled this far before the cell centre
constexpr uint16_t WALL_SAMPLE_WINDOW_MM = 30; // ...over this much travel

// ---- Moves ---------------------------------------------------- [MEASURED] --
// A turn that ends up to ~5 deg short is left there: static friction stops the wheels below duty
// ~160, so a tighter tolerance only waits for the integral to wind up. The next straight holds the
// grid heading and takes the remainder out. The first 4 s timeout fits a 300 mm move (2.4 s). A
// stop tolerance of 3 mm timed out in the mission: the target is anchored to the front sensor,
// which jitters by +-4 mm, and the wheels stick, so the robot hunted at the wall. A centre needs
// ~+-10 mm.
constexpr float STEP_TOLERANCE_MM = 8.0f;  // a stop this close to the target counts as arrived
constexpr float TURN_TOLERANCE_DEG = 6.0f; // a turn this close to the target counts as done
constexpr uint16_t TURN_SETTLE_MS = 150;   // ...once it has stayed inside the tolerance this long
constexpr uint16_t MOVE_TIMEOUT_MS = 4000; // a move longer than this is a fault; 0 = no limit

// ---- Stall protection --------------------------------- [FROM TESTS] --
// Duty at or above STALL_DUTY with the robot not moving for STALL_TIME_MS cuts the motors.
// The tuned moves peak at duty ~280 while moving, and sit below ~80 when stuck near a target, so
// 250 for 0.6 s means "pushing hard and going nowhere". Not yet tested by actually stalling.
constexpr uint16_t STALL_DUTY = 250; // 0 = disabled
constexpr uint16_t STALL_TIME_MS = 600;
constexpr float STALL_MIN_SPEED_MM_S = 5.0f;
constexpr float STALL_MIN_YAW_DPS = 5.0f;

// Hard cap on any wheel's duty in main.cpp, whatever the controller asks for (the timer's TOP is
// 799). The tuned moves stay below ~300.
constexpr int16_t DRIVE_MAX_DUTY = 450;

// A status line over serial this often while the mission runs (phase, odometer, speed, heading,
// duty, the three ultrasonics, the step target). 0 = off.
constexpr uint16_t RUN_LOG_EVERY_MS = 200;

// ---- Line zone ---------------------------------------- <<TBD CALIBRATION>> --
// The approach marker is a 90 mm white square, so a short run of non-black floor; a line tile
// is a long one.
constexpr float MARKER_RUN_MIN_MM = 0.0f;
constexpr float MARKER_RUN_MAX_MM = 0.0f;     // a longer run is a line tile, not the marker
constexpr float LINE_LEAVE_MARGIN_MM = 40.0f; // leaving a white tile, the array this far past its
                                              // edge still on white = going out along the line
constexpr float LINE_ENTER_RUN_MM = 0.0f;     // non-black this far = the line zone has been entered
// Bare floor this long under the array = the line zone has ended. Must exceed the 30 mm line,
// which can cross the array inside a zone.
constexpr float LINE_END_ZONE_MM = 0.0f;
// After the line's end the AXLE drives on this far, into Section B, and stops. The end is measured
// under the array, which is IR_ARRAY_FORWARD_OFFSET_MM ahead of the axle, so without this the robot
// would stop with its wheels still on the line.
constexpr float LINE_EXIT_MM = 0.0f;
// Bare floor must last this far to end a run of white. Wider than the 30 mm line, so a line
// crossing a white tile does not split it in two; narrower than the 80 mm of floor between the
// approach marker and the bridge tile, so those stay two runs.
constexpr float FLOOR_RUN_GAP_MM = 45.0f;
constexpr uint16_t LINE_LOST_TIMEOUT_MS = 0; // no line this long while searching = fault; 0 = none
constexpr float LINE_SEARCH_RATE_DPS = 0.0f; // spin rate when the line is lost
constexpr uint8_t LINE_SLOWDOWN_PCT = 50;    // speed kept at the largest line error (0..100)
constexpr float LINE_SLOWDOWN_ERR_MM = 0.0f; // line error at which that slowdown is reached

// ---- Ramp --------------------------------------------- <<TBD CALIBRATION>> --
// Tilt of the gravity vector from the board's Z axis. Above this the IR array may be lifted off
// the floor, so "all black" no longer means the line zone ended.
constexpr float RAMP_TILT_DEG = 0.0f;
