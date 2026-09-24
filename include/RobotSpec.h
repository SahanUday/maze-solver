// ============================================================================
//  RobotSpec.h  --  competition geometry, timing, motion and control
//  constants. No pins, no hardware framework.
//
//  RULES FOR THIS FILE (SW-01):
//    1. Nothing here may depend on Arduino.h, avr/io.h, or any hardware
//       framework. This is what lets lib/maze (env:native) include it
//       directly for geometry constants without dragging in a framework
//       that doesn't exist on the host build.
//    2. Pin assignments live in RobotConfig.h, not here - see that file's
//       own rules for why pins and constants are split.
//    3. Use `constexpr`, never `#define`, and put the unit in the name
//       (_MM, _MS, _US, _DEG, _MV, _PWM) - same rules as RobotConfig.h.
//    4. Values marked  <<TBD HW-01>>  /  <<TBD CAL-01>>  are placeholders,
//       filled in from docs/HARDWARE_TRUTH_SHEET.md once the robot is
//       measured. Run `scripts/list-tbds.sh` to see what's still unknown -
//       don't rely on spotting the comment.
//
//  Two kinds of constant below:
//    - Rules-of-the-competition values: fixed by the spec, not by the robot.
//      Safe to hardcode now.
//    - Robot-physical values: depend on parts we have not measured or tuned
//      yet. Left as <<TBD HW-01>> / <<TBD CAL-01>> placeholders. Filling
//      these in without a real measurement just moves the bug from
//      "obviously missing" to "silently wrong".
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

// ---- Control loop timing ------------------------------------ [SW-01] --
// A fixed period is what makes the PID's dt a known constant instead of
// something that drifts with whatever else the loop happens to be doing that
// tick. 10 ms (100 Hz) clears the "20 Hz or better" Phase-1 gate with margin;
// CAL-01 may retune once real sensor/motor timing is known.
constexpr uint16_t CONTROL_LOOP_PERIOD_MS = 10;

// ---- Debug serial ------------------------------------------- [SW-01] --
// Exact divisor of 16 MHz, so it has 0.0% baud error, unlike 115200 (-3.5%).
// Named here so main.cpp and platformio.ini's monitor_speed never drift
// apart - if you change one, change both from this constant's value.
constexpr uint32_t DEBUG_SERIAL_BAUD = 250000;

// ---- Wheel / drivetrain geometry ---------------------- <<TBD HW-01>> --
constexpr uint16_t WHEEL_DIAMETER_MM = 0;      // TBD HW-01: rolled circumference / pi
constexpr uint16_t TRACK_WIDTH_MM = 0;         // TBD HW-01: measured, not nominal
constexpr uint16_t ENCODER_COUNTS_PER_REV = 0; // TBD HW-01: one full wheel turn
// mm travelled per encoder count. Derived from the two rows above once they
// exist; kept as its own named constant so nothing recomputes it slightly
// differently in two places.
//
// Left un-derived (not `WHEEL_DIAMETER_MM * PI / ENCODER_COUNTS_PER_REV`) on
// purpose while both inputs are 0: that expression would silently evaluate
// to 0/0 territory before HW-01 exists. HW-01 should replace this literal
// with the real derived expression once both inputs are measured, not fill
// in a third independently-guessed number.
constexpr float MM_PER_ENCODER_COUNT = 0.0f; // TBD HW-01

// ---- Motor limits -------------------------------------- <<TBD HW-01>> --
constexpr uint8_t MOTOR_MIN_PWM_L = 0; // TBD HW-01: minimum PWM that starts the wheel
constexpr uint8_t MOTOR_MIN_PWM_R = 0; // TBD HW-01

// ---- Ultrasonic sensing --------------------------------- <<TBD HW-01>> --
constexpr uint16_t US_MIN_RANGE_MM = 0; // TBD HW-01: closest reliable reading
constexpr uint16_t US_MAX_RANGE_MM = 0; // TBD HW-01

// ---- Motion PID gains ----------------------------------- <<TBD CAL-01>> --
// Left at zero on purpose: an untuned PID that is "on" can drive the robot
// into a wall just as easily as one that is "off". CAL-01 fills these in
// from the calibration campaign, not from guessing.
//
// These six constants presume the controller will be a classic PID split
// into distance/turn axes - a design choice, not just a measurement, made
// here ahead of SW-02. That's a deliberate exception to "don't design ahead
// of need": PID-for-a-differential-drive-robot is close to the only
// reasonable choice for this hardware, not a speculative one, so naming the
// slots now costs nothing and keeps RobotConfig.h's "one source of truth"
// rule intact once SW-02 needs them.
constexpr float KP_DISTANCE = 0.0f;
constexpr float KI_DISTANCE = 0.0f;
constexpr float KD_DISTANCE = 0.0f;
constexpr float KP_TURN = 0.0f;
constexpr float KI_TURN = 0.0f;
constexpr float KD_TURN = 0.0f;
