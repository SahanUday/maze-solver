// ============================================================================
//  RobotConfig.h  --  the single source of truth for pins and constants.
//
//  RULES FOR THIS FILE (SW-01):
//    1. Every pin number and every tunable number in the firmware lives HERE.
//       If you find a numeric literal anywhere else in the codebase, it is a
//       bug: move it here and give it a name.
//    2. Use `constexpr`, never `#define`. constexpr values are typed, obey
//       scope, and are visible to the debugger. Macros are none of those.
//    3. Put the unit in the name: _MM, _MS, _US, _DEG, _MV, _PWM.
//       "TRACK_WIDTH = 92" is a bug waiting to happen. "TRACK_WIDTH_MM" is not.
//    4. Values marked  <<TBD HW-01>>  are placeholders, filled in from
//       docs/HARDWARE_TRUTH_SHEET.md once the robot is measured.
//       Grep for "TBD" to see what is still unknown.
// ============================================================================

#pragma once

#include <Arduino.h>   // for the A0..A9 pin names
#include <stdint.h>

// ============================================================================
//  SECTION 1 -- PIN ASSIGNMENTS
//
//  Board: Arduino Mega 2560.
//
//  Pins that are NOT ours to allocate:
//    0, 1     USB serial (Serial0). Never reuse - uploading fails.
//    20, 21   I2C SDA/SCL, hardwired for the MPU-6050.
//    50-53    Hardware SPI for the SD card.
//
//  The Mega has exactly SIX external-interrupt pins:
//    2 (INT4), 3 (INT5), 18 (INT3), 19 (INT2), 20 (INT1), 21 (INT0)
//  Four go to the encoders and two are consumed by I2C. There are none left,
//  which is why nothing else in this project may call attachInterrupt().
//
//  Status of each block:
//    [FIXED]    committed in the HW-01 pin plan; do not change
//    [PROPOSED] chosen here; must be confirmed against the real wiring and
//               recorded in docs/HARDWARE_TRUTH_SHEET.md
// ============================================================================

// ---- Encoders -------------------------------------------------- [FIXED] --
// Channel A must sit on an interrupt pin: it is what fires the ISR.
// Channel B is only read inside that ISR to decide direction, so it does not
// strictly need an interrupt pin - but we spend one on it anyway, because that
// leaves the door open to full 4x quadrature decoding later without rewiring.
constexpr uint8_t PIN_ENC_L_A = 2;    // INT4
constexpr uint8_t PIN_ENC_L_B = 3;    // INT5
constexpr uint8_t PIN_ENC_R_A = 18;   // INT3
constexpr uint8_t PIN_ENC_R_B = 19;   // INT2

// ---- I2C bus (MPU-6050 gyro) ----------------------------------- [FIXED] --
// Listed for documentation only. The Wire library owns these pins; you never
// reference them directly.
constexpr uint8_t PIN_I2C_SDA = 20;   // INT1 - consumed by I2C
constexpr uint8_t PIN_I2C_SCL = 21;   // INT0 - consumed by I2C

// ---- Motor driver ------------------------------------------- [PROPOSED] --
// Assumes a two-pin-direction driver (L298N / TB6612 style): one PWM speed pin
// plus two direction pins per motor. If the issued driver uses a single DIR
// pin instead, drop the _IN2 constants and update the driver code.
//
// PWM pins 44/45/46 are all on Timer5, so both motors share one timer and one
// PWM frequency - which is what you want, or the two wheels respond
// differently to the same duty cycle. Do NOT move motor PWM to pins 4 or 13:
// those are Timer0, and Timer0 is what millis() counts on.
constexpr uint8_t PIN_MOTOR_L_PWM = 44;   // OC5C
constexpr uint8_t PIN_MOTOR_L_IN1 = 22;
constexpr uint8_t PIN_MOTOR_L_IN2 = 23;
constexpr uint8_t PIN_MOTOR_R_PWM = 45;   // OC5B
constexpr uint8_t PIN_MOTOR_R_IN1 = 24;
constexpr uint8_t PIN_MOTOR_R_IN2 = 25;

// ---- Ultrasonic wall sensors -------------------------------- [PROPOSED] --
// Three HC-SR04-style sensors: front, left, right.
// Fired one at a time, never together - a neighbour's burst arriving at this
// sensor's receiver reads as a wall that is not there (cross-talk).
constexpr uint8_t PIN_US_FRONT_TRIG = 30;
constexpr uint8_t PIN_US_FRONT_ECHO = 31;
constexpr uint8_t PIN_US_LEFT_TRIG  = 32;
constexpr uint8_t PIN_US_LEFT_ECHO  = 33;
constexpr uint8_t PIN_US_RIGHT_TRIG = 34;
constexpr uint8_t PIN_US_RIGHT_ECHO = 35;

// ---- 8-element IR array ------------------------------------- [PROPOSED] --
// Wired as an ANALOG array (QTR-8A style): eight analog outputs plus one
// emitter-enable line. If the issued array is the RC/digital type instead,
// these become digital pins and the driver changes - but the pin count and the
// left-to-right ordering stay the same.
// A0..A7 are digital numbers 54..61 on the Mega; the Ax names are clearer.
constexpr uint8_t PIN_IR[8]      = { A0, A1, A2, A3, A4, A5, A6, A7 };
constexpr uint8_t PIN_IR_EMITTER = 36;    // drive HIGH to turn the LEDs on

// ---- Operator controls -------------------------------------- [PROPOSED] --
// The rules forbid reprogramming the robot between trials, so every run-time
// choice (which section, which start heading, which speed profile) has to be
// selectable by hand. That is what these are for.
constexpr uint8_t PIN_BTN_START = 38;                  // PULLUP: pressed = LOW
constexpr uint8_t PIN_DIP[4]    = { 40, 41, 42, 43 };  // PULLUP: on = LOW
constexpr uint8_t PIN_POT       = A8;                  // read once at boot

// ---- Indicators --------------------------------------------- [PROPOSED] --
// You cannot attach a laptop during a run, so the LEDs and buzzer are the only
// way the robot tells you what it thinks is happening.
constexpr uint8_t PIN_LED_STATUS = 26;    // heartbeat / current phase
constexpr uint8_t PIN_LED_ERROR  = 27;    // something is wrong
constexpr uint8_t PIN_BUZZER     = 8;     // tone() uses Timer2 - no conflict

// ---- Battery monitor ---------------------------------------- [PROPOSED] --
// A resistor divider from the pack down to a safe analog input. Motors that
// slow as the pack sags turn less than they did at full charge; you want to
// see that coming rather than discover it mid-run.
constexpr uint8_t PIN_VBAT_SENSE = A9;

// ---- SD card (SPI) --------------------------------------------- [FIXED] --
// Bench logging only. The module is physically unplugged for official runs.
// PIN_SD_CS must be configured as OUTPUT even when unused: on AVR, leaving the
// hardware SS pin as an input lets the SPI peripheral drop into slave mode and
// the bus stops working, with no error message.
constexpr uint8_t PIN_SD_MISO = 50;
constexpr uint8_t PIN_SD_MOSI = 51;
constexpr uint8_t PIN_SD_SCK  = 52;
constexpr uint8_t PIN_SD_CS   = 53;

// ============================================================================
//  SECTION 2 -- GEOMETRY, MOTION, CONTROL, MAZE
//  Added in SW-01 step 4.
// ============================================================================
