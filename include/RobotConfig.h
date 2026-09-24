// ============================================================================
//  RobotConfig.h  --  single source of truth for pin assignments.
//  Non-pin constants live in RobotSpec.h.
//
//  Rules:
//    1. Every pin number lives HERE.
//    2. Use `constexpr`, never `#define`.
//    3. Put the unit in the name: _MM, _MS, _US, _DEG, _MV, _PWM.
//  `<<TBD HARDWARE>>` = not yet measured. See scripts/list-tbds.sh.
// ============================================================================

#pragma once

#include <Arduino.h> // for the A0..A9 pin names
#include <stdint.h>

// ============================================================================
//  SECTION 1 -- PIN ASSIGNMENTS  (Arduino Mega 2560)
//
//  Reserved pins:
//    0, 1     USB serial (Serial0)
//    20, 21   I2C SDA/SCL (MPU-6050)
//    50-53    Hardware SPI (SD card)
//
//  6 external-interrupt pins total: 4 used by encoders, 2 by I2C. None left
//  for attachInterrupt() elsewhere.
//
//  [FIXED]    locked in; do not change
//  [PROPOSED] must be confirmed against real wiring
// ============================================================================

// ---- Encoders -------------------------------------------------- [FIXED] --
// Channel B reserved for future 4x quadrature decoding.
constexpr uint8_t PIN_ENC_L_A = 2;  // INT4
constexpr uint8_t PIN_ENC_L_B = 3;  // INT5
constexpr uint8_t PIN_ENC_R_A = 18; // INT3
constexpr uint8_t PIN_ENC_R_B = 19; // INT2

// ---- I2C bus (MPU-6050 gyro) ----------------------------------- [FIXED] --
// Owned by the Wire library - do not reference directly.
constexpr uint8_t PIN_I2C_SDA = 20; // INT1
constexpr uint8_t PIN_I2C_SCL = 21; // INT0

// ---- Motor driver ------------------------------------------- [PROPOSED] --
// Two-pin-direction driver (L298N/TB6612 style).
// PWM 44/45/46 share Timer5. Do not use pins 4/13 (Timer0 - used by millis()).
constexpr uint8_t PIN_MOTOR_L_PWM = 44; // OC5C
constexpr uint8_t PIN_MOTOR_L_IN1 = 22;
constexpr uint8_t PIN_MOTOR_L_IN2 = 23;
constexpr uint8_t PIN_MOTOR_R_PWM = 45; // OC5B
constexpr uint8_t PIN_MOTOR_R_IN1 = 24;
constexpr uint8_t PIN_MOTOR_R_IN2 = 25;

// ---- Ultrasonic wall sensors -------------------------------- [PROPOSED] --
// HC-SR04-style: front, left, right. Fire one at a time (avoid cross-talk).
constexpr uint8_t PIN_US_FRONT_TRIG = 30;
constexpr uint8_t PIN_US_FRONT_ECHO = 31;
constexpr uint8_t PIN_US_LEFT_TRIG = 32;
constexpr uint8_t PIN_US_LEFT_ECHO = 33;
constexpr uint8_t PIN_US_RIGHT_TRIG = 34;
constexpr uint8_t PIN_US_RIGHT_ECHO = 35;

// ---- 8-element IR array ------------------------------------- [PROPOSED] --
// Analog array (QTR-8A style), left to right.
constexpr uint8_t PIN_IR[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
constexpr uint8_t PIN_IR_EMITTER = 36; // HIGH = emitters on

// ---- Operator controls -------------------------------------- [PROPOSED] --
// Physical switches - no reprogramming allowed between trials.
constexpr uint8_t PIN_BTN_START = 38;            // PULLUP: pressed = LOW
constexpr uint8_t PIN_DIP[4] = {40, 41, 42, 43}; // PULLUP: on = LOW
constexpr uint8_t PIN_POT = A8;                  // read once at boot

// ---- Indicators --------------------------------------------- [PROPOSED] --
constexpr uint8_t PIN_LED_STATUS = 26; // heartbeat / current phase
constexpr uint8_t PIN_LED_ERROR = 27;  // something is wrong
constexpr uint8_t PIN_BUZZER = 8;      // tone() uses Timer2

// ---- Battery monitor ---------------------------------------- [PROPOSED] --
constexpr uint8_t PIN_VBAT_SENSE = A9; // resistor divider

// ---- SD card (SPI) --------------------------------------------- [FIXED] --
// Bench logging only; unplugged for official runs.
// PIN_SD_CS must stay OUTPUT even when unused, or SPI drops into slave mode.
constexpr uint8_t PIN_SD_MISO = 50;
constexpr uint8_t PIN_SD_MOSI = 51;
constexpr uint8_t PIN_SD_SCK = 52;
constexpr uint8_t PIN_SD_CS = 53;
