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

#include <Arduino.h> // for the A0..A15 pin names
#include <stdint.h>

// ============================================================================
//  SECTION 1 -- PIN ASSIGNMENTS  (Arduino Mega 2560)
//
//  Reserved pins:
//    0, 1     USB serial (Serial0)
//    20, 21   I2C SDA/SCL (MPU-6050)
//
//  Interrupts: INT2-INT5 (D19, D18, D2, D3) are the encoders; INT0/INT1 are
//  the I2C pins. Ultrasonic echoes use the PCINT2 bank (PORTK = A8-A15).
//  Serial1 (D18/D19) is therefore unavailable.
//
//  Timers: Timer0 Arduino millis(); Timer1 right motor PWM; Timer2 unused;
//  Timer3 reserved (ultrasonic timestamp); Timer4 left motor PWM; Timer5
//  spare. See decisions/0002.
//
//  [FIXED]    locked in; do not change
//  [PROPOSED] must be confirmed against real wiring
// ============================================================================

// ---- Encoders ----------------------------------------------- [PROPOSED] --
// JGA25-370 Hall quadrature, 4x decoded on all four edges. Each motor's A/B sit
// on one AVR port with A on the LOWER bit, so one PINx read yields (B<<1)|A.
// "+" = A leads B; flip per side with ENC_*_INVERT in RobotSpec.h.
constexpr uint8_t PIN_ENC_L_A = 2;  // PE4, INT4
constexpr uint8_t PIN_ENC_L_B = 3;  // PE5, INT5
constexpr uint8_t PIN_ENC_R_A = 19; // PD2, INT2
constexpr uint8_t PIN_ENC_R_B = 18; // PD3, INT3

// ---- I2C bus (MPU-6050 gyro) ----------------------------------- [FIXED] --
// Owned by the Wire library - do not reference directly.
constexpr uint8_t PIN_I2C_SDA = 20; // INT1
constexpr uint8_t PIN_I2C_SCL = 21; // INT0

// ---- Motor drivers ------------------------------------------ [PROPOSED] --
// Two HW-039 / IBT_2 (BTS7960) boards. Per motor: RPWM = forward PWM, LPWM =
// reverse PWM, and R_EN + L_EN tied together on one GPIO (EN low = coast).
// Left on Timer4 (OC4A/OC4B), right on Timer1 (OC1A/OC1B).
constexpr uint8_t PIN_MOTOR_L_RPWM = 6;  // PH3, OC4A
constexpr uint8_t PIN_MOTOR_L_LPWM = 7;  // PH4, OC4B
constexpr uint8_t PIN_MOTOR_L_EN = 8;    // PH5
constexpr uint8_t PIN_MOTOR_R_EN = 10;   // PB4
constexpr uint8_t PIN_MOTOR_R_RPWM = 11; // PB5, OC1A
constexpr uint8_t PIN_MOTOR_R_LPWM = 12; // PB6, OC1B

// ---- Ultrasonic wall sensors -------------------------------- [PROPOSED] --
// HC-SR04-style: front, left, right. Fire one at a time (avoid cross-talk).
// ECHO pins are on PORTK (PCINT21-23) so edges can be timestamped by interrupt.
constexpr uint8_t PIN_US_FRONT_TRIG = 30;
constexpr uint8_t PIN_US_LEFT_TRIG = 32;
constexpr uint8_t PIN_US_RIGHT_TRIG = 34;
constexpr uint8_t PIN_US_FRONT_ECHO = A13; // PK5
constexpr uint8_t PIN_US_LEFT_ECHO = A14;  // PK6
constexpr uint8_t PIN_US_RIGHT_ECHO = A15; // PK7

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
