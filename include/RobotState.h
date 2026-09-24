// ============================================================================
//  RobotState.h  --  boundary between drivers and algorithms.
//
//  Drivers write this struct once per tick. Algorithm code only reads it -
//  never a pin, analogRead(), digitalWrite(), or millis() directly.
// ============================================================================

#pragma once

#include <stdint.h>

struct RobotState {
    // ---- Timing ------------------------------------------------------
    uint32_t timestampMs = 0; // millis() at the start of this tick

    // ---- Odometry (from the encoder driver) ---------------------------
    int32_t encoderCountL = 0;
    int32_t encoderCountR = 0;

    // ---- Orientation (from the gyro driver) ---------------------------
    float headingDeg = 0.0f; // fused/integrated yaw, 0 = start heading

    // ---- Wall sensing (from the ultrasonic driver) --------------------
    uint16_t distFrontMm = 0;
    uint16_t distLeftMm = 0;
    uint16_t distRightMm = 0;

    // ---- Floor sensing (from the IR array driver) ---------------------
    uint16_t irRaw[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    // ---- Power (from the battery-monitor driver) ----------------------
    uint16_t batteryMv = 0;

    // ---- Operator controls (read once at boot) ------------------------
    bool startButtonPressed = false;
    uint8_t dipSwitches = 0; // one bit per switch in PIN_DIP
    uint8_t speedPotRaw = 0; // 0-255, scaled from the raw ADC read
};
