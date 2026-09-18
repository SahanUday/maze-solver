// ============================================================================
//  RobotState.h  --  the boundary between drivers and algorithms (SW-01).
//
//  RULE: algorithm code (path-finding, exploration, mode logic) never calls
//  analogRead(), digitalWrite() or millis() directly, and never touches a
//  sensor library. It only reads a RobotState. Drivers are the only code
//  allowed to write one, once per control-loop tick.
//
//  Why: this is what lets lib/maze compile and run on a laptop with no
//  robot attached (SIM-01) - the algorithm's input is a plain struct, not a
//  pin. It also means a bug is either "the driver filled RobotState wrong"
//  or "the algorithm read RobotState wrong", never both at once.
// ============================================================================

#pragma once

#include <stdint.h>

struct RobotState {
    // ---- Timing ------------------------------------------------------
    uint32_t timestampMs = 0;   // millis() at the start of this tick

    // ---- Odometry (from encoder driver, SW-02) ------------------------
    int32_t encoderCountL = 0;
    int32_t encoderCountR = 0;

    // ---- Orientation (from gyro driver, SW-02) ------------------------
    float headingDeg = 0.0f;   // fused/integrated yaw, 0 = start heading

    // ---- Wall sensing (from ultrasonic driver, SW-03) ------------------
    uint16_t distFrontMm = 0;
    uint16_t distLeftMm  = 0;
    uint16_t distRightMm = 0;

    // ---- Floor sensing (from IR array driver, SW-03c) ------------------
    uint16_t irRaw[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    // ---- Power (from battery-monitor driver, HW-03) --------------------
    uint16_t batteryMv = 0;

    // ---- Operator controls (read once at boot, HW-03) ------------------
    bool    startButtonPressed = false;
    uint8_t dipSwitches        = 0;   // one bit per switch in PIN_DIP
    uint8_t speedPotRaw        = 0;   // 0-255, scaled from the raw ADC read
};
