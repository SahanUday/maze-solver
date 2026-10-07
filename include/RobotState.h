// ============================================================================
//  RobotState.h  --  boundary between drivers and algorithms.
//
//  Drivers write this struct once per tick. Algorithm code only reads it -
//  never a pin, analogRead(), digitalWrite(), or millis() directly.
// ============================================================================

#pragma once

#include <stdint.h>

// One burst read of the MPU-6050's data registers, in the sensor's own axes and
// its own counts. Which axis is "forward" and which sign is nose-up depend on
// how the board is bolted down; the driver does not interpret either.
struct ImuRaw {
    int16_t accel[3] = {0, 0, 0}; // X, Y, Z
    int16_t temp = 0;             // chip temperature, for bias-drift diagnostics
    int16_t gyro[3] = {0, 0, 0};  // X, Y, Z; index IMU_GYRO_AXIS_YAW is the yaw rate
};

struct RobotState {
    // ---- Timing ------------------------------------------------------
    uint32_t timestampMs = 0; // millis() at the start of this tick

    // ---- Odometry (from the encoder driver) ---------------------------
    int32_t encoderCountL = 0; // signed 4x counts since boot, + = wheel-forward
    int32_t encoderCountR = 0;

    // ---- Orientation (from the gyro driver) ---------------------------
    float headingDeg = 0.0f; // fused/integrated yaw, 0 = start heading

    // ---- Inertial sensing (from the IMU driver) -----------------------
    // A straight mirror of the MPU-6050's 0x3B..0x48 block, raw counts, no
    // angles (see decisions/0007). Scales: ACCEL_COUNTS_PER_G per g,
    // GYRO_COUNTS_PER_DPS per deg/s.
    ImuRaw imu;

    // ---- Wall sensing (from the ultrasonic driver) --------------------
    uint16_t distFrontMm = 0;
    uint16_t distLeftMm = 0;
    uint16_t distRightMm = 0;

    // ---- Floor sensing (from the IR array driver) ---------------------
    // Raw 10-bit ADC counts, index 0 = A0 = D1. Lower = more reflected light (white);
    // ~1020 = nothing in range (black, too far, or emitters off).
    uint16_t irRaw[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    // ---- Operator controls (read once at boot) ------------------------
    bool startButtonPressed = false;
    uint8_t dipSwitches = 0; // one bit per switch in PIN_DIP
    uint8_t speedPotRaw = 0; // 0-255, scaled from the raw ADC read
};
