// ============================================================================
//  imu.h  --  MPU-6050 6-axis IMU over I2C (Wire.h, not registers: ADR 0001).
//  docs/architecture/modules/imu.md
// ============================================================================

#pragma once

#include "RobotState.h" // ImuRaw

// Starts the bus, checks WHO_AM_I and configures the chip. Returns false if the
// sensor does not answer or answers wrongly; nothing else here is usable then.
// Blocks for IMU_STARTUP_DELAY_MS. Call once.
bool imuInit();

// One burst read of the 14 data registers into raw counts. Returns false on a
// failed transfer, leaving `out` untouched rather than half-written.
// Blocks for ~0.4 ms at IMU_I2C_CLOCK_HZ. Not callable from an ISR.
bool imuRead(ImuRaw &out);
