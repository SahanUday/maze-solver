// ============================================================================
//  heading_estimator.h  --  the gyro heading for the whole run: the IMU, the bias captured at
//  boot, and the yaw integration that fills RobotState each tick. Hardware glue (Arduino
//  framework, blocking boot capture); the arithmetic is motion/Heading.h.
//  docs/architecture/modules/heading.md
// ============================================================================

#pragma once

#include "core/RobotState.h"

// Starts the IMU and captures the gyro bias with the robot held still (blocking, ~1.4 s per
// attempt). False if the IMU did not answer: the heading then stays at 0.
bool headingEstimatorInit();

// One tick: reads the IMU into state.imu, integrates the yaw and sets state.yawRateDps and
// state.headingDeg. A failed transfer keeps the previous sample and skips the integration.
void headingEstimatorRead(RobotState &state);

// Subtract the corridor heading the mission measured from the heading (StepCell.h).
void headingEstimatorApplyFix(float fixDeg);

// Capture the gyro bias again and call the current direction 0, with the robot held still
// (blocking, as at boot). For the bench firmware; the mission zeroes once, at boot. False if the
// IMU is absent or every capture was rejected: the old bias and heading are then kept.
bool headingEstimatorRezero();
