// ============================================================================
//  heading_estimator.cpp  --  see heading_estimator.h.
// ============================================================================

#include "app/heading_estimator.h"

#include <Arduino.h>

#include "hal/imu.h"
#include "motion/Heading.h"
#include "spec/fixed/Hardware.h"
#include "spec/measurements/Robot.h"
#include "spec/tuning/Imu.h"

namespace
{

// The IMU is the one driver that can be absent (an unplugged I2C device answers nothing), so the
// loop has to cope with it never coming up.
bool g_imuPresent = false;
GyroBias g_gyroBias;
Heading g_heading;

// Average the yaw rate with the robot held still: an uncorrected gyro bias integrates into heading
// without bound (decisions/0008). Blocking, and only at boot - the control loop has not started
// yet. ~1.4 s per attempt: the chip updates at 1 kHz, so 1 ms between reads keeps the samples
// independent. A capture whose readings scatter (the robot was being moved) is rejected by
// gyroBiasFinish() and repeated, up to GYRO_BIAS_MAX_ATTEMPTS times: a wrong zero is silent, a
// missing one is obvious.
// Returns false, with g_gyroBias left as it was, if every attempt was rejected.
bool calibrateGyroBias()
{
    for (uint8_t attempt = 1; attempt <= GYRO_BIAS_MAX_ATTEMPTS; ++attempt) {
        GyroBias capture;
        gyroBiasReset(capture);
        for (uint16_t i = 0; i < GYRO_BIAS_SAMPLES; ++i) {
            ImuRaw sample;
            if (imuRead(sample)) {
                gyroBiasAccumulate(capture, sample.gyro[IMU_GYRO_AXIS_YAW]);
            }
            delay(1);
        }
        if (gyroBiasFinish(capture)) {
            g_gyroBias = capture;
            Serial.print(F("imu: gyro bias "));
            Serial.print(g_gyroBias.counts);
            Serial.print(F(" counts, spread "));
            Serial.println(g_gyroBias.spread);
            return true;
        }
        if (capture.samples < GYRO_BIAS_MIN_SAMPLES) {
            Serial.print(F("imu: gyro bias capture got too few readings (attempt "));
            Serial.print(attempt);
            Serial.println(F(")"));
        } else {
            Serial.print(F("imu: gyro bias capture rejected, spread "));
            Serial.print(capture.spread);
            Serial.print(F(" counts (attempt "));
            Serial.print(attempt);
            Serial.println(F(") - hold the robot still"));
        }
    }
    return false;
}

} // namespace

bool headingEstimatorInit()
{
    g_imuPresent = imuInit();
    if (g_imuPresent) {
        // Hold the robot still through this.
        if (!calibrateGyroBias()) {
            Serial.println(F("imu: gyro bias calibration failed - heading stays at 0"));
        }
    } else {
        Serial.println(F("imu: MPU-6050 did not answer - no heading"));
    }
    return g_imuPresent;
}

void headingEstimatorRead(RobotState &state)
{
    // A failed transfer leaves state.imu and state.yawRateDps holding the previous sample, and
    // skips the integration rather than integrating a stale rate twice. A dropped tick loses
    // whatever rotation happened during it; nothing counts it yet.
    if (g_imuPresent && imuRead(state.imu)) {
        state.yawRateDps = headingUpdate(g_heading, g_gyroBias, state.imu.gyro[IMU_GYRO_AXIS_YAW],
                                         CONTROL_LOOP_PERIOD_MS);
    }
    state.headingDeg = g_heading.deg;
}

void headingEstimatorApplyFix(float fixDeg)
{
    headingSet(g_heading, g_heading.deg - fixDeg);
}

bool headingEstimatorRezero()
{
    if (!g_imuPresent || !calibrateGyroBias()) {
        Serial.println(F("imu: gyro bias calibration failed - old bias and heading kept"));
        return false;
    }
    headingSet(g_heading, 0.0f);
    return true;
}
