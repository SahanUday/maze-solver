// ============================================================================
//  main.cpp  --  fixed-period control loop scheduler.
//  Ticks at CONTROL_LOOP_PERIOD_MS: readSensors() then runAlgorithm().
//  Hardware is reached only through src/hal/* drivers - see RobotState.h.
// ============================================================================

#include <Arduino.h>

#include "Heading.h"
#include "RobotConfig.h"
#include "RobotSpec.h"
#include "RobotState.h"
#include "Scheduler.h"
#include "hal/encoders.h"
#include "hal/imu.h"
#include "hal/line_sensors.h"
#include "hal/motors.h"
#include "hal/ultrasonic.h"

static RobotState g_state;

// The IMU is the one driver that can be absent (an unplugged I2C device answers
// nothing), so the loop has to cope with it never coming up.
static bool g_imuPresent = false;
static GyroBias g_gyroBias;
static Heading g_heading;

// Average the yaw rate with the robot held still: an uncorrected gyro bias
// integrates into heading without bound (decisions/0007). Blocking, and only at
// boot - the control loop has not started yet. ~1.4 s: the chip updates at 1 kHz,
// so 1 ms between reads keeps the samples independent.
static void calibrateGyroBias()
{
    gyroBiasReset(g_gyroBias);
    for (uint16_t i = 0; i < GYRO_BIAS_SAMPLES; ++i) {
        ImuRaw sample;
        if (imuRead(sample)) {
            gyroBiasAccumulate(g_gyroBias, sample.gyro[IMU_GYRO_AXIS_YAW]);
        }
        delay(1);
    }
    if (gyroBiasFinish(g_gyroBias)) {
        Serial.print(F("imu: gyro bias "));
        Serial.print(g_gyroBias.counts);
        Serial.println(F(" counts"));
    } else {
        Serial.println(F("imu: gyro bias calibration failed - heading stays at 0"));
    }
}

void setup()
{
    Serial.begin(DEBUG_SERIAL_BAUD);
    pinMode(PIN_LED_STATUS, OUTPUT);
    motorsInit(); // EN stays low (coast) until the control layer enables it
    encodersInit();
    lineSensorsInit();

    g_imuPresent = imuInit();
    if (g_imuPresent) {
        calibrateGyroBias(); // hold the robot still through this
    } else {
        Serial.println(F("imu: MPU-6050 did not answer - no heading"));
    }

    ultrasonicInit();
    Serial.println(F("maze-solver: fixed-period loop starting"));
}

// Drivers populate g_state from hardware.
static void readSensors(RobotState &state)
{
    encodersRead(state.encoderCountL, state.encoderCountR);
    lineSensorsRead(state.irRaw);

    // A failed transfer leaves state.imu holding the previous sample, and skips the
    // integration rather than integrating a stale rate twice. A dropped tick loses
    // whatever rotation happened during it; nothing counts it yet.
    if (g_imuPresent && imuRead(state.imu)) {
        headingUpdate(g_heading, g_gyroBias, state.imu.gyro[IMU_GYRO_AXIS_YAW],
                      CONTROL_LOOP_PERIOD_MS);
    }
    state.headingDeg = g_heading.deg;
    ultrasonicUpdate();
    state.distFrontMm = ultrasonicDistanceMm(Ultrasonic::Front);
    state.distLeftMm = ultrasonicDistanceMm(Ultrasonic::Left);
    state.distRightMm = ultrasonicDistanceMm(Ultrasonic::Right);
    state.usValid = ultrasonicValidMask();
}

// Algorithms read g_state and decide what to do next. Empty until real
// algorithms exist.
static void runAlgorithm(const RobotState &state)
{
    (void)state;
}

void loop()
{
    static uint32_t nextTickMs = 0;
    const uint32_t nowMs = millis();

    if (!tickDue(nowMs, nextTickMs, CONTROL_LOOP_PERIOD_MS)) {
        return;
    }

    g_state.timestampMs = nowMs;
    readSensors(g_state);
    runAlgorithm(g_state);

    // Heartbeat LED - proves the loop is alive.
    static uint32_t lastToggleMs = 0;
    static bool ledOn = false;
    if (nowMs - lastToggleMs >= 500) {
        lastToggleMs = nowMs;
        ledOn = !ledOn;
        digitalWrite(PIN_LED_STATUS, ledOn);
    }
}
