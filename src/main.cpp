// ============================================================================
//  main.cpp  --  fixed-period control loop scheduler.
//  Ticks at CONTROL_LOOP_PERIOD_MS: readSensors() then runAlgorithm().
//  Hardware is reached only through src/hal/* drivers - see RobotState.h.
//  The algorithm is the Section A run (include/RunLoop.h): after the gyro is zeroed at power-up
//  the robot explores from where it stands until the approach marker, or every reachable tile
//  without a line calibration.
// ============================================================================

#include <Arduino.h>

#include "Heading.h"
#include "RobotConfig.h"
#include "RobotSpec.h"
#include "RobotState.h"
#include "RunLoop.h"
#include "Scheduler.h"
#include "hal/encoders.h"
#include "hal/imu.h"
#include "hal/line_sensors.h"
#include "hal/motors.h"
#include "hal/ultrasonic.h"
#include "line_cal_store.h"

static RobotState g_state;
static RunLoop g_run;
static bool g_motorsEnabled = false;

// The IMU is the one driver that can be absent (an unplugged I2C device answers
// nothing), so the loop has to cope with it never coming up.
static bool g_imuPresent = false;
static GyroBias g_gyroBias;
static Heading g_heading;

// Average the yaw rate with the robot held still: an uncorrected gyro bias
// integrates into heading without bound (decisions/0008). Blocking, and only at
// boot - the control loop has not started yet. ~1.4 s per attempt: the chip
// updates at 1 kHz, so 1 ms between reads keeps the samples independent.
// A capture whose readings scatter (the robot was being moved) is rejected by
// gyroBiasFinish() and repeated, up to GYRO_BIAS_MAX_ATTEMPTS times: a wrong zero
// is silent, a missing one is obvious.
static void calibrateGyroBias()
{
    for (uint8_t attempt = 1; attempt <= GYRO_BIAS_MAX_ATTEMPTS; ++attempt) {
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
            Serial.print(F(" counts, spread "));
            Serial.println(g_gyroBias.spread);
            return;
        }
        if (g_gyroBias.samples < GYRO_BIAS_MIN_SAMPLES) {
            Serial.print(F("imu: gyro bias capture got too few readings (attempt "));
            Serial.print(attempt);
            Serial.println(F(")"));
        } else {
            Serial.print(F("imu: gyro bias capture rejected, spread "));
            Serial.print(g_gyroBias.spread);
            Serial.print(F(" counts (attempt "));
            Serial.print(attempt);
            Serial.println(F(") - hold the robot still"));
        }
    }
    Serial.println(F("imu: gyro bias calibration failed - heading stays at 0"));
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

    if (RUN_START_BUTTON_FITTED) {
        pinMode(PIN_BTN_START, INPUT_PULLUP);
    }
    pinMode(PIN_LED_ERROR, OUTPUT);
    // The IR calibration is taken once per lighting with the probe and kept in EEPROM
    // (line_cal_store.h). Without one that looks real the run reads walls only: it explores every
    // reachable tile and stops, and cannot see the marker.
    LineCalibration cal;
    const bool haveCal = lineCalLoad(cal) && lineCalibrationLooksReal(cal);
    runLoopInit(g_run, g_imuPresent, haveCal ? &cal : nullptr);
    if (!g_imuPresent) {
        Serial.println(F("run: no gyro, will not start"));
    } else {
        Serial.println(haveCal ? F("run: starts by itself, line calibration loaded")
                               : F("run: starts by itself (walls only: no line calibration)"));
    }
    Serial.println(F("maze-solver: fixed-period loop starting"));
}

// Drivers populate g_state from hardware.
static void readSensors(RobotState &state)
{
    encodersRead(state.encoderCountL, state.encoderCountR);
    lineSensorsRead(state.irRaw);

    // A failed transfer leaves state.imu and state.yawRateDps holding the previous sample, and
    // skips the integration rather than integrating a stale rate twice. A dropped tick loses
    // whatever rotation happened during it; nothing counts it yet.
    if (g_imuPresent && imuRead(state.imu)) {
        state.yawRateDps = headingUpdate(g_heading, g_gyroBias, state.imu.gyro[IMU_GYRO_AXIS_YAW],
                                         CONTROL_LOOP_PERIOD_MS);
    }
    state.headingDeg = g_heading.deg;
    ultrasonicUpdate();
    state.distFrontMm = ultrasonicDistanceMm(Ultrasonic::Front);
    state.distLeftMm = ultrasonicDistanceMm(Ultrasonic::Left);
    state.distRightMm = ultrasonicDistanceMm(Ultrasonic::Right);
    state.usValid = ultrasonicValidMask();
}

// The Section A run: perception, mission, then the motors.
static void runAlgorithm(const RobotState &state)
{
    const bool button = RUN_START_BUTTON_FITTED && digitalRead(PIN_BTN_START) == LOW;
    float headingFixDeg = 0.0f;
    const DriveCommand cmd = runLoopTick(g_run, state, state.timestampMs, button, headingFixDeg);

    // The corridor's measured direction, handed back to the gyro (StepCell.h).
    if (headingFixDeg != 0.0f) {
        headingSet(g_heading, g_heading.deg - headingFixDeg);
    }

    if (cmd.enable != g_motorsEnabled) {
        motorsEnable(cmd.enable);
        g_motorsEnabled = cmd.enable;
    }
    if (cmd.enable) {
        motorSet(Motor::Left, cmd.left);
        motorSet(Motor::Right, cmd.right);
    }

    static SectionAPhase loggedPhase = SectionAPhase::WaitStart;
    const SectionAPhase phase = g_run.mission.phase;
    if (phase != loggedPhase) {
        loggedPhase = phase;
        Serial.print(F("run: phase "));
        Serial.print(static_cast<uint8_t>(phase));
        Serial.print(F(" fault "));
        Serial.print(static_cast<uint8_t>(g_run.mission.fault));
        Serial.print(F(" moves "));
        Serial.print(g_run.mission.moves);
        Serial.print(F(" turns "));
        Serial.println(g_run.mission.turns);
        digitalWrite(PIN_LED_ERROR, phase == SectionAPhase::Fault);
    }
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
