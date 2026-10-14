// ============================================================================
//  main.cpp  --  fixed-period control loop scheduler.
//  Ticks at CONTROL_LOOP_PERIOD_MS: readSensors(), runAlgorithm(), then the
//  DriveCommand it returns is applied to the motors.
//  Hardware is reached only through src/hal/* drivers - see RobotState.h.
// ============================================================================

#include <Arduino.h>

#include "DriveCommand.h"
#include "Heading.h"
#include "Mission.h"
#include "NavParams.h"
#include "Perception.h"
#include "RobotConfig.h"
#include "RobotSpec.h"
#include "RobotState.h"
#include "Scheduler.h"
#include "hal/encoders.h"
#include "hal/imu.h"
#include "hal/line_sensors.h"
#include "hal/motors.h"
#include "hal/ultrasonic.h"
#include "line_cal_store.h"
#include "operator_io.h"

static RobotState g_state;

// Algorithm layer: derived perception and the run-phase state machine. Static, not locals - the
// Mission holds the maze map and is far larger than the 128 B stack budget of a function.
static Perceiver g_perceiver;
static Perception g_perception;
static PerceiveParams g_perceiveParams;
static Mission g_mission;
static MissionParams g_missionParams;
static DriveCommand g_lastCmd; // what the last tick asked of the motors, for the status line

// The IMU is the one driver that can be absent (an unplugged I2C device answers
// nothing), so the loop has to cope with it never coming up.
static bool g_imuPresent = false;
static uint32_t g_bootDoneMs = 0; // when setup() finished: the auto-start delay counts from here
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

// Line calibration mode (DIP_LINE_CAL at power-up): slide the robot by hand across white and
// black until START is pressed, then keep the result in EEPROM and stop. Reset to run. The
// lighting at the arena is not the lighting in the lab, and nothing can be re-flashed there.
static void lineCalibrationMode()
{
    Serial.println(F("line cal: slide the array over white and black, then press START"));
    LineCalibration cal;
    lineCalibrationReset(cal);
    while (!operatorStartPressed()) {
        uint16_t counts[IR_CHANNEL_COUNT];
        lineSensorsRead(counts);
        lineCalibrationAccumulate(cal, counts);
        digitalWrite(PIN_LED_STATUS, (millis() / 100) & 1);
        delay(2);
    }
    const uint8_t usable = lineCalibrationFinish(cal);
    if (usable == 0) {
        Serial.println(F("line cal: no channel saw both white and black - nothing saved"));
        digitalWrite(PIN_LED_ERROR, HIGH);
    } else {
        lineCalSave(cal);
        Serial.print(F("line cal: saved, usable channels 0x"));
        Serial.println(usable, HEX);
        digitalWrite(PIN_LED_STATUS, HIGH);
    }
    for (;;) {
    }
}

void setup()
{
    Serial.begin(DEBUG_SERIAL_BAUD);
    operatorInit();
    motorsInit(); // EN stays low (coast) until the control layer enables it
    encodersInit();
    lineSensorsInit();

    // The DIP switches are only read when RUN_USE_DIP_SWITCHES is set; without them there is no
    // line-calibration mode and the hand is RUN_HAND.
    const uint8_t dip = RUN_USE_DIP_SWITCHES ? operatorReadDip() : 0;
    g_state.dipSwitches = dip;
    if ((dip & DIP_LINE_CAL) != 0) {
        lineCalibrationMode(); // does not return
    }

    g_imuPresent = imuInit();
    if (g_imuPresent) {
        calibrateGyroBias(); // hold the robot still through this
    } else {
        Serial.println(F("imu: MPU-6050 did not answer - no heading"));
    }

    ultrasonicInit();

    // The run: the mission refuses to start (motors stay off, error LED on) if a precondition
    // is missing.
    g_perceiveParams = specPerceive();
    const int8_t hand = RUN_USE_DIP_SWITCHES ? ((dip & DIP_HAND_RIGHT) != 0 ? -1 : 1) : RUN_HAND;
    g_missionParams = specMission(hand);
    missionReset(g_mission);
    perceiverReset(g_perceiver);
    LineCalibration cal;
    if (lineCalLoad(cal)) {
        perceiverSetCalibration(g_perceiver, cal);
    } else if (RUN_USE_FLOOR_SENSING) {
        Serial.println(F("line cal: none stored - set DIP 4, calibrate, reset"));
        missionFail(g_mission, FaultCode::NoLineCal);
    } else {
        Serial.println(F("line cal: none stored - not needed, floor sensing is off"));
    }
    if (!g_imuPresent) {
        missionFail(g_mission, FaultCode::NoImu);
    }
    g_bootDoneMs = millis();
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

    // START is polled every tick, the DIPs were read once at power-up. Without the button the run
    // starts by itself RUN_AUTOSTART_MS after boot.
    if (RUN_USE_START_BUTTON) {
        state.startButtonPressed = operatorStartPressed();
    } else {
        state.startButtonPressed = (millis() - g_bootDoneMs) >= RUN_AUTOSTART_MS;
    }
}

// Algorithms read g_state and decide what to do next: derive perception, step the mission.
static DriveCommand runAlgorithm(const RobotState &state)
{
    perceive(g_perceiver, state, g_perceiveParams, g_perception);
    g_lastCmd = missionStep(g_mission, g_missionParams, g_perception);
    return g_lastCmd;
}

// Whatever the controller asks for, no wheel gets more than DRIVE_MAX_DUTY.
static int16_t clampDuty(int16_t d)
{
    return d > DRIVE_MAX_DUTY ? DRIVE_MAX_DUTY : (d < -DRIVE_MAX_DUTY ? -DRIVE_MAX_DUTY : d);
}

// The only place the algorithm layer's answer touches the motors.
static void applyDrive(const DriveCommand &cmd)
{
    static bool enabled = false;
    if (cmd.enable != enabled) {
        motorsEnable(cmd.enable);
        enabled = cmd.enable;
    }
    if (enabled) {
        motorSet(Motor::Left, clampDuty(cmd.left));
        motorSet(Motor::Right, clampDuty(cmd.right));
    }
}

// Any byte on the serial line stops the mission (the motors cut and the phase goes to Fault), and
// phase and cell changes are printed, so a run can be watched and stopped from the PC.
static void serialSupervise()
{
    static Phase lastPhase = Phase::WaitStart;
    static int8_t lastCx = -99;
    static int8_t lastCy = -99;

    if (Serial.available() > 0) {
        while (Serial.available() > 0) {
            Serial.read();
        }
        if (g_mission.phase != Phase::Fault) {
            missionFail(g_mission, FaultCode::Aborted);
        }
    }
    if (g_mission.phase != lastPhase) {
        lastPhase = g_mission.phase;
        Serial.print(F("mission: phase "));
        Serial.print(static_cast<uint8_t>(lastPhase));
        Serial.print(F(" fault "));
        Serial.println(static_cast<uint8_t>(g_mission.fault));
    }
    static uint32_t lastLogMs = 0;
    if (RUN_LOG_EVERY_MS > 0 && g_mission.phase != Phase::WaitStart &&
        g_state.timestampMs - lastLogMs >= RUN_LOG_EVERY_MS) {
        lastLogMs = g_state.timestampMs;
        const MotionFeedback &m = g_perception.motion;
        Serial.print(F("  t="));
        Serial.print(g_state.timestampMs);
        Serial.print(F(" ph="));
        Serial.print(static_cast<uint8_t>(g_mission.phase));
        Serial.print(F(" sub="));
        Serial.print(static_cast<uint8_t>(g_mission.sub));
        Serial.print(F(" d="));
        Serial.print(m.distMm, 0);
        Serial.print(F(" tgt="));
        Serial.print(g_mission.step.straight.targetMm, 0);
        Serial.print(F(" v="));
        Serial.print(m.speedMmS, 0);
        Serial.print(F(" h="));
        Serial.print(m.headingDeg, 1);
        Serial.print(F(" duty="));
        Serial.print(g_lastCmd.left);
        Serial.print('/');
        Serial.print(g_lastCmd.right);
        Serial.print(F(" us="));
        Serial.print(g_state.distFrontMm);
        Serial.print('/');
        Serial.print(g_state.distLeftMm);
        Serial.print('/');
        Serial.println(g_state.distRightMm);
    }
    if (g_mission.cx != lastCx || g_mission.cy != lastCy) {
        lastCx = g_mission.cx;
        lastCy = g_mission.cy;
        Serial.print(F("mission: cell "));
        Serial.print(lastCx);
        Serial.print(',');
        Serial.print(lastCy);
        Serial.print(F(" facing "));
        Serial.println(static_cast<uint8_t>(g_mission.facing));
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
    applyDrive(runAlgorithm(g_state));
    digitalWrite(PIN_LED_ERROR, g_mission.phase == Phase::Fault);
    serialSupervise();

    // Heartbeat LED - proves the loop is alive.
    static uint32_t lastToggleMs = 0;
    static bool ledOn = false;
    if (nowMs - lastToggleMs >= 500) {
        lastToggleMs = nowMs;
        ledOn = !ledOn;
        digitalWrite(PIN_LED_STATUS, ledOn);
    }
}
