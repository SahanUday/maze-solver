// ============================================================================
//  calibration/main.cpp  --  entry point of env:calibrate.
//  Same fixed-period loop as src/main.cpp (readSensors() then the algorithm), but
//  the algorithm is a run on RunLayer: the safe run (recorded by flight_log) or the
//  sweep (streamed as CSV). The run is picked over serial until the self-test starts.
//  Hardware is reached only through src/hal/* drivers - see RobotState.h.
// ============================================================================

#include <Arduino.h>

#include "Heading.h"
#include "RobotConfig.h"
#include "RobotSpec.h"
#include "RobotState.h"
#include "RunLayer.h"
#include "SafeRun.h"
#include "Scheduler.h"
#include "StatusLed.h"
#include "SweepRun.h"
#include "flight_log.h"
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

static RunLayer g_layer;
static SafeRun g_safeRun;
static SweepRun g_sweep;
static bool g_motorsEnabled = false;

enum class RunKind : uint8_t { Safe, Sweep };
static RunKind g_kind = RunKind::Safe;

// Same procedure as src/main.cpp.
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
    flightLogDump(); // the previous run, if one was saved
    pinMode(PIN_LED_STATUS, OUTPUT);
    pinMode(PIN_LED_ERROR, OUTPUT);
    pinMode(PIN_LED_ONBOARD, OUTPUT);
    pinMode(PIN_BTN_START, INPUT_PULLUP);
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
    runLayerInit(g_layer, RUN_START_BUTTON_FITTED);
    safeRunInit(g_safeRun, g_imuPresent && g_gyroBias.ready);
    sweepRunInit(g_sweep);
    Serial.println(F("calibrate: 's' = safe run (default), 'w' = sweep (wheels OFF the ground)"));
    if (RUN_START_BUTTON_FITTED) {
        Serial.println(F("calibrate: ready - press START (wheels stay off until then)"));
    } else {
        Serial.println(
            F("calibrate: no start button - run begins by itself, wheels stay off until then"));
    }
}

// Drivers populate g_state from hardware. Same as src/main.cpp, plus the button.
static void readSensors(RobotState &state)
{
    encodersRead(state.encoderCountL, state.encoderCountR);
    lineSensorsRead(state.irRaw);

    // A failed transfer leaves state.imu holding the previous sample, and skips the
    // integration rather than integrating a stale rate twice.
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
    // Polled every tick, not read once: it is also the emergency stop.
    state.startButtonPressed = digitalRead(PIN_BTN_START) == LOW;
}

static const __FlashStringHelper *phaseName(RunPhase phase)
{
    switch (phase) {
        case RunPhase::WaitStart:
            return F("wait-start");
        case RunPhase::Countdown:
            return F("countdown");
        case RunPhase::SelfTest:
            return F("self-test");
        case RunPhase::Cruise:
            return F("cruise");
        case RunPhase::Settle:
            return F("settle");
        case RunPhase::Turn:
            return F("turn");
        case RunPhase::Stopped:
            return F("STOPPED");
        case RunPhase::Fault:
            return F("FAULT");
        case RunPhase::Sweep:
            return F("sweep");
    }
    return F("?");
}

static const __FlashStringHelper *reasonName(RunReason reason)
{
    switch (reason) {
        case RunReason::None:
            return F("-");
        case RunReason::EStop:
            return F("button pressed");
        case RunReason::TimeLimit:
            return F("run time limit");
        case RunReason::FrontWall:
            return F("wall ahead, cannot turn");
        case RunReason::NoClearSide:
            return F("wall ahead, side sensor silent");
        case RunReason::TooManyTurns:
            return F("too many turns in a row");
        case RunReason::Complete:
            return F("sweep complete");
        case RunReason::SelfTestReversed:
            return F("a wheel's encoder counted DOWN while driven forward - flip ENC_*_INVERT");
        case RunReason::SelfTestNoMotion:
            return F("a wheel did not move - check wiring/EN, or MOTOR_MIN_PWM");
        case RunReason::Stall:
            return F("a wheel stalled");
        case RunReason::FrontBlind:
            return F("no front ultrasonic echo");
        case RunReason::TurnTimeout:
            return F("turn did not finish - gyro/wheels?");
    }
    return F("?");
}

// Applies the run's output to the motors; EN follows `enable`, and low also zeroes both commands.
static void driveMotors(const RunOutput &out)
{
    if (out.enable != g_motorsEnabled) {
        motorsEnable(out.enable);
        g_motorsEnabled = out.enable;
    }
    if (out.enable) {
        motorSet(Motor::Left, out.left);
        motorSet(Motor::Right, out.right);
    }
}

// Notes the run, including the tick it ends on but not the idle ticks after it, and streams it
// to EEPROM once the wheel self-test has passed (so the motors are powered: a real run).
static void recordFlight(const RobotState &state, RunPhase before, const RunOutput &out)
{
    const bool wasEnded = before == RunPhase::Stopped || before == RunPhase::Fault;
    if (g_layer.phase != RunPhase::WaitStart && !wasEnded) {
        flightLogRecord(state, g_layer.phase, g_layer.reason, out.left, out.right);
    }
    if (g_layer.phase == RunPhase::Cruise && before != RunPhase::Cruise) {
        flightLogCommit();
    }
    flightLogService();
}

// Sensor readout for the bench. The wheels are off in these phases, so printing costs nothing.
static void printBench(const RobotState &state)
{
    static uint32_t lastPrintMs = 0;
    const bool wheelsOff =
        g_layer.phase == RunPhase::WaitStart || g_layer.phase == RunPhase::Countdown;
    if (!wheelsOff || state.timestampMs - lastPrintMs < 500) {
        return;
    }
    lastPrintMs = state.timestampMs;
    Serial.print(F("us mm F="));
    Serial.print((state.usValid & RUN_US_FRONT) ? state.distFrontMm : 0);
    Serial.print(F(" L="));
    Serial.print((state.usValid & RUN_US_LEFT) ? state.distLeftMm : 0);
    Serial.print(F(" R="));
    Serial.print((state.usValid & RUN_US_RIGHT) ? state.distRightMm : 0);
    Serial.println(F("  (0 = no echo)"));
}

static void printPhaseChange(const RobotState &state)
{
    Serial.print(F("run: "));
    Serial.print(phaseName(g_layer.phase));
    if (g_layer.reason != RunReason::None) {
        Serial.print(F(" - "));
        Serial.print(reasonName(g_layer.reason));
        Serial.print(F(" (enc L="));
        Serial.print(state.encoderCountL);
        Serial.print(F(" R="));
        Serial.print(state.encoderCountR);
        Serial.print(F(")"));
    }
    Serial.println();
}

// The run is picked while nothing is driving; after that the choice is locked.
static void pollRunSelect()
{
    if (g_layer.phase != RunPhase::WaitStart && g_layer.phase != RunPhase::Countdown) {
        return;
    }
    while (Serial.available() > 0) {
        const int c = Serial.read();
        if (c == 'w' || c == 's') {
            g_kind = c == 'w' ? RunKind::Sweep : RunKind::Safe;
            Serial.println(g_kind == RunKind::Sweep ? F("calibrate: sweep selected")
                                                    : F("calibrate: safe run selected"));
        }
    }
}

// Lines starting with '#' are notes; the rest is CSV that tools/scripts read by its shape. No
// battery sense exists, so the pack voltage and the floor are for the person to fill in.
static void printSweepHeader()
{
    Serial.println(F("# sweep v1"));
    Serial.print(F("# built "));
    Serial.println(F(__DATE__ " " __TIME__));
    Serial.print(F("# step_ms="));
    Serial.print(SWEEP_STEP_MS);
    Serial.print(F(" measure_ms="));
    Serial.print(SWEEP_MEASURE_MS);
    Serial.print(F(" duty="));
    Serial.print(SWEEP_DUTY_START);
    Serial.print(F(".."));
    Serial.print(SWEEP_DUTY_MAX);
    Serial.print(F("/"));
    Serial.println(SWEEP_DUTY_STEP);
    Serial.print(F("# counts_per_rev="));
    Serial.print(ENCODER_COUNTS_PER_REV);
    Serial.print(F(" wheel_mm="));
    Serial.print(WHEEL_DIAMETER_MM);
    Serial.print(F(" pwm_hz="));
    Serial.println(MOTOR_PWM_FREQ_HZ);
    Serial.println(F("# pack_v= surface="));
    Serial.println(F("motor,direction,sweep,duty,counts,dt_ms"));
}

static void printSweepRow(const SweepRow &row)
{
    Serial.print(row.right ? 'R' : 'L');
    Serial.print(row.reverse ? F(",reverse,") : F(",forward,"));
    Serial.print(row.up ? F("up,") : F("down,"));
    Serial.print(row.duty);
    Serial.print(',');
    Serial.print(row.counts);
    Serial.print(',');
    Serial.println(row.dtMs);
}

// Steps whichever run is picked. Drives the motors only through the layer's output.
static void runSelected(const RobotState &state)
{
    pollRunSelect();
    const RunPhase before = g_layer.phase;
    RunOutput out;
    if (g_kind == RunKind::Sweep) {
        out = runLayerStep(g_layer, g_sweep, state);
        SweepRow row;
        if (g_layer.phase == RunPhase::Sweep && before != RunPhase::Sweep) {
            printSweepHeader();
        }
        if (sweepRunTakeRow(g_sweep, row)) {
            printSweepRow(row);
        }
    } else {
        out = runLayerStep(g_layer, g_safeRun, state);
    }

    driveMotors(out);
    if (g_kind == RunKind::Safe) {
        recordFlight(state, before, out); // a sweep must not replace the last safe run's log
    }

    digitalWrite(PIN_LED_ERROR, g_layer.phase == RunPhase::Fault);
    digitalWrite(PIN_LED_ONBOARD, statusLedOn(g_layer.phase, g_layer.reason, state.timestampMs));

    printBench(state);
    if (g_layer.phase != before) {
        printPhaseChange(state);
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
    runSelected(g_state);

    // Heartbeat LED - proves the loop is alive.
    static uint32_t lastToggleMs = 0;
    static bool ledOn = false;
    if (nowMs - lastToggleMs >= 500) {
        lastToggleMs = nowMs;
        ledOn = !ledOn;
        digitalWrite(PIN_LED_STATUS, ledOn);
    }
}
