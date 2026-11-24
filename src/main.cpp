// ============================================================================
//  main.cpp  --  fixed-period control loop scheduler.
//  Ticks at CONTROL_LOOP_PERIOD_MS: readSensors() then runAlgorithm().
//  Hardware is reached only through src/hal/* drivers and the glue in src/app/ - see
//  RobotState.h. The algorithm is the Section A run (include/mission/RunLoop.h): after the gyro is
//  zeroed at power-up the robot explores from where it stands until the approach marker, or every
//  reachable tile without a line calibration.
// ============================================================================

#include <Arduino.h>

#include "app/heading_estimator.h"
#include "app/operator_io.h"
#include "app/telemetry.h"
#include "core/RobotState.h"
#include "core/Scheduler.h"
#include "hal/encoders.h"
#include "hal/line_sensors.h"
#include "hal/motors.h"
#include "hal/ultrasonic.h"
#include "line_cal_store.h"
#include "mission/RunLoop.h"
#include "panel/PanelLeds.h"
#include "spec/fixed/Hardware.h"
#include "spec/tuning/Run.h"

static RobotState g_state;
static RunLoop g_run;
static bool g_motorsEnabled = false;
static bool g_haveCal = false; // a usable IR line calibration was loaded at boot
static uint8_t g_dip = 0;      // the DIP bank, read while the run waits for its start

// The mission's stage as the panel shows it; main is the one place that knows both.
static PanelRun panelRunOf(RunStage stage)
{
    switch (stage) {
        case RunStage::WaitStart:
            return PanelRun::Waiting;
        case RunStage::Running:
            return PanelRun::Running;
        case RunStage::Done:
            return PanelRun::EndedOk;
        case RunStage::Fault:
            return PanelRun::EndedBad;
    }
    return PanelRun::EndedBad;
}

void setup()
{
    Serial.begin(DEBUG_SERIAL_BAUD);
    operatorIoInit();
    motorsInit(); // EN stays low (coast) until the control layer enables it
    encodersInit();
    lineSensorsInit();
    const bool imuPresent = headingEstimatorInit();
    ultrasonicInit();

    // The IR calibration is kept in EEPROM (line_cal_store.h). Without one that looks real the run
    // reads walls only: it explores every reachable tile and stops, and cannot see the marker.
    LineCalibration cal;
    g_haveCal = lineCalLoad(cal) && lineCalibrationLooksReal(cal);
    runLoopInit(g_run, imuPresent, g_haveCal ? &cal : nullptr,
                PartRange{RUN_PART_FIRST, RUN_PART_LAST});
    if (!imuPresent) {
        Serial.println(F("run: no gyro, will not start"));
    } else {
        Serial.println(g_haveCal ? F("run: starts by itself, line calibration loaded")
                                 : F("run: starts by itself (walls only: no line calibration)"));
    }
    Serial.println(F("maze-solver: fixed-period loop starting"));
}

// Drivers populate g_state from hardware.
static void readSensors(RobotState &state)
{
    encodersRead(state.encoderCountL, state.encoderCountR);
    lineSensorsRead(state.irRaw);
    headingEstimatorRead(state);
    ultrasonicUpdate();
    state.distFrontMm = ultrasonicDistanceMm(Ultrasonic::Front);
    state.distLeftMm = ultrasonicDistanceMm(Ultrasonic::Left);
    state.distRightMm = ultrasonicDistanceMm(Ultrasonic::Right);
    state.usValid = ultrasonicValidMask();
}

// The Section A run: perception, mission, then the motors.
static void runAlgorithm(const RobotState &state)
{
    float headingFixDeg = 0.0f;
    const DriveCommand cmd =
        runLoopTick(g_run, state, state.timestampMs, operatorIoStartButton(), headingFixDeg);

    // The corridor's measured direction, handed back to the gyro (StepCell.h).
    if (headingFixDeg != 0.0f) {
        headingEstimatorApplyFix(headingFixDeg);
    }

    if (cmd.enable != g_motorsEnabled) {
        motorsEnable(cmd.enable);
        g_motorsEnabled = cmd.enable;
    }
    if (cmd.enable) {
        motorSet(Motor::Left, cmd.left);
        motorSet(Motor::Right, cmd.right);
    }

    telemetryTick(g_run, state, cmd);

    // The DIP bank is read only while waiting for the start.
    if (g_run.mission.stage == RunStage::WaitStart) {
        g_dip = operatorIoDip();
    }
    PanelView view;
    view.run = panelRunOf(g_run.mission.stage);
    view.mode = panelDecode(g_dip);
    view.noCalibration = !g_haveCal;
    operatorIoTick(state.timestampMs, view);
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
}
