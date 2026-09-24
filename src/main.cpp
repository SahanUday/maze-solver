// ============================================================================
//  main.cpp  --  fixed-period control loop scheduler.
//  Ticks at CONTROL_LOOP_PERIOD_MS: readSensors() then runAlgorithm().
//  Only this file may touch pins directly - see RobotState.h.
// ============================================================================

#include <Arduino.h>

#include "RobotConfig.h"
#include "RobotSpec.h"
#include "RobotState.h"
#include "Scheduler.h"

static RobotState g_state;

void setup()
{
    Serial.begin(DEBUG_SERIAL_BAUD);
    pinMode(PIN_LED_STATUS, OUTPUT);
    Serial.println(F("maze-solver: fixed-period loop starting"));
}

// Drivers populate g_state from hardware. Empty until real drivers exist.
static void readSensors(RobotState &state)
{
    (void)state;
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
