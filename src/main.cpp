// ============================================================================
//  main.cpp  --  fixed-period control loop (SW-01).
//
//  This is the scheduler only: it ticks at CONTROL_LOOP_PERIOD_MS and gives
//  drivers and algorithms a place to run, in order, every tick. It does not
//  yet drive any sensor or motor - those arrive with their own tasks
//  (HW-03, SW-02, SW-03...). Until then the hooks below are empty on
//  purpose: adding "just a quick test read" here is how magic numbers and
//  direct hardware calls creep into what is supposed to stay a thin
//  scheduler.
//
//  Structural rule (see RobotState.h): this file, and only this file's
//  driver step, may touch pins directly. Everything downstream reads a
//  RobotState.
// ============================================================================

#include <Arduino.h>

#include "RobotConfig.h"
#include "RobotSpec.h"
#include "RobotState.h"

static RobotState g_state;

void setup()
{
    Serial.begin(DEBUG_SERIAL_BAUD);
    pinMode(PIN_LED_STATUS, OUTPUT);
    Serial.println(F("maze-solver: fixed-period loop starting"));
}

// Runs once per tick: drivers populate g_state from hardware.
// Empty until HW-03 wiring + SW-02/SW-03 drivers exist.
static void readSensors(RobotState &state)
{
    (void)state;
}

// Runs once per tick, after readSensors(): algorithms read g_state only,
// and decide what the robot should do next.
// Empty until SW-02 (motion primitives) exists.
static void runAlgorithm(const RobotState &state)
{
    (void)state;
}

void loop()
{
    static uint32_t nextTickMs = 0;
    const uint32_t nowMs = millis();

    // Not yet time for the next tick: do nothing rather than busy-loop on
    // work, so a later addition here cannot turn into an accidental delay().
    if (static_cast<int32_t>(nowMs - nextTickMs) < 0) {
        return;
    }

    // Advance from the previous target, not from nowMs: that's what keeps
    // the period fixed under jitter instead of drifting by however long the
    // last tick's work took. If a slow tick has put us more than one period
    // behind, resync to now instead of firing a burst of back-to-back ticks
    // trying to catch up.
    nextTickMs += CONTROL_LOOP_PERIOD_MS;
    if (static_cast<int32_t>(nowMs - nextTickMs) > 0) {
        nextTickMs = nowMs;
    }

    g_state.timestampMs = nowMs;
    readSensors(g_state);
    runAlgorithm(g_state);

    // Heartbeat: proves the loop is still ticking, without a blocking
    // delay() stealing time from the control loop. Elapsed-time check
    // (matches the tick-due check above) rather than a tick-count modulo,
    // so it has no dependency on CONTROL_LOOP_PERIOD_MS evenly dividing
    // anything - and a remembered state bit instead of a digitalRead()
    // round trip to invert it.
    static uint32_t lastToggleMs = 0;
    static bool ledOn = false;
    if (nowMs - lastToggleMs >= 500) {
        lastToggleMs = nowMs;
        ledOn = !ledOn;
        digitalWrite(PIN_LED_STATUS, ledOn);
    }
}
