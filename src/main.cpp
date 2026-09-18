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
#include "RobotState.h"

static RobotState g_state;

void setup() {
    Serial.begin(250000);
    pinMode(PIN_LED_STATUS, OUTPUT);
    Serial.println(F("maze-solver: fixed-period loop starting"));
}

// Runs once per tick: drivers populate g_state from hardware.
// Empty until HW-03 wiring + SW-02/SW-03 drivers exist.
static void readSensors(RobotState &state) {
    (void)state;
}

// Runs once per tick, after readSensors(): algorithms read g_state only,
// and decide what the robot should do next.
// Empty until SW-02 (motion primitives) exists.
static void runAlgorithm(const RobotState &state) {
    (void)state;
}

void loop() {
    static uint32_t nextTickMs = 0;
    const uint32_t nowMs = millis();

    // Not yet time for the next tick: do nothing rather than busy-loop on
    // work, so a later addition here cannot turn into an accidental delay().
    if (static_cast<int32_t>(nowMs - nextTickMs) < 0) {
        return;
    }
    nextTickMs = nowMs + CONTROL_LOOP_PERIOD_MS;

    g_state.timestampMs = nowMs;
    readSensors(g_state);
    runAlgorithm(g_state);

    // Heartbeat: proves the loop is still ticking at rate, without a
    // blocking delay() stealing time from the control loop.
    static uint16_t tickCount = 0;
    ++tickCount;
    const uint16_t ticksPerHalfSecond = 500 / CONTROL_LOOP_PERIOD_MS;
    if (tickCount % ticksPerHalfSecond == 0) {
        digitalWrite(PIN_LED_STATUS, !digitalRead(PIN_LED_STATUS));
    }
}
