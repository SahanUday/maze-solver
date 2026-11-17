// ============================================================================
//  operator_io.cpp  --  see operator_io.h.
// ============================================================================

#include "app/operator_io.h"

#include <Arduino.h>

#include "config/RobotConfig.h"
#include "spec/Mission.h"

namespace
{
constexpr uint32_t HEARTBEAT_HALF_PERIOD_MS = 500;
} // namespace

void operatorIoInit()
{
    pinMode(PIN_LED_STATUS, OUTPUT);
    pinMode(PIN_LED_ERROR, OUTPUT);
    if (RUN_START_BUTTON_FITTED) {
        pinMode(PIN_BTN_START, INPUT_PULLUP);
    }
}

bool operatorIoStartButton()
{
    return RUN_START_BUTTON_FITTED && digitalRead(PIN_BTN_START) == LOW;
}

void operatorIoTick(uint32_t nowMs, bool fault)
{
    static uint32_t lastToggleMs = 0;
    static bool heartbeatOn = false;
    if (nowMs - lastToggleMs >= HEARTBEAT_HALF_PERIOD_MS) {
        lastToggleMs = nowMs;
        heartbeatOn = !heartbeatOn;
        digitalWrite(PIN_LED_STATUS, heartbeatOn);
    }

    static bool errorOn = false;
    if (fault != errorOn) {
        errorOn = fault;
        digitalWrite(PIN_LED_ERROR, errorOn);
    }
}
