// ============================================================================
//  operator_io.cpp  --  see operator_io.h.
// ============================================================================

#include "app/operator_io.h"

#include <Arduino.h>

#include "panel/PanelMode.h"
#include "spec/pins/PinMap.h"
#include "spec/pins/Wired.h"

static_assert(sizeof(PIN_DIP) / sizeof(PIN_DIP[0]) == PANEL_DIP_COUNT,
              "PIN_DIP and PANEL_DIP_COUNT differ in size");

void operatorIoInit()
{
    pinMode(PIN_LED_STATUS, OUTPUT);
    pinMode(PIN_LED_ERROR, OUTPUT);
    for (uint8_t i = 0; i < PANEL_DIP_COUNT; ++i) {
        pinMode(PIN_DIP[i], INPUT_PULLUP);
    }
    if (BUTTON_WIRED) {
        pinMode(PIN_BTN_START, INPUT_PULLUP);
    }
}

uint8_t operatorIoDip()
{
    uint8_t bits = 0;
    for (uint8_t i = 0; i < PANEL_DIP_COUNT; ++i) {
        if (digitalRead(PIN_DIP[i]) == LOW) { // ON = pin to GND
            bits |= static_cast<uint8_t>(1u << i);
        }
    }
    return bits;
}

bool operatorIoStartButton()
{
    return BUTTON_WIRED && digitalRead(PIN_BTN_START) == LOW;
}

void operatorIoTick(uint32_t nowMs, const PanelView &view)
{
    const PanelLeds leds = panelLeds(view, nowMs);

    static bool statusOn = false;
    if (leds.status != statusOn) {
        statusOn = leds.status;
        digitalWrite(PIN_LED_STATUS, statusOn);
    }

    static bool errorOn = false;
    if (leds.error != errorOn) {
        errorOn = leds.error;
        digitalWrite(PIN_LED_ERROR, errorOn);
    }
}
