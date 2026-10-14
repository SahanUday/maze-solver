// ============================================================================
//  operator_io.cpp  --  see operator_io.h.
// ============================================================================

#include "operator_io.h"

#include <Arduino.h>

#include "RobotConfig.h"

void operatorInit()
{
    pinMode(PIN_BTN_START, INPUT_PULLUP);
    for (const uint8_t pin : PIN_DIP) {
        pinMode(pin, INPUT_PULLUP);
    }
    pinMode(PIN_LED_STATUS, OUTPUT);
    pinMode(PIN_LED_ERROR, OUTPUT);
}

bool operatorStartPressed()
{
    return digitalRead(PIN_BTN_START) == LOW;
}

uint8_t operatorReadDip()
{
    uint8_t dip = 0;
    for (uint8_t i = 0; i < sizeof(PIN_DIP); ++i) {
        if (digitalRead(PIN_DIP[i]) == LOW) {
            dip = static_cast<uint8_t>(dip | (1u << i));
        }
    }
    return dip;
}
