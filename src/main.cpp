// ============================================================================
//  main.cpp  --  TEMPORARY toolchain check.
//
//  This file exists only to prove that the project compiles and uploads.
//  It is replaced in SW-01 step 6 by the real fixed-period control loop.
// ============================================================================

#include <Arduino.h>

static const uint8_t PIN_HEARTBEAT = LED_BUILTIN;   // pin 13 on the Mega

void setup() {
    Serial.begin(250000);
    pinMode(PIN_HEARTBEAT, OUTPUT);
    Serial.println(F("maze-solver: toolchain OK"));
}

void loop() {
    digitalWrite(PIN_HEARTBEAT, HIGH);
    delay(500);
    digitalWrite(PIN_HEARTBEAT, LOW);
    delay(500);
}
