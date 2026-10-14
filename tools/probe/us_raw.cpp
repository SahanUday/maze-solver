// Bench probe: pulse each HC-SR04 TRIG and time its ECHO with plain Arduino calls (no HAL driver),
// to tell a wiring or power fault from a driver fault. Motors stay off.
#include <Arduino.h>

#include "RobotConfig.h"
#include "RobotSpec.h"

static const uint8_t TRIG[3] = {PIN_US_FRONT_TRIG, PIN_US_LEFT_TRIG, PIN_US_RIGHT_TRIG};
static const uint8_t ECHO[3] = {PIN_US_FRONT_ECHO, PIN_US_LEFT_ECHO, PIN_US_RIGHT_ECHO};

void setup()
{
    Serial.begin(DEBUG_SERIAL_BAUD);
    for (uint8_t i = 0; i < 3; ++i) {
        pinMode(TRIG[i], OUTPUT);
        digitalWrite(TRIG[i], LOW);
        pinMode(ECHO[i], INPUT);
    }
    Serial.println(F("# front,left,right: echo idle level, pulse us (0=none), mm"));
}

void loop()
{
    for (uint8_t i = 0; i < 3; ++i) {
        const int idle = digitalRead(ECHO[i]);
        digitalWrite(TRIG[i], HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG[i], LOW);
        const unsigned long us = pulseIn(ECHO[i], HIGH, 30000UL);
        Serial.print(idle);
        Serial.print(' ');
        Serial.print(us);
        Serial.print(' ');
        Serial.print(us * 343UL / 2000UL);
        Serial.print(i < 2 ? F(" | ") : F("\n"));
        delay(60);
    }
}
