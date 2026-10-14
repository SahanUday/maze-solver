// ============================================================================
//  flight_log.h  --  records a run so it can be read back over USB afterwards.
//
//  While the robot runs on its own battery there is no serial monitor, so the loop notes
//  the phase, both encoders, the three wall distances, the heading and the motor commands
//  at a fixed interval and whenever the phase changes. Samples are streamed into the
//  Mega's EEPROM one byte per tick while the run is under way, so cutting the power
//  mid-run keeps everything up to about the last half second. The next boot prints the log
//  on Serial.
//  Debug glue: it uses the Arduino framework and avr-libc's EEPROM routines.
// ============================================================================

#pragma once

#include <stdint.h>

#include "RobotState.h"
#include "RunLayer.h"

// Notes one sample if the interval has passed, or the phase or reason changed since the last. Call
// every tick while a run is under way; it drops samples once the log is full.
void flightLogRecord(const RobotState &state, RunPhase phase, RunReason reason, int16_t outLeft,
                     int16_t outRight);

// Marks the run as real: from now on the log is streamed to EEPROM, replacing the previous one.
// Call once the wheels are known to turn (the self-test passed). Until then samples are only kept
// in RAM, so a power-up that never reaches a run (USB plugged in with the motors unpowered, say)
// does not erase the last run's log.
void flightLogCommit();

// Moves at most one byte of the log into EEPROM, and only if the EEPROM is idle, so it never waits.
// Call every tick, including after the run has ended, until the log is flushed.
void flightLogService();

// Prints the log saved in EEPROM, if there is one. Call once at boot, after Serial.begin().
void flightLogDump();
