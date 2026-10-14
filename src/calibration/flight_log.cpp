// ============================================================================
//  flight_log.cpp  --  see flight_log.h.
// ============================================================================

#include "flight_log.h"

#include <Arduino.h>
#include <avr/eeprom.h>
#include <stddef.h>

namespace
{

constexpr uint16_t kMagic = 0xF117;
constexpr uint16_t kMax = 200; // 200 x 350 ms = 70 s: the 8 s countdown and the 60 s run
constexpr uint16_t kIntervalMs = 350;

struct Record {
    uint16_t t10;        // ms since the first sample, in units of 10 ms
    uint8_t phaseReason; // phase in the high nibble, reason in the low
    uint8_t usValid;
    int16_t encL;
    int16_t encR;
    uint16_t front; // mm, 0 = no echo
    uint16_t left;
    uint16_t right;
    int16_t head10; // heading in tenths of a degree
    int8_t outL4;   // motor command / 4
    int8_t outR4;
};

struct Header {
    uint16_t magic;
    uint16_t count;
    uint32_t reserved;
};

static_assert(sizeof(Header) + sizeof(Record) * kMax <= 4096,
              "the log must fit the Mega's 4 KB EEPROM");

Record g_log[kMax];
uint16_t g_count = 0; // samples noted
uint32_t g_firstMs = 0;
uint32_t g_lastMs = 0;
uint8_t g_lastPhaseReason = 0xFF;

// EEPROM streaming state. The header goes first, then each record in turn, and after each record
// the count byte, so the stored count never claims more than has fully landed. The count is below
// 256 and its high byte is written as 0 with the header, so one byte write updates it atomically.
constexpr uint8_t kHeaderBytes = sizeof(Header);
constexpr uint16_t kCountAddress = offsetof(Header, count);
uint8_t g_headerWritten = 0;
uint16_t g_flushed = 0; // records fully in EEPROM
uint8_t g_byteInRecord = 0;
bool g_countPending = false;
bool g_committed = false;

int16_t clamp16(int32_t v)
{
    return static_cast<int16_t>(v > 32767 ? 32767 : (v < -32768 ? -32768 : v));
}

} // namespace

void flightLogRecord(const RobotState &s, RunPhase phase, RunReason reason, int16_t outLeft,
                     int16_t outRight)
{
    if (g_count >= kMax) {
        return;
    }
    const uint8_t phaseReasonCode = static_cast<uint8_t>((static_cast<uint8_t>(phase) << 4) |
                                                         (static_cast<uint8_t>(reason) & 0xF));
    if (g_count == 0) {
        g_firstMs = s.timestampMs;
    } else if (s.timestampMs - g_lastMs < kIntervalMs && phaseReasonCode == g_lastPhaseReason) {
        return;
    }
    g_lastPhaseReason = phaseReasonCode;
    g_lastMs = s.timestampMs;
    Record &r = g_log[g_count++];
    r.t10 = static_cast<uint16_t>((s.timestampMs - g_firstMs) / 10);
    r.phaseReason = phaseReasonCode;
    r.usValid = s.usValid;
    r.encL = clamp16(s.encoderCountL);
    r.encR = clamp16(s.encoderCountR);
    r.front = (s.usValid & RUN_US_FRONT) ? s.distFrontMm : 0;
    r.left = (s.usValid & RUN_US_LEFT) ? s.distLeftMm : 0;
    r.right = (s.usValid & RUN_US_RIGHT) ? s.distRightMm : 0;
    r.head10 = clamp16(static_cast<int32_t>(s.headingDeg * 10.0f));
    r.outL4 = static_cast<int8_t>(outLeft / 4);
    r.outR4 = static_cast<int8_t>(outRight / 4);
}

void flightLogCommit()
{
    g_committed = true;
}

void flightLogService()
{
    if (!g_committed || g_count == 0 || !eeprom_is_ready()) {
        return;
    }
    if (g_headerWritten < kHeaderBytes) {
        const Header h = {kMagic, 0, 0};
        eeprom_update_byte(reinterpret_cast<uint8_t *>(g_headerWritten),
                           reinterpret_cast<const uint8_t *>(&h)[g_headerWritten]);
        ++g_headerWritten;
        return;
    }
    if (g_countPending) {
        eeprom_update_byte(reinterpret_cast<uint8_t *>(kCountAddress),
                           static_cast<uint8_t>(g_flushed));
        g_countPending = false;
        return;
    }
    if (g_flushed < g_count) {
        const uint16_t address = kHeaderBytes + g_flushed * sizeof(Record) + g_byteInRecord;
        eeprom_update_byte(reinterpret_cast<uint8_t *>(address),
                           reinterpret_cast<const uint8_t *>(&g_log[g_flushed])[g_byteInRecord]);
        if (++g_byteInRecord == sizeof(Record)) {
            g_byteInRecord = 0;
            ++g_flushed;
            g_countPending = true;
        }
    }
}

void flightLogDump()
{
    Header h;
    eeprom_read_block(&h, reinterpret_cast<const void *>(0), sizeof(h));
    if (h.magic != kMagic || h.count == 0 || h.count > kMax) {
        return;
    }
    Serial.print(F("LOG begin, samples "));
    Serial.println(h.count);
    Serial.println(F("L t_ms phase reason encL encR front left right heading_deg outL outR"));
    for (uint16_t i = 0; i < h.count; ++i) {
        Record r;
        eeprom_read_block(&r, reinterpret_cast<const void *>(sizeof(h) + i * sizeof(Record)),
                          sizeof(r));
        Serial.print(F("L "));
        Serial.print(static_cast<uint32_t>(r.t10) * 10);
        Serial.print(' ');
        Serial.print(r.phaseReason >> 4);
        Serial.print(' ');
        Serial.print(r.phaseReason & 0xF);
        Serial.print(' ');
        Serial.print(r.encL);
        Serial.print(' ');
        Serial.print(r.encR);
        Serial.print(' ');
        Serial.print(r.front);
        Serial.print(' ');
        Serial.print(r.left);
        Serial.print(' ');
        Serial.print(r.right);
        Serial.print(' ');
        Serial.print(r.head10 / 10);
        Serial.print('.');
        Serial.print(abs(r.head10) % 10);
        Serial.print(' ');
        Serial.print(r.outL4 * 4);
        Serial.print(' ');
        Serial.println(r.outR4 * 4);
    }
    Serial.println(F("LOG end"));
}
