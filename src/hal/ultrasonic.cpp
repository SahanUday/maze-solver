// ============================================================================
//  ultrasonic.cpp  --  3x HC-SR04, round-robin, PCINT echo timing.
//  docs/architecture/decisions/0007-non-blocking-ultrasonic-ranging.md
// ============================================================================

#include "ultrasonic.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/atomic.h>
#include <util/delay.h>

#include "RobotConfig.h"
#include "RobotSpec.h"
#include "UltrasonicMath.h"

namespace hal
{
namespace ultrasonic
{

static_assert(US_SENSOR_COUNT == 3, "front, left, right");

// The register bits below are hand-mapped to these pins; moving one must break the build.
static_assert(PIN_US_FRONT_TRIG == 30, "front trigger must be pin 30 (PC7)");
static_assert(PIN_US_LEFT_TRIG == 32, "left trigger must be pin 32 (PC5)");
static_assert(PIN_US_RIGHT_TRIG == 34, "right trigger must be pin 34 (PC3)");
static_assert(PIN_US_FRONT_ECHO == A10, "front echo must be A10 (PK2, PCINT18)");
static_assert(PIN_US_LEFT_ECHO == A11, "left echo must be A11 (PK3, PCINT19)");
static_assert(PIN_US_RIGHT_ECHO == A12, "right echo must be A12 (PK4, PCINT20)");
static_assert(US_SLOT_MS % CONTROL_LOOP_PERIOD_MS == 0, "slot must be whole ticks");
static_assert(US_TIMER_PRESCALER == 8, "TCCR1B below hardcodes CS11");

// Trigger pins 30/32/34 on PORTC.
constexpr uint8_t kTrigBits[US_SENSOR_COUNT] = {_BV(PC7), _BV(PC5), _BV(PC3)};

// Echo pins A10/A11/A12 on PORTK.
constexpr uint8_t kEchoBits[US_SENSOR_COUNT] = {_BV(PK2), _BV(PK3), _BV(PK4)};

// Mounting offsets, same order as the Sensor enum.
constexpr int16_t kOffsetMm[US_SENSOR_COUNT] = {US_OFFSET_FRONT_MM, US_OFFSET_LEFT_MM,
                                                US_OFFSET_RIGHT_MM};

constexpr uint8_t kTrigMask = kTrigBits[0] | kTrigBits[1] | kTrigBits[2];
constexpr uint8_t kEchoMask = kEchoBits[0] | kEchoBits[1] | kEchoBits[2];
constexpr uint8_t kSlotTicks = US_SLOT_MS / CONTROL_LOOP_PERIOD_MS;

// Written by the ISR.
static volatile uint16_t g_startTicks = 0;
static volatile uint16_t g_endTicks = 0;
static volatile bool g_haveStart = false;
static volatile bool g_haveEnd = false;
static volatile uint8_t g_activeEcho = 0;

namespace
{

uint8_t g_sensor = 0;
uint8_t g_tickInSlot = 0;
bool g_slotStored = false;

uint16_t g_distanceMm[US_SENSOR_COUNT] = {0};
uint8_t g_validMask = 0;

void startPing(uint8_t s)
{
    g_slotStored = false;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        g_activeEcho = kEchoBits[s];
        g_haveStart = false;
        g_haveEnd = false;
        TCNT1 = 0;
        PCMSK2 = g_activeEcho; // listen to this sensor only
        PCIFR = _BV(PCIF2);    // drop any edge already pending
    }

    // 10us trigger pulse. Read-modify-write: PC1 is the IR emitter.
    PORTC |= kTrigBits[s];
    _delay_us(10);
    PORTC = static_cast<uint8_t>(PORTC & ~kTrigBits[s]);
}

void storeResult(uint8_t s, ranging::Reading r)
{
    const uint8_t bit = static_cast<uint8_t>(1u << s);
    if (r.valid) {
        g_distanceMm[s] = r.distanceMm;
        g_validMask = static_cast<uint8_t>(g_validMask | bit);
    } else {
        g_distanceMm[s] = 0;
        g_validMask = static_cast<uint8_t>(g_validMask & ~bit);
    }
}

} // namespace

void begin()
{
    // Triggers are outputs, held low.
    DDRC |= kTrigMask;
    PORTC = static_cast<uint8_t>(PORTC & ~kTrigMask);

    // Echoes are inputs, pull-ups off.
    // PK0/PK1 are the pot and battery sense.
    DDRK = static_cast<uint8_t>(DDRK & ~kEchoMask);
    PORTK = static_cast<uint8_t>(PORTK & ~kEchoMask);

    // Timer1 normal mode, prescaler 8.
    // 0.5us per tick, wraps at 32.7ms.
    TCCR1A = 0;
    TCCR1B = _BV(CS11);
    TIMSK1 = 0;

    PCMSK2 = 0;
    PCICR |= _BV(PCIE2);

    g_sensor = 0;
    g_tickInSlot = 0;
    startPing(g_sensor);
}

void update()
{
    // Harvest as soon as the echo lands.
    if (!g_slotStored && g_haveEnd) {
        uint16_t start = 0;
        uint16_t end = 0;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
        {
            start = g_startTicks;
            end = g_endTicks;
        }
        const uint16_t ticks = static_cast<uint16_t>(end - start);
        storeResult(g_sensor, ranging::fromTicks(ticks, kOffsetMm[g_sensor]));
        g_slotStored = true;
    }

    if (++g_tickInSlot < kSlotTicks) {
        return;
    }
    g_tickInSlot = 0;

    // Silence is not an empty corridor. Report it as invalid.
    if (!g_slotStored) {
        storeResult(g_sensor, ranging::Reading{});
    }

    g_sensor = static_cast<uint8_t>((g_sensor + 1) % US_SENSOR_COUNT);
    startPing(g_sensor);
}

uint16_t distanceMm(Sensor s)
{
    return g_distanceMm[s];
}

uint8_t validMask()
{
    return g_validMask;
}

} // namespace ultrasonic
} // namespace hal

// Shared by all of PORTK.
// PCMSK2 admits only the active sensor.
ISR(PCINT2_vect)
{
    const uint16_t now = TCNT1;
    if (PINK & hal::ultrasonic::g_activeEcho) {
        hal::ultrasonic::g_startTicks = now;
        hal::ultrasonic::g_haveStart = true;
    } else if (hal::ultrasonic::g_haveStart) {
        hal::ultrasonic::g_endTicks = now;
        hal::ultrasonic::g_haveEnd = true;
        PCMSK2 = 0; // stop listening until the next ping
    }
}
