// ============================================================================
//  encoders.cpp  --  4x quadrature on four external-interrupt pins.
//  Each motor's A/B share one port (A on the lower bit), so every ISR takes a
//  single PINx snapshot of both channels and looks up the step from the last
//  state. See docs/architecture/modules/encoders.md.
// ============================================================================

#include "hal/encoders.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/atomic.h>

#include "Quadrature.h"
#include "RobotConfig.h"
#include "RobotSpec.h"

// The register code below is hand-mapped to these pins.
static_assert(PIN_ENC_L_A == 2 && PIN_ENC_L_B == 3, "left encoder must be PE4/PE5 (INT4/INT5)");
static_assert(PIN_ENC_R_A == 19 && PIN_ENC_R_B == 18, "right encoder must be PD2/PD3 (INT2/INT3)");

static volatile int32_t g_countL = 0;
static volatile int32_t g_countR = 0;
static volatile uint8_t g_stateL = 0;
static volatile uint8_t g_stateR = 0;

// (B << 1) | A
static inline uint8_t readStateL()
{
    return (PINE >> PE4) & 3;
}
static inline uint8_t readStateR()
{
    return (PIND >> PD2) & 3;
}

static inline void stepLeft()
{
    const uint8_t s = readStateL();
    const int8_t d = quadratureDelta(g_stateL, s);
    g_countL = g_countL + (ENC_L_INVERT ? -d : d);
    g_stateL = s;
}

static inline void stepRight()
{
    const uint8_t s = readStateR();
    const int8_t d = quadratureDelta(g_stateR, s);
    g_countR = g_countR + (ENC_R_INVERT ? -d : d);
    g_stateR = s;
}

ISR(INT4_vect)
{
    stepLeft();
}
ISR(INT5_vect)
{
    stepLeft();
}
ISR(INT2_vect)
{
    stepRight();
}
ISR(INT3_vect)
{
    stepRight();
}

void encodersInit()
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        DDRE &= ~((1 << PE4) | (1 << PE5));
        PORTE |= (1 << PE4) | (1 << PE5);
        DDRD &= ~((1 << PD2) | (1 << PD3));
        PORTD |= (1 << PD2) | (1 << PD3);

        // ISCn1:0 = 01 -> any logical change. INT7:4 live in EICRB, INT3:0 in EICRA.
        EICRB = (EICRB & ~((1 << ISC41) | (1 << ISC51))) | (1 << ISC40) | (1 << ISC50);
        EICRA = (EICRA & ~((1 << ISC21) | (1 << ISC31))) | (1 << ISC20) | (1 << ISC30);

        __builtin_avr_delay_cycles(160); // 10 us for the pull-ups to settle before sampling
        g_stateL = readStateL();
        g_stateR = readStateR();
        g_countL = 0;
        g_countR = 0;

        EIFR = (1 << INTF2) | (1 << INTF3) | (1 << INTF4) | (1 << INTF5);
        EIMSK |= (1 << INT2) | (1 << INT3) | (1 << INT4) | (1 << INT5);
    }
}

void encodersRead(int32_t &left, int32_t &right)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        left = g_countL;
        right = g_countR;
    }
}
