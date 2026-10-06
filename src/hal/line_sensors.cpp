// ============================================================================
//  line_sensors.cpp  --  8-channel analog IR array, direct ADC register access.
//  docs/architecture/modules/line_sensors.md
// ============================================================================

#include "line_sensors.h"

#include <avr/io.h>
#include <util/delay.h>

#include "RobotConfig.h"
#include "RobotSpec.h"

namespace hal
{
namespace line_sensors
{

namespace
{

// The register code below is hand-mapped to these pins.
constexpr bool irChannelsAreAdc0To7(const uint8_t (&pins)[IR_CHANNEL_COUNT])
{
    for (uint8_t i = 0; i < IR_CHANNEL_COUNT; ++i) {
        if (pins[i] != A0 + i) {
            return false;
        }
    }
    return true;
}
static_assert(IR_CHANNEL_COUNT == 8, "line_sensors owns all of PORTF = ADC0..ADC7");
static_assert(irChannelsAreAdc0To7(PIN_IR), "IR channels must be A0..A7 (ADC0..ADC7), in order");
static_assert(PIN_IR_EMITTER == 36, "IR emitter enable must be pin 36 (PC1)");

constexpr uint8_t kEmitterBit = _BV(PC1);

// ADPS2:0 is log2 of the ADC clock division factor (datasheet table 127).
constexpr uint8_t adcPrescalerBits(uint8_t divisor)
{
    uint8_t bits = 0;
    while ((1u << bits) < divisor) {
        ++bits;
    }
    return bits;
}
static_assert(IR_ADC_PRESCALER >= 2 && IR_ADC_PRESCALER <= 128 &&
                  (IR_ADC_PRESCALER & (IR_ADC_PRESCALER - 1)) == 0,
              "IR_ADC_PRESCALER must be a power of two from 2 to 128");
// Only validated up to 1 MHz on the bench (decisions/0005).
static_assert(F_CPU / IR_ADC_PRESCALER <= 1000000UL,
              "IR_ADC_PRESCALER gives an ADC clock above 1 MHz");

constexpr uint8_t kAdcPrescalerBits = adcPrescalerBits(IR_ADC_PRESCALER);

inline uint16_t convert(uint8_t channel)
{
    // Every conversion picks its own channel group: the Arduino core's analogRead() leaves
    // MUX5 set after reading A8-A15 (PIN_POT, say), which would silently turn this into ADC8-15.
    ADCSRB = static_cast<uint8_t>(ADCSRB & ~_BV(MUX5));
    ADMUX = _BV(REFS0) | channel; // AVcc reference, right-adjusted, MUX4:0 = channel
    ADCSRA |= _BV(ADSC);
    while (ADCSRA & _BV(ADSC)) {
    }
    return ADC;
}

} // namespace

void setEmitters(bool on)
{
    const bool isOn = (PORTC & kEmitterBit) != 0;
    if (on == isOn) {
        return;
    }
    // Read-modify-write: PORTC 7-2 are the ultrasonic pins. A single-bit change to a low I/O
    // register compiles to one sbi/cbi, so an ISR can't land in the middle of it.
    if (on) {
        PORTC |= kEmitterBit;
    } else {
        PORTC = static_cast<uint8_t>(PORTC & ~kEmitterBit);
    }
    _delay_us(IR_EMITTER_SETTLE_US);
}

void begin()
{
    DDRF = 0x00;  // all 8 channels are inputs
    PORTF = 0x00; // pull-ups off: the module has its own, ours would shift every reading ~20%
    DIDR0 = 0xFF; // analog-only: no digital input buffer on ADC0-7

    ADCSRA = _BV(ADEN) | kAdcPrescalerBits;
    convert(0); // first conversion after ADEN is longer and the result is not trusted

    // PORTC first so the pin goes straight to HIGH, not through a LOW glitch.
    PORTC |= kEmitterBit;
    DDRC |= kEmitterBit;
    _delay_us(IR_EMITTER_SETTLE_US);
}

void read(uint16_t (&counts)[IR_CHANNEL_COUNT])
{
    for (uint8_t ch = 0; ch < IR_CHANNEL_COUNT; ++ch) {
        counts[ch] = convert(ch);
    }
}

} // namespace line_sensors
} // namespace hal
