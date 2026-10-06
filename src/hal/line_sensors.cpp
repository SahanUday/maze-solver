// ============================================================================
//  line_sensors.cpp  --  8-channel digital IR array, direct PORTF access.
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

static_assert(IR_CHANNEL_COUNT == 8, "line_sensors reads one 8-bit port (PINF)");
static_assert(IR_DEBOUNCE_SAMPLES % 2 == 1, "IR_DEBOUNCE_SAMPLES must be odd");
static_assert(IR_DEBOUNCE_SAMPLES >= 1, "IR_DEBOUNCE_SAMPLES must be at least 1");
static_assert(PIN_IR[0] == A0 && PIN_IR[1] == A1 && PIN_IR[2] == A2 && PIN_IR[3] == A3 &&
                  PIN_IR[4] == A4 && PIN_IR[5] == A5 && PIN_IR[6] == A6 && PIN_IR[7] == A7,
              "line sensors must stay on PF0..PF7 (A0..A7) for one-byte PINF reads");
static_assert(PIN_IR_EMITTER == 36, "IR emitter enable must stay on PC1");

// A channel wins the vote with strictly more than this.
constexpr uint8_t kVoteThreshold = IR_DEBOUNCE_SAMPLES / 2;

// PIN_IR_EMITTER, Arduino pin 36, is PC1.
constexpr uint8_t kEmitterBit = _BV(PC1);

} // namespace

void setEmitters(bool on)
{
    // Read-modify-write: PORTC 7-2 are the ultrasonic pins.
    if (on) {
        PORTC |= kEmitterBit;
    } else {
        PORTC = static_cast<uint8_t>(PORTC & ~kEmitterBit);
    }
}

void begin()
{
    DDRF = 0x00;  // all 8 channels are inputs
    PORTF = 0x00; // internal pull-ups off

    DDRC |= kEmitterBit; // emitter enable is an output
    setEmitters(true);
}

uint8_t read()
{
    uint8_t highCount[IR_CHANNEL_COUNT] = {0};

    for (uint8_t sample = 0; sample < IR_DEBOUNCE_SAMPLES; ++sample) {
        if (sample != 0) {
            _delay_us(IR_DEBOUNCE_SPACING_US);
        }
        const uint8_t levels = PINF; // all 8 channels, one instruction
        for (uint8_t ch = 0; ch < IR_CHANNEL_COUNT; ++ch) {
            highCount[ch] = static_cast<uint8_t>(highCount[ch] + ((levels >> ch) & 1u));
        }
    }

    uint8_t voted = 0;
    for (uint8_t ch = 0; ch < IR_CHANNEL_COUNT; ++ch) {
        if (highCount[ch] > kVoteThreshold) {
            voted = static_cast<uint8_t>(voted | (1u << ch));
        }
    }

    return voted;
}

} // namespace line_sensors
} // namespace hal
