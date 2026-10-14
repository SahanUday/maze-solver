// ============================================================================
//  line_cal_store.cpp  --  see line_cal_store.h.
// ============================================================================

#include "line_cal_store.h"

#include <EEPROM.h>
#include <stddef.h>

namespace
{

constexpr uint16_t MAGIC = 0x4C43; // "LC"
constexpr int ADDRESS = 0;

struct Stored {
    uint16_t magic;
    LineCalibration cal;
    uint16_t check;
};

// Sum of every byte before the check field; enough to notice a blank (0xFF) or torn write.
uint16_t checksum(const Stored &s)
{
    const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&s);
    uint16_t sum = 0;
    for (size_t i = 0; i < offsetof(Stored, check); ++i) {
        sum = static_cast<uint16_t>(sum + bytes[i] + 1);
    }
    return sum;
}

} // namespace

bool lineCalLoad(LineCalibration &cal)
{
    Stored s;
    EEPROM.get(ADDRESS, s);
    if (s.magic != MAGIC || s.check != checksum(s)) {
        return false;
    }
    for (uint8_t ch = 0; ch < IR_CHANNEL_COUNT; ++ch) {
        if (s.cal.scale[ch] != 0) {
            cal = s.cal;
            return true;
        }
    }
    return false;
}

void lineCalSave(const LineCalibration &cal)
{
    Stored s;
    s.magic = MAGIC;
    s.cal = cal;
    s.check = checksum(s);
    EEPROM.put(ADDRESS, s);
}
