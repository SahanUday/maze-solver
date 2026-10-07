// ============================================================================
//  imu.cpp  --  MPU-6050 over Wire.h. One burst read per tick, raw counts out.
//  docs/architecture/modules/imu.md
// ============================================================================

#include "hal/imu.h"

#include <Wire.h>
#include <avr/io.h>
#include <util/delay.h>

#include "RobotConfig.h"
#include "RobotSpec.h"

namespace
{

// imuInit() clears the AVR's internal I2C pull-ups through PORTD, so the pin map
// is this module's business even though Wire.h drives the bus.
static_assert(PIN_I2C_SDA == 20, "I2C SDA must be pin 20 (PD1) - imu clears its pull-up in PORTD");
static_assert(PIN_I2C_SCL == 21, "I2C SCL must be pin 21 (PD0) - imu clears its pull-up in PORTD");

// Register addresses (InvenSense MPU-6000/6050 register map).
constexpr uint8_t kRegSmplrtDiv = 0x19;
constexpr uint8_t kRegConfig = 0x1A;
constexpr uint8_t kRegGyroConfig = 0x1B;
constexpr uint8_t kRegAccelConfig = 0x1C;
constexpr uint8_t kRegAccelXoutH = 0x3B; // first of the 14 contiguous data bytes
constexpr uint8_t kRegPwrMgmt1 = 0x6B;
constexpr uint8_t kRegWhoAmI = 0x75;

constexpr uint8_t kWhoAmIValue = 0x68;
// Clears SLEEP (the chip boots asleep and reads zeros until then) and selects the
// gyro's own PLL over the internal oscillator, as the register map recommends.
constexpr uint8_t kPwrMgmt1ClockPllGyroX = 0x01;
constexpr uint8_t kConfigDlpf20Hz = 0x04;   // 20 Hz gyro / 21 Hz accel, 1 kHz internal rate
constexpr uint8_t kGyroConfig500Dps = 0x08; // FS_SEL = 1
constexpr uint8_t kAccelConfig2G = 0x00;    // AFS_SEL = 0
constexpr uint8_t kSmplrtDivNone = 0x00;    // keep the internal 1 kHz
constexpr uint8_t kBurstBytes = 14;         // accel[3], temp, gyro[3], 2 bytes each
constexpr uint8_t kValues = kBurstBytes / 2;

static_assert(IMU_GYRO_AXIS_YAW < 3, "IMU_GYRO_AXIS_YAW indexes ImuRaw::gyro");
static_assert(kBurstBytes <= BUFFER_LENGTH, "the burst must fit one Wire transfer");
// _delay_ms() below is computed from the F_CPU macro.
static_assert(CPU_HZ == F_CPU, "CPU_HZ in RobotSpec.h must match the F_CPU the build uses");

bool writeRegister(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(IMU_I2C_ADDRESS);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

bool readRegister(uint8_t reg, uint8_t &value)
{
    Wire.beginTransmission(IMU_I2C_ADDRESS);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) { // false = repeated start, keep the bus
        return false;
    }
    if (Wire.requestFrom(IMU_I2C_ADDRESS, static_cast<uint8_t>(1)) != 1) {
        return false;
    }
    value = static_cast<uint8_t>(Wire.read());
    return true;
}

} // namespace

bool imuInit()
{
    Wire.begin();

    // The MPU-6050's logic pins are not 5 V tolerant and the breakout's own 4.7k
    // pull-ups go to its 3.3 V rail, but Wire.begin() switches the AVR's internal
    // pull-ups on, pulling the bus toward 5 V. digitalWrite() is banned in src/hal
    // (ADR 0001) and the register write is the right form anyway. encoders.cpp also
    // read-modify-writes PORTD (PD2/PD3): both run once at init, in sequence, on
    // disjoint bits, and no ISR writes PORTD.
    PORTD = static_cast<uint8_t>(PORTD & ~(_BV(PD0) | _BV(PD1)));

    Wire.setClock(IMU_I2C_CLOCK_HZ);

    uint8_t who = 0;
    if (!readRegister(kRegWhoAmI, who) || who != kWhoAmIValue) {
        return false;
    }
    if (!writeRegister(kRegPwrMgmt1, kPwrMgmt1ClockPllGyroX)) {
        return false;
    }
    // Gyro start-up, and time for the PLL just selected to lock, before the
    // configuration below and the first read.
    _delay_ms(IMU_STARTUP_DELAY_MS);

    return writeRegister(kRegConfig, kConfigDlpf20Hz) &&
           writeRegister(kRegGyroConfig, kGyroConfig500Dps) &&
           writeRegister(kRegAccelConfig, kAccelConfig2G) &&
           writeRegister(kRegSmplrtDiv, kSmplrtDivNone);
}

bool imuRead(ImuRaw &out)
{
    Wire.beginTransmission(IMU_I2C_ADDRESS);
    Wire.write(kRegAccelXoutH);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }
    // One transaction for all 14 bytes: the chip holds its data registers still for
    // the duration of a single read, so separate reads can straddle two samples and
    // return a combination that never physically occurred.
    if (Wire.requestFrom(IMU_I2C_ADDRESS, kBurstBytes) != kBurstBytes) {
        return false;
    }

    int16_t value[kValues];
    for (uint8_t i = 0; i < kValues; ++i) {
        const uint8_t high = static_cast<uint8_t>(Wire.read()); // big-endian on the wire
        const uint8_t low = static_cast<uint8_t>(Wire.read());
        value[i] = static_cast<int16_t>((static_cast<uint16_t>(high) << 8) | low);
    }

    // Published only once every byte is in, so a caller never sees a half-written sample.
    out.accel[0] = value[0];
    out.accel[1] = value[1];
    out.accel[2] = value[2];
    out.temp = value[3];
    out.gyro[0] = value[4];
    out.gyro[1] = value[5];
    out.gyro[2] = value[6];
    return true;
}
