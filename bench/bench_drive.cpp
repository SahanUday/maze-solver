// ============================================================================
//  bench_drive.cpp  --  manual bring-up harness for the motors + encoders.
//  NOT the competition firmware: built only by `pio run -e bench_drive`.
//  Serial 250000 baud, newline-terminated commands:
//    e        enable drivers          x        disable (coast)
//    l <n>    left command            r <n>    right command   (+-kBenchMaxCmd)
//    b        both commands to 0 (brakes while enabled)
//    z        zero the printed counts ?        help
//  Drivers auto-disable if no command arrives for kTimeoutMs.
// ============================================================================

#include <Arduino.h>

#include "RobotConfig.h"
#include "RobotSpec.h"
#include "hal/encoders.h"
#include "hal/motors.h"

constexpr int16_t kBenchMaxCmd = 400; // about half of MOTOR_PWM_TOP
constexpr uint32_t kTimeoutMs = 5000;
constexpr uint32_t kPrintPeriodMs = 100;

static int16_t g_cmdL = 0;
static int16_t g_cmdR = 0;
static bool g_enabled = false;
static int32_t g_zeroL = 0;
static int32_t g_zeroR = 0;
static uint32_t g_lastCmdMs = 0;
static char g_line[24];
static uint8_t g_len = 0;

static void apply()
{
    motorSet(Motor::Left, g_cmdL);
    motorSet(Motor::Right, g_cmdR);
}

static void setEnabled(bool on)
{
    g_enabled = on;
    motorsEnable(on);
}

static void printHelp()
{
    Serial.println(F("e/x enable/disable | l <n> r <n> command | b brake | z zero counts"));
}

static void handleLine(char *line)
{
    g_lastCmdMs = millis();
    switch (line[0]) {
        case 'e':
            setEnabled(true);
            break;
        case 'x':
            setEnabled(false);
            break;
        case 'b':
            g_cmdL = g_cmdR = 0;
            apply();
            break;
        case 'l':
        case 'r': {
            const long n = constrain(strtol(line + 1, nullptr, 10), -kBenchMaxCmd, kBenchMaxCmd);
            (line[0] == 'l' ? g_cmdL : g_cmdR) = static_cast<int16_t>(n);
            apply();
            break;
        }
        case 'z':
            encodersRead(g_zeroL, g_zeroR);
            break;
        default:
            printHelp();
            break;
    }
}

void setup()
{
    Serial.begin(DEBUG_SERIAL_BAUD);
    motorsInit(); // EN low: motors coast until 'e'
    encodersInit();
    Serial.println(F("bench_drive: drivers DISABLED. Wheels off the ground before 'e'."));
    printHelp();
}

void loop()
{
    while (Serial.available() > 0) {
        const char c = static_cast<char>(Serial.read());
        if (c == '\n' || c == '\r') {
            if (g_len > 0) {
                g_line[g_len] = '\0';
                handleLine(g_line);
                g_len = 0;
            }
        } else if (g_len < sizeof(g_line) - 1) {
            g_line[g_len++] = c;
        }
    }

    const uint32_t now = millis();
    if (g_enabled && now - g_lastCmdMs > kTimeoutMs) {
        g_cmdL = g_cmdR = 0;
        apply();
        setEnabled(false);
        Serial.println(F("TIMEOUT: drivers disabled"));
    }

    static uint32_t lastPrintMs = 0;
    if (now - lastPrintMs >= kPrintPeriodMs) {
        lastPrintMs = now;
        int32_t l, r;
        encodersRead(l, r);
        Serial.print(F("L="));
        Serial.print(l - g_zeroL);
        Serial.print(F(" R="));
        Serial.print(r - g_zeroR);
        Serial.print(F(" cmdL="));
        Serial.print(g_cmdL);
        Serial.print(F(" cmdR="));
        Serial.print(g_cmdR);
        Serial.print(F(" en="));
        Serial.print(g_enabled ? 1 : 0);
        Serial.print(F(" | LA="));
        Serial.print(digitalRead(PIN_ENC_L_A));
        Serial.print(F(" LB="));
        Serial.print(digitalRead(PIN_ENC_L_B));
        Serial.print(F(" RA="));
        Serial.print(digitalRead(PIN_ENC_R_A));
        Serial.print(F(" RB="));
        Serial.println(digitalRead(PIN_ENC_R_B));
    }
}
