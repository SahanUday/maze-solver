// ============================================================================
//  Bench probe: the IR array, motors off. Measures the raw channels, captures a calibration the way
//  the mission would, and shows the live line view. Not part of the mission build.
//
//  Commands (one line, end with newline):
//    h                  help
//    p                  print the settings
//    set pitch <mm>     channel pitch used for the centroid (default: IR_SENSOR_PITCH_MM, or 8)
//    m <label>          measure ~1.5 s standing still: per channel mean, sd, min, max, and tilt
//    k                  start a calibration capture: slide the array over white AND black
//    f                  finish the capture: prints each channel's white, black, range, scale
//    v                  live view (5 Hz): raw, normalised, mask, line view, tilt. Any byte stops it
//    x                  stop whatever is running
// ============================================================================

#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

#include "LineFollow.h"
#include "LineSense.h"
#include "RobotConfig.h"
#include "RobotSpec.h"
#include "RobotState.h"
#include "Scheduler.h"
#include "Tilt.h"
#include "hal/imu.h"
#include "hal/line_sensors.h"
#include "hal/motors.h"

namespace
{

constexpr uint16_t MEASURE_TICKS = 150; // 1.5 s at 10 ms

LineCalibration g_cal;
bool g_haveCal = false;
bool g_capturing = false;
uint8_t g_usable = 0;
uint8_t g_mask = 0;
float g_pitch = IR_SENSOR_PITCH_MM > 0.0f ? IR_SENSOR_PITCH_MM : 8.0f;
bool g_imuPresent = false;
float g_tilt = 0.0f;

bool g_measuring = false;
uint16_t g_n = 0;
uint32_t g_sum[IR_CHANNEL_COUNT];
uint32_t g_sumSq[IR_CHANNEL_COUNT];
uint16_t g_min[IR_CHANNEL_COUNT];
uint16_t g_max[IR_CHANNEL_COUNT];
float g_tiltSum = 0.0f;
char g_label[16] = "";

bool g_view = false;
uint8_t g_viewTick = 0;

void startMeasure(const char *label)
{
    strncpy(g_label, label != nullptr ? label : "", sizeof(g_label) - 1);
    g_label[sizeof(g_label) - 1] = '\0';
    for (uint8_t i = 0; i < IR_CHANNEL_COUNT; ++i) {
        g_sum[i] = 0;
        g_sumSq[i] = 0;
        g_min[i] = 0xFFFF;
        g_max[i] = 0;
    }
    g_n = 0;
    g_tiltSum = 0.0f;
    g_measuring = true;
}

void finishMeasure()
{
    g_measuring = false;
    Serial.print(F("# measure "));
    Serial.print(g_label);
    Serial.print(F(" n="));
    Serial.print(g_n);
    Serial.print(F(" tilt="));
    Serial.println(g_tiltSum / static_cast<float>(g_n), 2);
    Serial.println(F("# ch,mean,sd,min,max"));
    for (uint8_t i = 0; i < IR_CHANNEL_COUNT; ++i) {
        const float mean = static_cast<float>(g_sum[i]) / static_cast<float>(g_n);
        const float var = static_cast<float>(g_sumSq[i]) / static_cast<float>(g_n) - mean * mean;
        Serial.print(i);
        Serial.print(',');
        Serial.print(mean, 1);
        Serial.print(',');
        Serial.print(var > 0.0f ? sqrtf(var) : 0.0f, 2);
        Serial.print(',');
        Serial.print(g_min[i]);
        Serial.print(',');
        Serial.println(g_max[i]);
    }
    Serial.println(F("# done"));
}

void finishCapture()
{
    g_capturing = false;
    g_usable = lineCalibrationFinish(g_cal);
    g_haveCal = true;
    g_mask = 0;
    Serial.println(F("# calibration: ch,white(low),black(high),range,scale,usable"));
    for (uint8_t i = 0; i < IR_CHANNEL_COUNT; ++i) {
        Serial.print(i);
        Serial.print(',');
        Serial.print(g_cal.white[i]);
        Serial.print(',');
        Serial.print(g_cal.black[i]);
        Serial.print(',');
        Serial.print(static_cast<int>(g_cal.black[i]) - static_cast<int>(g_cal.white[i]));
        Serial.print(',');
        Serial.print(g_cal.scale[i]);
        Serial.print(',');
        Serial.println((g_usable >> i) & 1u);
    }
    Serial.print(F("# usable mask 0x"));
    Serial.println(g_usable, HEX);
}

void printSettings()
{
    Serial.print(F("# pitch="));
    Serial.print(g_pitch, 1);
    Serial.print(F(" mm  ride height="));
    Serial.print(IR_RIDE_HEIGHT_MM, 1);
    Serial.print(F(" mm  mask on/off="));
    Serial.print(LINE_MASK_ON_PCT);
    Serial.print('/');
    Serial.print(LINE_MASK_OFF_PCT);
    Serial.print(F("%  min span="));
    Serial.print(LINE_CAL_MIN_SPAN_COUNTS);
    Serial.print(F("  imu="));
    Serial.print(g_imuPresent ? F("ok") : F("MISSING"));
    Serial.print(F("  calibration="));
    Serial.println(g_haveCal ? F("captured") : F("none"));
}

void help()
{
    Serial.println(F("# h | p | set pitch <mm> | m <label> | k | f | v | x"));
}

void handleLine(char *line)
{
    char *cmd = strtok(line, " \t");
    if (cmd == nullptr) {
        return;
    }
    if (strcmp(cmd, "h") == 0) {
        help();
    } else if (strcmp(cmd, "p") == 0) {
        printSettings();
    } else if (strcmp(cmd, "x") == 0) {
        g_view = false;
        g_measuring = false;
        g_capturing = false;
        Serial.println(F("# idle"));
    } else if (strcmp(cmd, "set") == 0) {
        const char *key = strtok(nullptr, " \t");
        const char *val = strtok(nullptr, " \t");
        if (key != nullptr && val != nullptr && strcmp(key, "pitch") == 0) {
            g_pitch = static_cast<float>(atof(val));
            printSettings();
        } else {
            Serial.println(F("# usage: set pitch <mm>"));
        }
    } else if (strcmp(cmd, "m") == 0) {
        startMeasure(strtok(nullptr, " \t"));
        Serial.println(F("# measuring..."));
    } else if (strcmp(cmd, "k") == 0) {
        lineCalibrationReset(g_cal);
        g_capturing = true;
        Serial.println(F("# capturing: slide over white AND black, then f"));
    } else if (strcmp(cmd, "f") == 0) {
        if (g_capturing) {
            finishCapture();
        } else {
            Serial.println(F("# no capture running (k first)"));
        }
    } else if (strcmp(cmd, "v") == 0) {
        g_view = true;
        g_viewTick = 0;
        Serial.println(F("# view: raw | normalised | mask | line view | tilt   (any byte stops)"));
    } else {
        Serial.println(F("# unknown command (h for help)"));
    }
}

void pollSerial()
{
    static char line[40];
    static uint8_t len = 0;
    while (Serial.available() > 0) {
        const char ch = static_cast<char>(Serial.read());
        if (g_view) {
            g_view = false;
            Serial.println(F("# view stopped"));
            len = 0;
            continue;
        }
        if (ch == '\n' || ch == '\r') {
            line[len] = '\0';
            len = 0;
            handleLine(line);
        } else if (len < sizeof(line) - 1) {
            line[len++] = ch;
        }
    }
}

void printView(const uint16_t (&raw)[IR_CHANNEL_COUNT], const uint8_t (&norm)[IR_CHANNEL_COUNT],
               const LineView &v)
{
    Serial.print(F("raw"));
    for (uint8_t i = 0; i < IR_CHANNEL_COUNT; ++i) {
        Serial.print(' ');
        Serial.print(raw[i]);
    }
    Serial.print(F(" | n"));
    for (uint8_t i = 0; i < IR_CHANNEL_COUNT; ++i) {
        Serial.print(' ');
        Serial.print(norm[i]);
    }
    Serial.print(F(" | "));
    for (uint8_t i = 0; i < IR_CHANNEL_COUNT; ++i) {
        Serial.print(((g_mask >> i) & 1u) ? '#' : '.');
    }
    Serial.print(F(" | "));
    Serial.print(v.allBlack ? F("allBlack") : (v.allWhite ? F("allWhite") : F("onLine")));
    Serial.print(F(" err="));
    Serial.print(v.errMm, 1);
    Serial.print(F(" | tilt="));
    Serial.println(g_tilt, 1);
}

} // namespace

void setup()
{
    Serial.begin(DEBUG_SERIAL_BAUD);
    motorsInit(); // EN low: the wheels stay off
    lineSensorsInit();
    g_imuPresent = imuInit();
    lineCalibrationReset(g_cal);
    Serial.println(F("# IR probe ready - motors off. h for help"));
    printSettings();
}

void loop()
{
    static uint32_t nextTickMs = 0;
    const uint32_t nowMs = millis();
    if (!tickDue(nowMs, nextTickMs, CONTROL_LOOP_PERIOD_MS)) {
        return;
    }

    uint16_t raw[IR_CHANNEL_COUNT];
    lineSensorsRead(raw);
    if (g_imuPresent) {
        ImuRaw imu;
        if (imuRead(imu)) {
            g_tilt = tiltDeg(imu);
        }
    }
    pollSerial();

    if (g_capturing) {
        lineCalibrationAccumulate(g_cal, raw);
    }
    if (g_measuring) {
        for (uint8_t i = 0; i < IR_CHANNEL_COUNT; ++i) {
            g_sum[i] += raw[i];
            g_sumSq[i] += static_cast<uint32_t>(raw[i]) * raw[i];
            g_min[i] = raw[i] < g_min[i] ? raw[i] : g_min[i];
            g_max[i] = raw[i] > g_max[i] ? raw[i] : g_max[i];
        }
        g_tiltSum += g_tilt;
        if (++g_n >= MEASURE_TICKS) {
            finishMeasure();
        }
    }
    if (g_view && ++g_viewTick >= 20) {
        g_viewTick = 0;
        uint8_t norm[IR_CHANNEL_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0};
        if (g_haveCal) {
            lineNormalize(g_cal, raw, norm);
            g_mask = lineMaskUpdate(norm, g_mask);
        }
        printView(raw, norm, lineView(norm, g_mask, g_usable, g_pitch, IR_D1_ON_RIGHT));
    }
}
