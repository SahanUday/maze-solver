// ============================================================================
//  test_panel.cpp  --  the DIP decode and the LED patterns of the operator panel
//  (include/panel/). Pure logic, no hardware needed.
// ============================================================================

#include <stdio.h>
#include <unity.h>

#include "panel/PanelLeds.h"
#include "panel/PanelControls.h"

// Every setting of the four positions (bit 0 RUN, 1 FAST, 2 CAL, 3 RESERVED) and its mode.
static void test_decode_all_sixteen_settings()
{
    static const PanelMode expected[16] = {
        PanelMode::Explore,   PanelMode::SpeedSafe, PanelMode::Explore, PanelMode::SpeedFast,
        PanelMode::Calibrate, PanelMode::Refused,   PanelMode::Refused, PanelMode::Refused,
        PanelMode::Refused,   PanelMode::Refused,   PanelMode::Refused, PanelMode::Refused,
        PanelMode::Refused,   PanelMode::Refused,   PanelMode::Refused, PanelMode::Refused};
    for (uint8_t dip = 0; dip < 16; ++dip) {
        char msg[32];
        snprintf(msg, sizeof msg, "dip %u", dip);
        TEST_ASSERT_EQUAL_MESSAGE(static_cast<int>(expected[dip]),
                                  static_cast<int>(panelDecode(dip)), msg);
        // Bits above the fourth are not switches.
        TEST_ASSERT_EQUAL_MESSAGE(static_cast<int>(expected[dip]),
                                  static_cast<int>(panelDecode(static_cast<uint8_t>(dip | 0xA0))),
                                  msg);
    }

    // The blink code and what is built: only explore is, so the rest show as refused for now.
    static const struct {
        PanelMode mode;
        uint8_t code;
        bool built;
    } modes[] = {{PanelMode::Explore, 1, true},
                 {PanelMode::SpeedSafe, 2, false},
                 {PanelMode::SpeedFast, 3, false},
                 {PanelMode::Calibrate, 4, false},
                 {PanelMode::Refused, 0, false}};
    for (const auto &m : modes) {
        TEST_ASSERT_EQUAL_UINT8(m.code, panelModeCode(m.mode));
        TEST_ASSERT_EQUAL(m.built, panelModeBuilt(m.mode));
    }
}

// What the two LEDs show, at times that fall on and off the edges of each pattern.
static void test_led_patterns()
{
    static const struct {
        PanelRun run;
        PanelMode mode;
        bool noCal;
        uint32_t nowMs;
        bool status;
        bool error;
    } rows[] = {
        // Waiting, explore (CODE 1: on 0-149, off to 399, pause to 1599, repeat).
        {PanelRun::Waiting, PanelMode::Explore, false, 0, true, false},
        {PanelRun::Waiting, PanelMode::Explore, false, 149, true, false},
        {PanelRun::Waiting, PanelMode::Explore, false, 150, false, false},
        {PanelRun::Waiting, PanelMode::Explore, false, 400, false, false},
        {PanelRun::Waiting, PanelMode::Explore, false, 1599, false, false},
        {PanelRun::Waiting, PanelMode::Explore, false, 1600, true, false},
        // The same with no usable calibration: the error LED beats (500 ms on, 500 ms off).
        {PanelRun::Waiting, PanelMode::Explore, true, 0, true, true},
        {PanelRun::Waiting, PanelMode::Explore, true, 500, false, false},
        {PanelRun::Waiting, PanelMode::Explore, true, 1000, false, true},
        // Waiting with a refused or not-built mode: status off, error FAST BLINK.
        {PanelRun::Waiting, PanelMode::Refused, false, 0, false, true},
        {PanelRun::Waiting, PanelMode::Refused, false, 100, false, false},
        {PanelRun::Waiting, PanelMode::SpeedSafe, false, 99, false, true},
        {PanelRun::Waiting, PanelMode::Calibrate, true, 200, false, true},
        // Running: the status LED beats, the error LED is off.
        {PanelRun::Running, PanelMode::Explore, false, 0, true, false},
        {PanelRun::Running, PanelMode::Explore, true, 499, true, false},
        {PanelRun::Running, PanelMode::Explore, false, 500, false, false},
        // Ended: SOLID, and the error LED too when it ended badly, at any time.
        {PanelRun::EndedOk, PanelMode::Explore, false, 0, true, false},
        {PanelRun::EndedOk, PanelMode::Explore, true, 777, true, false},
        {PanelRun::EndedBad, PanelMode::Explore, false, 0, true, true},
        {PanelRun::EndedBad, PanelMode::Refused, true, 12345, true, true},
    };
    for (unsigned i = 0; i < sizeof rows / sizeof rows[0]; ++i) {
        const auto &r = rows[i];
        PanelView view;
        view.run = r.run;
        view.mode = r.mode;
        view.noCalibration = r.noCal;
        const PanelLeds leds = panelLeds(view, r.nowMs);
        char msg[32];
        snprintf(msg, sizeof msg, "row %u (t=%lu)", i, static_cast<unsigned long>(r.nowMs));
        TEST_ASSERT_EQUAL_MESSAGE(r.status, leds.status, msg);
        TEST_ASSERT_EQUAL_MESSAGE(r.error, leds.error, msg);
    }
}

// CODE N shows N separate blinks per cycle, each PANEL_CODE_ON_MS long; CODE 0 shows none.
static void test_code_blinks_n_times_per_cycle()
{
    for (uint8_t n = 0; n <= 4; ++n) {
        const uint32_t cycleMs = n * (PANEL_CODE_ON_MS + PANEL_CODE_OFF_MS) + PANEL_CODE_PAUSE_MS;
        bool previous = false;
        uint32_t blinks = 0;
        uint32_t onMs = 0;
        for (uint32_t t = 0; t < cycleMs; ++t) {
            const bool on = panelCode(t, n);
            blinks += (on && !previous) ? 1 : 0;
            onMs += on ? 1 : 0;
            previous = on;
        }
        TEST_ASSERT_EQUAL_UINT32(n, blinks);
        TEST_ASSERT_EQUAL_UINT32(n * PANEL_CODE_ON_MS, onMs);
    }
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_decode_all_sixteen_settings);
    RUN_TEST(test_led_patterns);
    RUN_TEST(test_code_blinks_n_times_per_cycle);
    return UNITY_END();
}
