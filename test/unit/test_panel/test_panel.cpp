// ============================================================================
//  test_panel.cpp  --  the DIP decode, the LED patterns and the start rules of the operator
//  panel (include/panel/). Pure logic, no hardware needed.
// ============================================================================

#include <stdio.h>
#include <unity.h>

#include "panel/PanelLeds.h"
#include "panel/PanelMode.h"
#include "panel/PanelStart.h"
#include "spec/fixed/Hardware.h"

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
        // Countdown after the press: the status LED blinks fast, the error LED is off.
        {PanelRun::Countdown, PanelMode::Explore, true, 0, true, false},
        {PanelRun::Countdown, PanelMode::Explore, false, 100, false, false},
        {PanelRun::Countdown, PanelMode::SpeedSafe, false, 200, true, false},
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

// ---- the start: PanelStart ------------------------------------------------------------------

constexpr uint32_t TICK_MS = CONTROL_LOOP_PERIOD_MS;

// `n` ticks with the button down or up and the DIP at `mode`; `t` moves on.
static void ticks(PanelStart &ps, uint32_t &t, uint32_t n, bool down,
                  PanelMode mode = PanelMode::Explore)
{
    for (uint32_t i = 0; i < n; ++i, t += TICK_MS) {
        panelStartTick(ps, t, down, mode);
    }
}

// No button wired: the first tick on a setting that can run is accepted and `go` follows the delay.
static void test_without_a_button_go_follows_the_delay()
{
    PanelStart ps(false);
    uint32_t t = 1500; // the gyro zeroing took this long
    // A floating pin reading low must not matter.
    panelStartTick(ps, t, true, PanelMode::Explore);
    TEST_ASSERT_TRUE(ps.counting);
    const uint32_t acceptedAt = t;
    for (t += TICK_MS; t < acceptedAt + PANEL_START_DELAY_MS; t += TICK_MS) {
        panelStartTick(ps, t, false, PanelMode::Explore);
        TEST_ASSERT_FALSE(ps.go);
    }
    panelStartTick(ps, t, false, PanelMode::Explore);
    TEST_ASSERT_TRUE(ps.go);

    // A setting that is not built is never accepted; the delay runs from the first that is.
    PanelStart refused(false);
    t = 0;
    ticks(refused, t, 300, false, PanelMode::SpeedSafe);
    TEST_ASSERT_FALSE(refused.counting);
    panelStartTick(refused, t, false, PanelMode::Explore);
    TEST_ASSERT_TRUE(refused.counting);
    TEST_ASSERT_EQUAL_UINT32(t, refused.acceptedAtMs);
}

// A press counts after DEBOUNCE ticks in a row, and only after the button has been up.
static void test_a_press_needs_a_release_and_debounce()
{
    PanelStart ps(true);
    uint32_t t = 0;
    ticks(ps, t, 100, false);
    TEST_ASSERT_FALSE(ps.counting); // no press, nothing happens
    for (uint32_t i = 0; i < 40; ++i, t += TICK_MS) {
        panelStartTick(ps, t, i % 3 == 2, PanelMode::Explore); // never DEBOUNCE in a row
    }
    TEST_ASSERT_FALSE(ps.counting);

    PanelStart held(true); // down since boot: a stuck or shorted line
    t = 0;
    ticks(held, t, 50, true);
    TEST_ASSERT_FALSE(held.counting);
    ticks(held, t, 1, false);
    ticks(held, t, PANEL_START_DEBOUNCE_TICKS - 1, true);
    TEST_ASSERT_FALSE(held.counting);
    ticks(held, t, 1, true);
    TEST_ASSERT_TRUE(held.counting); // on the press, not the release
}

// An accepted press counts down, then `go`; releasing and the DIP change nothing from then on.
static void test_an_accepted_press_counts_down_then_goes()
{
    PanelStart ps(true);
    uint32_t t = 0;
    ticks(ps, t, 1, false);
    ticks(ps, t, PANEL_START_DEBOUNCE_TICKS, true);
    TEST_ASSERT_TRUE(ps.counting);
    const uint32_t acceptedAt = ps.acceptedAtMs;
    for (; t < acceptedAt + PANEL_START_DELAY_MS; t += TICK_MS) {
        panelStartTick(ps, t, false, PanelMode::SpeedFast); // released, the DIP moved
        TEST_ASSERT_FALSE(ps.go);
    }
    panelStartTick(ps, t, true, PanelMode::Calibrate);
    TEST_ASSERT_TRUE(ps.go);
}

// A press on a setting that is not built is used up: it needs a release and a new press.
static void test_a_refused_setting_uses_up_the_press()
{
    PanelStart ps(true);
    uint32_t t = 0;
    ticks(ps, t, 1, false);
    ticks(ps, t, 10, true, PanelMode::SpeedSafe);
    ticks(ps, t, 10, true, PanelMode::Explore); // fixed while still held
    TEST_ASSERT_FALSE(ps.counting);
    ticks(ps, t, 1, false);
    ticks(ps, t, PANEL_START_DEBOUNCE_TICKS, true);
    TEST_ASSERT_TRUE(ps.counting);
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_decode_all_sixteen_settings);
    RUN_TEST(test_led_patterns);
    RUN_TEST(test_code_blinks_n_times_per_cycle);
    RUN_TEST(test_without_a_button_go_follows_the_delay);
    RUN_TEST(test_a_press_needs_a_release_and_debounce);
    RUN_TEST(test_an_accepted_press_counts_down_then_goes);
    RUN_TEST(test_a_refused_setting_uses_up_the_press);
    return UNITY_END();
}
