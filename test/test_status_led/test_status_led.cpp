// ============================================================================
//  test_status_led.cpp  --  LED patterns. No hardware needed.
// ============================================================================

#include <unity.h>

#include "StatusLed.h"

// Counts rising edges of the LED over one full cycle sampled every millisecond.
static int flashesInCycle(RunPhase phase, RunReason reason, uint32_t cycleMs)
{
    int flashes = 0;
    bool prev = statusLedOn(phase, reason, cycleMs - 1); // end of the previous cycle: off
    for (uint32_t t = 0; t < cycleMs; ++t) {
        const bool on = statusLedOn(phase, reason, t);
        flashes += (on && !prev) ? 1 : 0;
        prev = on;
    }
    return flashes;
}

static void test_waiting_blinks_at_one_hertz()
{
    TEST_ASSERT_TRUE(statusLedOn(RunPhase::WaitStart, RunReason::None, 100));
    TEST_ASSERT_FALSE(statusLedOn(RunPhase::WaitStart, RunReason::None, 600));
    TEST_ASSERT_TRUE(statusLedOn(RunPhase::Countdown, RunReason::None, 1100));
}

static void test_driving_is_solid()
{
    const RunPhase driving[] = {RunPhase::SelfTest, RunPhase::Cruise, RunPhase::Settle,
                                RunPhase::Turn, RunPhase::Sweep};
    for (RunPhase p : driving) {
        for (uint32_t t = 0; t < 3000; t += 37) {
            TEST_ASSERT_TRUE(statusLedOn(p, RunReason::None, t));
        }
    }
}

static void test_blink_count_matches_the_documented_codes()
{
    TEST_ASSERT_EQUAL_UINT8(1, statusBlinkCount(RunPhase::Stopped, RunReason::EStop));
    TEST_ASSERT_EQUAL_UINT8(2, statusBlinkCount(RunPhase::Stopped, RunReason::TimeLimit));
    TEST_ASSERT_EQUAL_UINT8(3, statusBlinkCount(RunPhase::Stopped, RunReason::FrontWall));
    TEST_ASSERT_EQUAL_UINT8(4, statusBlinkCount(RunPhase::Stopped, RunReason::NoClearSide));
    TEST_ASSERT_EQUAL_UINT8(5, statusBlinkCount(RunPhase::Stopped, RunReason::TooManyTurns));
    TEST_ASSERT_EQUAL_UINT8(6, statusBlinkCount(RunPhase::Stopped, RunReason::Complete));
    TEST_ASSERT_EQUAL_UINT8(1, statusBlinkCount(RunPhase::Fault, RunReason::SelfTestReversed));
    TEST_ASSERT_EQUAL_UINT8(2, statusBlinkCount(RunPhase::Fault, RunReason::SelfTestNoMotion));
    TEST_ASSERT_EQUAL_UINT8(3, statusBlinkCount(RunPhase::Fault, RunReason::Stall));
    TEST_ASSERT_EQUAL_UINT8(4, statusBlinkCount(RunPhase::Fault, RunReason::FrontBlind));
    TEST_ASSERT_EQUAL_UINT8(5, statusBlinkCount(RunPhase::Fault, RunReason::TurnTimeout));
    TEST_ASSERT_EQUAL_UINT8(0, statusBlinkCount(RunPhase::Cruise, RunReason::None));
}

static void test_stopped_flashes_n_times_then_pauses()
{
    const uint32_t cycle = 3UL * 2 * STATUS_STOPPED_HALF_PERIOD_MS + STATUS_PAUSE_MS;
    TEST_ASSERT_EQUAL(3, flashesInCycle(RunPhase::Stopped, RunReason::FrontWall, cycle));
    // Dark for the whole pause.
    for (uint32_t t = 3UL * 2 * STATUS_STOPPED_HALF_PERIOD_MS; t < cycle; t += 50) {
        TEST_ASSERT_FALSE(statusLedOn(RunPhase::Stopped, RunReason::FrontWall, t));
    }
}

static void test_fault_flashes_faster_than_stopped()
{
    const uint32_t cycle = 2UL * 2 * STATUS_FAULT_HALF_PERIOD_MS + STATUS_PAUSE_MS;
    TEST_ASSERT_EQUAL(2, flashesInCycle(RunPhase::Fault, RunReason::SelfTestNoMotion, cycle));
    TEST_ASSERT_TRUE(STATUS_FAULT_HALF_PERIOD_MS < STATUS_STOPPED_HALF_PERIOD_MS);
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_waiting_blinks_at_one_hertz);
    RUN_TEST(test_driving_is_solid);
    RUN_TEST(test_blink_count_matches_the_documented_codes);
    RUN_TEST(test_stopped_flashes_n_times_then_pauses);
    RUN_TEST(test_fault_flashes_faster_than_stopped);
    return UNITY_END();
}
