// ============================================================================
//  test_scheduler.cpp  --  tickDue() timing tests. No hardware needed.
// ============================================================================

#include <unity.h>

#include "Scheduler.h"

static void test_not_due_yet()
{
    uint32_t nextTickMs = 100;
    TEST_ASSERT_FALSE(tickDue(99, nextTickMs, 10));
    TEST_ASSERT_EQUAL_UINT32(100, nextTickMs); // unchanged
}

static void test_exactly_due_advances_by_one_period()
{
    uint32_t nextTickMs = 100;
    TEST_ASSERT_TRUE(tickDue(100, nextTickMs, 10));
    TEST_ASSERT_EQUAL_UINT32(110, nextTickMs);
}

static void test_jitter_advances_from_previous_target_not_now()
{
    // Fired 3ms late (jitter) - must advance from the old target (100+10),
    // not from now (103+10), or the period drifts.
    uint32_t nextTickMs = 100;
    TEST_ASSERT_TRUE(tickDue(103, nextTickMs, 10));
    TEST_ASSERT_EQUAL_UINT32(110, nextTickMs);
}

static void test_far_behind_resyncs_instead_of_catch_up_burst()
{
    // 25ms late on a 10ms period (more than one period behind) - must
    // resync to now, not advance to 110 and stay behind.
    uint32_t nextTickMs = 100;
    TEST_ASSERT_TRUE(tickDue(125, nextTickMs, 10));
    TEST_ASSERT_EQUAL_UINT32(125, nextTickMs);
}

static void test_millis_wraparound()
{
    // nextTickMs near the top of uint32_t range, nowMs wrapped back near 0.
    uint32_t nextTickMs = 0xFFFFFFF0;
    TEST_ASSERT_TRUE(tickDue(5, nextTickMs, 10)); // wrapped 21 ms past target
    TEST_ASSERT_EQUAL_UINT32(5, nextTickMs);      // more than one period behind -> resync
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_not_due_yet);
    RUN_TEST(test_exactly_due_advances_by_one_period);
    RUN_TEST(test_jitter_advances_from_previous_target_not_now);
    RUN_TEST(test_far_behind_resyncs_instead_of_catch_up_burst);
    RUN_TEST(test_millis_wraparound);
    return UNITY_END();
}
