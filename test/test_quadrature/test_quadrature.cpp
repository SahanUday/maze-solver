// ============================================================================
//  test_quadrature.cpp  --  quadratureDelta() decode tests. No hardware needed.
// ============================================================================

#include <unity.h>

#include "Quadrature.h"

// (B << 1) | A, A leading: 00 -> 01 -> 11 -> 10 -> 00
static const uint8_t kForward[] = {0, 1, 3, 2, 0};

static void test_no_change_is_zero()
{
    for (uint8_t s = 0; s < 4; s++) {
        TEST_ASSERT_EQUAL_INT8(0, quadratureDelta(s, s));
    }
}

static void test_forward_cycle_counts_four()
{
    int32_t total = 0;
    for (uint8_t i = 0; i < 4; i++) {
        TEST_ASSERT_EQUAL_INT8(1, quadratureDelta(kForward[i], kForward[i + 1]));
        total += quadratureDelta(kForward[i], kForward[i + 1]);
    }
    TEST_ASSERT_EQUAL_INT32(4, total);
}

static void test_reverse_cycle_counts_minus_four()
{
    int32_t total = 0;
    for (uint8_t i = 4; i > 0; i--) {
        TEST_ASSERT_EQUAL_INT8(-1, quadratureDelta(kForward[i], kForward[i - 1]));
        total += quadratureDelta(kForward[i], kForward[i - 1]);
    }
    TEST_ASSERT_EQUAL_INT32(-4, total);
}

static void test_two_state_jump_is_ignored()
{
    TEST_ASSERT_EQUAL_INT8(0, quadratureDelta(0, 3));
    TEST_ASSERT_EQUAL_INT8(0, quadratureDelta(3, 0));
    TEST_ASSERT_EQUAL_INT8(0, quadratureDelta(1, 2));
    TEST_ASSERT_EQUAL_INT8(0, quadratureDelta(2, 1));
}

static void test_jitter_on_one_edge_nets_zero()
{
    // Contact bounce: A rises, falls, rises again - must net exactly one count.
    int32_t total = 0;
    total += quadratureDelta(0, 1);
    total += quadratureDelta(1, 0);
    total += quadratureDelta(0, 1);
    TEST_ASSERT_EQUAL_INT32(1, total);
}

static void test_every_transition_is_covered()
{
    // All 16 (prev, curr) pairs: 4 forward, 4 reverse, 8 zero.
    int plus = 0, minus = 0, zero = 0;
    for (uint8_t p = 0; p < 4; p++) {
        for (uint8_t c = 0; c < 4; c++) {
            const int8_t d = quadratureDelta(p, c);
            plus += d == 1;
            minus += d == -1;
            zero += d == 0;
        }
    }
    TEST_ASSERT_EQUAL_INT(4, plus);
    TEST_ASSERT_EQUAL_INT(4, minus);
    TEST_ASSERT_EQUAL_INT(8, zero);
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_no_change_is_zero);
    RUN_TEST(test_forward_cycle_counts_four);
    RUN_TEST(test_reverse_cycle_counts_minus_four);
    RUN_TEST(test_two_state_jump_is_ignored);
    RUN_TEST(test_jitter_on_one_edge_nets_zero);
    RUN_TEST(test_every_transition_is_covered);
    return UNITY_END();
}
