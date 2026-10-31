// ============================================================================
//  test_odometry.cpp  --  odometryUpdate() tests. No hardware needed.
// ============================================================================

#include <unity.h>

#include "sense/Odometry.h"

static void test_first_update_only_primes()
{
    Odometry o;
    odometryUpdate(o, 5000, 5000, 0.1f, 10);
    TEST_ASSERT_TRUE(o.primed);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, o.distMm);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, o.speedMmS);
}

static void test_distance_is_mean_of_wheels()
{
    Odometry o;
    odometryUpdate(o, 0, 0, 0.5f, 10);
    odometryUpdate(o, 100, 60, 0.5f, 10); // mean 80 counts * 0.5
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 40.0f, o.distMm);
}

static void test_pivot_adds_no_distance()
{
    Odometry o;
    odometryUpdate(o, 0, 0, 0.5f, 10);
    odometryUpdate(o, -50, 50, 0.5f, 10);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, o.distMm);
}

static void test_reverse_is_negative()
{
    Odometry o;
    odometryUpdate(o, 0, 0, 1.0f, 10);
    odometryUpdate(o, -10, -10, 1.0f, 10);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -10.0f, o.distMm);
}

static void test_speed_converges_to_the_true_speed()
{
    Odometry o;
    odometryUpdate(o, 0, 0, 1.0f, 10);
    // 2 counts per 10 ms tick = 200 mm/s
    int32_t c = 0;
    for (int i = 0; i < 20; ++i) {
        c += 2;
        odometryUpdate(o, c, c, 1.0f, 10);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 200.0f, o.speedMmS);
}

static void test_zero_scale_means_not_moving()
{
    Odometry o;
    odometryUpdate(o, 0, 0, 0.0f, 10);
    odometryUpdate(o, 1000, 1000, 0.0f, 10);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, o.distMm);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, o.speedMmS);
}

static void test_zero_dt_keeps_distance_but_not_speed()
{
    Odometry o;
    odometryUpdate(o, 0, 0, 1.0f, 10);
    odometryUpdate(o, 10, 10, 1.0f, 0);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 10.0f, o.distMm);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, o.speedMmS);
}

static void test_distance_does_not_drift_over_a_long_run()
{
    // 8 minutes at 150 mm/s: 6.6 counts a tick, 72 m. Adding a float step to a float total
    // loses about 22 mm by the end.
    const float mm = 0.2274f;
    Odometry o;
    odometryUpdate(o, 0, 0, mm, 10);
    double carry = 0.0;
    int32_t c = 0;
    for (int i = 0; i < 48000; ++i) {
        carry += 150.0 * 0.01 / mm;
        const int32_t n = static_cast<int32_t>(carry);
        carry -= n;
        c += n;
        odometryUpdate(o, c, c, mm, 10);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.1f, static_cast<float>(static_cast<double>(c) * mm), o.distMm);
}

static void test_the_counts_may_wrap_around_int32()
{
    // The encoder counters are free-running: a step across the wrap is still a small step.
    Odometry o;
    odometryUpdate(o, INT32_MAX - 4, INT32_MAX - 4, 1.0f, 10);
    odometryUpdate(o, INT32_MIN + 5, INT32_MIN + 5, 1.0f, 10); // +10 counts
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 10.0f, o.distMm);
    odometryUpdate(o, INT32_MAX - 4, INT32_MAX - 4, 1.0f, 10); // and back
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, o.distMm);
}

static void test_reset_clears_everything()
{
    Odometry o;
    odometryUpdate(o, 0, 0, 1.0f, 10);
    odometryUpdate(o, 10, 10, 1.0f, 10);
    odometryReset(o);
    TEST_ASSERT_FALSE(o.primed);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, o.distMm);
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_first_update_only_primes);
    RUN_TEST(test_distance_is_mean_of_wheels);
    RUN_TEST(test_pivot_adds_no_distance);
    RUN_TEST(test_reverse_is_negative);
    RUN_TEST(test_speed_converges_to_the_true_speed);
    RUN_TEST(test_zero_scale_means_not_moving);
    RUN_TEST(test_zero_dt_keeps_distance_but_not_speed);
    RUN_TEST(test_distance_does_not_drift_over_a_long_run);
    RUN_TEST(test_the_counts_may_wrap_around_int32);
    RUN_TEST(test_reset_clears_everything);
    return UNITY_END();
}
