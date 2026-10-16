// ============================================================================
//  test_tilt.cpp  --  inclineCounts() and the level capture. No hardware needed.
// ============================================================================

#include <math.h>
#include <unity.h>

#include "sense/Tilt.h"
#include "spec/Imu.h"

constexpr int16_t LEVEL = IMU_ACCEL_FORWARD_LEVEL_COUNTS;

// A reading with the forward axis `slope` counts nose-up from level (negative = nose down).
static ImuRaw noseUp(int16_t slope, int16_t level = LEVEL)
{
    ImuRaw r;
    r.accel[IMU_ACCEL_AXIS_FORWARD] =
        static_cast<int16_t>(IMU_ACCEL_NOSE_UP_NEGATIVE ? level - slope : level + slope);
    return r;
}

static void test_level_is_zero()
{
    TEST_ASSERT_EQUAL_INT16(0, inclineCounts(noseUp(0), LEVEL));
}

static void test_nose_up_is_positive_and_nose_down_negative()
{
    TEST_ASSERT_EQUAL_INT16(2387, inclineCounts(noseUp(2387), LEVEL));
    TEST_ASSERT_EQUAL_INT16(-2387, inclineCounts(noseUp(-2387), LEVEL));
}

static void test_the_level_reading_is_taken_off()
{
    // The same physical slope against a level reference 155 counts further along the axis.
    TEST_ASSERT_EQUAL_INT16(3000, inclineCounts(noseUp(3000, 400), 400));
    TEST_ASSERT_EQUAL_INT16(3000 + (IMU_ACCEL_NOSE_UP_NEGATIVE ? 155 : -155),
                            inclineCounts(noseUp(3000, 400), 555));
}

static void test_only_the_forward_axis_counts()
{
    ImuRaw r = noseUp(3000);
    for (int axis = 0; axis < 3; ++axis) {
        if (axis != IMU_ACCEL_AXIS_FORWARD) {
            r.accel[axis] = 12000; // a sideways lean or a changed Z must not read as slope
        }
    }
    TEST_ASSERT_EQUAL_INT16(3000, inclineCounts(r, LEVEL));
}

static void test_the_bench_hand_lift_reads_about_26_degrees()
{
    // docs/calibration: nose raised by hand, forward axis -7250 against +245 level.
    ImuRaw r;
    r.accel[IMU_ACCEL_AXIS_FORWARD] = IMU_ACCEL_NOSE_UP_NEGATIVE ? -7250 : 7250;
    const int16_t c = inclineCounts(r, IMU_ACCEL_NOSE_UP_NEGATIVE ? 245 : -245);
    TEST_ASSERT_EQUAL_INT16(7495, c);
    const float deg = asinf(static_cast<float>(c) / IMU_ACCEL_ONE_G_COUNTS) * 57.29578f;
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 25.9f, deg);
}

static void test_the_result_saturates()
{
    ImuRaw r;
    r.accel[IMU_ACCEL_AXIS_FORWARD] = IMU_ACCEL_NOSE_UP_NEGATIVE ? INT16_MIN : INT16_MAX;
    TEST_ASSERT_EQUAL_INT16(INT16_MAX, inclineCounts(r, IMU_ACCEL_NOSE_UP_NEGATIVE ? 245 : -245));
}

static void feed(InclineLevel &l, int16_t value, int n)
{
    ImuRaw r;
    r.accel[IMU_ACCEL_AXIS_FORWARD] = value;
    for (int i = 0; i < n; ++i) {
        inclineLevelAccumulate(l, r);
    }
}

static void test_the_level_is_the_average_of_a_still_capture()
{
    InclineLevel l;
    feed(l, 300, 60);
    feed(l, 340, 60);
    TEST_ASSERT_TRUE(inclineLevelFinish(l));
    TEST_ASSERT_TRUE(l.ready);
    TEST_ASSERT_EQUAL_INT16(320, l.counts);
}

static void test_a_capture_with_too_few_samples_is_refused()
{
    InclineLevel l;
    feed(l, 900, INCLINE_LEVEL_MIN_SAMPLES - 1);
    TEST_ASSERT_FALSE(inclineLevelFinish(l));
    TEST_ASSERT_FALSE(l.ready);
    TEST_ASSERT_EQUAL_INT16(IMU_ACCEL_FORWARD_LEVEL_COUNTS, l.counts);
}

static void test_a_capture_while_moving_is_refused()
{
    InclineLevel l;
    feed(l, 0, 60);
    feed(l, static_cast<int16_t>(INCLINE_LEVEL_MAX_RANGE_COUNTS + 1), 60);
    TEST_ASSERT_FALSE(inclineLevelFinish(l));
    TEST_ASSERT_EQUAL_INT16(IMU_ACCEL_FORWARD_LEVEL_COUNTS, l.counts);
}

static void test_a_spread_at_the_limit_is_accepted()
{
    InclineLevel l;
    feed(l, 0, 60);
    feed(l, static_cast<int16_t>(INCLINE_LEVEL_MAX_RANGE_COUNTS), 60);
    TEST_ASSERT_TRUE(inclineLevelFinish(l));
}

static void test_negative_readings_average_correctly()
{
    InclineLevel l;
    feed(l, -400, 100);
    TEST_ASSERT_TRUE(inclineLevelFinish(l));
    TEST_ASSERT_EQUAL_INT16(-400, l.counts);
}

static void test_an_overrun_capture_saturates_instead_of_overflowing()
{
    InclineLevel l;
    feed(l, 100, 70000);
    TEST_ASSERT_EQUAL_UINT16(UINT16_MAX, l.samples);
    TEST_ASSERT_TRUE(inclineLevelFinish(l));
    TEST_ASSERT_EQUAL_INT16(100, l.counts);
}

static void test_reset_forgets_the_capture()
{
    InclineLevel l;
    feed(l, 500, 100);
    inclineLevelFinish(l);
    inclineLevelReset(l);
    TEST_ASSERT_FALSE(l.ready);
    TEST_ASSERT_EQUAL_UINT16(0, l.samples);
    TEST_ASSERT_EQUAL_INT16(IMU_ACCEL_FORWARD_LEVEL_COUNTS, l.counts);
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_level_is_zero);
    RUN_TEST(test_nose_up_is_positive_and_nose_down_negative);
    RUN_TEST(test_the_level_reading_is_taken_off);
    RUN_TEST(test_only_the_forward_axis_counts);
    RUN_TEST(test_the_bench_hand_lift_reads_about_26_degrees);
    RUN_TEST(test_the_result_saturates);
    RUN_TEST(test_the_level_is_the_average_of_a_still_capture);
    RUN_TEST(test_a_capture_with_too_few_samples_is_refused);
    RUN_TEST(test_a_capture_while_moving_is_refused);
    RUN_TEST(test_a_spread_at_the_limit_is_accepted);
    RUN_TEST(test_negative_readings_average_correctly);
    RUN_TEST(test_an_overrun_capture_saturates_instead_of_overflowing);
    RUN_TEST(test_reset_forgets_the_capture);
    return UNITY_END();
}
