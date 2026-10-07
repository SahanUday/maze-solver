// ============================================================================
//  test_heading.cpp  --  gyro bias averaging and yaw integration. No hardware.
//  The fixtures are synthetic: no real sensor has been read yet, so these test
//  the arithmetic and its guards, not any measured behaviour.
// ============================================================================

#include <unity.h>

#include "Heading.h"

// A bias calibration of `samples` readings, each `counts`.
static GyroBias calibrated(int16_t counts, uint16_t samples)
{
    GyroBias bias;
    gyroBiasReset(bias);
    for (uint16_t i = 0; i < samples; ++i) {
        gyroBiasAccumulate(bias, counts);
    }
    gyroBiasFinish(bias);
    return bias;
}

static void test_reset_leaves_nothing_sampled_and_not_ready()
{
    GyroBias bias;
    gyroBiasAccumulate(bias, 123);
    gyroBiasReset(bias);
    TEST_ASSERT_EQUAL_INT32(0, bias.sum);
    TEST_ASSERT_EQUAL_UINT16(0, bias.samples);
    TEST_ASSERT_FALSE(bias.ready);
}

static void test_finish_averages_the_samples()
{
    GyroBias bias = calibrated(40, GYRO_BIAS_SAMPLES);
    TEST_ASSERT_TRUE(bias.ready);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 40.0f, bias.counts);
}

static void test_the_average_is_not_the_integer_division()
{
    GyroBias bias;
    gyroBiasReset(bias);
    // 501 samples: 251 of 1 and 250 of 0, so the mean is just over a half count.
    for (uint16_t i = 0; i < 251; ++i) {
        gyroBiasAccumulate(bias, 1);
    }
    for (uint16_t i = 0; i < 250; ++i) {
        gyroBiasAccumulate(bias, 0);
    }
    TEST_ASSERT_TRUE(gyroBiasFinish(bias));
    TEST_ASSERT_FLOAT_WITHIN(0.0005f, 251.0f / 501.0f, bias.counts);
}

static void test_a_negative_bias_averages_negative()
{
    GyroBias bias = calibrated(-37, GYRO_BIAS_SAMPLES);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -37.0f, bias.counts);
}

static void test_too_few_samples_is_a_failed_calibration()
{
    GyroBias bias;
    gyroBiasReset(bias);
    for (uint16_t i = 0; i < GYRO_BIAS_MIN_SAMPLES - 1; ++i) {
        gyroBiasAccumulate(bias, 40);
    }
    TEST_ASSERT_FALSE(gyroBiasFinish(bias));
    TEST_ASSERT_FALSE(bias.ready);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, bias.counts);
}

static void test_the_minimum_sample_count_itself_succeeds()
{
    GyroBias bias;
    gyroBiasReset(bias);
    for (uint16_t i = 0; i < GYRO_BIAS_MIN_SAMPLES; ++i) {
        gyroBiasAccumulate(bias, 40);
    }
    TEST_ASSERT_TRUE(gyroBiasFinish(bias));
}

static void test_accumulating_past_the_limit_is_ignored_not_overflowed()
{
    GyroBias bias;
    gyroBiasReset(bias);
    for (uint32_t i = 0; i < GYRO_BIAS_SAMPLES * 3UL; ++i) {
        gyroBiasAccumulate(bias, 32767);
    }
    TEST_ASSERT_EQUAL_UINT16(GYRO_BIAS_SAMPLES, bias.samples);
    TEST_ASSERT_EQUAL_INT32(32767L * GYRO_BIAS_SAMPLES, bias.sum);
    TEST_ASSERT_TRUE(gyroBiasFinish(bias));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 32767.0f, bias.counts);
}

static void test_a_still_robot_does_not_drift_whatever_the_bias()
{
    const GyroBias bias = calibrated(250, GYRO_BIAS_SAMPLES);
    Heading heading;
    for (uint16_t tick = 0; tick < 1000; ++tick) {
        headingUpdate(heading, bias, 250, CONTROL_LOOP_PERIOD_MS);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, heading.deg);
}

static void test_an_uncorrected_bias_is_what_would_drift()
{
    // The same readings against a zero bias: 250 counts is ~3.8 deg/s, and 10 s of
    // it is why gyroBiasFinish() exists at all.
    GyroBias none;
    gyroBiasReset(none);
    none.ready = true;
    Heading heading;
    for (uint16_t tick = 0; tick < 1000; ++tick) {
        headingUpdate(heading, none, 250, CONTROL_LOOP_PERIOD_MS);
    }
    TEST_ASSERT_TRUE(heading.deg > 30.0f || heading.deg < -30.0f);
}

static void test_a_constant_rate_integrates_to_rate_times_time()
{
    const GyroBias bias = calibrated(0, GYRO_BIAS_SAMPLES);
    // 90 deg/s held for 1 s = 90 deg, in 100 ticks of 10 ms.
    const int16_t counts = static_cast<int16_t>(90.0f * GYRO_COUNTS_PER_DPS);
    Heading heading;
    for (uint16_t tick = 0; tick < 100; ++tick) {
        headingUpdate(heading, bias, counts, CONTROL_LOOP_PERIOD_MS);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.5f, GYRO_YAW_INVERT ? -90.0f : 90.0f, heading.deg);
}

static void test_the_bias_is_subtracted_from_the_rate()
{
    const GyroBias bias = calibrated(100, GYRO_BIAS_SAMPLES);
    const int16_t counts = static_cast<int16_t>(100.0f + 90.0f * GYRO_COUNTS_PER_DPS);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, GYRO_YAW_INVERT ? -90.0f : 90.0f,
                             gyroYawRateDps(bias, counts));
}

static void test_rotating_back_returns_to_where_it_started()
{
    const GyroBias bias = calibrated(0, GYRO_BIAS_SAMPLES);
    const int16_t counts = static_cast<int16_t>(90.0f * GYRO_COUNTS_PER_DPS);
    Heading heading;
    for (uint16_t tick = 0; tick < 50; ++tick) {
        headingUpdate(heading, bias, counts, CONTROL_LOOP_PERIOD_MS);
    }
    for (uint16_t tick = 0; tick < 50; ++tick) {
        headingUpdate(heading, bias, static_cast<int16_t>(-counts), CONTROL_LOOP_PERIOD_MS);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, heading.deg);
}

static void test_nothing_integrates_before_the_bias_is_ready()
{
    GyroBias bias;
    gyroBiasReset(bias);
    Heading heading;
    headingUpdate(heading, bias, 20000, CONTROL_LOOP_PERIOD_MS);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, heading.deg);
}

static void test_wrap_keeps_the_angle_in_minus_180_to_180()
{
    TEST_ASSERT_EQUAL_FLOAT(0.0f, headingWrapDeg(360.0f));
    TEST_ASSERT_EQUAL_FLOAT(-170.0f, headingWrapDeg(190.0f));
    TEST_ASSERT_EQUAL_FLOAT(170.0f, headingWrapDeg(-190.0f));
    TEST_ASSERT_EQUAL_FLOAT(-180.0f, headingWrapDeg(180.0f));
    TEST_ASSERT_EQUAL_FLOAT(-180.0f, headingWrapDeg(-180.0f));
    TEST_ASSERT_EQUAL_FLOAT(179.0f, headingWrapDeg(179.0f));
}

static void test_integrating_past_half_a_turn_wraps_rather_than_growing()
{
    const GyroBias bias = calibrated(0, GYRO_BIAS_SAMPLES);
    const int16_t counts = static_cast<int16_t>(180.0f * GYRO_COUNTS_PER_DPS);
    Heading heading;
    for (uint16_t tick = 0; tick < 100; ++tick) { // 180 deg/s for 1 s
        headingUpdate(heading, bias, counts, CONTROL_LOOP_PERIOD_MS);
    }
    TEST_ASSERT_TRUE(heading.deg >= -180.0f && heading.deg < 180.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, -180.0f, heading.deg);
}

static void test_set_overrides_the_integral_and_wraps()
{
    Heading heading;
    headingSet(heading, 90.0f);
    TEST_ASSERT_EQUAL_FLOAT(90.0f, heading.deg);
    headingSet(heading, 270.0f);
    TEST_ASSERT_EQUAL_FLOAT(-90.0f, heading.deg);
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_reset_leaves_nothing_sampled_and_not_ready);
    RUN_TEST(test_finish_averages_the_samples);
    RUN_TEST(test_the_average_is_not_the_integer_division);
    RUN_TEST(test_a_negative_bias_averages_negative);
    RUN_TEST(test_too_few_samples_is_a_failed_calibration);
    RUN_TEST(test_the_minimum_sample_count_itself_succeeds);
    RUN_TEST(test_accumulating_past_the_limit_is_ignored_not_overflowed);
    RUN_TEST(test_a_still_robot_does_not_drift_whatever_the_bias);
    RUN_TEST(test_an_uncorrected_bias_is_what_would_drift);
    RUN_TEST(test_a_constant_rate_integrates_to_rate_times_time);
    RUN_TEST(test_the_bias_is_subtracted_from_the_rate);
    RUN_TEST(test_rotating_back_returns_to_where_it_started);
    RUN_TEST(test_nothing_integrates_before_the_bias_is_ready);
    RUN_TEST(test_wrap_keeps_the_angle_in_minus_180_to_180);
    RUN_TEST(test_integrating_past_half_a_turn_wraps_rather_than_growing);
    RUN_TEST(test_set_overrides_the_integral_and_wraps);
    return UNITY_END();
}
