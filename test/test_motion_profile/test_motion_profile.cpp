// ============================================================================
//  test_motion_profile.cpp  --  profileStep() tests. No hardware needed.
// ============================================================================

#include <unity.h>

#include "motion/MotionProfile.h"

constexpr float DT = 0.01f;

// Run a whole move; returns the distance covered and leaves the final speed in `v`.
static float run(float target, float vMax, float accel, float vEnd, float &v, int maxTicks = 5000)
{
    float pos = 0.0f;
    for (int i = 0; i < maxTicks; ++i) {
        v = profileStep(v, target - pos, vMax, accel, vEnd, DT);
        pos += v * DT;
        if (vEnd == 0.0f && v == 0.0f) {
            break;
        }
    }
    return pos;
}

static void test_accelerates_by_one_step()
{
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 10.0f, profileStep(0.0f, 1000.0f, 500.0f, 1000.0f, 0.0f, DT));
}

static void test_cruises_at_the_cap()
{
    float v = 0.0f;
    for (int i = 0; i < 200; ++i) {
        v = profileStep(v, 10000.0f, 300.0f, 1000.0f, 0.0f, DT);
    }
    TEST_ASSERT_EQUAL_FLOAT(300.0f, v);
}

static void test_stops_at_the_target_without_overshoot()
{
    float v = 0.0f;
    const float pos = run(250.0f, 300.0f, 1000.0f, 0.0f, v);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, v);
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 250.0f, pos);
    TEST_ASSERT_TRUE(pos <= 251.0f);
}

static void test_short_move_never_reaches_the_cap()
{
    float v = 0.0f;
    float peak = 0.0f;
    float pos = 0.0f;
    for (int i = 0; i < 2000 && !(i > 5 && v == 0.0f); ++i) {
        v = profileStep(v, 20.0f - pos, 500.0f, 1000.0f, 0.0f, DT);
        pos += v * DT;
        peak = v > peak ? v : peak;
    }
    TEST_ASSERT_TRUE(peak < 500.0f);
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 20.0f, pos);
}

static void test_pass_through_keeps_speed()
{
    float v = 200.0f;
    // Plenty of road ahead, vEnd = vMax: no braking.
    v = profileStep(v, 250.0f, 200.0f, 1000.0f, 200.0f, DT);
    TEST_ASSERT_EQUAL_FLOAT(200.0f, v);
}

static void test_brakes_late_toward_a_non_zero_end_speed()
{
    // 100 mm/s at the target, 300 mm/s now, only 10 mm left: must already be slowing.
    const float v = profileStep(300.0f, 10.0f, 300.0f, 1000.0f, 100.0f, DT);
    TEST_ASSERT_TRUE(v < 300.0f);
    TEST_ASSERT_TRUE(v >= 100.0f);
}

static void test_negative_remaining_drives_backwards()
{
    float v = 0.0f;
    const float pos = run(-100.0f, 300.0f, 1000.0f, 0.0f, v);
    TEST_ASSERT_FLOAT_WITHIN(2.0f, -100.0f, pos);
}

static void test_moving_the_wrong_way_turns_around()
{
    const float v = profileStep(-50.0f, 100.0f, 300.0f, 1000.0f, 0.0f, DT);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, -40.0f, v); // brakes first, by accel*dt
}

static void test_zero_accel_goes_nowhere()
{
    TEST_ASSERT_EQUAL_FLOAT(0.0f, profileStep(0.0f, 500.0f, 300.0f, 0.0f, 0.0f, DT));
}

static void test_end_speed_is_clamped_to_the_cap()
{
    // vEnd above vMax must not make the profile exceed vMax.
    const float v = profileStep(0.0f, 1000.0f, 100.0f, 1e6f, 500.0f, DT);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, v);
}

static void test_negative_cap_and_end_are_floored_to_zero()
{
    TEST_ASSERT_EQUAL_FLOAT(0.0f, profileStep(0.0f, 100.0f, -5.0f, 1000.0f, -5.0f, DT));
}

static void test_stop_from_speed_is_a_ramp_not_a_cliff()
{
    // Zero cap with plenty of road: a plain "slow down to nothing" must take several ticks.
    float v = 300.0f;
    v = profileStep(v, 1e9f, 0.0f, 1500.0f, 0.0f, DT);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 285.0f, v);
    int ticks = 0;
    while (v != 0.0f && ticks < 1000) {
        v = profileStep(v, 1e9f, 0.0f, 1500.0f, 0.0f, DT);
        ++ticks;
    }
    TEST_ASSERT_TRUE(ticks > 15 && ticks < 25); // 300 / (1500 * 0.01) = 20
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_accelerates_by_one_step);
    RUN_TEST(test_cruises_at_the_cap);
    RUN_TEST(test_stops_at_the_target_without_overshoot);
    RUN_TEST(test_short_move_never_reaches_the_cap);
    RUN_TEST(test_pass_through_keeps_speed);
    RUN_TEST(test_brakes_late_toward_a_non_zero_end_speed);
    RUN_TEST(test_negative_remaining_drives_backwards);
    RUN_TEST(test_moving_the_wrong_way_turns_around);
    RUN_TEST(test_zero_accel_goes_nowhere);
    RUN_TEST(test_end_speed_is_clamped_to_the_cap);
    RUN_TEST(test_negative_cap_and_end_are_floored_to_zero);
    RUN_TEST(test_stop_from_speed_is_a_ramp_not_a_cliff);
    return UNITY_END();
}
