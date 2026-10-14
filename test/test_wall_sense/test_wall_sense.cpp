// ============================================================================
//  test_wall_sense.cpp  --  wallView(), wallCenterError(), wallSteerTrim() tests. No hardware
//  needed.
// ============================================================================

#include <unity.h>

#include "WallSense.h"

static WallParams params()
{
    WallParams p;
    p.sideMm = 150;
    p.frontMm = 250;
    p.centerDistMm = 100;
    p.kp = 0.5f;
    p.steerMaxDeg = 10.0f;
    return p;
}

static RobotState state(uint16_t f, uint16_t l, uint16_t r, uint8_t valid = 0x07)
{
    RobotState s;
    s.distFrontMm = f;
    s.distLeftMm = l;
    s.distRightMm = r;
    s.usValid = valid;
    return s;
}

static void test_the_defaults_are_the_spec_values()
{
    const WallParams p;
    TEST_ASSERT_EQUAL_UINT16(WALL_PRESENT_SIDE_MM, p.sideMm);
    TEST_ASSERT_EQUAL_UINT16(WALL_PRESENT_FRONT_MM, p.frontMm);
    TEST_ASSERT_EQUAL_UINT16(WALL_CENTER_DIST_MM, p.centerDistMm);
    TEST_ASSERT_EQUAL_FLOAT(KP_WALL, p.kp);
    TEST_ASSERT_EQUAL_FLOAT(STEER_MAX_DEG, p.steerMaxDeg);
}

static void test_view_applies_thresholds()
{
    const WallView v = wallView(state(200, 100, 300), params());
    TEST_ASSERT_TRUE(v.front);
    TEST_ASSERT_TRUE(v.left);
    TEST_ASSERT_FALSE(v.right);
    TEST_ASSERT_EQUAL_UINT16(200, v.frontMm);
}

static void test_view_threshold_is_exclusive()
{
    const WallView v = wallView(state(250, 150, 150), params());
    TEST_ASSERT_FALSE(v.front);
    TEST_ASSERT_FALSE(v.left);
    TEST_ASSERT_FALSE(v.right);
}

static void test_no_echo_is_neither_wall_nor_open()
{
    // Bit 0 front, 1 left, 2 right. Left has no echo: its distance is stale and must not count.
    const WallView v = wallView(state(100, 10, 100, 0x05), params());
    TEST_ASSERT_TRUE(v.frontValid);
    TEST_ASSERT_FALSE(v.leftValid);
    TEST_ASSERT_TRUE(v.rightValid);
    TEST_ASSERT_FALSE(v.left);
    TEST_ASSERT_TRUE(v.right);
}

static void test_each_valid_bit_maps_to_its_own_sensor()
{
    TEST_ASSERT_TRUE(wallView(state(0, 0, 0, 0x01), params()).frontValid);
    TEST_ASSERT_TRUE(wallView(state(0, 0, 0, 0x02), params()).leftValid);
    TEST_ASSERT_TRUE(wallView(state(0, 0, 0, 0x04), params()).rightValid);
    TEST_ASSERT_FALSE(wallView(state(0, 0, 0, 0x00), params()).frontValid);
}

static void test_center_error_with_both_walls()
{
    float e = 0.0f;
    // Left 60, right 100: nearer the left wall, so negative error (steer right).
    TEST_ASSERT_TRUE(wallCenterError(wallView(state(400, 60, 100), params()), params(), e));
    TEST_ASSERT_EQUAL_FLOAT(-20.0f, e);
}

static void test_center_error_with_one_wall_uses_the_target_distance()
{
    float e = 0.0f;
    TEST_ASSERT_TRUE(wallCenterError(wallView(state(400, 120, 400), params()), params(), e));
    TEST_ASSERT_EQUAL_FLOAT(20.0f, e); // 20 mm too far from the left wall: robot is right of centre
    TEST_ASSERT_TRUE(wallCenterError(wallView(state(400, 400, 80), params()), params(), e));
    TEST_ASSERT_EQUAL_FLOAT(20.0f,
                            e); // 20 mm too close to the right wall: robot is right of centre
}

static void test_center_error_needs_a_wall()
{
    float e = 7.0f;
    TEST_ASSERT_FALSE(wallCenterError(wallView(state(400, 400, 400), params()), params(), e));
}

static void test_trim_steers_away_from_the_near_wall()
{
    WallSteer w;
    const WallParams p = params();
    float trim = 0.0f;
    for (int i = 0; i < 20; ++i) {
        trim = wallSteerTrim(w, wallView(state(400, 60, 100), p), p);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.1f, -10.0f, trim); // 0.5 * -20 = -10, clamped at the same value
}

static void test_trim_is_clamped()
{
    WallSteer w;
    const WallParams p = params();
    float trim = 0.0f;
    for (int i = 0; i < 20; ++i) {
        trim = wallSteerTrim(w, wallView(state(400, 20, 140), p), p); // err -60
    }
    TEST_ASSERT_EQUAL_FLOAT(-10.0f, trim);
    for (int i = 0; i < 20; ++i) {
        trim = wallSteerTrim(w, wallView(state(400, 140, 20), p), p);
    }
    TEST_ASSERT_EQUAL_FLOAT(10.0f, trim);
}

static void test_no_wall_means_no_trim_and_resets()
{
    WallSteer w;
    const WallParams p = params();
    wallSteerTrim(w, wallView(state(400, 60, 100), p), p);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, wallSteerTrim(w, wallView(state(400, 400, 400), p), p));
    TEST_ASSERT_TRUE(w.mode == WallMode::None);
}

static void test_both_walls_mode()
{
    WallSteer w;
    const WallParams p = params();
    wallSteerTrim(w, wallView(state(400, 60, 100), p), p);
    TEST_ASSERT_TRUE(w.mode == WallMode::Both);
}

static void test_filter_restarts_when_the_wall_set_changes()
{
    WallSteer w;
    WallParams p = params();
    p.kp = 1.0f;
    p.steerMaxDeg = 100.0f;
    for (int i = 0; i < 20; ++i) {
        wallSteerTrim(w, wallView(state(400, 60, 100), p), p); // both walls, err -20
    }
    // The right wall ends: error now comes from the left wall alone. No half-way blend.
    const float trim = wallSteerTrim(w, wallView(state(400, 130, 400), p), p);
    TEST_ASSERT_EQUAL_FLOAT(30.0f, trim);
    TEST_ASSERT_TRUE(w.mode == WallMode::LeftOnly);
}

static void test_right_only_mode()
{
    WallSteer w;
    const WallParams p = params();
    wallSteerTrim(w, wallView(state(400, 400, 80), p), p);
    TEST_ASSERT_TRUE(w.mode == WallMode::RightOnly);
}

static void test_reset()
{
    WallSteer w;
    w.mode = WallMode::LeftOnly;
    w.filtMm = 5.0f;
    wallSteerReset(w);
    TEST_ASSERT_TRUE(w.mode == WallMode::None);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, w.filtMm);
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_the_defaults_are_the_spec_values);
    RUN_TEST(test_view_applies_thresholds);
    RUN_TEST(test_view_threshold_is_exclusive);
    RUN_TEST(test_no_echo_is_neither_wall_nor_open);
    RUN_TEST(test_each_valid_bit_maps_to_its_own_sensor);
    RUN_TEST(test_center_error_with_both_walls);
    RUN_TEST(test_center_error_with_one_wall_uses_the_target_distance);
    RUN_TEST(test_center_error_needs_a_wall);
    RUN_TEST(test_trim_steers_away_from_the_near_wall);
    RUN_TEST(test_trim_is_clamped);
    RUN_TEST(test_no_wall_means_no_trim_and_resets);
    RUN_TEST(test_both_walls_mode);
    RUN_TEST(test_filter_restarts_when_the_wall_set_changes);
    RUN_TEST(test_right_only_mode);
    RUN_TEST(test_reset);
    return UNITY_END();
}
