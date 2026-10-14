// ============================================================================
//  test_wall_sense.cpp  --  wallView(), wallCenterError(), wallSteerTrim() tests. No hardware
//  needed.
// ============================================================================

#include <unity.h>

#include "WallSense.h"

// Proportional only, wide pair tolerance: the tests that add the integral or the gate say so.
static WallParams params()
{
    WallParams p;
    p.sideMm = 150;
    p.frontMm = 250;
    p.centerDistMm = 100;
    p.pairTolMm = 1000;
    p.kp = 0.5f;
    p.ki = 0.0f;
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

static float trimOf(WallSteer &w, const RobotState &s, const WallParams &p, float headingDeg = 0.0f,
                    uint16_t dtMs = 10)
{
    return wallSteerTrim(w, wallView(s, p), p, headingDeg, dtMs);
}

static void test_the_defaults_are_the_spec_values()
{
    const WallParams p;
    TEST_ASSERT_EQUAL_UINT16(WALL_PRESENT_SIDE_MM, p.sideMm);
    TEST_ASSERT_EQUAL_UINT16(WALL_PRESENT_FRONT_MM, p.frontMm);
    TEST_ASSERT_EQUAL_UINT16(WALL_CENTER_DIST_MM, p.centerDistMm);
    TEST_ASSERT_EQUAL_UINT16(WALL_PAIR_TOL_MM, p.pairTolMm);
    TEST_ASSERT_EQUAL_FLOAT(KP_WALL, p.kp);
    TEST_ASSERT_EQUAL_FLOAT(KI_WALL, p.ki);
    TEST_ASSERT_EQUAL_FLOAT(WALL_BIAS_MAX_DEG, p.biasMaxDeg);
    TEST_ASSERT_EQUAL_FLOAT(WALL_BIAS_RESET_DEG, p.biasResetDeg);
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

static void test_a_pair_that_disagrees_with_the_corridor_width_is_not_used()
{
    WallParams p = params();
    p.pairTolMm = 30; // 2 * centre = 200
    float e = 0.0f;
    TEST_ASSERT_TRUE(wallCenterError(wallView(state(400, 100, 100), p), p, e));  // sum 200
    TEST_ASSERT_TRUE(wallCenterError(wallView(state(400, 115, 114), p), p, e));  // sum 229, within
    TEST_ASSERT_FALSE(wallCenterError(wallView(state(400, 60, 100), p), p, e));  // sum 160: wrong
    TEST_ASSERT_FALSE(wallCenterError(wallView(state(400, 140, 100), p), p, e)); // sum 240: wrong
    // The edge itself is accepted.
    TEST_ASSERT_TRUE(wallCenterError(wallView(state(400, 115, 115), p), p, e)); // sum 230
}

static void test_the_pair_gate_does_not_touch_a_single_wall()
{
    WallParams p = params();
    p.pairTolMm = 5;
    float e = 0.0f;
    TEST_ASSERT_TRUE(wallCenterError(wallView(state(400, 60, 400), p), p, e));
    TEST_ASSERT_EQUAL_FLOAT(-40.0f, e);
}

static void test_trim_steers_away_from_the_near_wall()
{
    WallSteer w;
    const WallParams p = params();
    float trim = 0.0f;
    for (int i = 0; i < 20; ++i) {
        trim = trimOf(w, state(400, 60, 100), p);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.1f, -10.0f, trim); // 0.5 * -20 = -10, clamped at the same value
}

static void test_trim_is_clamped()
{
    WallSteer w;
    const WallParams p = params();
    float trim = 0.0f;
    for (int i = 0; i < 20; ++i) {
        trim = trimOf(w, state(400, 20, 140), p); // err -60
    }
    TEST_ASSERT_EQUAL_FLOAT(-10.0f, trim);
    for (int i = 0; i < 20; ++i) {
        trim = trimOf(w, state(400, 140, 20), p);
    }
    TEST_ASSERT_EQUAL_FLOAT(10.0f, trim);
}

static void test_no_wall_means_no_trim_and_resets_the_mode()
{
    WallSteer w;
    const WallParams p = params();
    trimOf(w, state(400, 60, 100), p);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, trimOf(w, state(400, 400, 400), p));
    TEST_ASSERT_TRUE(w.mode == WallMode::None);
}

static void test_both_walls_mode()
{
    WallSteer w;
    const WallParams p = params();
    trimOf(w, state(400, 60, 100), p);
    TEST_ASSERT_TRUE(w.mode == WallMode::Both);
}

static void test_filter_restarts_when_the_wall_set_changes()
{
    WallSteer w;
    WallParams p = params();
    p.kp = 1.0f;
    p.steerMaxDeg = 100.0f;
    for (int i = 0; i < 20; ++i) {
        trimOf(w, state(400, 60, 100), p); // both walls, err -20
    }
    // The right wall ends: error now comes from the left wall alone. No half-way blend.
    const float trim = trimOf(w, state(400, 130, 400), p);
    TEST_ASSERT_EQUAL_FLOAT(30.0f, trim);
    TEST_ASSERT_TRUE(w.mode == WallMode::LeftOnly);
}

static void test_right_only_mode()
{
    WallSteer w;
    const WallParams p = params();
    trimOf(w, state(400, 400, 80), p);
    TEST_ASSERT_TRUE(w.mode == WallMode::RightOnly);
}

// ---- The integral: the offset a proportional trim cannot remove --------------------------

static void test_an_offset_that_stays_makes_the_bias_grow()
{
    WallSteer w;
    WallParams p = params();
    p.ki = 0.06f;
    for (int i = 0; i < 100; ++i) {        // 1 s at 10 mm right of centre
        trimOf(w, state(400, 110, 90), p); // err +10
    }
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.06f * 10.0f * 1.0f, w.biasDeg);
}

static void test_a_zero_ki_learns_nothing()
{
    WallSteer w;
    const WallParams p = params();
    for (int i = 0; i < 100; ++i) {
        trimOf(w, state(400, 110, 90), p);
    }
    TEST_ASSERT_EQUAL_FLOAT(0.0f, w.biasDeg);
}

static void test_the_bias_is_clamped()
{
    WallSteer w;
    WallParams p = params();
    p.ki = 5.0f;
    p.biasMaxDeg = 4.0f;
    p.steerMaxDeg = 100.0f;
    p.kp = 0.0f;
    for (int i = 0; i < 500; ++i) {
        trimOf(w, state(400, 150, 50), p); // err +50 for 5 s
    }
    TEST_ASSERT_EQUAL_FLOAT(4.0f, w.biasDeg);
}

static void test_a_saturated_trim_does_not_wind_the_bias_up()
{
    WallSteer w;
    WallParams p = params();
    p.ki = 1.0f;
    p.kp = 1.0f;
    p.steerMaxDeg = 5.0f; // err 40 -> P alone is 40 deg: pinned from the first tick
    p.biasMaxDeg = 100.0f;
    for (int i = 0; i < 200; ++i) {
        trimOf(w, state(400, 140, 60), p);
    }
    TEST_ASSERT_EQUAL_FLOAT(5.0f, trimOf(w, state(400, 140, 60), p));
    TEST_ASSERT_TRUE(w.biasDeg < 1.0f);
}

static void test_the_bias_keeps_applying_with_no_wall_in_view()
{
    WallSteer w;
    w.biasDeg = 3.0f;
    w.anchored = true;
    const WallParams p = params();
    TEST_ASSERT_EQUAL_FLOAT(3.0f, trimOf(w, state(400, 400, 400), p));
    TEST_ASSERT_EQUAL_FLOAT(3.0f, w.biasDeg);
}

static void test_the_bias_does_not_grow_with_no_wall_in_view()
{
    WallSteer w;
    WallParams p = params();
    p.ki = 1.0f;
    trimOf(w, state(400, 110, 90), p);
    const float learned = w.biasDeg;
    for (int i = 0; i < 100; ++i) {
        trimOf(w, state(400, 400, 400), p);
    }
    TEST_ASSERT_EQUAL_FLOAT(learned, w.biasDeg);
}

static void test_the_bias_is_held_through_a_gap_then_faded()
{
    WallSteer w;
    WallParams p = params();
    p.biasHoldMs = 500;
    p.biasFadeDegPerS = 2.0f;
    w.biasDeg = 3.0f;
    w.anchored = true;
    for (int i = 0; i < 50; ++i) { // 500 ms with no wall: still held
        trimOf(w, state(400, 400, 400), p);
    }
    TEST_ASSERT_EQUAL_FLOAT(3.0f, w.biasDeg);
    for (int i = 0; i < 100; ++i) { // 1 s past the hold: 2 deg/s faded
        trimOf(w, state(400, 400, 400), p);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 1.0f, w.biasDeg);
    for (int i = 0; i < 300; ++i) { // and it stops at zero
        trimOf(w, state(400, 400, 400), p);
    }
    TEST_ASSERT_EQUAL_FLOAT(0.0f, w.biasDeg);
}

static void test_a_negative_bias_fades_toward_zero_too()
{
    WallSteer w;
    WallParams p = params();
    p.biasHoldMs = 0;
    p.biasFadeDegPerS = 2.0f;
    w.biasDeg = -3.0f;
    w.anchored = true;
    for (int i = 0; i < 100; ++i) {
        trimOf(w, state(400, 400, 400), p);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.05f, -1.0f, w.biasDeg);
}

static void test_seeing_a_wall_again_restarts_the_hold()
{
    WallSteer w;
    WallParams p = params();
    p.biasHoldMs = 500;
    p.biasFadeDegPerS = 2.0f;
    w.biasDeg = 3.0f;
    w.anchored = true;
    for (int i = 0; i < 40; ++i) {
        trimOf(w, state(400, 400, 400), p);
    }
    trimOf(w, state(400, 100, 100), p); // a wall confirms it
    for (int i = 0; i < 40; ++i) {      // 400 ms again: held, because the count restarted
        trimOf(w, state(400, 400, 400), p);
    }
    TEST_ASSERT_EQUAL_FLOAT(3.0f, w.biasDeg);
}

static void test_a_turn_drops_the_bias_by_itself()
{
    WallSteer w;
    WallParams p = params();
    p.ki = 0.06f;
    for (int i = 0; i < 100; ++i) {
        trimOf(w, state(400, 110, 90), p, 0.0f);
    }
    TEST_ASSERT_TRUE(w.biasDeg > 0.1f);
    // Heading wanders by the trim's own authority: the bias stays.
    trimOf(w, state(400, 110, 90), p, 8.0f);
    TEST_ASSERT_TRUE(w.biasDeg > 0.1f);
    // A quarter turn: the reference is new, so what was learned is not valid.
    trimOf(w, state(400, 100, 100), p, 90.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, w.biasDeg);
    TEST_ASSERT_EQUAL_FLOAT(90.0f, w.anchorDeg);
}

static void test_the_heading_comparison_wraps_at_180()
{
    WallSteer w;
    WallParams p = params();
    p.ki = 0.06f;
    for (int i = 0; i < 100; ++i) {
        trimOf(w, state(400, 110, 90), p, 175.0f);
    }
    TEST_ASSERT_TRUE(w.biasDeg > 0.1f);
    trimOf(w, state(400, 110, 90), p, -175.0f); // 10 deg on, across the wrap
    TEST_ASSERT_TRUE(w.biasDeg > 0.1f);
}

static void test_reset()
{
    WallSteer w;
    w.mode = WallMode::LeftOnly;
    w.filtMm = 5.0f;
    w.biasDeg = 2.0f;
    w.anchored = true;
    wallSteerReset(w);
    TEST_ASSERT_TRUE(w.mode == WallMode::None);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, w.filtMm);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, w.biasDeg);
    TEST_ASSERT_FALSE(w.anchored);
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
    RUN_TEST(test_a_pair_that_disagrees_with_the_corridor_width_is_not_used);
    RUN_TEST(test_the_pair_gate_does_not_touch_a_single_wall);
    RUN_TEST(test_trim_steers_away_from_the_near_wall);
    RUN_TEST(test_trim_is_clamped);
    RUN_TEST(test_no_wall_means_no_trim_and_resets_the_mode);
    RUN_TEST(test_both_walls_mode);
    RUN_TEST(test_filter_restarts_when_the_wall_set_changes);
    RUN_TEST(test_right_only_mode);
    RUN_TEST(test_an_offset_that_stays_makes_the_bias_grow);
    RUN_TEST(test_a_zero_ki_learns_nothing);
    RUN_TEST(test_the_bias_is_clamped);
    RUN_TEST(test_a_saturated_trim_does_not_wind_the_bias_up);
    RUN_TEST(test_the_bias_keeps_applying_with_no_wall_in_view);
    RUN_TEST(test_the_bias_does_not_grow_with_no_wall_in_view);
    RUN_TEST(test_the_bias_is_held_through_a_gap_then_faded);
    RUN_TEST(test_a_negative_bias_fades_toward_zero_too);
    RUN_TEST(test_seeing_a_wall_again_restarts_the_hold);
    RUN_TEST(test_a_turn_drops_the_bias_by_itself);
    RUN_TEST(test_the_heading_comparison_wraps_at_180);
    RUN_TEST(test_reset);
    return UNITY_END();
}
