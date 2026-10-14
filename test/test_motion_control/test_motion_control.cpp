// ============================================================================
//  test_motion_control.cpp  --  motionControlStep() tests. No hardware needed.
// ============================================================================

#include <unity.h>

#include "MotionControl.h"

static MotionControlParams params()
{
    MotionControlParams p;
    p.pwmTop = 799;
    return p;
}

static void test_all_zero_gains_output_nothing()
{
    MotionControl mc;
    MotionTarget t;
    t.distRefMm = 100.0f;
    t.speedRefMmS = 200.0f;
    const MotionOutput o = motionControlStep(mc, params(), t, MotionFeedback{}, 10);
    TEST_ASSERT_EQUAL_INT16(0, o.left);
    TEST_ASSERT_EQUAL_INT16(0, o.right);
}

static void test_speed_feedforward_drives_both_wheels_equally()
{
    MotionControl mc;
    MotionControlParams p = params();
    p.ffPwmPerMmS = 1.0f;
    MotionTarget t;
    t.speedRefMmS = 300.0f;
    const MotionOutput o = motionControlStep(mc, p, t, MotionFeedback{}, 10);
    TEST_ASSERT_EQUAL_INT16(300, o.left);
    TEST_ASSERT_EQUAL_INT16(300, o.right);
}

static void test_distance_error_pushes_forward()
{
    MotionControl mc;
    MotionControlParams p = params();
    p.kpDist = 2.0f;
    MotionTarget t;
    t.distRefMm = 50.0f;
    const MotionOutput o = motionControlStep(mc, p, t, MotionFeedback{}, 10);
    TEST_ASSERT_EQUAL_INT16(100, o.left);
    TEST_ASSERT_EQUAL_INT16(100, o.right);
}

static void test_speed_error_term_uses_the_derivative_gain()
{
    MotionControl mc;
    MotionControlParams p = params();
    p.kdDist = 0.5f;
    MotionTarget t;
    t.speedRefMmS = 200.0f;
    MotionFeedback fb;
    fb.speedMmS = 100.0f;
    const MotionOutput o = motionControlStep(mc, p, t, fb, 10);
    TEST_ASSERT_EQUAL_INT16(50, o.left);
}

static void test_positive_heading_error_speeds_the_right_wheel()
{
    // Heading is CCW positive: wanting +10 deg means turning left, which needs the right
    // wheel faster.
    MotionControl mc;
    MotionControlParams p = params();
    p.kpTurn = 5.0f;
    MotionTarget t;
    t.headingRefDeg = 10.0f;
    const MotionOutput o = motionControlStep(mc, p, t, MotionFeedback{}, 10);
    TEST_ASSERT_EQUAL_INT16(-50, o.left);
    TEST_ASSERT_EQUAL_INT16(50, o.right);
}

static void test_heading_error_takes_the_short_way_round()
{
    MotionControl mc;
    MotionControlParams p = params();
    p.kpTurn = 1.0f;
    MotionTarget t;
    t.headingRefDeg = -170.0f;
    MotionFeedback fb;
    fb.headingDeg = 170.0f; // 20 deg CCW away across the wrap, not 340 CW
    const MotionOutput o = motionControlStep(mc, p, t, fb, 10);
    TEST_ASSERT_EQUAL_INT16(20, o.right);
    TEST_ASSERT_EQUAL_INT16(-20, o.left);
}

static void test_yaw_rate_feedforward_and_damping()
{
    MotionControl mc;
    MotionControlParams p = params();
    p.ffPwmPerDps = 2.0f;
    p.kdTurn = 1.0f;
    MotionTarget t;
    t.yawRateRefDps = 100.0f;
    MotionFeedback fb;
    fb.yawRateDps = 40.0f;
    const MotionOutput o = motionControlStep(mc, p, t, fb, 10);
    TEST_ASSERT_EQUAL_INT16(260, o.right); // 2*100 + 1*(100-40)
    TEST_ASSERT_EQUAL_INT16(-260, o.left);
}

static void test_duty_is_clamped_to_top()
{
    MotionControl mc;
    MotionControlParams p = params();
    p.kpDist = 100.0f;
    MotionTarget t;
    t.distRefMm = 1000.0f;
    const MotionOutput o = motionControlStep(mc, p, t, MotionFeedback{}, 10);
    TEST_ASSERT_EQUAL_INT16(799, o.left);
    TEST_ASSERT_EQUAL_INT16(799, o.right);
}

static void test_deadband_lifts_small_commands_past_the_start_threshold()
{
    MotionControl mc;
    MotionControlParams p = params();
    p.kpDist = 1.0f;
    p.minPwmL = 100;
    p.minPwmR = 200;
    MotionTarget t;
    t.distRefMm = 10.0f;
    const MotionOutput o = motionControlStep(mc, p, t, MotionFeedback{}, 10);
    TEST_ASSERT_TRUE(o.left >= 100 && o.left < 120);
    TEST_ASSERT_TRUE(o.right >= 200 && o.right < 220);
}

static void test_deadband_keeps_the_sign_and_never_exceeds_top()
{
    MotionControl mc;
    MotionControlParams p = params();
    p.kpDist = 1.0f;
    p.minPwmL = 100;
    p.minPwmR = 100;
    MotionTarget t;
    t.distRefMm = -5000.0f;
    const MotionOutput o = motionControlStep(mc, p, t, MotionFeedback{}, 10);
    TEST_ASSERT_EQUAL_INT16(-799, o.left);
    TEST_ASSERT_EQUAL_INT16(-799, o.right);
}

static void test_tiny_command_is_zero_not_a_start_pulse()
{
    MotionControl mc;
    MotionControlParams p = params();
    p.kpDist = 0.01f;
    p.minPwmL = 100;
    p.minPwmR = 100;
    MotionTarget t;
    t.distRefMm = 10.0f; // 0.1 duty
    const MotionOutput o = motionControlStep(mc, p, t, MotionFeedback{}, 10);
    TEST_ASSERT_EQUAL_INT16(0, o.left);
}

static void test_no_pwm_top_means_no_output()
{
    MotionControl mc;
    MotionControlParams p;
    p.kpDist = 5.0f;
    MotionTarget t;
    t.distRefMm = 100.0f;
    TEST_ASSERT_EQUAL_INT16(0, motionControlStep(mc, p, t, MotionFeedback{}, 10).left);
}

static void test_integral_accumulates_and_is_clamped()
{
    MotionControl mc;
    MotionControlParams p = params();
    p.kiDist = 1.0f;
    MotionTarget t;
    t.distRefMm = 100.0f;
    MotionOutput o;
    for (int i = 0; i < 1000; ++i) {
        o = motionControlStep(mc, p, t, MotionFeedback{}, 10);
    }
    TEST_ASSERT_EQUAL_INT16(static_cast<int16_t>(MOTION_INTEGRAL_MAX_DUTY), o.left);
}

static void test_integral_is_off_when_its_gain_is_zero()
{
    MotionControl mc;
    MotionControlParams p = params();
    MotionTarget t;
    t.distRefMm = 100.0f;
    motionControlStep(mc, p, t, MotionFeedback{}, 10);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, mc.distIntegral);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, mc.turnIntegral);
}

static void test_turn_integral_acts_on_heading_error()
{
    MotionControl mc;
    MotionControlParams p = params();
    p.kiTurn = 10.0f;
    MotionTarget t;
    t.headingRefDeg = 10.0f;
    MotionOutput o;
    for (int i = 0; i < 10; ++i) {
        o = motionControlStep(mc, p, t, MotionFeedback{}, 10);
    }
    TEST_ASSERT_TRUE(o.right > 0);
    TEST_ASSERT_TRUE(o.left < 0);
}

static MotionControlParams stallParams()
{
    MotionControlParams p = params();
    p.kpDist = 10.0f;
    p.stallDuty = 100;
    p.stallTimeMs = 50;
    p.stallMinSpeedMmS = 5.0f;
    p.stallMinYawDps = 5.0f;
    return p;
}

static void test_stall_cuts_the_output_and_latches()
{
    MotionControl mc;
    const MotionControlParams p = stallParams();
    MotionTarget t;
    t.distRefMm = 100.0f;
    MotionOutput o;
    for (int i = 0; i < 10; ++i) {
        o = motionControlStep(mc, p, t, MotionFeedback{}, 10);
    }
    TEST_ASSERT_TRUE(mc.stalled);
    TEST_ASSERT_EQUAL_INT16(0, o.left);
    // Stays latched even when the target is reached.
    t.distRefMm = 0.0f;
    o = motionControlStep(mc, p, t, MotionFeedback{}, 10);
    TEST_ASSERT_TRUE(mc.stalled);
    TEST_ASSERT_EQUAL_INT16(0, o.right);
}

static void test_moving_robot_is_not_stalled()
{
    MotionControl mc;
    const MotionControlParams p = stallParams();
    MotionTarget t;
    t.distRefMm = 100.0f;
    MotionFeedback fb;
    fb.speedMmS = 50.0f;
    for (int i = 0; i < 100; ++i) {
        motionControlStep(mc, p, t, fb, 10);
    }
    TEST_ASSERT_FALSE(mc.stalled);
}

static void test_a_rotating_robot_is_not_stalled()
{
    MotionControl mc;
    const MotionControlParams p = stallParams();
    MotionTarget t;
    t.distRefMm = 100.0f;
    MotionFeedback fb;
    fb.yawRateDps = 90.0f; // pivoting: translation is zero but the robot is not stuck
    for (int i = 0; i < 100; ++i) {
        motionControlStep(mc, p, t, fb, 10);
    }
    TEST_ASSERT_FALSE(mc.stalled);
}

static void test_a_brief_stop_resets_the_stall_timer()
{
    MotionControl mc;
    const MotionControlParams p = stallParams();
    MotionTarget t;
    t.distRefMm = 100.0f;
    MotionFeedback moving;
    moving.speedMmS = 50.0f;
    for (int round = 0; round < 10; ++round) {
        for (int i = 0; i < 3; ++i) {
            motionControlStep(mc, p, t, MotionFeedback{}, 10); // 30 ms still: under the limit
        }
        motionControlStep(mc, p, t, moving, 10);
    }
    TEST_ASSERT_FALSE(mc.stalled);
}

static void test_stall_protection_is_off_by_default()
{
    MotionControl mc;
    MotionControlParams p = params();
    p.kpDist = 10.0f;
    MotionTarget t;
    t.distRefMm = 100.0f;
    for (int i = 0; i < 1000; ++i) {
        motionControlStep(mc, p, t, MotionFeedback{}, 10);
    }
    TEST_ASSERT_FALSE(mc.stalled);
}

static void test_reset_clears_a_latched_stall()
{
    MotionControl mc;
    mc.stalled = true;
    mc.distIntegral = 5.0f;
    motionControlReset(mc);
    TEST_ASSERT_FALSE(mc.stalled);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, mc.distIntegral);
}

static void test_clearing_the_integrals_keeps_the_stall_latch()
{
    MotionControl mc;
    mc.distIntegral = 12.0f;
    mc.turnIntegral = -30.0f;
    mc.stalled = true;
    motionControlClearIntegrals(mc);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, mc.distIntegral);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, mc.turnIntegral);
    TEST_ASSERT_TRUE(mc.stalled);
}

static void test_a_new_move_starts_from_zero_integrals()
{
    MotionControlParams p = params();
    p.kiDist = 2.0f;
    p.kiTurn = 20.0f;
    p.kiHold = 2.0f;
    MotionControl mc;
    mc.distIntegral = 10.0f; // wound up by the last move
    mc.turnIntegral = 1.0f;
    MotionTarget t;
    t.newMove = true;
    const MotionOutput o = motionControlStep(mc, p, t, MotionFeedback{}, 10);
    TEST_ASSERT_EQUAL_INT16(0, o.left);
    TEST_ASSERT_EQUAL_INT16(0, o.right);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, mc.distIntegral);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, mc.turnIntegral);
}

static void test_a_move_in_progress_keeps_its_integrals()
{
    MotionControlParams p = params();
    p.kiDist = 2.0f;
    MotionControl mc;
    mc.distIntegral = 10.0f;
    const MotionOutput o = motionControlStep(mc, p, MotionTarget{}, MotionFeedback{}, 10);
    TEST_ASSERT_EQUAL_INT16(20, o.left); // kiDist * integral
    TEST_ASSERT_EQUAL_FLOAT(10.0f, mc.distIntegral);
}

static void test_a_new_move_keeps_the_stall_latch()
{
    MotionControlParams p = params();
    MotionControl mc;
    mc.stalled = true;
    MotionTarget t;
    t.newMove = true;
    motionControlStep(mc, p, t, MotionFeedback{}, 10);
    TEST_ASSERT_TRUE(mc.stalled);
}

// The heading integral has its own gain while driving: the turn-sized one rings on a straight.
static float heldAngular(const MotionControlParams &p, bool pivot)
{
    MotionControl mc;
    MotionTarget t;
    t.headingRefDeg = 10.0f; // 10 deg to turn left, nothing else asked
    t.pivot = pivot;
    MotionOutput o;
    for (int i = 0; i < 100; ++i) { // one second
        o = motionControlStep(mc, p, t, MotionFeedback{}, 10);
    }
    return static_cast<float>(o.right - o.left) * 0.5f;
}

static void test_a_straight_uses_the_hold_integral_and_a_pivot_the_turn_one()
{
    MotionControlParams p = params();
    p.kiTurn = 20.0f;
    p.kiHold = 2.0f;
    const float straight = heldAngular(p, false); // 2 * 10 deg * 1 s = 20
    const float pivot = heldAngular(p, true);     // 20 * 10 deg * 1 s = 200
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 20.0f, straight);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 200.0f, pivot);
}

// The integral breaks the wheels loose; once the error has flipped it points the wrong way.
static void test_a_pivot_drops_an_integral_that_points_against_the_error()
{
    MotionControl mc;
    MotionControlParams p = params();
    p.kiTurn = 20.0f;
    MotionTarget t;
    t.pivot = true;
    t.headingRefDeg = 10.0f;
    for (int i = 0; i < 50; ++i) {
        motionControlStep(mc, p, t, MotionFeedback{}, 10); // wound up against +10 deg
    }
    TEST_ASSERT_TRUE(mc.turnIntegral > 0.0f);
    MotionFeedback fb;
    fb.headingDeg = 12.0f; // overshot: the error is now -2 deg
    motionControlStep(mc, p, t, fb, 10);
    TEST_ASSERT_TRUE(mc.turnIntegral <= 0.0f);
}

static void test_a_straight_keeps_its_heading_integral_through_an_overshoot()
{
    MotionControl mc;
    MotionControlParams p = params();
    p.kiHold = 2.0f;
    MotionTarget t;
    t.headingRefDeg = 10.0f;
    for (int i = 0; i < 50; ++i) {
        motionControlStep(mc, p, t, MotionFeedback{}, 10);
    }
    const float wound = mc.turnIntegral;
    MotionFeedback fb;
    fb.headingDeg = 10.5f;
    motionControlStep(mc, p, t, fb, 10);
    TEST_ASSERT_TRUE(mc.turnIntegral > 0.9f * wound); // it holds a steady disturbance
}

static void test_a_negative_hold_gain_falls_back_to_the_turn_gain()
{
    MotionControlParams p = params();
    p.kiTurn = 20.0f; // kiHold stays at its default, -1
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 200.0f, heldAngular(p, false));
}

static void test_the_right_scale_multiplies_only_the_right_command()
{
    MotionControl mc;
    MotionControlParams p = params();
    p.ffPwmPerMmS = 1.0f;
    p.ffScaleR = 1.2f;
    MotionTarget t;
    t.speedRefMmS = 200.0f;
    const MotionOutput o = motionControlStep(mc, p, t, MotionFeedback{}, 10);
    TEST_ASSERT_EQUAL_INT16(200, o.left);
    TEST_ASSERT_EQUAL_INT16(240, o.right);
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_a_straight_uses_the_hold_integral_and_a_pivot_the_turn_one);
    RUN_TEST(test_a_negative_hold_gain_falls_back_to_the_turn_gain);
    RUN_TEST(test_a_pivot_drops_an_integral_that_points_against_the_error);
    RUN_TEST(test_a_straight_keeps_its_heading_integral_through_an_overshoot);
    RUN_TEST(test_the_right_scale_multiplies_only_the_right_command);
    RUN_TEST(test_clearing_the_integrals_keeps_the_stall_latch);
    RUN_TEST(test_all_zero_gains_output_nothing);
    RUN_TEST(test_speed_feedforward_drives_both_wheels_equally);
    RUN_TEST(test_distance_error_pushes_forward);
    RUN_TEST(test_speed_error_term_uses_the_derivative_gain);
    RUN_TEST(test_positive_heading_error_speeds_the_right_wheel);
    RUN_TEST(test_heading_error_takes_the_short_way_round);
    RUN_TEST(test_yaw_rate_feedforward_and_damping);
    RUN_TEST(test_duty_is_clamped_to_top);
    RUN_TEST(test_deadband_lifts_small_commands_past_the_start_threshold);
    RUN_TEST(test_deadband_keeps_the_sign_and_never_exceeds_top);
    RUN_TEST(test_tiny_command_is_zero_not_a_start_pulse);
    RUN_TEST(test_no_pwm_top_means_no_output);
    RUN_TEST(test_integral_accumulates_and_is_clamped);
    RUN_TEST(test_integral_is_off_when_its_gain_is_zero);
    RUN_TEST(test_turn_integral_acts_on_heading_error);
    RUN_TEST(test_stall_cuts_the_output_and_latches);
    RUN_TEST(test_moving_robot_is_not_stalled);
    RUN_TEST(test_a_rotating_robot_is_not_stalled);
    RUN_TEST(test_a_brief_stop_resets_the_stall_timer);
    RUN_TEST(test_stall_protection_is_off_by_default);
    RUN_TEST(test_reset_clears_a_latched_stall);
    RUN_TEST(test_a_new_move_starts_from_zero_integrals);
    RUN_TEST(test_a_move_in_progress_keeps_its_integrals);
    RUN_TEST(test_a_new_move_keeps_the_stall_latch);
    return UNITY_END();
}
