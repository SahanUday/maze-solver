// ============================================================================
//  test_line_follow.cpp  --  the line follower: steering, searching, the ramp, the end. No
//  hardware.
// ============================================================================

#include <unity.h>

#include "motion/LineFollow.h"

static LineParams params()
{
    LineParams p;
    p.kp = 0.3f;
    p.kd = 0.0f;
    p.steerMaxDeg = 15.0f;
    p.speedMmS = 130.0f;
    p.accelMmS2 = 400.0f;
    p.searchRateDps = 60.0f;
    p.searchSpeedMmS = 0.0f;
    p.lostTimeoutMs = 3000;
    return p;
}

static LineView onLine(float errMm)
{
    LineView v;
    v.valid = true;
    v.onLine = true;
    v.errMm = errMm;
    return v;
}

static LineView allWhite()
{
    LineView v;
    v.valid = true;
    v.allWhite = true;
    return v;
}

static LineView allBlack()
{
    LineView v;
    v.valid = true;
    v.allBlack = true;
    return v;
}

constexpr uint16_t DT = 10;

static void test_a_line_to_the_left_steers_left_by_kp_times_the_error()
{
    LineFollow lf;
    MotionFeedback fb;
    fb.headingDeg = 10.0f;
    lineFollowBegin(lf, fb, true);
    MotionTarget t;
    const LineStatus st = lineFollowStep(lf, params(), fb, onLine(10.0f), 0, DT, t);
    TEST_ASSERT_EQUAL(static_cast<int>(LineStatus::Following), static_cast<int>(st));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f + 0.3f * 10.0f, t.headingRefDeg); // CCW = left
    TEST_ASSERT_TRUE(t.newMove);
    lineFollowStep(lf, params(), fb, onLine(10.0f), 0, DT, t);
    TEST_ASSERT_FALSE(t.newMove);
}

static void test_the_steering_offset_is_clamped()
{
    LineFollow lf;
    MotionFeedback fb;
    lineFollowBegin(lf, fb, true);
    MotionTarget t;
    LineParams p = params();
    p.kp = 5.0f;
    lineFollowStep(lf, p, fb, onLine(-30.0f), 0, DT, t);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -15.0f, t.headingRefDeg);
}

static void test_the_speed_ramps_to_the_line_speed_and_is_a_reference_not_a_distance()
{
    LineFollow lf;
    MotionFeedback fb;
    lineFollowBegin(lf, fb, true);
    MotionTarget t;
    for (int i = 0; i < 200; ++i) {
        lineFollowStep(lf, params(), fb, onLine(0.0f), 0, DT, t);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 130.0f, t.speedRefMmS);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, fb.distMm, t.distRefMm);
    TEST_ASSERT_FALSE(t.pivot);
}

static void test_a_lost_line_is_searched_for_toward_the_side_it_was_last_seen()
{
    LineFollow lf;
    MotionFeedback fb;
    lineFollowBegin(lf, fb, true);
    MotionTarget t;
    lineFollowStep(lf, params(), fb, onLine(-12.0f), 0, DT, t); // last seen on the right
    const LineStatus st = lineFollowStep(lf, params(), fb, allWhite(), 0, DT, t);
    TEST_ASSERT_EQUAL(static_cast<int>(LineStatus::Searching), static_cast<int>(st));
    TEST_ASSERT_TRUE(t.yawRateRefDps < 0.0f); // clockwise
    TEST_ASSERT_TRUE(t.headingRefDeg < fb.headingDeg);
}

static void test_the_search_gives_up_after_the_timeout()
{
    LineFollow lf;
    MotionFeedback fb;
    lineFollowBegin(lf, fb, true);
    MotionTarget t;
    LineStatus st = LineStatus::Following;
    int ticks = 0;
    while (st != LineStatus::Lost && ticks < 1000) {
        st = lineFollowStep(lf, params(), fb, allWhite(), 0, DT, t);
        ++ticks;
    }
    TEST_ASSERT_EQUAL(static_cast<int>(LineStatus::Lost), static_cast<int>(st));
    TEST_ASSERT_INT_WITHIN(2, 300, ticks); // 3000 ms at 10 ms
}

static void test_finding_the_line_again_ends_the_search()
{
    LineFollow lf;
    MotionFeedback fb;
    lineFollowBegin(lf, fb, true);
    MotionTarget t;
    for (int i = 0; i < 50; ++i) {
        lineFollowStep(lf, params(), fb, allWhite(), 0, DT, t);
    }
    const LineStatus st = lineFollowStep(lf, params(), fb, onLine(3.0f), 0, DT, t);
    TEST_ASSERT_EQUAL(static_cast<int>(LineStatus::Following), static_cast<int>(st));
}

static void test_all_black_is_not_a_lost_line()
{
    LineFollow lf;
    MotionFeedback fb;
    fb.headingDeg = 20.0f;
    lineFollowBegin(lf, fb, true);
    MotionTarget t;
    lineFollowStep(lf, params(), fb, onLine(0.0f), 0, DT, t);
    LineStatus st = LineStatus::Searching;
    for (int i = 0; i < 400; ++i) {
        st = lineFollowStep(lf, params(), fb, allBlack(), 0, DT, t);
    }
    TEST_ASSERT_EQUAL(static_cast<int>(LineStatus::Following), static_cast<int>(st));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 20.0f, t.headingRefDeg); // along the last good heading
}

static void test_all_white_on_the_ramp_is_ignored()
{
    LineParams p = params();
    p.rampInclineCounts = 1800;
    LineFollow lf;
    MotionFeedback fb;
    lineFollowBegin(lf, fb, true);
    MotionTarget t;
    LineStatus st = LineStatus::Searching;
    for (int i = 0; i < 400; ++i) {
        st = lineFollowStep(lf, p, fb, allWhite(), 3000, DT, t); // a lifted array on the slope
    }
    TEST_ASSERT_EQUAL(static_cast<int>(LineStatus::Following), static_cast<int>(st));
    st = lineFollowStep(lf, p, fb, allWhite(), 0, DT, t); // flat again: now it is lost ground
    TEST_ASSERT_EQUAL(static_cast<int>(LineStatus::Searching), static_cast<int>(st));
}

static void test_a_line_gone_for_the_end_distance_ends_the_run()
{
    LineParams p = params();
    p.endLostMm = 100.0f;
    p.searchSpeedMmS = 60.0f;
    LineFollow lf;
    MotionFeedback fb;
    lineFollowBegin(lf, fb, true);
    MotionTarget t;
    lineFollowStep(lf, p, fb, onLine(0.0f), 0, DT, t);
    LineStatus st = LineStatus::Following;
    for (int i = 0; i < 2000 && st != LineStatus::Ended; ++i) {
        fb.distMm += t.speedRefMmS * 0.01f;
        fb.speedMmS = t.speedRefMmS;
        st = lineFollowStep(lf, p, fb, allWhite(), 0, DT, t);
    }
    TEST_ASSERT_EQUAL(static_cast<int>(LineStatus::Ended), static_cast<int>(st));
}

static void test_the_end_rule_is_off_when_the_line_was_never_seen()
{
    LineParams p = params();
    p.endLostMm = 50.0f;
    p.searchSpeedMmS = 60.0f;
    LineFollow lf;
    MotionFeedback fb;
    lineFollowBegin(lf, fb, false);
    MotionTarget t;
    LineStatus st = LineStatus::Following;
    for (int i = 0; i < 200; ++i) {
        fb.distMm += 1.0f;
        st = lineFollowStep(lf, p, fb, allWhite(), 0, DT, t);
    }
    TEST_ASSERT_TRUE(st != LineStatus::Ended);
}

static void test_a_big_error_slows_the_robot_when_asked()
{
    LineParams p = params();
    p.slowdownErrMm = 20.0f;
    p.slowdownPct = 50;
    LineFollow lf;
    MotionFeedback fb;
    lineFollowBegin(lf, fb, true);
    MotionTarget t;
    for (int i = 0; i < 300; ++i) {
        lineFollowStep(lf, p, fb, onLine(20.0f), 0, DT, t);
    }
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 65.0f, t.speedRefMmS);
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_a_line_to_the_left_steers_left_by_kp_times_the_error);
    RUN_TEST(test_the_steering_offset_is_clamped);
    RUN_TEST(test_the_speed_ramps_to_the_line_speed_and_is_a_reference_not_a_distance);
    RUN_TEST(test_a_lost_line_is_searched_for_toward_the_side_it_was_last_seen);
    RUN_TEST(test_the_search_gives_up_after_the_timeout);
    RUN_TEST(test_finding_the_line_again_ends_the_search);
    RUN_TEST(test_all_black_is_not_a_lost_line);
    RUN_TEST(test_all_white_on_the_ramp_is_ignored);
    RUN_TEST(test_a_line_gone_for_the_end_distance_ends_the_run);
    RUN_TEST(test_the_end_rule_is_off_when_the_line_was_never_seen);
    RUN_TEST(test_a_big_error_slows_the_robot_when_asked);
    return UNITY_END();
}
