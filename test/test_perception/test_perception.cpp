// ============================================================================
//  test_perception.cpp  --  perceive(): RobotState in, distance / speed / yaw rate / walls / tilt /
//  line view out. No hardware needed.
// ============================================================================

#include <unity.h>

#include "Perception.h"

constexpr uint16_t DT_MS = 10;

static PerceiveParams params()
{
    PerceiveParams p;
    p.mmPerCount = 0.5f;
    p.dtMs = DT_MS;
    p.wall.sideMm = 200;
    p.wall.frontMm = 280;
    p.wall.centerDistMm = 120;
    p.irPitchMm = 10.0f;
    return p;
}

struct Rig {
    Perceiver pc;
    Perception out;
    PerceiveParams p = params();
    RobotState s;

    Rig()
    {
        perceiverReset(pc);
    }

    void tick()
    {
        perceive(pc, s, p, out);
    }
};

static void test_the_first_tick_only_records_the_counts()
{
    Rig r;
    r.s.encoderCountL = 5000;
    r.s.encoderCountR = 5000;
    r.tick();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, r.out.motion.distMm);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, r.out.motion.speedMmS);
}

static void test_distance_is_the_mean_of_both_wheels_in_mm()
{
    Rig r;
    r.tick();
    r.s.encoderCountL = 100;
    r.s.encoderCountR = 300;
    r.tick();
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 100.0f, r.out.motion.distMm); // 200 counts * 0.5 mm
}

static void test_speed_settles_at_the_true_speed()
{
    Rig r;
    r.tick();
    for (int i = 1; i <= 60; ++i) {
        r.s.encoderCountL = 2 * i; // 1 mm per tick = 100 mm/s
        r.s.encoderCountR = 2 * i;
        r.tick();
    }
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 100.0f, r.out.motion.speedMmS);
}

static void test_yaw_rate_comes_from_the_heading_change()
{
    Rig r;
    r.tick();
    for (int i = 1; i <= 60; ++i) {
        r.s.headingDeg = 0.1f * static_cast<float>(i); // 0.1 deg per 10 ms = 10 deg/s
        r.tick();
    }
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 10.0f, r.out.motion.yawRateDps);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 6.0f, r.out.motion.headingDeg);
}

static void test_yaw_rate_is_not_fooled_by_the_wrap_at_180()
{
    Rig r;
    r.s.headingDeg = 179.95f;
    r.tick();
    for (int i = 0; i < 60; ++i) {
        r.s.headingDeg = (i % 2 == 0) ? -179.95f : 179.95f; // +-0.1 deg across the wrap, in turn
        r.tick();
    }
    TEST_ASSERT_TRUE(r.out.motion.yawRateDps < 12.0f && r.out.motion.yawRateDps > -12.0f);
}

static void test_the_wall_view_uses_the_thresholds()
{
    Rig r;
    r.s.usValid = 0x07;
    r.s.distFrontMm = 100;
    r.s.distLeftMm = 90;
    r.s.distRightMm = 400;
    r.tick();
    TEST_ASSERT_TRUE(r.out.walls.front);
    TEST_ASSERT_TRUE(r.out.walls.left);
    TEST_ASSERT_FALSE(r.out.walls.right);
}

static void test_a_silent_sensor_is_neither_wall_nor_open()
{
    Rig r;
    r.s.usValid = 0x03; // right has no echo
    r.s.distFrontMm = 500;
    r.s.distLeftMm = 90;
    r.s.distRightMm = 0;
    r.tick();
    TEST_ASSERT_FALSE(r.out.walls.rightValid);
    TEST_ASSERT_FALSE(r.out.walls.right);
}

static void test_tilt_is_the_angle_from_vertical()
{
    Rig r;
    r.s.imu.accel[2] = 16384;
    r.tick();
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, r.out.tiltDeg);
    r.s.imu.accel[0] = 16384;
    r.tick();
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 45.0f, r.out.tiltDeg);
}

static void test_a_calibration_marks_its_usable_channels()
{
    Rig r;
    LineCalibration cal{};
    cal.scale[0] = 1;
    cal.scale[3] = 1;
    cal.scale[7] = 1;
    perceiverSetCalibration(r.pc, cal);
    TEST_ASSERT_EQUAL_HEX8(0b10001001, r.pc.usable);
}

static void test_without_a_calibration_the_floor_is_never_reported()
{
    Rig r;
    for (auto &c : r.s.irRaw) {
        c = 900;
    }
    r.tick();
    TEST_ASSERT_FALSE(r.out.line.allBlack);
}

static void test_the_start_button_is_passed_through()
{
    Rig r;
    r.tick();
    TEST_ASSERT_FALSE(r.out.startPressed);
    r.s.startButtonPressed = true;
    r.tick();
    TEST_ASSERT_TRUE(r.out.startPressed);
}

static void test_reset_forgets_the_run()
{
    Rig r;
    r.tick();
    r.s.encoderCountL = 400;
    r.s.encoderCountR = 400;
    r.tick();
    TEST_ASSERT_TRUE(r.out.motion.distMm > 0.0f);
    perceiverReset(r.pc);
    r.tick();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, r.out.motion.distMm);
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_the_first_tick_only_records_the_counts);
    RUN_TEST(test_distance_is_the_mean_of_both_wheels_in_mm);
    RUN_TEST(test_speed_settles_at_the_true_speed);
    RUN_TEST(test_yaw_rate_comes_from_the_heading_change);
    RUN_TEST(test_yaw_rate_is_not_fooled_by_the_wrap_at_180);
    RUN_TEST(test_the_wall_view_uses_the_thresholds);
    RUN_TEST(test_a_silent_sensor_is_neither_wall_nor_open);
    RUN_TEST(test_tilt_is_the_angle_from_vertical);
    RUN_TEST(test_a_calibration_marks_its_usable_channels);
    RUN_TEST(test_without_a_calibration_the_floor_is_never_reported);
    RUN_TEST(test_the_start_button_is_passed_through);
    RUN_TEST(test_reset_forgets_the_run);
    return UNITY_END();
}
