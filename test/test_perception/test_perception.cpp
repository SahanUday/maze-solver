// ============================================================================
//  test_perception.cpp  --  perceive(): RobotState in, distance / speed / yaw rate / walls /
//  incline / line view out. No hardware needed.
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

static void test_heading_and_yaw_rate_are_passed_through()
{
    Rig r;
    r.s.headingDeg = 42.0f;
    r.s.yawRateDps = -17.5f;
    r.tick();
    TEST_ASSERT_EQUAL_FLOAT(42.0f, r.out.motion.headingDeg);
    TEST_ASSERT_EQUAL_FLOAT(-17.5f, r.out.motion.yawRateDps);
}

static void test_a_heading_snap_is_not_a_yaw_rate()
{
    // The mission snaps the heading to the grid; the robot is not turning.
    Rig r;
    r.s.headingDeg = 0.0f;
    r.s.yawRateDps = 0.0f;
    r.tick();
    r.s.headingDeg = 3.0f;
    r.tick();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, r.out.motion.yawRateDps);
    TEST_ASSERT_EQUAL_FLOAT(3.0f, r.out.motion.headingDeg);
}

static void test_the_defaults_are_the_spec_values()
{
    const PerceiveParams d;
    TEST_ASSERT_EQUAL_FLOAT(MM_PER_ENCODER_COUNT, d.mmPerCount);
    TEST_ASSERT_EQUAL_UINT16(CONTROL_LOOP_PERIOD_MS, d.dtMs);
    TEST_ASSERT_EQUAL_FLOAT(IR_SENSOR_PITCH_MM, d.irPitchMm);
    TEST_ASSERT_EQUAL_UINT16(WALL_PRESENT_SIDE_MM, d.wall.sideMm);
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

static void test_the_incline_is_the_forward_axis_above_level()
{
    Rig r;
    r.s.imu.accel[IMU_ACCEL_AXIS_FORWARD] = IMU_ACCEL_FORWARD_LEVEL_COUNTS;
    r.tick();
    TEST_ASSERT_EQUAL_INT16(0, r.out.inclineCounts);
    r.s.imu.accel[IMU_ACCEL_AXIS_FORWARD] = static_cast<int16_t>(
        IMU_ACCEL_FORWARD_LEVEL_COUNTS + (IMU_ACCEL_NOSE_UP_NEGATIVE ? -2400 : 2400));
    r.tick();
    TEST_ASSERT_EQUAL_INT16(2400, r.out.inclineCounts);
}

static void test_a_captured_level_replaces_the_bench_value()
{
    Rig r;
    perceiverSetLevel(r.pc, 1000);
    r.s.imu.accel[IMU_ACCEL_AXIS_FORWARD] = 1000;
    r.tick();
    TEST_ASSERT_EQUAL_INT16(0, r.out.inclineCounts);
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

static void test_without_a_calibration_the_line_view_reports_nothing()
{
    Rig r;
    for (auto &c : r.s.irRaw) {
        c = 900;
    }
    r.tick();
    TEST_ASSERT_FALSE(r.out.line.valid);
    TEST_ASSERT_FALSE(r.out.line.allBlack);
    TEST_ASSERT_FALSE(r.out.line.allWhite);
    TEST_ASSERT_FALSE(r.out.line.onLine);
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
    RUN_TEST(test_heading_and_yaw_rate_are_passed_through);
    RUN_TEST(test_a_heading_snap_is_not_a_yaw_rate);
    RUN_TEST(test_the_defaults_are_the_spec_values);
    RUN_TEST(test_the_wall_view_uses_the_thresholds);
    RUN_TEST(test_a_silent_sensor_is_neither_wall_nor_open);
    RUN_TEST(test_the_incline_is_the_forward_axis_above_level);
    RUN_TEST(test_a_captured_level_replaces_the_bench_value);
    RUN_TEST(test_a_calibration_marks_its_usable_channels);
    RUN_TEST(test_without_a_calibration_the_line_view_reports_nothing);
    RUN_TEST(test_the_start_button_is_passed_through);
    RUN_TEST(test_reset_forgets_the_run);
    return UNITY_END();
}
