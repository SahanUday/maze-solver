// ============================================================================
//  test_perception.cpp  --  perceive(): RobotState in, distance / speed / yaw rate / walls /
//  incline / line view out. No hardware needed.
// ============================================================================

#include <initializer_list>
#include <unity.h>

#include "sense/Perception.h"
#include "spec/Drivetrain.h"
#include "spec/Imu.h"
#include "spec/LineSensor.h"
#include "spec/Platform.h"
#include "spec/WallFollow.h"

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

static void test_a_side_sensor_that_never_changes_is_frozen()
{
    Rig r;
    r.s.usValid = 0x07;
    r.s.distLeftMm = 100;
    for (int i = 0; i < WALL_FROZEN_MS / DT_MS + 2; ++i) {
        r.s.distRightMm = (i & 1) ? 110 : 112; // scatters, like a real reading
        r.tick();
    }
    TEST_ASSERT_FALSE(r.out.walls.leftValid);
    TEST_ASSERT_FALSE(r.out.walls.left); // not a wall, and not "open" either
    TEST_ASSERT_TRUE(r.out.walls.rightValid);
    TEST_ASSERT_EQUAL_UINT16(0, r.out.sideBlindMs);
}

static void test_a_reading_that_scatters_is_not_frozen()
{
    Rig r;
    r.s.usValid = 0x07;
    for (int i = 0; i < 300; ++i) {
        r.s.distLeftMm = (i & 1) ? 100 : 103;
        r.s.distRightMm = (i & 1) ? 120 : 117;
        r.tick();
    }
    TEST_ASSERT_TRUE(r.out.walls.leftValid);
    TEST_ASSERT_TRUE(r.out.walls.rightValid);
}

// One stray echo among agreeing neighbours is dropped; a real change comes through once two echoes
// agree on it.
static void test_a_stray_echo_is_not_passed_on()
{
    Rig r;
    r.s.usValid = 0x07;
    r.s.distFrontMm = 400;
    r.s.distRightMm = 120;
    uint16_t seen = 0;
    for (uint16_t mm : {117, 119, 118, 780, 117, 119, 118}) {
        r.s.distLeftMm = mm;
        r.tick();
        seen = r.out.walls.leftMm > seen ? r.out.walls.leftMm : seen;
        TEST_ASSERT_TRUE(r.out.walls.left);
    }
    TEST_ASSERT_TRUE(seen < 200);
}

static void test_a_real_change_passes_after_two_echoes()
{
    Rig r;
    r.s.usValid = 0x07;
    r.s.distFrontMm = 400;
    r.s.distRightMm = 120;
    for (uint16_t mm : {117, 119, 118, 117}) {
        r.s.distLeftMm = mm;
        r.tick();
    }
    r.s.distLeftMm = 402; // the wall ends
    r.tick();
    TEST_ASSERT_TRUE(r.out.walls.left); // one echo is not yet believed
    r.s.distLeftMm = 399;
    r.tick();
    TEST_ASSERT_FALSE(r.out.walls.left);
    TEST_ASSERT_TRUE(r.out.walls.leftMm >= 399);
}

static void test_a_front_reading_that_does_not_change_while_moving_is_frozen()
{
    Rig r;
    r.s.usValid = 0x07;
    r.s.distFrontMm = 300;
    r.tick();
    TEST_ASSERT_TRUE(r.out.walls.frontValid);
    for (int i = 1; i <= 100; ++i) { // 0.5 mm a count: 1 mm a tick
        r.s.encoderCountL = 2 * i;
        r.s.encoderCountR = 2 * i;
        r.tick();
    }
    TEST_ASSERT_FALSE(r.out.walls.frontValid);
    TEST_ASSERT_FALSE(r.out.walls.front);
    r.s.distFrontMm = 240; // the value moves: it is a sensor again
    r.tick();
    TEST_ASSERT_TRUE(r.out.walls.frontValid);
}

static void test_a_steady_front_reading_at_rest_is_not_frozen()
{
    Rig r;
    r.s.usValid = 0x07;
    r.s.distFrontMm = 117;
    for (int i = 0; i < 500; ++i) {
        r.tick();
    }
    TEST_ASSERT_TRUE(r.out.walls.frontValid);
}

static void test_a_frozen_sensor_comes_back_when_the_value_moves()
{
    Rig r;
    r.s.usValid = 0x07;
    r.s.distLeftMm = 100;
    for (int i = 0; i < WALL_FROZEN_MS / DT_MS + 5; ++i) {
        r.tick();
    }
    TEST_ASSERT_FALSE(r.out.walls.leftValid);
    r.s.distLeftMm = 103;
    r.tick();
    TEST_ASSERT_TRUE(r.out.walls.leftValid);
    TEST_ASSERT_TRUE(r.out.walls.left);
}

static void test_the_blind_time_counts_only_while_no_side_has_a_reading()
{
    Rig r;
    r.s.usValid = 0x01; // front only: both sides silent
    for (int i = 0; i < 25; ++i) {
        r.tick();
    }
    TEST_ASSERT_EQUAL_UINT16(25 * DT_MS, r.out.sideBlindMs);
    r.s.usValid = 0x03;
    r.s.distLeftMm = 100;
    r.tick();
    TEST_ASSERT_EQUAL_UINT16(0, r.out.sideBlindMs);
}

static void test_an_open_side_is_not_blind()
{
    // Far readings that arrive are valid: an opening is information, not a failure.
    Rig r;
    r.s.usValid = 0x07;
    for (int i = 0; i < 100; ++i) {
        r.s.distLeftMm = static_cast<uint16_t>(400 + (i & 3));
        r.s.distRightMm = static_cast<uint16_t>(405 + (i & 1));
        r.tick();
    }
    TEST_ASSERT_EQUAL_UINT16(0, r.out.sideBlindMs);
    TEST_ASSERT_TRUE(r.out.walls.leftValid);
    TEST_ASSERT_FALSE(r.out.walls.left);
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

// Calibrated on white 600 and black 1000 on every channel.
static void calibrate(Rig &r)
{
    LineCalibration cal;
    lineCalibrationReset(cal);
    uint16_t counts[IR_CHANNEL_COUNT];
    for (auto &c : counts) {
        c = 600;
    }
    lineCalibrationAccumulate(cal, counts);
    for (auto &c : counts) {
        c = 1000;
    }
    lineCalibrationAccumulate(cal, counts);
    lineCalibrationFinish(cal);
    perceiverSetCalibration(r.pc, cal);
}

static void test_the_floor_is_black_white_or_mixed_by_the_line_view()
{
    Rig r;
    calibrate(r);
    for (auto &c : r.s.irRaw) {
        c = 1000;
    }
    r.tick();
    r.tick();
    TEST_ASSERT_EQUAL(static_cast<int>(FloorClass::Black), static_cast<int>(r.out.floor));
    for (auto &c : r.s.irRaw) {
        c = 600;
    }
    for (int i = 0; i < 4; ++i) {
        r.tick(); // the mask has hysteresis: it needs a few sweeps to settle
    }
    TEST_ASSERT_EQUAL(static_cast<int>(FloorClass::White), static_cast<int>(r.out.floor));
    r.s.irRaw[0] = 1000; // one end on black
    r.tick();
    TEST_ASSERT_EQUAL(static_cast<int>(FloorClass::Mixed), static_cast<int>(r.out.floor));
}

static void test_a_saturated_array_is_air_even_with_no_calibration()
{
    Rig r;
    for (auto &c : r.s.irRaw) {
        c = 1020;
    }
    r.tick();
    TEST_ASSERT_EQUAL(static_cast<int>(FloorClass::Air), static_cast<int>(r.out.floor));
}

static void test_the_array_end_that_is_d1_gives_the_sign_of_the_error()
{
    Rig r;
    calibrate(r);
    for (auto &c : r.s.irRaw) {
        c = 600;
    }
    r.s.irRaw[0] = 1000; // D1, the robot's right end
    for (int i = 0; i < 4; ++i) {
        r.tick();
    }
    TEST_ASSERT_TRUE(r.out.line.onLine);
    TEST_ASSERT_TRUE(r.out.line.errMm < 0.0f); // positive is left; D1 is on the right
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
    RUN_TEST(test_a_side_sensor_that_never_changes_is_frozen);
    RUN_TEST(test_a_reading_that_scatters_is_not_frozen);
    RUN_TEST(test_a_stray_echo_is_not_passed_on);
    RUN_TEST(test_a_real_change_passes_after_two_echoes);
    RUN_TEST(test_a_front_reading_that_does_not_change_while_moving_is_frozen);
    RUN_TEST(test_a_steady_front_reading_at_rest_is_not_frozen);
    RUN_TEST(test_a_frozen_sensor_comes_back_when_the_value_moves);
    RUN_TEST(test_the_blind_time_counts_only_while_no_side_has_a_reading);
    RUN_TEST(test_an_open_side_is_not_blind);
    RUN_TEST(test_the_incline_is_the_forward_axis_above_level);
    RUN_TEST(test_a_captured_level_replaces_the_bench_value);
    RUN_TEST(test_a_calibration_marks_its_usable_channels);
    RUN_TEST(test_without_a_calibration_the_line_view_reports_nothing);
    RUN_TEST(test_the_floor_is_black_white_or_mixed_by_the_line_view);
    RUN_TEST(test_a_saturated_array_is_air_even_with_no_calibration);
    RUN_TEST(test_the_array_end_that_is_d1_gives_the_sign_of_the_error);
    RUN_TEST(test_the_start_button_is_passed_through);
    RUN_TEST(test_reset_forgets_the_run);
    return UNITY_END();
}
