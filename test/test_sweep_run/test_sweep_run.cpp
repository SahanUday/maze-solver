// ============================================================================
//  test_sweep_run.cpp  --  the sweep run, driven through RunLayer against a
//  simple motor model (dead zone, then linear). No hardware needed.
//  One tick is CONTROL_LOOP_PERIOD_MS.
// ============================================================================

#include <unity.h>

#include "SweepRun.h"

namespace
{

constexpr int16_t kDeadZone = 100;
constexpr float kCountsPerDuty = 0.02f; // per tick, above the dead zone

float wheelRate(int16_t duty)
{
    const int16_t mag = duty < 0 ? static_cast<int16_t>(-duty) : duty;
    if (mag <= kDeadZone) {
        return 0.0f;
    }
    const float rate = (mag - kDeadZone) * kCountsPerDuty;
    return duty < 0 ? -rate : rate;
}

struct Result {
    SweepRow rows[320];
    uint16_t rowCount = 0;
    int16_t peakDuty = 0;
    bool idleWheelDriven = false;
    bool movedWhileSettling = false;
    bool negativeOutsideReverse = false;
    uint32_t endMs = 0;
    RunPhase phase = RunPhase::WaitStart;
    RunReason reason = RunReason::None;
    bool enableAtEnd = true;
};

Result simulate(bool wheelsMove = true)
{
    Result r;
    RunLayer layer;
    SweepRun run;
    RobotState s;
    RunOutput out;
    float encL = 0.0f;
    float encR = 0.0f;
    runLayerInit(layer, false); // no button: it starts itself
    sweepRunInit(run);

    while (!runLayerEnded(layer) && s.timestampMs < 400000) {
        s.timestampMs += CONTROL_LOOP_PERIOD_MS;
        if (wheelsMove && out.enable) {
            encL += wheelRate(out.left);
            encR += wheelRate(out.right);
            s.encoderCountL = static_cast<int32_t>(encL);
            s.encoderCountR = static_cast<int32_t>(encR);
        }
        out = runLayerStep(layer, run, s);

        SweepRow row;
        if (sweepRunTakeRow(run, row) && r.rowCount < 320) {
            r.rows[r.rowCount++] = row;
        }
        if (layer.phase == RunPhase::Sweep) {
            const bool right = run.combo >= 2;
            const bool reverse = (run.combo & 1) != 0;
            const int16_t idle = right ? out.left : out.right;
            const int16_t driven = right ? out.right : out.left;
            if (idle != 0) {
                r.idleWheelDriven = true;
            }
            if (run.settling && (out.left != 0 || out.right != 0)) {
                r.movedWhileSettling = true;
            }
            if ((driven < 0) != reverse && driven != 0) {
                r.negativeOutsideReverse = true;
            }
            const int16_t mag = driven < 0 ? static_cast<int16_t>(-driven) : driven;
            if (mag > r.peakDuty) {
                r.peakDuty = mag;
            }
        }
    }
    r.endMs = s.timestampMs;
    r.phase = layer.phase;
    r.reason = layer.reason;
    r.enableAtEnd = out.enable;
    return r;
}

} // namespace

static void test_the_ladder_goes_up_and_comes_back_from_the_top()
{
    TEST_ASSERT_EQUAL_INT16(SWEEP_DUTY_START, sweepDutyAt(0));
    TEST_ASSERT_EQUAL_INT16(SWEEP_DUTY_MAX, sweepDutyAt(SWEEP_STEPS_PER_RAMP - 1));
    TEST_ASSERT_EQUAL_INT16(SWEEP_DUTY_MAX, sweepDutyAt(SWEEP_STEPS_PER_RAMP));
    TEST_ASSERT_EQUAL_INT16(SWEEP_DUTY_START, sweepDutyAt(2 * SWEEP_STEPS_PER_RAMP - 1));
}

static void test_a_full_sweep_gives_every_row_in_order()
{
    const Result r = simulate();
    const uint16_t perCombo = 2 * SWEEP_STEPS_PER_RAMP;
    TEST_ASSERT_EQUAL_UINT16(SWEEP_COMBOS * perCombo, r.rowCount);
    for (uint16_t i = 0; i < r.rowCount; ++i) {
        const uint8_t combo = static_cast<uint8_t>(i / perCombo);
        const uint8_t step = static_cast<uint8_t>(i % perCombo);
        TEST_ASSERT_EQUAL(combo >= 2, r.rows[i].right);
        TEST_ASSERT_EQUAL((combo & 1) != 0, r.rows[i].reverse);
        TEST_ASSERT_EQUAL(step < SWEEP_STEPS_PER_RAMP, r.rows[i].up);
        TEST_ASSERT_EQUAL_INT16(sweepDutyAt(step), r.rows[i].duty);
    }
}

static void test_each_row_is_measured_over_the_window_and_matches_the_model()
{
    const Result r = simulate();
    const float ticks = SWEEP_MEASURE_MS / static_cast<float>(CONTROL_LOOP_PERIOD_MS);
    for (uint16_t i = 0; i < r.rowCount; ++i) {
        TEST_ASSERT_INT_WITHIN(CONTROL_LOOP_PERIOD_MS, SWEEP_MEASURE_MS, r.rows[i].dtMs);
        const float duty = r.rows[i].reverse ? -r.rows[i].duty : r.rows[i].duty;
        const float expected = wheelRate(static_cast<int16_t>(duty)) * ticks;
        const float tolerance =
            1.0f +
            (r.rows[i].duty > kDeadZone ? (r.rows[i].duty - kDeadZone) * kCountsPerDuty : 0.0f);
        TEST_ASSERT_FLOAT_WITHIN(tolerance, expected, static_cast<float>(r.rows[i].counts));
    }
}

static void test_reverse_rows_count_down()
{
    const Result r = simulate();
    for (uint16_t i = 0; i < r.rowCount; ++i) {
        if (r.rows[i].duty > kDeadZone + 20) {
            TEST_ASSERT_EQUAL(r.rows[i].reverse, r.rows[i].counts < 0);
        }
    }
}

static void test_only_the_swept_wheel_is_driven_and_the_sign_follows_the_direction()
{
    const Result r = simulate();
    TEST_ASSERT_FALSE(r.idleWheelDriven);
    TEST_ASSERT_FALSE(r.negativeOutsideReverse);
}

static void test_the_wheels_are_stopped_while_settling()
{
    const Result r = simulate();
    TEST_ASSERT_FALSE(r.movedWhileSettling);
}

static void test_the_duty_reaches_the_top_and_never_passes_it()
{
    const Result r = simulate();
    TEST_ASSERT_EQUAL_INT16(SWEEP_DUTY_MAX, r.peakDuty);
}

static void test_the_sweep_ends_complete_inside_its_time_limit()
{
    const Result r = simulate();
    TEST_ASSERT_EQUAL(static_cast<int>(RunPhase::Stopped), static_cast<int>(r.phase));
    TEST_ASSERT_EQUAL(static_cast<int>(RunReason::Complete), static_cast<int>(r.reason));
    TEST_ASSERT_FALSE(r.enableAtEnd);
    TEST_ASSERT_TRUE(r.endMs < RUN_AUTOSTART_MS + SWEEP_TIME_LIMIT_MS);
}

static void test_wheels_that_do_not_turn_fault_in_the_self_test_before_any_row()
{
    const Result r = simulate(false);
    TEST_ASSERT_EQUAL(static_cast<int>(RunPhase::Fault), static_cast<int>(r.phase));
    TEST_ASSERT_EQUAL(static_cast<int>(RunReason::SelfTestNoMotion), static_cast<int>(r.reason));
    TEST_ASSERT_EQUAL_UINT16(0, r.rowCount);
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_the_ladder_goes_up_and_comes_back_from_the_top);
    RUN_TEST(test_a_full_sweep_gives_every_row_in_order);
    RUN_TEST(test_each_row_is_measured_over_the_window_and_matches_the_model);
    RUN_TEST(test_reverse_rows_count_down);
    RUN_TEST(test_only_the_swept_wheel_is_driven_and_the_sign_follows_the_direction);
    RUN_TEST(test_the_wheels_are_stopped_while_settling);
    RUN_TEST(test_the_duty_reaches_the_top_and_never_passes_it);
    RUN_TEST(test_the_sweep_ends_complete_inside_its_time_limit);
    RUN_TEST(test_wheels_that_do_not_turn_fault_in_the_self_test_before_any_row);
    return UNITY_END();
}
