// ============================================================================
//  test_run_layer.cpp  --  the layer under a stub run, so these hold for any run,
//  not just SafeRun. No hardware needed. One tick is CONTROL_LOOP_PERIOD_MS.
// ============================================================================

#include <unity.h>

#include "RunLayer.h"

namespace
{

// A run that asks for whatever the test sets, and records how it was called.
struct StubRun {
    RunLimits limits;
    RunReason ready = RunReason::None;
    RunWant want;
    int begins = 0;
    int steps = 0;
};

// Found by argument-dependent lookup, so they sit in StubRun's namespace.
inline RunLimits runLimits(const StubRun &run)
{
    return run.limits;
}

inline RunReason runReady(const StubRun &run, const RobotState &)
{
    return run.ready;
}

inline RunWant runBegin(StubRun &run, const RobotState &)
{
    ++run.begins;
    RunWant want;
    want.hardStop = true;
    want.phase = RunPhase::Settle;
    return want;
}

inline RunWant runStep(StubRun &run, const RobotState &)
{
    ++run.steps;
    return run.want;
}

struct Rig {
    RunLayer layer;
    StubRun run;
    RobotState s;
    RunOutput out;
    bool wheelsTurn = true;
};

Rig makeRig(bool buttonFitted = true)
{
    Rig r;
    runLayerInit(r.layer, buttonFitted);
    r.run.limits.maxDuty = 600;
    r.run.limits.rampPerTick = 50;
    r.run.limits.timeLimitMs = 5000;
    r.run.limits.selfTestDuty = 300;
    r.run.want.phase = RunPhase::Cruise;
    return r;
}

void tick(Rig &r)
{
    r.s.timestampMs += CONTROL_LOOP_PERIOD_MS;
    if (r.wheelsTurn && r.out.enable) {
        r.s.encoderCountL += 5;
        r.s.encoderCountR += 5;
    }
    r.out = runLayerStep(r.layer, r.run, r.s);
}

void run(Rig &r, uint32_t ms)
{
    for (uint32_t t = 0; t < ms; t += CONTROL_LOOP_PERIOD_MS) {
        tick(r);
    }
}

// Release, press, count down and self-test: the next tick asks the run for its first want.
void startRun(Rig &r)
{
    r.s.startButtonPressed = false;
    run(r, 50);
    r.s.startButtonPressed = true;
    run(r, 50);
    r.s.startButtonPressed = false;
    r.s.usValid = RUN_US_FRONT;
    r.s.distFrontMm = 500;
    run(r, RUN_COUNTDOWN_MS + RUN_SELFTEST_MS + 100);
}

int phase(const Rig &r)
{
    return static_cast<int>(r.layer.phase);
}

} // namespace

static void test_nothing_is_driven_until_the_button()
{
    Rig r = makeRig();
    run(r, 1000);
    TEST_ASSERT_EQUAL(static_cast<int>(RunPhase::WaitStart), phase(r));
    TEST_ASSERT_FALSE(r.out.enable);
    TEST_ASSERT_EQUAL(0, r.run.steps);
}

static void test_a_run_that_is_not_ready_faults_with_its_reason_and_never_drives()
{
    Rig r = makeRig();
    r.run.ready = RunReason::FrontBlind;
    r.s.startButtonPressed = false;
    run(r, 50);
    r.s.startButtonPressed = true;
    run(r, 50);
    r.s.startButtonPressed = false;
    run(r, RUN_COUNTDOWN_MS + 100);
    TEST_ASSERT_EQUAL(static_cast<int>(RunPhase::Fault), phase(r));
    TEST_ASSERT_EQUAL(static_cast<int>(RunReason::FrontBlind), static_cast<int>(r.layer.reason));
    TEST_ASSERT_FALSE(r.out.enable);
}

static void test_self_test_runs_at_the_runs_duty_then_hands_over()
{
    Rig r = makeRig();
    startRun(r);
    TEST_ASSERT_EQUAL(1, r.run.begins);
    TEST_ASSERT_TRUE(r.run.steps > 0);
    TEST_ASSERT_EQUAL(static_cast<int>(RunPhase::Cruise), phase(r));
}

static void test_wheel_that_does_not_move_in_the_self_test_faults()
{
    Rig r = makeRig();
    r.wheelsTurn = false;
    startRun(r);
    TEST_ASSERT_EQUAL(static_cast<int>(RunReason::SelfTestNoMotion),
                      static_cast<int>(r.layer.reason));
    TEST_ASSERT_EQUAL(0, r.run.begins);
}

static void test_the_runs_cap_is_enforced_on_every_wheel()
{
    Rig r = makeRig();
    r.run.limits.maxDuty = 400;
    r.run.want.left = 9000;
    r.run.want.right = -9000;
    startRun(r);
    run(r, 500);
    TEST_ASSERT_EQUAL_INT16(400, r.out.left);
    TEST_ASSERT_EQUAL_INT16(-400, r.out.right);
}

static void test_the_cap_never_exceeds_the_pwm_range()
{
    Rig r = makeRig();
    r.run.limits.maxDuty = 30000;
    r.run.want.left = 30000;
    startRun(r);
    run(r, 2000);
    TEST_ASSERT_EQUAL_INT16(static_cast<int16_t>(MOTOR_PWM_TOP), r.out.left);
}

static void test_a_run_with_a_higher_cap_is_not_held_to_the_safe_one()
{
    Rig r = makeRig();
    r.run.limits.maxDuty = 760;
    r.run.want.left = 760;
    r.run.want.right = 760;
    startRun(r);
    run(r, 2000);
    TEST_ASSERT_EQUAL_INT16(760, r.out.left);
}

static void test_output_ramps_by_the_runs_step()
{
    Rig r = makeRig();
    r.run.limits.rampPerTick = 10;
    r.run.want.left = 300;
    startRun(r);
    run(r, CONTROL_LOOP_PERIOD_MS);
    const int16_t first = r.out.left;
    run(r, CONTROL_LOOP_PERIOD_MS);
    TEST_ASSERT_EQUAL_INT16(10, r.out.left - first);
}

static void test_a_hard_stop_zeroes_the_wheels_at_once()
{
    Rig r = makeRig();
    r.run.want.left = 500;
    r.run.want.right = 500;
    startRun(r);
    run(r, 1000);
    TEST_ASSERT_TRUE(r.out.left > 100);
    r.run.want.left = 0;
    r.run.want.right = 0;
    r.run.want.hardStop = true;
    run(r, CONTROL_LOOP_PERIOD_MS);
    TEST_ASSERT_EQUAL_INT16(0, r.out.left);
    TEST_ASSERT_EQUAL_INT16(0, r.out.right);
    TEST_ASSERT_TRUE(r.out.enable);
}

static void test_a_run_ending_with_a_reason_stops_or_faults_by_kind()
{
    Rig r = makeRig();
    startRun(r);
    r.run.want.end = RunReason::TooManyTurns;
    run(r, CONTROL_LOOP_PERIOD_MS);
    TEST_ASSERT_EQUAL(static_cast<int>(RunPhase::Stopped), phase(r));
    TEST_ASSERT_FALSE(r.out.enable);

    Rig f = makeRig();
    startRun(f);
    f.run.want.end = RunReason::Stall;
    run(f, CONTROL_LOOP_PERIOD_MS);
    TEST_ASSERT_EQUAL(static_cast<int>(RunPhase::Fault), phase(f));
    TEST_ASSERT_EQUAL(static_cast<int>(RunReason::Stall), static_cast<int>(f.layer.reason));
}

static void test_the_run_time_limit_comes_from_the_run()
{
    Rig r = makeRig();
    r.run.limits.timeLimitMs = 2000;
    startRun(r);
    run(r, 3000);
    TEST_ASSERT_EQUAL(static_cast<int>(RunPhase::Stopped), phase(r));
    TEST_ASSERT_EQUAL(static_cast<int>(RunReason::TimeLimit), static_cast<int>(r.layer.reason));
}

static void test_the_button_stops_a_live_run()
{
    Rig r = makeRig();
    r.run.want.left = 300;
    startRun(r);
    run(r, 500);
    TEST_ASSERT_TRUE(r.out.enable);
    r.s.startButtonPressed = true;
    run(r, 100);
    TEST_ASSERT_EQUAL(static_cast<int>(RunPhase::Stopped), phase(r));
    TEST_ASSERT_EQUAL(static_cast<int>(RunReason::EStop), static_cast<int>(r.layer.reason));
    TEST_ASSERT_FALSE(r.out.enable);
}

static void test_without_a_button_the_run_starts_itself()
{
    Rig r = makeRig(false);
    r.s.usValid = RUN_US_FRONT;
    r.s.distFrontMm = 500;
    run(r, RUN_AUTOSTART_MS - 500);
    TEST_ASSERT_EQUAL(static_cast<int>(RunPhase::Countdown), phase(r));
    run(r, 1000);
    TEST_ASSERT_EQUAL(static_cast<int>(RunPhase::SelfTest), phase(r));
}

static void test_a_run_without_a_front_stop_ignores_a_close_wall_in_the_self_test()
{
    Rig r = makeRig();
    r.run.limits.frontStopMm = 0;
    r.s.startButtonPressed = false;
    run(r, 50);
    r.s.startButtonPressed = true;
    run(r, 50);
    r.s.startButtonPressed = false;
    r.s.usValid = RUN_US_FRONT;
    r.s.distFrontMm = 20;
    run(r, RUN_COUNTDOWN_MS + RUN_SELFTEST_MS + 100);
    TEST_ASSERT_EQUAL(1, r.run.begins);
}

static void test_encoders_counting_down_in_the_self_test_fault()
{
    Rig r = makeRig();
    r.s.startButtonPressed = false;
    run(r, 50);
    r.s.startButtonPressed = true;
    run(r, 50);
    r.s.startButtonPressed = false;
    r.s.usValid = RUN_US_FRONT;
    r.s.distFrontMm = 500;
    run(r, RUN_COUNTDOWN_MS + 20);
    for (uint32_t t = 0; t < RUN_SELFTEST_MS + 50; t += CONTROL_LOOP_PERIOD_MS) {
        r.s.encoderCountL -= 5;
        r.s.encoderCountR -= 5;
        r.s.timestampMs += CONTROL_LOOP_PERIOD_MS;
        r.out = runLayerStep(r.layer, r.run, r.s);
    }
    TEST_ASSERT_EQUAL(static_cast<int>(RunReason::SelfTestReversed),
                      static_cast<int>(r.layer.reason));
    TEST_ASSERT_FALSE(r.out.enable);
}

static void test_a_wall_inside_the_runs_front_stop_ends_the_self_test()
{
    Rig r = makeRig();
    r.run.limits.frontStopMm = 70;
    r.s.startButtonPressed = false;
    run(r, 50);
    r.s.startButtonPressed = true;
    run(r, 50);
    r.s.startButtonPressed = false;
    r.s.usValid = RUN_US_FRONT;
    r.s.distFrontMm = 60;
    run(r, RUN_COUNTDOWN_MS + RUN_SELFTEST_MS + 100);
    TEST_ASSERT_EQUAL(static_cast<int>(RunPhase::Stopped), phase(r));
    TEST_ASSERT_EQUAL(static_cast<int>(RunReason::FrontWall), static_cast<int>(r.layer.reason));
    TEST_ASSERT_EQUAL(0, r.run.begins);
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_nothing_is_driven_until_the_button);
    RUN_TEST(test_a_run_that_is_not_ready_faults_with_its_reason_and_never_drives);
    RUN_TEST(test_self_test_runs_at_the_runs_duty_then_hands_over);
    RUN_TEST(test_wheel_that_does_not_move_in_the_self_test_faults);
    RUN_TEST(test_the_runs_cap_is_enforced_on_every_wheel);
    RUN_TEST(test_the_cap_never_exceeds_the_pwm_range);
    RUN_TEST(test_a_run_with_a_higher_cap_is_not_held_to_the_safe_one);
    RUN_TEST(test_output_ramps_by_the_runs_step);
    RUN_TEST(test_a_hard_stop_zeroes_the_wheels_at_once);
    RUN_TEST(test_a_run_ending_with_a_reason_stops_or_faults_by_kind);
    RUN_TEST(test_the_run_time_limit_comes_from_the_run);
    RUN_TEST(test_the_button_stops_a_live_run);
    RUN_TEST(test_without_a_button_the_run_starts_itself);
    RUN_TEST(test_a_run_without_a_front_stop_ignores_a_close_wall_in_the_self_test);
    RUN_TEST(test_encoders_counting_down_in_the_self_test_fault);
    RUN_TEST(test_a_wall_inside_the_runs_front_stop_ends_the_self_test);
    return UNITY_END();
}
