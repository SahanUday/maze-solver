// ============================================================================
//  test_motion_primitives.cpp  --  Straight and Turn, closed loop against a small simulated
//  plant (sim.h). No hardware needed.
// ============================================================================

#include <unity.h>

#include "sim.h"

constexpr uint16_t DT_MS = 10;
constexpr float DT_S = 0.01f;

struct Rig {
    Sim sim;
    MotionControl mc;
    MotionControlParams cp = simControlParams();
    PlanParams plan = simPlanParams();

    // Step the loop once with the target a primitive produced.
    void drive(const MotionTarget &t)
    {
        sim.step(motionControlStep(mc, cp, t, sim.feedback(), DT_MS), DT_S);
    }
};

static void test_straight_stops_on_the_target()
{
    Rig r;
    Straight s;
    straightBegin(s, r.sim.feedback(), 250.0f, 300.0f, 0.0f, 0.0f);
    MoveStatus st = MoveStatus::Running;
    for (int i = 0; i < 400 && st == MoveStatus::Running; ++i) {
        MotionTarget t;
        st = straightStep(s, r.plan, r.sim.feedback(), 0.0f, DT_MS, t);
        r.drive(t);
    }
    TEST_ASSERT_EQUAL(static_cast<int>(MoveStatus::Done), static_cast<int>(st));
    TEST_ASSERT_FLOAT_WITHIN(3.0f, 250.0f, r.sim.distMm);
}

static void test_straight_holds_heading_against_a_disturbance()
{
    Rig r;
    r.sim.headingDeg = 8.0f; // kicked off the line
    Straight s;
    straightBegin(s, r.sim.feedback(), 250.0f, 300.0f, 0.0f, 0.0f);
    for (int i = 0; i < 200; ++i) {
        MotionTarget t;
        straightStep(s, r.plan, r.sim.feedback(), 0.0f, DT_MS, t);
        r.drive(t);
    }
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, r.sim.headingDeg);
}

static void test_straight_trim_steers_the_heading()
{
    Rig r;
    Straight s;
    straightBegin(s, r.sim.feedback(), 1000.0f, 200.0f, 0.0f, 0.0f);
    for (int i = 0; i < 100; ++i) {
        MotionTarget t;
        straightStep(s, r.plan, r.sim.feedback(), 5.0f, DT_MS, t);
        r.drive(t);
    }
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 5.0f, r.sim.headingDeg);
}

static void test_pass_through_is_done_at_speed()
{
    Rig r;
    Straight s;
    straightBegin(s, r.sim.feedback(), 250.0f, 300.0f, 300.0f, 0.0f);
    MoveStatus st = MoveStatus::Running;
    for (int i = 0; i < 400 && st == MoveStatus::Running; ++i) {
        MotionTarget t;
        st = straightStep(s, r.plan, r.sim.feedback(), 0.0f, DT_MS, t);
        r.drive(t);
    }
    TEST_ASSERT_EQUAL(static_cast<int>(MoveStatus::Done), static_cast<int>(st));
    TEST_ASSERT_TRUE(r.sim.speedMmS > 200.0f); // still rolling
}

static void test_extend_chains_a_second_cell_without_stopping()
{
    Rig r;
    Straight s;
    straightBegin(s, r.sim.feedback(), 250.0f, 300.0f, 300.0f, 0.0f);
    float minSpeedMid = 1e9f;
    bool extended = false;
    MoveStatus st = MoveStatus::Running;
    for (int i = 0; i < 800 && st == MoveStatus::Running; ++i) {
        if (!extended && r.sim.distMm > 150.0f) {
            straightExtend(s, 250.0f, 0.0f); // decided late: carry on one more cell, then stop
            extended = true;
        }
        MotionTarget t;
        st = straightStep(s, r.plan, r.sim.feedback(), 0.0f, DT_MS, t);
        r.drive(t);
        if (r.sim.distMm > 200.0f && r.sim.distMm < 300.0f) {
            minSpeedMid = r.sim.speedMmS < minSpeedMid ? r.sim.speedMmS : minSpeedMid;
        }
    }
    TEST_ASSERT_EQUAL(static_cast<int>(MoveStatus::Done), static_cast<int>(st));
    TEST_ASSERT_FLOAT_WITHIN(3.0f, 500.0f, r.sim.distMm);
    TEST_ASSERT_TRUE(minSpeedMid > 200.0f);
}

static void test_retarget_moves_the_stop()
{
    Rig r;
    Straight s;
    straightBegin(s, r.sim.feedback(), 250.0f, 300.0f, 0.0f, 0.0f);
    bool done = false;
    for (int i = 0; i < 400 && !done; ++i) {
        if (i == 30) {
            straightRetarget(s, 220.0f, 0.0f); // a wall said the centre is 30 mm nearer
        }
        MotionTarget t;
        done = straightStep(s, r.plan, r.sim.feedback(), 0.0f, DT_MS, t) == MoveStatus::Done;
        r.drive(t);
    }
    TEST_ASSERT_TRUE(done);
    TEST_ASSERT_FLOAT_WITHIN(3.0f, 220.0f, r.sim.distMm);
}

static void test_straight_times_out()
{
    Rig r;
    r.plan.moveTimeoutMs = 100;
    r.plan.accelMmS2 = 0.0f; // cannot move
    Straight s;
    straightBegin(s, r.sim.feedback(), 250.0f, 300.0f, 0.0f, 0.0f);
    MoveStatus st = MoveStatus::Running;
    for (int i = 0; i < 20 && st == MoveStatus::Running; ++i) {
        MotionTarget t;
        st = straightStep(s, r.plan, r.sim.feedback(), 0.0f, DT_MS, t);
    }
    TEST_ASSERT_EQUAL(static_cast<int>(MoveStatus::Timeout), static_cast<int>(st));
}

static void test_straight_without_a_timeout_never_gives_up()
{
    Rig r;
    r.plan.moveTimeoutMs = 0;
    r.plan.accelMmS2 = 0.0f;
    Straight s;
    straightBegin(s, r.sim.feedback(), 250.0f, 300.0f, 0.0f, 0.0f);
    MoveStatus st = MoveStatus::Running;
    for (int i = 0; i < 1000; ++i) {
        MotionTarget t;
        st = straightStep(s, r.plan, r.sim.feedback(), 0.0f, DT_MS, t);
    }
    TEST_ASSERT_EQUAL(static_cast<int>(MoveStatus::Running), static_cast<int>(st));
}

static void test_straight_begins_at_the_measured_speed()
{
    Rig r;
    r.sim.speedMmS = 150.0f;
    Straight s;
    straightBegin(s, r.sim.feedback(), 250.0f, 300.0f, 0.0f, 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(150.0f, s.refSpeedMmS);
}

// Run a turn to completion; returns the status it ended on.
static MoveStatus runTurn(Rig &r, float targetDeg, float deltaDeg, int maxTicks = 600)
{
    Turn t;
    turnBegin(t, r.sim.feedback(), targetDeg, deltaDeg);
    MoveStatus st = MoveStatus::Running;
    for (int i = 0; i < maxTicks && st == MoveStatus::Running; ++i) {
        MotionTarget tg;
        st = turnStep(t, r.plan, r.sim.feedback(), DT_MS, tg);
        r.drive(tg);
    }
    return st;
}

static void test_turn_left_90()
{
    Rig r;
    TEST_ASSERT_EQUAL(static_cast<int>(MoveStatus::Done),
                      static_cast<int>(runTurn(r, 90.0f, 90.0f)));
    TEST_ASSERT_FLOAT_WITHIN(2.5f, 90.0f, r.sim.headingDeg);
}

static void test_turn_right_90()
{
    Rig r;
    TEST_ASSERT_EQUAL(static_cast<int>(MoveStatus::Done),
                      static_cast<int>(runTurn(r, -90.0f, -90.0f)));
    TEST_ASSERT_FLOAT_WITHIN(2.5f, -90.0f, r.sim.headingDeg);
}

static void test_turn_180_goes_the_way_it_was_told()
{
    Rig r;
    // Told to go CCW: it must pass +90 on the way, not -90.
    Turn t;
    turnBegin(t, r.sim.feedback(), 180.0f, 180.0f);
    bool sawPositive = false;
    MoveStatus st = MoveStatus::Running;
    for (int i = 0; i < 800 && st == MoveStatus::Running; ++i) {
        MotionTarget tg;
        st = turnStep(t, r.plan, r.sim.feedback(), DT_MS, tg);
        r.drive(tg);
        sawPositive = sawPositive || (r.sim.headingDeg > 80.0f && r.sim.headingDeg < 100.0f);
    }
    TEST_ASSERT_EQUAL(static_cast<int>(MoveStatus::Done), static_cast<int>(st));
    TEST_ASSERT_TRUE(sawPositive);
    TEST_ASSERT_TRUE(r.sim.headingDeg > 177.0f || r.sim.headingDeg < -177.0f);
}

static void test_turn_across_the_wrap_point()
{
    Rig r;
    r.sim.headingDeg = 170.0f;
    TEST_ASSERT_EQUAL(static_cast<int>(MoveStatus::Done),
                      static_cast<int>(runTurn(r, -100.0f, 90.0f)));
    TEST_ASSERT_FLOAT_WITHIN(2.5f, -100.0f, r.sim.headingDeg);
}

static void test_turn_corrects_start_error_by_aiming_at_the_grid_heading()
{
    Rig r;
    r.sim.headingDeg = 6.0f; // drifted off the grid before the turn
    runTurn(r, 90.0f, 90.0f);
    TEST_ASSERT_FLOAT_WITHIN(2.5f, 90.0f, r.sim.headingDeg); // not 96
}

static void test_turn_stays_on_the_spot()
{
    Rig r;
    runTurn(r, 90.0f, 90.0f);
    TEST_ASSERT_FLOAT_WITHIN(3.0f, 0.0f, r.sim.distMm);
}

static void test_turn_times_out()
{
    Rig r;
    r.plan.moveTimeoutMs = 100;
    r.plan.turnAccelDps2 = 0.0f;
    TEST_ASSERT_EQUAL(static_cast<int>(MoveStatus::Timeout),
                      static_cast<int>(runTurn(r, 90.0f, 90.0f, 50)));
}

static void test_zero_degree_turn_finishes_immediately()
{
    Rig r;
    TEST_ASSERT_EQUAL(static_cast<int>(MoveStatus::Done),
                      static_cast<int>(runTurn(r, 0.0f, 0.0f, 20)));
}

static void test_turn_waits_out_the_settle_time()
{
    Rig r;
    r.plan.turnSettleMs = 200;
    Turn t;
    turnBegin(t, r.sim.feedback(), 0.0f, 0.0f);
    MotionTarget tg;
    // Already on target, but not for long enough yet.
    TEST_ASSERT_EQUAL(static_cast<int>(MoveStatus::Running),
                      static_cast<int>(turnStep(t, r.plan, r.sim.feedback(), DT_MS, tg)));
}

// A chain of cells is one Straight that keeps being extended; its timeout is per leg.
static void test_extending_a_straight_restarts_its_timeout()
{
    PlanParams plan = simPlanParams();
    plan.moveTimeoutMs = 1000;
    Straight s;
    MotionFeedback fb;
    straightBegin(s, fb, 250.0f, 150.0f, 0.0f, 0.0f);
    MotionTarget t;
    for (int i = 0; i < 80; ++i) { // 800 ms of a 1000 ms limit
        TEST_ASSERT_EQUAL(static_cast<int>(MoveStatus::Running),
                          static_cast<int>(straightStep(s, plan, fb, 0.0f, 10, t)));
    }
    straightExtend(s, 250.0f, 0.0f);
    for (int i = 0; i < 80; ++i) { // another 800 ms: 1600 ms in all, but only 800 on this leg
        TEST_ASSERT_EQUAL(static_cast<int>(MoveStatus::Running),
                          static_cast<int>(straightStep(s, plan, fb, 0.0f, 10, t)));
    }
}

static void test_only_a_turn_asks_for_the_pivot_integral()
{
    const PlanParams plan = simPlanParams();
    MotionFeedback fb;
    MotionTarget t;
    Straight s;
    straightBegin(s, fb, 250.0f, 150.0f, 0.0f, 0.0f);
    straightStep(s, plan, fb, 0.0f, 10, t);
    TEST_ASSERT_FALSE(t.pivot);
    Turn turn;
    turnBegin(turn, fb, 90.0f, 90.0f);
    turnStep(turn, plan, fb, 10, t);
    TEST_ASSERT_TRUE(t.pivot);
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_only_a_turn_asks_for_the_pivot_integral);
    RUN_TEST(test_extending_a_straight_restarts_its_timeout);
    RUN_TEST(test_straight_stops_on_the_target);
    RUN_TEST(test_straight_holds_heading_against_a_disturbance);
    RUN_TEST(test_straight_trim_steers_the_heading);
    RUN_TEST(test_pass_through_is_done_at_speed);
    RUN_TEST(test_extend_chains_a_second_cell_without_stopping);
    RUN_TEST(test_retarget_moves_the_stop);
    RUN_TEST(test_straight_times_out);
    RUN_TEST(test_straight_without_a_timeout_never_gives_up);
    RUN_TEST(test_straight_begins_at_the_measured_speed);
    RUN_TEST(test_turn_left_90);
    RUN_TEST(test_turn_right_90);
    RUN_TEST(test_turn_180_goes_the_way_it_was_told);
    RUN_TEST(test_turn_across_the_wrap_point);
    RUN_TEST(test_turn_corrects_start_error_by_aiming_at_the_grid_heading);
    RUN_TEST(test_turn_stays_on_the_spot);
    RUN_TEST(test_turn_times_out);
    RUN_TEST(test_zero_degree_turn_finishes_immediately);
    RUN_TEST(test_turn_waits_out_the_settle_time);
    return UNITY_END();
}
