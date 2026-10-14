// ============================================================================
//  test_motion_primitives.cpp  --  Straight and Turn, closed loop against a small simulated
//  plant (sim.h). No hardware needed.
// ============================================================================

#include <unity.h>

#include "../support/sim.h"

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

static void test_reverse_pass_through_is_done_at_speed()
{
    Rig r;
    Straight s;
    straightBegin(s, r.sim.feedback(), -250.0f, 300.0f, 300.0f, 0.0f);
    MoveStatus st = MoveStatus::Running;
    int ticks = 0;
    for (; ticks < 400 && st == MoveStatus::Running; ++ticks) {
        MotionTarget t;
        st = straightStep(s, r.plan, r.sim.feedback(), 0.0f, DT_MS, t);
        r.drive(t);
    }
    TEST_ASSERT_EQUAL(static_cast<int>(MoveStatus::Done), static_cast<int>(st));
    TEST_ASSERT_TRUE(ticks > 20); // not "already there" on the first tick
    TEST_ASSERT_FLOAT_WITHIN(10.0f, -250.0f, r.sim.distMm);
    TEST_ASSERT_TRUE(r.sim.speedMmS < -200.0f); // still rolling backward
}

static void test_reverse_chain_does_not_stop_between_cells()
{
    Rig r;
    Straight s;
    straightBegin(s, r.sim.feedback(), -250.0f, 300.0f, 300.0f, 0.0f);
    float maxSpeedMid = -1e9f;
    bool extended = false;
    MoveStatus st = MoveStatus::Running;
    for (int i = 0; i < 800 && st == MoveStatus::Running; ++i) {
        if (!extended && r.sim.distMm < -150.0f) {
            straightExtend(s, -250.0f, 0.0f); // carry on one more cell, then stop
            extended = true;
        }
        MotionTarget t;
        st = straightStep(s, r.plan, r.sim.feedback(), 0.0f, DT_MS, t);
        r.drive(t);
        if (r.sim.distMm < -200.0f && r.sim.distMm > -300.0f) {
            maxSpeedMid = r.sim.speedMmS > maxSpeedMid ? r.sim.speedMmS : maxSpeedMid;
        }
    }
    TEST_ASSERT_EQUAL(static_cast<int>(MoveStatus::Done), static_cast<int>(st));
    TEST_ASSERT_FLOAT_WITHIN(3.0f, -500.0f, r.sim.distMm);
    TEST_ASSERT_TRUE(maxSpeedMid < -200.0f);
}

static void test_forward_pass_through_begun_while_rolling_backward_is_not_done_at_once()
{
    Rig r;
    r.sim.vL = r.sim.vR = r.sim.speedMmS = -50.0f;
    Straight s;
    straightBegin(s, r.sim.feedback(), 250.0f, 300.0f, 300.0f, 0.0f);
    MotionTarget t;
    TEST_ASSERT_EQUAL(static_cast<int>(MoveStatus::Running),
                      static_cast<int>(straightStep(s, r.plan, r.sim.feedback(), 0.0f, DT_MS, t)));
}

static void test_retarget_moves_the_stop()
{
    Rig r;
    Straight s;
    straightBegin(s, r.sim.feedback(), 250.0f, 300.0f, 0.0f, 0.0f);
    bool done = false;
    for (int i = 0; i < 400 && !done; ++i) {
        if (i == 30) {
            straightRetarget(s, 220.0f, 0.0f,
                             r.plan.accelMmS2); // a wall said the centre is 30 mm nearer
        }
        MotionTarget t;
        done = straightStep(s, r.plan, r.sim.feedback(), 0.0f, DT_MS, t) == MoveStatus::Done;
        r.drive(t);
    }
    TEST_ASSERT_TRUE(done);
    TEST_ASSERT_FLOAT_WITHIN(3.0f, 220.0f, r.sim.distMm);
}

// A finish line pulled in nearer than the profile can stop is taken as the braking point: the robot
// brakes as hard as the plan allows and stops there, it does not run through and come back.
static void test_retarget_inside_the_braking_distance_does_not_reverse()
{
    Rig r;
    Straight s;
    straightBegin(s, r.sim.feedback(), 250.0f, 300.0f, 0.0f, 0.0f);
    float atRetarget = 0.0f;
    float backtrack = 0.0f;
    float last = 0.0f;
    bool done = false;
    for (int i = 0; i < 400 && !done; ++i) {
        if (i == 40) {
            atRetarget = s.refMm;
            straightRetarget(s, s.refMm - 60.0f, 0.0f, r.plan.accelMmS2); // 60 mm behind it
        }
        MotionTarget t;
        done = straightStep(s, r.plan, r.sim.feedback(), 0.0f, DT_MS, t) == MoveStatus::Done;
        r.drive(t);
        backtrack = r.sim.distMm < last - backtrack ? last - r.sim.distMm : backtrack;
        last = r.sim.distMm;
    }
    const float brakeMm = 300.0f * 300.0f / (2.0f * r.plan.accelMmS2);
    TEST_ASSERT_TRUE(done);
    TEST_ASSERT_TRUE(backtrack < 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(5.0f, atRetarget + brakeMm, r.sim.distMm);
}

static void test_retarget_beyond_the_braking_distance_is_taken_as_given()
{
    Straight s;
    MotionFeedback fb;
    fb.speedMmS = 150.0f;
    straightBegin(s, fb, 250.0f, 150.0f, 0.0f, 0.0f);
    straightRetarget(s, 100.0f, 0.0f, 500.0f); // braking from 150 mm/s at 500 takes 22.5 mm
    TEST_ASSERT_EQUAL_FLOAT(100.0f, s.targetMm);
    straightRetarget(s, 10.0f, 0.0f, 500.0f);
    TEST_ASSERT_EQUAL_FLOAT(22.5f, s.targetMm);
}

static void test_retarget_never_pushes_a_line_out_that_is_already_inside_the_braking_distance()
{
    Straight s;
    MotionFeedback fb;
    fb.speedMmS = 150.0f;
    straightBegin(s, fb, 250.0f, 150.0f, 0.0f, 0.0f);
    s.targetMm = 10.0f; // the profile is late on this line: braking from 150 needs 22.5 mm
    straightRetarget(s, 8.0f, 0.0f, 500.0f);
    TEST_ASSERT_EQUAL_FLOAT(10.0f, s.targetMm);
    straightRetarget(s, 60.0f, 0.0f, 500.0f); // moving it further out is still allowed
    TEST_ASSERT_EQUAL_FLOAT(60.0f, s.targetMm);
}

static void test_retarget_brakes_only_down_to_the_arrival_speed()
{
    Straight s;
    MotionFeedback fb;
    fb.speedMmS = 200.0f;
    straightBegin(s, fb, 250.0f, 200.0f, 100.0f, 0.0f);
    straightRetarget(s, 0.0f, 100.0f, 500.0f); // (200^2 - 100^2) / (2 * 500)
    TEST_ASSERT_EQUAL_FLOAT(30.0f, s.targetMm);
}

static void test_retarget_of_a_reverse_move_clamps_the_other_way()
{
    Straight s;
    MotionFeedback fb;
    fb.speedMmS = -150.0f;
    straightBegin(s, fb, -250.0f, 150.0f, 0.0f, 0.0f);
    straightRetarget(s, 40.0f, 0.0f, 500.0f); // behind the robot's direction of travel
    TEST_ASSERT_EQUAL_FLOAT(-22.5f, s.targetMm);
}

static void test_retarget_at_rest_follows_the_direction_of_the_move()
{
    Straight fwd;
    Straight rev;
    MotionFeedback fb;
    straightBegin(fwd, fb, 100.0f, 150.0f, 0.0f, 0.0f);
    straightBegin(rev, fb, -100.0f, 150.0f, 0.0f, 0.0f);
    straightRetarget(fwd, -30.0f, 0.0f, 500.0f);
    straightRetarget(rev, 30.0f, 0.0f, 500.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, fwd.targetMm);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, rev.targetMm);
}

static void test_straight_that_cannot_move_is_stuck()
{
    Rig r;
    r.plan.patienceMs = 100;
    r.plan.accelMmS2 = 0.0f; // cannot move
    Straight s;
    straightBegin(s, r.sim.feedback(), 250.0f, 300.0f, 0.0f, 0.0f);
    MoveStatus st = MoveStatus::Running;
    for (int i = 0; i < 20 && st == MoveStatus::Running; ++i) {
        MotionTarget t;
        st = straightStep(s, r.plan, r.sim.feedback(), 0.0f, DT_MS, t);
    }
    TEST_ASSERT_EQUAL(static_cast<int>(MoveStatus::Stuck), static_cast<int>(st));
}

// Run a straight against a robot whose odometer reads distAt(tick); returns the status reached.
template <typename F>
static MoveStatus runAgainstOdometer(const PlanParams &plan, float distMm, int ticks, F distAt)
{
    Straight s;
    MotionFeedback fb;
    straightBegin(s, fb, distMm, 100.0f, 0.0f, 0.0f);
    MoveStatus st = MoveStatus::Running;
    for (int i = 0; i < ticks && st == MoveStatus::Running; ++i) {
        fb.distMm = distAt(i);
        MotionTarget t;
        st = straightStep(s, plan, fb, 0.0f, DT_MS, t);
    }
    return st;
}

static void test_hunting_short_of_the_target_is_stuck()
{
    PlanParams plan = simPlanParams();
    plan.patienceMs = 500;
    // 15 mm short, jittering +-3 mm: moving, but never closing the gap.
    const MoveStatus st = runAgainstOdometer(
        plan, 100.0f, 200, [](int i) { return 85.0f + ((i & 1) ? 3.0f : -3.0f); });
    TEST_ASSERT_EQUAL(static_cast<int>(MoveStatus::Stuck), static_cast<int>(st));
}

static void test_moving_the_wrong_way_is_stuck()
{
    PlanParams plan = simPlanParams();
    plan.patienceMs = 500;
    const MoveStatus st =
        runAgainstOdometer(plan, 100.0f, 200, [](int i) { return -1.0f * static_cast<float>(i); });
    TEST_ASSERT_EQUAL(static_cast<int>(MoveStatus::Stuck), static_cast<int>(st));
}

static void test_a_slow_creep_is_not_stuck()
{
    PlanParams plan = simPlanParams();
    plan.patienceMs = 1000;
    // 3 mm/s: a new 2 mm low every ~0.7 s, inside the patience.
    const MoveStatus st =
        runAgainstOdometer(plan, 250.0f, 600, [](int i) { return 0.03f * static_cast<float>(i); });
    TEST_ASSERT_EQUAL(static_cast<int>(MoveStatus::Running), static_cast<int>(st));
}

static void test_a_long_healthy_leg_is_never_stuck()
{
    Rig r;
    r.plan.patienceMs = 300;
    Straight s;
    straightBegin(s, r.sim.feedback(), 1500.0f, 300.0f, 0.0f, 0.0f);
    MoveStatus st = MoveStatus::Running;
    for (int i = 0; i < 1500 && st == MoveStatus::Running; ++i) {
        MotionTarget t;
        st = straightStep(s, r.plan, r.sim.feedback(), 0.0f, DT_MS, t);
        r.drive(t);
    }
    TEST_ASSERT_EQUAL(static_cast<int>(MoveStatus::Done), static_cast<int>(st));
    TEST_ASSERT_FLOAT_WITHIN(3.0f, 1500.0f, r.sim.distMm);
}

static void test_a_robot_shoved_back_near_the_target_recovers()
{
    Rig r; // patience 1000 ms: the 100 mm shove has to be made up inside it
    Straight s;
    straightBegin(s, r.sim.feedback(), 400.0f, 300.0f, 0.0f, 0.0f);
    bool shoved = false;
    MoveStatus st = MoveStatus::Running;
    for (int i = 0; i < 800 && st == MoveStatus::Running; ++i) {
        MotionTarget t;
        st = straightStep(s, r.plan, r.sim.feedback(), 0.0f, DT_MS, t);
        r.drive(t);
        if (!shoved && s.targetMm - r.sim.distMm < 20.0f) {
            r.sim.distMm -= 100.0f;
            shoved = true;
        }
    }
    TEST_ASSERT_TRUE(shoved);
    TEST_ASSERT_EQUAL(static_cast<int>(MoveStatus::Done), static_cast<int>(st));
}

static void test_a_limit_cycle_through_the_target_is_stuck()
{
    PlanParams plan = simPlanParams();
    plan.patienceMs = 500;
    // +-15 mm around the target, 200 ms period: every swing crosses the tolerance band, and the
    // robot never settles.
    const MoveStatus st = runAgainstOdometer(plan, 100.0f, 400, [](int i) {
        return 100.0f + 15.0f * sinf(0.314159f * static_cast<float>(i));
    });
    TEST_ASSERT_EQUAL(static_cast<int>(MoveStatus::Stuck), static_cast<int>(st));
}

static void test_extending_a_straight_restarts_the_stuck_count()
{
    PlanParams plan = simPlanParams();
    plan.patienceMs = 100; // 10 ticks
    Straight s;
    MotionFeedback fb; // the robot does not move
    straightBegin(s, fb, 250.0f, 150.0f, 150.0f, 0.0f);
    MotionTarget t;
    for (int i = 0; i < 8; ++i) {
        TEST_ASSERT_EQUAL(static_cast<int>(MoveStatus::Running),
                          static_cast<int>(straightStep(s, plan, fb, 0.0f, DT_MS, t)));
    }
    straightExtend(s, 250.0f, 0.0f); // 8 ticks idle so far: the count starts again
    for (int i = 0; i < 8; ++i) {
        TEST_ASSERT_EQUAL(static_cast<int>(MoveStatus::Running),
                          static_cast<int>(straightStep(s, plan, fb, 0.0f, DT_MS, t)));
    }
    MoveStatus st = MoveStatus::Running;
    for (int i = 0; i < 5 && st == MoveStatus::Running; ++i) {
        st = straightStep(s, plan, fb, 0.0f, DT_MS, t);
    }
    TEST_ASSERT_EQUAL(static_cast<int>(MoveStatus::Stuck), static_cast<int>(st));
}

static void test_stuck_detection_off_never_gives_up()
{
    Rig r;
    r.plan.patienceMs = 0;
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

static void test_turn_that_cannot_move_is_stuck()
{
    Rig r;
    r.plan.patienceMs = 100;
    r.plan.turnAccelDps2 = 0.0f;
    TEST_ASSERT_EQUAL(static_cast<int>(MoveStatus::Stuck),
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

static void test_only_the_first_step_of_a_move_is_flagged_as_new()
{
    const PlanParams plan = simPlanParams();
    MotionFeedback fb;
    MotionTarget t;
    Straight s;
    straightBegin(s, fb, 250.0f, 150.0f, 150.0f, 0.0f);
    straightStep(s, plan, fb, 0.0f, 10, t);
    TEST_ASSERT_TRUE(t.newMove);
    straightStep(s, plan, fb, 0.0f, 10, t);
    TEST_ASSERT_FALSE(t.newMove);
    straightExtend(s, 250.0f, 0.0f); // a chain of cells keeps its momentum
    straightStep(s, plan, fb, 0.0f, 10, t);
    TEST_ASSERT_FALSE(t.newMove);
    straightRetarget(s, 400.0f, 0.0f, plan.accelMmS2);
    straightStep(s, plan, fb, 0.0f, 10, t);
    TEST_ASSERT_FALSE(t.newMove);
    straightBegin(s, fb, 100.0f, 150.0f, 0.0f, 0.0f);
    straightStep(s, plan, fb, 0.0f, 10, t);
    TEST_ASSERT_TRUE(t.newMove);

    Turn turn;
    turnBegin(turn, fb, 90.0f, 90.0f);
    turnStep(turn, plan, fb, 10, t);
    TEST_ASSERT_TRUE(t.newMove);
    turnStep(turn, plan, fb, 10, t);
    TEST_ASSERT_FALSE(t.newMove);
}

static void test_a_move_starts_without_the_previous_moves_integral()
{
    Rig r;
    r.cp.kiTurn = 60.0f;
    r.cp.kiHold = 8.0f;
    MotionFeedback fb;
    fb.headingDeg = -3.0f; // held 3 deg short, so the turn integral winds up
    Turn turn;
    turnBegin(turn, fb, 0.0f, 0.0f);
    MotionTarget t;
    for (int i = 0; i < 100; ++i) {
        turnStep(turn, r.plan, fb, DT_MS, t);
        motionControlStep(r.mc, r.cp, t, fb, DT_MS);
    }
    TEST_ASSERT_TRUE(r.mc.turnIntegral > 2.0f);

    fb.headingDeg = 0.0f; // the straight starts on its heading
    Straight s;
    straightBegin(s, fb, 250.0f, 150.0f, 0.0f, 0.0f);
    straightStep(s, r.plan, fb, 0.0f, DT_MS, t);
    motionControlStep(r.mc, r.cp, t, fb, DT_MS);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, r.mc.turnIntegral);
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_only_a_turn_asks_for_the_pivot_integral);
    RUN_TEST(test_only_the_first_step_of_a_move_is_flagged_as_new);
    RUN_TEST(test_a_move_starts_without_the_previous_moves_integral);
    RUN_TEST(test_straight_stops_on_the_target);
    RUN_TEST(test_straight_holds_heading_against_a_disturbance);
    RUN_TEST(test_straight_trim_steers_the_heading);
    RUN_TEST(test_pass_through_is_done_at_speed);
    RUN_TEST(test_extend_chains_a_second_cell_without_stopping);
    RUN_TEST(test_reverse_pass_through_is_done_at_speed);
    RUN_TEST(test_reverse_chain_does_not_stop_between_cells);
    RUN_TEST(test_forward_pass_through_begun_while_rolling_backward_is_not_done_at_once);
    RUN_TEST(test_retarget_moves_the_stop);
    RUN_TEST(test_retarget_inside_the_braking_distance_does_not_reverse);
    RUN_TEST(test_retarget_beyond_the_braking_distance_is_taken_as_given);
    RUN_TEST(test_retarget_never_pushes_a_line_out_that_is_already_inside_the_braking_distance);
    RUN_TEST(test_retarget_brakes_only_down_to_the_arrival_speed);
    RUN_TEST(test_retarget_of_a_reverse_move_clamps_the_other_way);
    RUN_TEST(test_retarget_at_rest_follows_the_direction_of_the_move);
    RUN_TEST(test_straight_that_cannot_move_is_stuck);
    RUN_TEST(test_hunting_short_of_the_target_is_stuck);
    RUN_TEST(test_moving_the_wrong_way_is_stuck);
    RUN_TEST(test_a_slow_creep_is_not_stuck);
    RUN_TEST(test_a_long_healthy_leg_is_never_stuck);
    RUN_TEST(test_a_robot_shoved_back_near_the_target_recovers);
    RUN_TEST(test_a_limit_cycle_through_the_target_is_stuck);
    RUN_TEST(test_extending_a_straight_restarts_the_stuck_count);
    RUN_TEST(test_stuck_detection_off_never_gives_up);
    RUN_TEST(test_straight_begins_at_the_measured_speed);
    RUN_TEST(test_turn_left_90);
    RUN_TEST(test_turn_right_90);
    RUN_TEST(test_turn_180_goes_the_way_it_was_told);
    RUN_TEST(test_turn_across_the_wrap_point);
    RUN_TEST(test_turn_corrects_start_error_by_aiming_at_the_grid_heading);
    RUN_TEST(test_turn_stays_on_the_spot);
    RUN_TEST(test_turn_that_cannot_move_is_stuck);
    RUN_TEST(test_zero_degree_turn_finishes_immediately);
    RUN_TEST(test_turn_waits_out_the_settle_time);
    return UNITY_END();
}
