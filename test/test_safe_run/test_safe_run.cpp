// ============================================================================
//  test_safe_run.cpp  --  first-run state machine. No hardware needed.
//  Each helper advances a simulated robot one 10 ms tick at a time.
// ============================================================================

#include <math.h>
#include <unity.h>

#include "SafeRun.h"

namespace
{

struct Sim {
    RunLayer layer;
    SafeRun run;
    RobotState s;
    RunOutput out;
    float encPerTick = 0.0f; // encoder counts added per tick at full cruise duty
    float degPerTick = 0.0f; // heading change per tick at turn duty (+ = CCW)
    float rightGain = 1.0f;  // right wheel counts per duty relative to the left
    bool wheelsMove = true;
    bool reverseEnc = false;
    float encL = 0.0f, encR = 0.0f;
};

void clearWalls(RobotState &s)
{
    s.usValid = 0x7;
    s.distFrontMm = 500;
    s.distLeftMm = 100;
    s.distRightMm = 100;
}

void tick(Sim &m)
{
    m.s.timestampMs += CONTROL_LOOP_PERIOD_MS;
    m.out = runLayerStep(m.layer, m.run, m.s);
    if (m.wheelsMove && m.out.enable) {
        const float k = m.encPerTick / SAFE_CRUISE_DUTY;
        const float sign = m.reverseEnc ? -1.0f : 1.0f;
        m.encL += sign * k * m.out.left;
        m.encR += sign * k * m.rightGain * m.out.right;
        m.s.encoderCountL = static_cast<int32_t>(m.encL);
        m.s.encoderCountR = static_cast<int32_t>(m.encR);
        // Right minus left wheel = rotation, + = CCW.
        m.s.headingDeg = headingWrapDeg(m.s.headingDeg +
                                        m.degPerTick * (m.rightGain * m.out.right - m.out.left) /
                                            (2.0f * SAFE_TURN_DUTY));
    }
}

void run(Sim &m, uint32_t ms)
{
    for (uint32_t t = 0; t < ms; t += CONTROL_LOOP_PERIOD_MS) {
        tick(m);
    }
}

Sim makeSim(bool canTurn = true)
{
    Sim m;
    runLayerInit(m.layer, true);
    safeRunInit(m.run, canTurn);
    clearWalls(m.s);
    m.encPerTick = 2.0f;
    m.degPerTick = 1.5f;
    return m;
}

void press(Sim &m)
{
    m.s.startButtonPressed = true;
    run(m, 50);
    m.s.startButtonPressed = false;
    run(m, 50);
}

// Release at boot, press, count down, self-test and settle into Cruise.
void startToCruise(Sim &m)
{
    m.s.startButtonPressed = false;
    run(m, 50);
    press(m);
    run(m, RUN_COUNTDOWN_MS + RUN_SELFTEST_MS + SAFE_SETTLE_MS + 100);
}

} // namespace

static void test_waits_for_button_and_drives_nothing()
{
    Sim m = makeSim();
    run(m, 1000);
    TEST_ASSERT_EQUAL(static_cast<int>(RunPhase::WaitStart), static_cast<int>(m.layer.phase));
    TEST_ASSERT_FALSE(m.out.enable);
}

static void test_heading_hold_corrects_a_disturbance()
{
    Sim m = makeSim();
    startToCruise(m);
    m.s.headingDeg += 8.0f; // a bump knocks it 8 degrees off
    run(m, 2000);
    TEST_ASSERT_TRUE(fabsf(headingWrapDeg(m.s.headingDeg - m.run.targetDeg)) < 1.5f);
}

static void test_a_weak_right_wheel_is_absorbed_by_the_integral()
{
    Sim m = makeSim();
    m.rightGain = 0.8f;
    m.encPerTick = 16.0f;
    startToCruise(m);
    run(m, 6000);
    TEST_ASSERT_TRUE(fabsf(headingWrapDeg(m.s.headingDeg - m.run.targetDeg)) < 1.5f);
    TEST_ASSERT_TRUE(m.run.headingI > 5.0f);
    TEST_ASSERT_TRUE(m.run.headingI <= SAFE_HEADING_I_MAX);
}

static void test_wall_tilt_steers_away_from_the_nearer_wall()
{
    Sim m = makeSim();
    startToCruise(m);
    m.run.learnDeg = 0.0f;
    m.run.headingI = 0.0f;
    m.s.headingDeg = m.run.targetDeg;
    m.s.distLeftMm = 60; // nearer the right wall: head left
    m.s.distRightMm = 30;
    TEST_ASSERT_TRUE(safeRunHeadingSteer(m.run, m.s) > 0);
    m.run.learnDeg = 0.0f;
    m.run.headingI = 0.0f;
    m.s.distLeftMm = 30;
    m.s.distRightMm = 60;
    TEST_ASSERT_TRUE(safeRunHeadingSteer(m.run, m.s) < 0);
}

static void test_wall_tilt_and_learning_are_capped()
{
    Sim m = makeSim();
    startToCruise(m);
    m.run.learnDeg = 0.0f;
    m.s.headingDeg = m.run.targetDeg;
    m.s.distLeftMm = 190;
    m.s.distRightMm = 5;
    for (int i = 0; i < 3000; ++i) {
        safeRunHeadingSteer(m.run, m.s);
    }
    TEST_ASSERT_TRUE(m.run.learnDeg <= SAFE_WALL_LEARN_MAX_DEG);
    TEST_ASSERT_TRUE(m.run.learnDeg > 1.0f);
    TEST_ASSERT_TRUE(safeRunHeadingSteer(m.run, m.s) <= SAFE_STEER_TOTAL_MAX);
}

static void test_no_wall_tilt_or_learning_when_a_side_is_open()
{
    Sim m = makeSim();
    startToCruise(m);
    m.run.learnDeg = 0.0f;
    m.s.headingDeg = m.run.targetDeg;
    m.s.distLeftMm = 40;
    m.s.distRightMm = 600; // an opening on the right
    for (int i = 0; i < 100; ++i) {
        safeRunHeadingSteer(m.run, m.s);
    }
    TEST_ASSERT_EQUAL_FLOAT(0.0f, m.run.learnDeg);
}

static void test_target_moves_by_the_ideal_angle_after_a_turn()
{
    Sim m = makeSim();
    startToCruise(m);
    const float before = m.run.targetDeg;
    m.s.distFrontMm = SAFE_FRONT_STOP_MM;
    m.s.distLeftMm = 400; // open: turn left 90
    m.s.distRightMm = 400;
    run(m, SAFE_SETTLE_MS + 100);
    m.s.distFrontMm = 500;
    run(m, 1500);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, headingWrapDeg(before + 90.0f), m.run.targetDeg);
}

static void test_a_turn_overshoot_is_corrected_after_the_turn()
{
    Sim m = makeSim();
    startToCruise(m);
    m.s.distFrontMm = SAFE_FRONT_STOP_MM;
    m.s.distLeftMm = 400;
    m.s.distRightMm = 400;
    run(m, SAFE_SETTLE_MS + 100);
    m.s.distFrontMm = 500;
    m.s.distLeftMm = 100; // the new corridor: walls both sides
    m.s.distRightMm = 100;
    while (m.layer.phase == RunPhase::Turn) {
        tick(m);
    }
    m.s.headingDeg += 20.0f; // it coasts 20 degrees past
    run(m, SAFE_SETTLE_MS + 2500);
    TEST_ASSERT_TRUE(fabsf(headingWrapDeg(m.s.headingDeg - m.run.targetDeg)) < 2.0f);
}

static void test_the_wheels_stop_at_once_when_a_turn_ends()
{
    Sim m = makeSim();
    startToCruise(m);
    m.s.distFrontMm = SAFE_FRONT_STOP_MM;
    m.s.distRightMm = 400;
    run(m, SAFE_SETTLE_MS + 100);
    while (m.layer.phase == RunPhase::Turn) {
        tick(m);
    }
    TEST_ASSERT_EQUAL(static_cast<int>(RunPhase::Settle), static_cast<int>(m.layer.phase));
    TEST_ASSERT_EQUAL_INT16(0, m.out.left);
    TEST_ASSERT_EQUAL_INT16(0, m.out.right);
}

static void test_the_wheels_stop_at_once_at_a_front_wall()
{
    Sim m = makeSim();
    startToCruise(m);
    run(m, 600); // let it get up to speed
    TEST_ASSERT_TRUE(m.out.left > 200);
    m.s.distFrontMm = SAFE_FRONT_STOP_MM;
    m.s.distLeftMm = 100;
    m.s.distRightMm = 100;
    tick(m);
    TEST_ASSERT_EQUAL_INT16(0, m.out.left);
    TEST_ASSERT_EQUAL_INT16(0, m.out.right);
}

static void test_without_a_gyro_the_walls_still_steer()
{
    Sim m = makeSim(false);
    startToCruise(m);
    m.s.distLeftMm = 60;
    m.s.distRightMm = 30;
    run(m, 300);
    TEST_ASSERT_TRUE(m.out.right > m.out.left);
}

static void test_without_a_button_the_run_starts_itself_after_the_delay()
{
    Sim m = makeSim();
    runLayerInit(m.layer, false);
    safeRunInit(m.run, true);
    run(m, RUN_AUTOSTART_MS - 500);
    TEST_ASSERT_EQUAL(static_cast<int>(RunPhase::Countdown), static_cast<int>(m.layer.phase));
    TEST_ASSERT_FALSE(m.out.enable);
    run(m, 1000);
    TEST_ASSERT_EQUAL(static_cast<int>(RunPhase::SelfTest), static_cast<int>(m.layer.phase));
    // A floating or noisy button input must not stop it.
    m.s.startButtonPressed = true;
    run(m, 200);
    TEST_ASSERT_TRUE(m.layer.phase != RunPhase::Stopped);
}

static void test_button_held_through_boot_does_not_start()
{
    Sim m = makeSim();
    m.s.startButtonPressed = true;
    run(m, 1000);
    TEST_ASSERT_EQUAL(static_cast<int>(RunPhase::WaitStart), static_cast<int>(m.layer.phase));
}

static void test_bounce_shorter_than_debounce_is_ignored()
{
    Sim m = makeSim();
    m.s.startButtonPressed = false;
    run(m, 50);
    m.s.startButtonPressed = true;
    run(m, 10);
    m.s.startButtonPressed = false;
    run(m, 100);
    TEST_ASSERT_EQUAL(static_cast<int>(RunPhase::WaitStart), static_cast<int>(m.layer.phase));
}

static void test_countdown_keeps_motors_off_then_self_tests()
{
    Sim m = makeSim();
    m.s.startButtonPressed = false;
    run(m, 50);
    press(m);
    TEST_ASSERT_EQUAL(static_cast<int>(RunPhase::Countdown), static_cast<int>(m.layer.phase));
    run(m, 2000);
    TEST_ASSERT_FALSE(m.out.enable);
    run(m, 1200);
    TEST_ASSERT_EQUAL(static_cast<int>(RunPhase::SelfTest), static_cast<int>(m.layer.phase));
    TEST_ASSERT_TRUE(m.out.enable);
}

static void test_countdown_without_front_echo_faults_before_driving()
{
    Sim m = makeSim();
    m.s.startButtonPressed = false;
    run(m, 50);
    press(m);
    m.s.usValid = 0x6; // front silent
    run(m, RUN_COUNTDOWN_MS + 100);
    TEST_ASSERT_EQUAL(static_cast<int>(RunPhase::Fault), static_cast<int>(m.layer.phase));
    TEST_ASSERT_EQUAL(static_cast<int>(RunReason::FrontBlind), static_cast<int>(m.layer.reason));
    TEST_ASSERT_FALSE(m.out.enable);
}

static void test_self_test_passes_and_cruise_starts()
{
    Sim m = makeSim();
    startToCruise(m);
    TEST_ASSERT_EQUAL(static_cast<int>(RunPhase::Cruise), static_cast<int>(m.layer.phase));
    TEST_ASSERT_TRUE(m.out.left > 0 && m.out.right > 0);
}

static void test_self_test_reversed_encoder_faults()
{
    Sim m = makeSim();
    m.reverseEnc = true;
    startToCruise(m);
    TEST_ASSERT_EQUAL(static_cast<int>(RunPhase::Fault), static_cast<int>(m.layer.phase));
    TEST_ASSERT_EQUAL(static_cast<int>(RunReason::SelfTestReversed),
                      static_cast<int>(m.layer.reason));
    TEST_ASSERT_FALSE(m.out.enable);
    TEST_ASSERT_EQUAL_INT16(0, m.out.left);
}

static void test_self_test_no_motion_faults()
{
    Sim m = makeSim();
    m.wheelsMove = false;
    startToCruise(m);
    TEST_ASSERT_EQUAL(static_cast<int>(RunReason::SelfTestNoMotion),
                      static_cast<int>(m.layer.reason));
}

static void test_self_test_one_dead_wheel_faults()
{
    Sim m = makeSim();
    m.s.startButtonPressed = false;
    run(m, 50);
    press(m);
    run(m, RUN_COUNTDOWN_MS + 20);
    // Only the left encoder counts.
    for (uint32_t t = 0; t < RUN_SELFTEST_MS + 50; t += CONTROL_LOOP_PERIOD_MS) {
        m.s.encoderCountL += 5;
        m.s.timestampMs += CONTROL_LOOP_PERIOD_MS;
        m.out = runLayerStep(m.layer, m.run, m.s);
    }
    TEST_ASSERT_EQUAL(static_cast<int>(RunReason::SelfTestNoMotion),
                      static_cast<int>(m.layer.reason));
}

static void test_self_test_stops_if_wall_already_ahead()
{
    Sim m = makeSim();
    m.s.startButtonPressed = false;
    run(m, 50);
    press(m);
    run(m, RUN_COUNTDOWN_MS + 20);
    m.s.distFrontMm = SAFE_FRONT_STOP_MM;
    run(m, 20);
    TEST_ASSERT_EQUAL(static_cast<int>(RunPhase::Stopped), static_cast<int>(m.layer.phase));
    TEST_ASSERT_EQUAL(static_cast<int>(RunReason::FrontWall), static_cast<int>(m.layer.reason));
}

static void test_output_ramps_instead_of_jumping()
{
    Sim m = makeSim();
    m.s.startButtonPressed = false;
    run(m, 50);
    press(m);
    while (m.layer.phase != RunPhase::SelfTest) {
        tick(m);
    }
    tick(m);
    TEST_ASSERT_EQUAL_INT16(SAFE_RAMP_PER_TICK, m.out.left);
    TEST_ASSERT_EQUAL_INT16(SAFE_RAMP_PER_TICK, m.out.right);
}

static void test_cruise_never_exceeds_cap()
{
    Sim m = makeSim();
    startToCruise(m);
    m.s.distLeftMm = 20;
    m.s.distRightMm = 190;
    run(m, 500);
    TEST_ASSERT_TRUE(m.out.left <= SAFE_MAX_DUTY && m.out.right <= SAFE_MAX_DUTY);
}

static void test_steers_away_from_the_nearer_wall()
{
    Sim m = makeSim();
    startToCruise(m);
    m.s.distLeftMm = 150; // far from left, near right -> steer left: right wheel faster
    m.s.distRightMm = 120;
    run(m, 500);
    TEST_ASSERT_TRUE(m.out.right > m.out.left);
    m.s.distLeftMm = 120;
    m.s.distRightMm = 150;
    run(m, 500);
    TEST_ASSERT_TRUE(m.out.left > m.out.right);
}

static void test_steering_is_clamped()
{
    Sim m = makeSim();
    m.s.distLeftMm = 190;
    m.s.distRightMm = 10;
    TEST_ASSERT_EQUAL_INT16(SAFE_STEER_MAX, safeRunSteer(m.s));
    m.s.distLeftMm = 10;
    m.s.distRightMm = 190;
    TEST_ASSERT_EQUAL_INT16(-SAFE_STEER_MAX, safeRunSteer(m.s));
}

static void test_goes_straight_when_a_side_is_open_or_silent()
{
    Sim m = makeSim();
    m.s.distLeftMm = 100;
    m.s.distRightMm = 600; // opening on the right
    TEST_ASSERT_EQUAL_INT16(0, safeRunSteer(m.s));
    m.s.distRightMm = 120;
    m.s.usValid = 0x5; // left silent
    TEST_ASSERT_EQUAL_INT16(0, safeRunSteer(m.s));
}

static void test_front_wall_turns_toward_the_open_side_and_resumes()
{
    Sim m = makeSim();
    startToCruise(m);
    m.s.distFrontMm = SAFE_FRONT_STOP_MM;
    m.s.distLeftMm = 100;  // wall
    m.s.distRightMm = 400; // open -> clockwise
    run(m, 100);
    TEST_ASSERT_EQUAL(static_cast<int>(RunPhase::Settle), static_cast<int>(m.layer.phase));
    run(m, SAFE_SETTLE_MS + 50);
    TEST_ASSERT_EQUAL(static_cast<int>(RunPhase::Turn), static_cast<int>(m.layer.phase));
    TEST_ASSERT_TRUE(m.out.left > 0 && m.out.right < 0);
    // The turn finishes on gyro angle; the wall is gone once it points down the new corridor.
    const float start = m.s.headingDeg;
    m.s.distFrontMm = 500;
    run(m, SAFE_TURN_TIMEOUT_MS - 500);
    TEST_ASSERT_TRUE(m.layer.phase != RunPhase::Fault);
    float turned = headingWrapDeg(m.s.headingDeg - start);
    TEST_ASSERT_TRUE(turned < -70.0f && turned > -110.0f);
    run(m, SAFE_SETTLE_MS + 100);
    TEST_ASSERT_EQUAL(static_cast<int>(RunPhase::Cruise), static_cast<int>(m.layer.phase));
}

static void test_left_open_turns_counter_clockwise()
{
    Sim m = makeSim();
    startToCruise(m);
    m.s.distFrontMm = SAFE_FRONT_STOP_MM;
    m.s.distLeftMm = 400;
    m.s.distRightMm = 400;
    run(m, SAFE_SETTLE_MS + 100);
    TEST_ASSERT_EQUAL(static_cast<int>(RunPhase::Turn), static_cast<int>(m.layer.phase));
    TEST_ASSERT_TRUE(m.out.left < 0 && m.out.right > 0);
}

static void test_dead_end_turns_180()
{
    Sim m = makeSim();
    startToCruise(m);
    m.s.distFrontMm = SAFE_FRONT_STOP_MM;
    const float start = m.s.headingDeg;
    run(m, SAFE_SETTLE_MS + 100);
    TEST_ASSERT_EQUAL(static_cast<int>(RunPhase::Turn), static_cast<int>(m.layer.phase));
    m.s.distFrontMm = 500;
    run(m, 2000);
    const float turned = headingWrapDeg(m.s.headingDeg - start);
    TEST_ASSERT_TRUE(turned > 160.0f || turned < -160.0f);
}

static void test_silent_side_at_front_wall_stops_instead_of_guessing()
{
    Sim m = makeSim();
    startToCruise(m);
    m.s.distFrontMm = SAFE_FRONT_STOP_MM;
    m.s.usValid = 0x3; // right silent, left is a wall
    run(m, 20);
    TEST_ASSERT_EQUAL(static_cast<int>(RunPhase::Stopped), static_cast<int>(m.layer.phase));
    TEST_ASSERT_EQUAL(static_cast<int>(RunReason::NoClearSide), static_cast<int>(m.layer.reason));
    TEST_ASSERT_FALSE(m.out.enable);
}

static void test_front_wall_without_gyro_stops()
{
    Sim m = makeSim(false);
    startToCruise(m);
    m.s.distFrontMm = SAFE_FRONT_STOP_MM;
    run(m, 20);
    TEST_ASSERT_EQUAL(static_cast<int>(RunReason::FrontWall), static_cast<int>(m.layer.reason));
}

static void test_turn_that_never_turns_faults()
{
    Sim m = makeSim();
    startToCruise(m);
    m.s.distFrontMm = SAFE_FRONT_STOP_MM;
    m.s.distRightMm = 400;
    m.degPerTick = 0.0f; // gyro dead
    run(m, SAFE_SETTLE_MS + SAFE_TURN_TIMEOUT_MS + 200);
    TEST_ASSERT_EQUAL(static_cast<int>(RunPhase::Fault), static_cast<int>(m.layer.phase));
    TEST_ASSERT_EQUAL(static_cast<int>(RunReason::TurnTimeout), static_cast<int>(m.layer.reason));
}

static void test_a_turn_stuck_against_a_wall_gives_up_early()
{
    Sim m = makeSim();
    startToCruise(m);
    m.s.distFrontMm = SAFE_FRONT_STOP_MM;
    m.s.distRightMm = 400;
    m.degPerTick = 0.05f; // the pivot barely turns: jammed, the wheels slip
    run(m, SAFE_SETTLE_MS + SAFE_TURN_PROGRESS_MS + 300);
    TEST_ASSERT_EQUAL(static_cast<int>(RunReason::TurnTimeout), static_cast<int>(m.layer.reason));
    TEST_ASSERT_FALSE(m.out.enable);
}

static void test_turn_with_blocked_wheels_faults_as_stall()
{
    Sim m = makeSim();
    startToCruise(m);
    m.s.distFrontMm = SAFE_FRONT_STOP_MM;
    m.s.distRightMm = 400;
    run(m, SAFE_SETTLE_MS + 50);
    m.wheelsMove = false;
    run(m, SAFE_STALL_WINDOW_MS + 100);
    TEST_ASSERT_EQUAL(static_cast<int>(RunReason::Stall), static_cast<int>(m.layer.reason));
}

static void test_repeated_turns_without_straight_run_stop()
{
    Sim m = makeSim();
    startToCruise(m);
    // A wall that follows the robot around: every turn finishes and the front is still blocked.
    for (int i = 0; i < 6 && m.layer.phase != RunPhase::Stopped; ++i) {
        m.s.distFrontMm = SAFE_FRONT_STOP_MM;
        m.s.distRightMm = 400;
        run(m, SAFE_SETTLE_MS + 100);
        m.s.distFrontMm = 500;
        run(m, 1500);
        m.s.distFrontMm = SAFE_FRONT_STOP_MM;
        run(m, 30);
    }
    TEST_ASSERT_EQUAL(static_cast<int>(RunReason::TooManyTurns), static_cast<int>(m.layer.reason));
}

static void test_long_straight_run_clears_the_turn_count()
{
    Sim m = makeSim();
    startToCruise(m);
    m.run.turnsInRow = SAFE_MAX_TURNS_IN_ROW;
    run(m, SAFE_TURN_RESET_MS + 100);
    TEST_ASSERT_EQUAL_UINT8(0, m.run.turnsInRow);
}

static void test_stalled_wheel_while_cruising_faults()
{
    Sim m = makeSim();
    startToCruise(m);
    m.wheelsMove = false;
    run(m, 2 * SAFE_STALL_WINDOW_MS + 100);
    TEST_ASSERT_EQUAL(static_cast<int>(RunReason::Stall), static_cast<int>(m.layer.reason));
    TEST_ASSERT_FALSE(m.out.enable);
}

static void test_front_echo_lost_while_cruising_faults()
{
    Sim m = makeSim();
    startToCruise(m);
    m.s.usValid = 0x6;
    run(m, SAFE_FRONT_BLIND_MS + 100);
    TEST_ASSERT_EQUAL(static_cast<int>(RunReason::FrontBlind), static_cast<int>(m.layer.reason));
}

static void test_brief_front_dropout_is_tolerated()
{
    Sim m = makeSim();
    startToCruise(m);
    m.s.usValid = 0x6;
    run(m, SAFE_FRONT_BLIND_MS / 2);
    m.s.usValid = 0x7;
    run(m, 200);
    TEST_ASSERT_EQUAL(static_cast<int>(RunPhase::Cruise), static_cast<int>(m.layer.phase));
}

static void test_button_during_run_stops_immediately()
{
    Sim m = makeSim();
    startToCruise(m);
    TEST_ASSERT_TRUE(m.out.left > 0);
    m.s.startButtonPressed = true;
    run(m, 50);
    TEST_ASSERT_EQUAL(static_cast<int>(RunReason::EStop), static_cast<int>(m.layer.reason));
    TEST_ASSERT_FALSE(m.out.enable);
    TEST_ASSERT_EQUAL_INT16(0, m.out.left);
    TEST_ASSERT_FALSE(runReasonIsFault(m.layer.reason));
}

static void test_button_during_countdown_cancels()
{
    Sim m = makeSim();
    m.s.startButtonPressed = false;
    run(m, 50);
    press(m);
    m.s.startButtonPressed = true;
    run(m, 50);
    TEST_ASSERT_EQUAL(static_cast<int>(RunReason::EStop), static_cast<int>(m.layer.reason));
}

static void test_run_time_limit_stops()
{
    Sim m = makeSim();
    startToCruise(m);
    run(m, SAFE_RUN_LIMIT_MS);
    TEST_ASSERT_EQUAL(static_cast<int>(RunReason::TimeLimit), static_cast<int>(m.layer.reason));
    TEST_ASSERT_FALSE(m.out.enable);
}

static void test_stopped_and_fault_are_final()
{
    Sim m = makeSim();
    startToCruise(m);
    m.s.startButtonPressed = true;
    run(m, 50);
    m.s.startButtonPressed = false;
    run(m, 100);
    m.s.startButtonPressed = true;
    run(m, 100);
    TEST_ASSERT_EQUAL(static_cast<int>(RunPhase::Stopped), static_cast<int>(m.layer.phase));
    TEST_ASSERT_FALSE(m.out.enable);
}

static void test_reasons_split_into_stops_and_faults()
{
    TEST_ASSERT_FALSE(runReasonIsFault(RunReason::TooManyTurns));
    TEST_ASSERT_TRUE(runReasonIsFault(RunReason::SelfTestReversed));
    TEST_ASSERT_TRUE(runReasonIsFault(RunReason::TurnTimeout));
}

static void test_slew_moves_toward_target_both_ways()
{
    TEST_ASSERT_EQUAL_INT16(8, runSlew(0, 240, 8));
    TEST_ASSERT_EQUAL_INT16(-8, runSlew(0, -240, 8));
    TEST_ASSERT_EQUAL_INT16(232, runSlew(240, 0, 8));
    TEST_ASSERT_EQUAL_INT16(5, runSlew(0, 5, 8));
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_waits_for_button_and_drives_nothing);
    RUN_TEST(test_heading_hold_corrects_a_disturbance);
    RUN_TEST(test_a_weak_right_wheel_is_absorbed_by_the_integral);
    RUN_TEST(test_wall_tilt_steers_away_from_the_nearer_wall);
    RUN_TEST(test_wall_tilt_and_learning_are_capped);
    RUN_TEST(test_no_wall_tilt_or_learning_when_a_side_is_open);
    RUN_TEST(test_target_moves_by_the_ideal_angle_after_a_turn);
    RUN_TEST(test_a_turn_overshoot_is_corrected_after_the_turn);
    RUN_TEST(test_the_wheels_stop_at_once_when_a_turn_ends);
    RUN_TEST(test_the_wheels_stop_at_once_at_a_front_wall);
    RUN_TEST(test_without_a_gyro_the_walls_still_steer);
    RUN_TEST(test_without_a_button_the_run_starts_itself_after_the_delay);
    RUN_TEST(test_button_held_through_boot_does_not_start);
    RUN_TEST(test_bounce_shorter_than_debounce_is_ignored);
    RUN_TEST(test_countdown_keeps_motors_off_then_self_tests);
    RUN_TEST(test_countdown_without_front_echo_faults_before_driving);
    RUN_TEST(test_self_test_passes_and_cruise_starts);
    RUN_TEST(test_self_test_reversed_encoder_faults);
    RUN_TEST(test_self_test_no_motion_faults);
    RUN_TEST(test_self_test_one_dead_wheel_faults);
    RUN_TEST(test_self_test_stops_if_wall_already_ahead);
    RUN_TEST(test_output_ramps_instead_of_jumping);
    RUN_TEST(test_cruise_never_exceeds_cap);
    RUN_TEST(test_steers_away_from_the_nearer_wall);
    RUN_TEST(test_steering_is_clamped);
    RUN_TEST(test_goes_straight_when_a_side_is_open_or_silent);
    RUN_TEST(test_front_wall_turns_toward_the_open_side_and_resumes);
    RUN_TEST(test_left_open_turns_counter_clockwise);
    RUN_TEST(test_dead_end_turns_180);
    RUN_TEST(test_silent_side_at_front_wall_stops_instead_of_guessing);
    RUN_TEST(test_front_wall_without_gyro_stops);
    RUN_TEST(test_turn_that_never_turns_faults);
    RUN_TEST(test_a_turn_stuck_against_a_wall_gives_up_early);
    RUN_TEST(test_turn_with_blocked_wheels_faults_as_stall);
    RUN_TEST(test_repeated_turns_without_straight_run_stop);
    RUN_TEST(test_long_straight_run_clears_the_turn_count);
    RUN_TEST(test_stalled_wheel_while_cruising_faults);
    RUN_TEST(test_front_echo_lost_while_cruising_faults);
    RUN_TEST(test_brief_front_dropout_is_tolerated);
    RUN_TEST(test_button_during_run_stops_immediately);
    RUN_TEST(test_button_during_countdown_cancels);
    RUN_TEST(test_run_time_limit_stops);
    RUN_TEST(test_stopped_and_fault_are_final);
    RUN_TEST(test_reasons_split_into_stops_and_faults);
    RUN_TEST(test_slew_moves_toward_target_both_ways);
    return UNITY_END();
}
