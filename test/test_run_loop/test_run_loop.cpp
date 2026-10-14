// ============================================================================
//  test_run_loop.cpp  --  the tick main.cpp runs: start button, perception, mission, run limit.
//  No hardware needed.
// ============================================================================

#include <initializer_list>
#include <unity.h>

#include "RunLoop.h"
#include "../support/world.h"

using world::World;

static DriveCommand tick(RunLoop &rl, const RobotState &s, uint32_t nowMs, bool button,
                         float *fix = nullptr)
{
    float f = 0.0f;
    const DriveCommand c = runLoopTick(rl, s, nowMs, button, f);
    if (fix != nullptr) {
        *fix = f;
    }
    return c;
}

static void test_nothing_moves_until_the_button_is_held()
{
    RunLoop rl;
    runLoopInit(rl, true, nullptr);
    RobotState s;
    for (uint32_t t = 0; t < 100; ++t) {
        const DriveCommand c = tick(rl, s, t * 10, false);
        TEST_ASSERT_FALSE(c.enable);
    }
    TEST_ASSERT_FALSE(rl.started);
    TEST_ASSERT_EQUAL(static_cast<int>(SectionAPhase::WaitStart),
                      static_cast<int>(rl.mission.phase));
}

static void test_a_bounce_does_not_start_a_run()
{
    RunLoop rl;
    runLoopInit(rl, true, nullptr);
    RobotState s;
    for (uint32_t t = 0; t < 40; ++t) {
        tick(rl, s, t * 10, (t % 3) != 0 && (t % 3) != 1 ? true : false); // never five in a row
    }
    TEST_ASSERT_FALSE(rl.started);
}

static void test_one_press_starts_the_run_and_releasing_does_not_stop_it()
{
    RunLoop rl;
    runLoopInit(rl, true, nullptr);
    RobotState s;
    uint32_t t = 0;
    for (; t < RUN_START_DEBOUNCE_TICKS + 2; ++t) {
        tick(rl, s, t * 10, true);
    }
    TEST_ASSERT_TRUE(rl.started);
    TEST_ASSERT_TRUE(rl.mission.phase != SectionAPhase::WaitStart);
    for (; t < RUN_START_DEBOUNCE_TICKS + 20; ++t) {
        tick(rl, s, t * 10, false);
    }
    TEST_ASSERT_TRUE(rl.started);
    TEST_ASSERT_TRUE(rl.mission.phase != SectionAPhase::WaitStart);
}

static void test_no_gyro_means_no_run()
{
    RunLoop rl;
    runLoopInit(rl, false, nullptr);
    RobotState s;
    DriveCommand c;
    for (uint32_t t = 0; t < 20; ++t) {
        c = tick(rl, s, t * 10, true);
    }
    TEST_ASSERT_EQUAL(static_cast<int>(SectionAPhase::Fault), static_cast<int>(rl.mission.phase));
    TEST_ASSERT_EQUAL(static_cast<int>(SectionAFault::NoImu), static_cast<int>(rl.mission.fault));
    TEST_ASSERT_FALSE(c.enable);
}

static void test_without_a_line_calibration_the_run_is_walls_only()
{
    RunLoop none;
    runLoopInit(none, true, nullptr);
    TEST_ASSERT_TRUE(none.params.wallsOnly);

    const LineCalibration cal = World::calibration();
    RunLoop with;
    runLoopInit(with, true, &cal);
    TEST_ASSERT_FALSE(with.params.wallsOnly);
}

static void test_the_run_limit_ends_the_run()
{
    RunLoop rl;
    runLoopInit(rl, true, nullptr);
    RobotState s;
    uint32_t t = 0;
    for (; t < RUN_START_DEBOUNCE_TICKS + 5; ++t) {
        tick(rl, s, t * 10, true);
    }
    const uint32_t startedAt = rl.startedAtMs;
    DriveCommand c = tick(rl, s, startedAt + RUN_LIMIT_MS - 10, false);
    TEST_ASSERT_TRUE(rl.mission.phase != SectionAPhase::Fault);
    c = tick(rl, s, startedAt + RUN_LIMIT_MS, false);
    TEST_ASSERT_EQUAL(static_cast<int>(SectionAPhase::Fault), static_cast<int>(rl.mission.phase));
    TEST_ASSERT_EQUAL(static_cast<int>(SectionAFault::TimeUp), static_cast<int>(rl.mission.fault));
    TEST_ASSERT_FALSE(c.enable);
}

// The whole thing in the simulated arena: button, perception from raw sensors, mission, motors.
static void test_a_run_from_the_button_to_the_white_tile()
{
    for (int facing : {0, 2}) {
        World w;
        w.arena.photo();
        w.start(2, 1, facing);
        w.state.startButtonPressed = false; // the loop sees the button, not the world
        const LineCalibration cal = World::calibration();
        RunLoop rl;
        runLoopInit(rl, true, &cal);
        bool done = false;
        for (uint32_t t = 0; t < RUN_LIMIT_MS / CONTROL_LOOP_PERIOD_MS && !done; ++t) {
            const bool button = t >= 20 && t < 40;
            const uint32_t nowMs = t * CONTROL_LOOP_PERIOD_MS;
            // The world's own perception is not used: the loop gets the raw state.
            (void)w.sense();
            float fix = 0.0f;
            const DriveCommand c = runLoopTick(rl, w.state, nowMs, button, fix);
            if (fix != 0.0f) {
                headingSet(w.heading, w.heading.deg - fix);
            }
            w.drive(c);
            done =
                rl.mission.phase == SectionAPhase::Done || rl.mission.phase == SectionAPhase::Fault;
        }
        TEST_ASSERT_EQUAL(static_cast<int>(SectionAPhase::Done),
                          static_cast<int>(rl.mission.phase));
        const float dx =
            static_cast<float>(w.x) - world::PITCH * static_cast<float>(w.arena.whiteI);
        const float dy =
            static_cast<float>(w.y) - world::PITCH * static_cast<float>(w.arena.whiteJ);
        TEST_ASSERT_TRUE(dx * dx + dy * dy < 40.0f * 40.0f);
        TEST_ASSERT_TRUE(w.minNoseMm > world::NOSE_MM - 1.0f);
    }
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_nothing_moves_until_the_button_is_held);
    RUN_TEST(test_a_bounce_does_not_start_a_run);
    RUN_TEST(test_one_press_starts_the_run_and_releasing_does_not_stop_it);
    RUN_TEST(test_no_gyro_means_no_run);
    RUN_TEST(test_without_a_line_calibration_the_run_is_walls_only);
    RUN_TEST(test_the_run_limit_ends_the_run);
    RUN_TEST(test_a_run_from_the_button_to_the_white_tile);
    return UNITY_END();
}
