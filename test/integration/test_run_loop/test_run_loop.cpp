// ============================================================================
//  test_run_loop.cpp  --  the tick main.cpp runs: start button, perception, mission, run limit.
//  No hardware needed.
// ============================================================================

#include <initializer_list>
#include <unity.h>

#include "mission/RunLoop.h"
#include "panel/PanelStart.h"
#include "../../support/world/Presets.h"
#include "spec/fixed/Hardware.h"
#include "spec/fixed/Rules.h"
#include "spec/tuning/Run.h"

using world::World;

// What main.cpp does each tick: the panel decides `go`, the mission gets it.
static DriveCommand tick(RunLoop &rl, PanelStart &ps, const RobotState &s, uint32_t nowMs,
                         bool button, float *fix = nullptr)
{
    panelStartTick(ps, nowMs, button, PanelMode::Explore);
    float f = 0.0f;
    const DriveCommand c = runLoopTick(rl, s, nowMs, ps.go, f);
    if (fix != nullptr) {
        *fix = f;
    }
    return c;
}

// A robot with no start button: it stands until the delay after power-up, then runs.
static void test_without_a_button_the_run_begins_after_the_delay()
{
    RunLoop rl;
    runLoopInit(rl, true, nullptr);
    PanelStart ps(false);
    RobotState s;
    const uint32_t bootMs = 1500; // the gyro zeroing took this long
    for (uint32_t t = bootMs; t < bootMs + PANEL_START_DELAY_MS; t += CONTROL_LOOP_PERIOD_MS) {
        const DriveCommand c = tick(rl, ps, s, t, false);
        TEST_ASSERT_FALSE(c.enable);
        TEST_ASSERT_FALSE(rl.mission.start.started);
    }
    tick(rl, ps, s, bootMs + PANEL_START_DELAY_MS, false);
    TEST_ASSERT_TRUE(rl.mission.start.started);
    TEST_ASSERT_TRUE(rl.mission.phase != MissionPhase::WaitStart);
}

// With a button: nothing moves until a press, and the run begins a delay after it is accepted.
static void test_with_a_button_the_run_begins_a_delay_after_the_press()
{
    RunLoop rl;
    runLoopInit(rl, true, nullptr);
    PanelStart ps(true);
    RobotState s;
    uint32_t t = 0;
    for (; t < 1000; t += CONTROL_LOOP_PERIOD_MS) {
        TEST_ASSERT_FALSE(tick(rl, ps, s, t, false).enable);
    }
    TEST_ASSERT_EQUAL(static_cast<int>(MissionPhase::WaitStart),
                      static_cast<int>(rl.mission.phase));

    for (uint32_t i = 0; i < PANEL_START_DEBOUNCE_TICKS; ++i, t += CONTROL_LOOP_PERIOD_MS) {
        tick(rl, ps, s, t, true);
    }
    TEST_ASSERT_TRUE(ps.counting);
    const uint32_t acceptedAt = ps.acceptedAtMs;
    for (; t < acceptedAt + PANEL_START_DELAY_MS; t += CONTROL_LOOP_PERIOD_MS) {
        TEST_ASSERT_FALSE(tick(rl, ps, s, t, false).enable); // released, hands off
        TEST_ASSERT_FALSE(rl.mission.start.started);
    }
    tick(rl, ps, s, t, false);
    TEST_ASSERT_TRUE(rl.mission.start.started);
    TEST_ASSERT_TRUE(rl.mission.phase != MissionPhase::WaitStart);
}

static void test_no_gyro_means_no_run()
{
    RunLoop rl;
    runLoopInit(rl, false, nullptr);
    PanelStart ps(false);
    RobotState s;
    DriveCommand c;
    for (uint32_t t = 0; t < PANEL_START_DELAY_MS / 10 + 20; ++t) {
        c = tick(rl, ps, s, t * 10, true);
        TEST_ASSERT_FALSE(c.enable);
    }
    TEST_ASSERT_EQUAL(static_cast<int>(MissionPhase::Fault), static_cast<int>(rl.mission.phase));
    TEST_ASSERT_EQUAL(static_cast<int>(MissionFault::NoImu), static_cast<int>(rl.mission.fault));
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
    PanelStart ps(false);
    RobotState s;
    for (uint32_t t = 0; t <= PANEL_START_DELAY_MS / 10 + 5; ++t) {
        tick(rl, ps, s, t * 10, false);
    }
    TEST_ASSERT_TRUE(rl.mission.start.started);
    const uint32_t startedAt = rl.mission.start.startedAtMs;
    DriveCommand c = tick(rl, ps, s, startedAt + RUN_LIMIT_MS - 10, false);
    TEST_ASSERT_TRUE(rl.mission.phase != MissionPhase::Fault);
    c = tick(rl, ps, s, startedAt + RUN_LIMIT_MS, false);
    TEST_ASSERT_EQUAL(static_cast<int>(MissionPhase::Fault), static_cast<int>(rl.mission.phase));
    TEST_ASSERT_EQUAL(static_cast<int>(MissionFault::TimeUp), static_cast<int>(rl.mission.fault));
    TEST_ASSERT_FALSE(c.enable);
}

// The whole thing in the simulated arena: power-up delay, perception from raw sensors, mission,
// motors; from a tile of the maze out over the gap and onto the line.
static void test_a_run_from_power_on_to_the_marker_and_onto_the_line()
{
    for (int facing : {0, 2}) {
        World w;
        world::photoMaze(w);
        w.start(2, 1, facing);
        const LineCalibration cal = World::calibration();
        RunLoop rl;
        runLoopInit(rl, true, &cal);
        PanelStart ps(false); // no button: it starts by itself
        uint32_t lineTicks = 0;
        float worstMm = 0.0f;
        for (uint32_t t = 0; t < RUN_LIMIT_MS / CONTROL_LOOP_PERIOD_MS && lineTicks < 600; ++t) {
            const uint32_t nowMs = t * CONTROL_LOOP_PERIOD_MS;
            // The world's own perception is not used: the loop gets the raw state.
            (void)w.sense();
            float fix = 0.0f;
            panelStartTick(ps, nowMs, false, PanelMode::Explore);
            const DriveCommand c = runLoopTick(rl, w.state, nowMs, ps.go, fix);
            if (fix != 0.0f) {
                headingSet(w.heading, w.heading.deg - fix);
            }
            w.drive(c);
            if (rl.mission.phase == MissionPhase::Fault) {
                break;
            }
            if (rl.mission.phase == MissionPhase::Line) {
                ++lineTicks;
                const float e = w.lineErrorMm();
                worstMm = e > worstMm ? e : worstMm;
            }
        }
        TEST_ASSERT_EQUAL(static_cast<int>(MissionPhase::Line), static_cast<int>(rl.mission.phase));
        TEST_ASSERT_TRUE(worstMm < 45.0f);
        TEST_ASSERT_TRUE(w.metrics.minNoseMm > model::NOSE_MM - 1.0f);
    }
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_without_a_button_the_run_begins_after_the_delay);
    RUN_TEST(test_with_a_button_the_run_begins_a_delay_after_the_press);
    RUN_TEST(test_no_gyro_means_no_run);
    RUN_TEST(test_without_a_line_calibration_the_run_is_walls_only);
    RUN_TEST(test_the_run_limit_ends_the_run);
    RUN_TEST(test_a_run_from_power_on_to_the_marker_and_onto_the_line);
    return UNITY_END();
}
