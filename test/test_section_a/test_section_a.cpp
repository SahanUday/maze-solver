// ============================================================================
//  test_section_a.cpp  --  the Section A mission in a simulated arena: the real planner, step,
//  turn, perception and motion controller against ray-cast walls and a plant with stiction.
//  No hardware needed.
// ============================================================================

#include <initializer_list>
#include <unity.h>

#include "SectionA.h"
#include "../support/world.h"

using world::World;

constexpr uint32_t RUN_TICKS = RUN_LIMIT_MS / CONTROL_LOOP_PERIOD_MS;

struct Outcome {
    SectionAPhase phase = SectionAPhase::WaitStart;
    SectionAFault fault = SectionAFault::None;
    uint32_t ticks = 0;
    float tileOffsetMm = 1.0e9f; // from the white tile's centre
    int visited = 0;
};

static int visitedCells(const SectionA &ms)
{
    int n = 0;
    for (uint8_t i = 0; i < maze::kCells; ++i) {
        n += (ms.map.cell[i] & maze::kVisited) != 0 ? 1 : 0;
    }
    return n;
}

static Outcome run(World &w, SectionA &ms, const SectionAParams &p, uint32_t maxTicks = RUN_TICKS)
{
    Outcome out;
    for (; out.ticks < maxTicks; ++out.ticks) {
        const Perception pc = w.sense();
        const DriveCommand cmd = sectionAStep(ms, p, pc);
        const float fix = sectionATakeHeadingFix(ms);
        if (fix != 0.0f) {
            headingSet(w.heading, w.heading.deg - fix);
        }
        w.drive(cmd);
        if (ms.phase == SectionAPhase::Done || ms.phase == SectionAPhase::Fault) {
            break;
        }
    }
    out.phase = ms.phase;
    out.fault = ms.fault;
    out.visited = visitedCells(ms);
    const float dx = static_cast<float>(w.x) - world::PITCH * static_cast<float>(w.arena.whiteI);
    const float dy = static_cast<float>(w.y) - world::PITCH * static_cast<float>(w.arena.whiteJ);
    out.tileOffsetMm = sqrtf(dx * dx + dy * dy);
    return out;
}

// Where the map says the robot is, against where it is: the map's North is the start heading.
static bool mapMatchesTruth(const World &w, const SectionA &ms, int startI, int startJ, int facing)
{
    static const int fwdI[4] = {1, 0, -1, 0};
    static const int fwdJ[4] = {0, 1, 0, -1};
    const int di = w.cellI() - startI;
    const int dj = w.cellJ() - startJ;
    const int fwd = di * fwdI[facing] + dj * fwdJ[facing];
    const int right = di * fwdJ[facing] - dj * fwdI[facing]; // right of the heading
    const int row = maze::kSide / 2 - fwd;
    const int col = maze::kSide / 2 + right;
    return ms.map.pos == row * maze::kSide + col;
}

static void photoWorld(World &w)
{
    w.arena.photo();
}

// ---- Photo maze ----------------------------------------------------------------------------
static void test_every_start_tile_and_heading_reaches_the_white_tile()
{
    int reentered = 0;
    int runs = 0;
    for (int i = 0; i < world::N; ++i) {
        for (int j = 0; j < world::N; ++j) {
            if (i == 0 && j == world::N - 1) {
                continue; // the white tile itself
            }
            for (int facing = 0; facing < 4; ++facing) {
                World w;
                photoWorld(w);
                w.start(i, j, facing);
                SectionA ms;
                sectionAReset(ms);
                const Outcome o = run(w, ms, sectionAParams());
                TEST_ASSERT_EQUAL_MESSAGE(static_cast<int>(SectionAPhase::Done),
                                          static_cast<int>(o.phase), "did not finish");
                TEST_ASSERT_TRUE_MESSAGE(o.tileOffsetMm < 40.0f, "stopped off the white tile");
                TEST_ASSERT_TRUE_MESSAGE(mapMatchesTruth(w, ms, i, j, facing),
                                         "map position wrong");
                TEST_ASSERT_TRUE_MESSAGE(w.minNoseMm > world::NOSE_MM - 1.0f, "hit a wall");
                TEST_ASSERT_TRUE_MESSAGE(w.minSideMm > world::BODY_HALF_MM - 15.0f,
                                         "rubbed a wall");
                reentered += w.startTileEntries > 0 ? 1 : 0;
                ++runs;
            }
        }
    }
    TEST_ASSERT_EQUAL(60, runs);
    // Starts whose only way on is through the start tile cross it; nothing else does.
    TEST_ASSERT_LESS_OR_EQUAL_INT(12, reentered);
}

static void test_a_walls_only_run_explores_every_tile_and_stops()
{
    World w;
    photoWorld(w);
    w.start(3, 0, 1);
    SectionA ms;
    sectionAReset(ms);
    SectionAParams p = sectionAParams();
    p.wallsOnly = true;
    const Outcome o = run(w, ms, p);
    TEST_ASSERT_EQUAL(static_cast<int>(SectionAPhase::Done), static_cast<int>(o.phase));
    TEST_ASSERT_EQUAL(16, o.visited);
}

static void test_without_a_white_tile_the_run_ends_in_no_route()
{
    World w;
    photoWorld(w);
    w.arena.whiteI = -1;
    w.start(2, 0, 1);
    SectionA ms;
    sectionAReset(ms);
    const SectionAParams p = sectionAParams();
    const Outcome o = run(w, ms, p);
    TEST_ASSERT_EQUAL(static_cast<int>(SectionAPhase::Fault), static_cast<int>(o.phase));
    TEST_ASSERT_EQUAL(static_cast<int>(SectionAFault::NoRoute), static_cast<int>(o.fault));
    TEST_ASSERT_EQUAL_UINT8(p.rescans, ms.rescans);
}

static void test_the_start_tile_is_left_when_it_is_white_unless_told_otherwise()
{
    World w;
    photoWorld(w);
    w.start(0, 3, 3); // on the white tile, facing south
    SectionA ms;
    sectionAReset(ms);
    const Outcome o = run(w, ms, sectionAParams(), 400);
    TEST_ASSERT_TRUE(o.phase != SectionAPhase::Done);
    TEST_ASSERT_TRUE(ms.moves > 0 || ms.turns > 0 || ms.phase == SectionAPhase::Sense);
}

static void test_a_run_that_begins_on_the_white_tile_can_finish_there()
{
    World w;
    photoWorld(w);
    w.start(0, 3, 3);
    SectionA ms;
    sectionAReset(ms);
    SectionAParams p = sectionAParams();
    p.finishAtStart = true;
    const Outcome o = run(w, ms, p);
    TEST_ASSERT_EQUAL(static_cast<int>(SectionAPhase::Done), static_cast<int>(o.phase));
    TEST_ASSERT_EQUAL_UINT16(0, ms.moves);
}

static void test_it_holds_still_once_done()
{
    World w;
    photoWorld(w);
    w.start(3, 3, 2);
    SectionA ms;
    sectionAReset(ms);
    const SectionAParams p = sectionAParams();
    const Outcome o = run(w, ms, p);
    TEST_ASSERT_EQUAL(static_cast<int>(SectionAPhase::Done), static_cast<int>(o.phase));
    const double x0 = w.x;
    const double y0 = w.y;
    DriveCommand last;
    for (int i = 0; i < 200; ++i) {
        last = sectionAStep(ms, p, w.sense());
        w.drive(last);
    }
    TEST_ASSERT_TRUE(last.enable); // holds the wheels
    TEST_ASSERT_FLOAT_WITHIN(3.0f, 0.0f, static_cast<float>(w.x - x0));
    TEST_ASSERT_FLOAT_WITHIN(3.0f, 0.0f, static_cast<float>(w.y - y0));
}

// ---- Edge cases ----------------------------------------------------------------------------
// 12 mm nearer the wall than the centre: the front sensor is blind there and never hears it.
static void test_a_wall_in_the_front_sensors_dead_zone_is_taken_as_a_wall()
{
    World w;
    photoWorld(w);
    w.start(1, 0, 2, -12.0f); // facing west at the wall between (0, 0) and (1, 0)
    SectionA ms;
    sectionAReset(ms);
    const Outcome o = run(w, ms, sectionAParams());
    TEST_ASSERT_EQUAL(static_cast<int>(SectionAPhase::Done), static_cast<int>(o.phase));
    TEST_ASSERT_TRUE(w.minNoseMm > world::NOSE_MM - 1.0f);
    TEST_ASSERT_EQUAL_UINT16(0, ms.blocked); // taken as a wall without driving at it
}

// Something held in front of the robot for half a second while it drives: it stops short of its
// centre with no wall to explain it, and carries on once the way is clear.
static void test_a_stop_no_wall_explains_is_resumed_to_the_centre()
{
    World w;
    photoWorld(w);
    w.faults.obstacleFromMs = 3500;
    w.faults.obstacleToMs = 3750;
    w.start(3, 0, 1);
    SectionA ms;
    sectionAReset(ms);
    const Outcome o = run(w, ms, sectionAParams());
    TEST_ASSERT_EQUAL(static_cast<int>(SectionAPhase::Done), static_cast<int>(o.phase));
    TEST_ASSERT_TRUE(ms.blocked > 0);
    TEST_ASSERT_TRUE(ms.resumed > 0);
    TEST_ASSERT_TRUE(ms.resumes == 0); // reset once the centre was reached
    TEST_ASSERT_TRUE(o.tileOffsetMm < 40.0f);
    TEST_ASSERT_TRUE(mapMatchesTruth(w, ms, 3, 0, 1));
    TEST_ASSERT_TRUE(w.minNoseMm > world::NOSE_MM - 1.0f);
}

static void test_dropped_readings_and_stray_echoes_cost_time_not_the_finish()
{
    for (int i : {0, 3}) {
        for (int j : {0, 1, 2}) {
            World w;
            photoWorld(w);
            w.faults.dropout = 0.30f;
            w.faults.strayNear = 0.05f;
            w.rng.s = 99u + static_cast<uint32_t>(i * 7 + j);
            w.start(i, j, (i + j) & 3);
            SectionA ms;
            sectionAReset(ms);
            const Outcome o = run(w, ms, sectionAParams());
            TEST_ASSERT_EQUAL(static_cast<int>(SectionAPhase::Done), static_cast<int>(o.phase));
            TEST_ASSERT_TRUE(o.tileOffsetMm < 40.0f);
            TEST_ASSERT_TRUE(w.minNoseMm > world::NOSE_MM - 1.0f);
            TEST_ASSERT_TRUE(mapMatchesTruth(w, ms, i, j, (i + j) & 3));
        }
    }
}

static void test_gyro_scale_and_drift_are_taken_out_by_the_walls()
{
    World w;
    photoWorld(w);
    w.faults.gyroScale = 1.02f;
    w.faults.gyroDriftDps = 0.15f;
    w.start(3, 0, 1);
    SectionA ms;
    sectionAReset(ms);
    const Outcome o = run(w, ms, sectionAParams());
    TEST_ASSERT_EQUAL(static_cast<int>(SectionAPhase::Done), static_cast<int>(o.phase));
    TEST_ASSERT_TRUE(w.minSideMm > world::BODY_HALF_MM - 15.0f);
}

// ---- Faults --------------------------------------------------------------------------------
static Outcome runWith(const SectionAParams &p, float sticking = 0.4f, int obstacleFrom = 0,
                       int obstacleTo = 0)
{
    World w;
    photoWorld(w);
    w.sticking = sticking;
    w.faults.obstacleFromMs = static_cast<uint32_t>(obstacleFrom);
    w.faults.obstacleToMs = static_cast<uint32_t>(obstacleTo);
    w.start(3, 0, 1);
    SectionA ms;
    sectionAReset(ms);
    return run(w, ms, p);
}

static void test_a_step_that_cannot_start_is_a_stuck_step()
{
    SectionAParams p = sectionAParams();
    p.step.plan.accelMmS2 = 0.0f; // the profile never leaves zero
    const Outcome o = runWith(p);
    TEST_ASSERT_EQUAL(static_cast<int>(SectionAPhase::Fault), static_cast<int>(o.phase));
    TEST_ASSERT_EQUAL(static_cast<int>(SectionAFault::StuckStep), static_cast<int>(o.fault));
}

static void test_a_turn_that_cannot_move_is_retried_then_a_stuck_turn()
{
    SectionAParams p = sectionAParams();
    p.step.plan.turnAccelDps2 = 0.0f;
    // Facing south on (3, 0) the first plan is a turn.
    World w;
    photoWorld(w);
    w.start(3, 0, 3);
    SectionA ms;
    sectionAReset(ms);
    const Outcome o = run(w, ms, p);
    TEST_ASSERT_EQUAL(static_cast<int>(SectionAPhase::Fault), static_cast<int>(o.phase));
    TEST_ASSERT_EQUAL(static_cast<int>(SectionAFault::StuckTurn), static_cast<int>(o.fault));
    TEST_ASSERT_EQUAL_UINT8(p.turnRetries + 1, ms.turnTries);
}

static void test_wheels_that_never_break_away_stall_the_motors()
{
    const Outcome o = runWith(sectionAParams(), 100.0f);
    TEST_ASSERT_EQUAL(static_cast<int>(SectionAPhase::Fault), static_cast<int>(o.phase));
    TEST_ASSERT_EQUAL(static_cast<int>(SectionAFault::Stall), static_cast<int>(o.fault));
}

static void test_too_many_cut_short_steps_end_the_run()
{
    SectionAParams p = sectionAParams();
    p.maxBlocked = 0;
    const Outcome o = runWith(p, 0.4f, 3500, 3750);
    TEST_ASSERT_EQUAL(static_cast<int>(SectionAPhase::Fault), static_cast<int>(o.phase));
    TEST_ASSERT_EQUAL(static_cast<int>(SectionAFault::TooManyStops), static_cast<int>(o.fault));
}

// ---- The state machine ----------------------------------------------------------------------
static void test_nothing_moves_until_the_start_button()
{
    World w;
    photoWorld(w);
    w.start(1, 1, 0);
    w.state.startButtonPressed = false;
    SectionA ms;
    sectionAReset(ms);
    const SectionAParams p = sectionAParams();
    DriveCommand cmd;
    for (int i = 0; i < 50; ++i) {
        w.state.startButtonPressed = false;
        cmd = sectionAStep(ms, p, w.sense());
        w.drive(cmd);
    }
    TEST_ASSERT_EQUAL(static_cast<int>(SectionAPhase::WaitStart), static_cast<int>(ms.phase));
    TEST_ASSERT_FALSE(cmd.enable);
    TEST_ASSERT_EQUAL_INT16(0, cmd.left);
}

static void test_a_failure_set_by_the_caller_cuts_the_motors()
{
    SectionA ms;
    sectionAReset(ms);
    sectionAFail(ms, SectionAFault::NoImu);
    World w;
    photoWorld(w);
    w.start(1, 1, 0);
    const DriveCommand cmd = sectionAStep(ms, sectionAParams(), w.sense());
    TEST_ASSERT_FALSE(cmd.enable);
    TEST_ASSERT_EQUAL(static_cast<int>(SectionAFault::NoImu), static_cast<int>(ms.fault));
}

static void test_the_sweep_goes_the_way_of_the_turn()
{
    using section_a_detail::sweepTo;
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -90.0f, sweepTo(0.0f, -90.0f, 1));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 90.0f, sweepTo(0.0f, 90.0f, 3));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 180.0f, sweepTo(0.0f, 180.0f, 2) > 0 ? 180.0f : -1.0f);
    // From 6 degrees off the grid, a right turn to -90 sweeps 96, not 90.
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -96.0f, sweepTo(6.0f, -90.0f, 1));
    // A right turn never comes out positive, a left one never negative.
    TEST_ASSERT_TRUE(sweepTo(-100.0f, -90.0f, 1) < 0.0f);
    TEST_ASSERT_TRUE(sweepTo(100.0f, 90.0f, 3) > 0.0f);
}

static void test_map_directions_run_clockwise_and_the_gyro_counter_clockwise()
{
    SectionA ms;
    ms.h0 = 10.0f;
    using section_a_detail::gridHeading;
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f, gridHeading(ms, maze::Dir::North));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -80.0f, gridHeading(ms, maze::Dir::East));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -170.0f, gridHeading(ms, maze::Dir::South));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, gridHeading(ms, maze::Dir::West));
}

static void test_the_default_parameters_are_the_spec()
{
    const SectionAParams p = sectionAParams();
    TEST_ASSERT_EQUAL_FLOAT(ACCEL_MM_S2, p.step.plan.accelMmS2);
    TEST_ASSERT_EQUAL_FLOAT(KP_TURN, p.control.kpTurn);
    TEST_ASSERT_EQUAL_UINT16(MOTOR_PWM_TOP, p.control.pwmTop);
    TEST_ASSERT_EQUAL_UINT16(SECTION_A_SENSE_MS, p.senseMs);
    TEST_ASSERT_EQUAL_UINT8(SECTION_A_RESCANS, p.rescans);
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_every_start_tile_and_heading_reaches_the_white_tile);
    RUN_TEST(test_a_walls_only_run_explores_every_tile_and_stops);
    RUN_TEST(test_without_a_white_tile_the_run_ends_in_no_route);
    RUN_TEST(test_the_start_tile_is_left_when_it_is_white_unless_told_otherwise);
    RUN_TEST(test_a_run_that_begins_on_the_white_tile_can_finish_there);
    RUN_TEST(test_it_holds_still_once_done);
    RUN_TEST(test_a_wall_in_the_front_sensors_dead_zone_is_taken_as_a_wall);
    RUN_TEST(test_a_stop_no_wall_explains_is_resumed_to_the_centre);
    RUN_TEST(test_dropped_readings_and_stray_echoes_cost_time_not_the_finish);
    RUN_TEST(test_gyro_scale_and_drift_are_taken_out_by_the_walls);
    RUN_TEST(test_a_step_that_cannot_start_is_a_stuck_step);
    RUN_TEST(test_a_turn_that_cannot_move_is_retried_then_a_stuck_turn);
    RUN_TEST(test_wheels_that_never_break_away_stall_the_motors);
    RUN_TEST(test_too_many_cut_short_steps_end_the_run);
    RUN_TEST(test_nothing_moves_until_the_start_button);
    RUN_TEST(test_a_failure_set_by_the_caller_cuts_the_motors);
    RUN_TEST(test_the_sweep_goes_the_way_of_the_turn);
    RUN_TEST(test_map_directions_run_clockwise_and_the_gyro_counter_clockwise);
    RUN_TEST(test_the_default_parameters_are_the_spec);
    return UNITY_END();
}
