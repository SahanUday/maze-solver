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
    const float dx = static_cast<float>(w.x) - world::PITCH * static_cast<float>(w.arena.markerI);
    const float dy = static_cast<float>(w.y) - world::PITCH * static_cast<float>(w.arena.markerJ);
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

// The navigation tests end on the marker's tile; the line is tested on its own below.
static SectionAParams stopAtMarker()
{
    SectionAParams p = sectionAParams();
    p.followLine = false;
    return p;
}

static void photoWorld(World &w)
{
    w.arena.photo();
}

// ---- Photo maze ----------------------------------------------------------------------------
static void test_every_start_tile_and_heading_reaches_the_marker()
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
                const Outcome o = run(w, ms, stopAtMarker());
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
    SectionAParams p = stopAtMarker();
    p.wallsOnly = true;
    const Outcome o = run(w, ms, p);
    TEST_ASSERT_EQUAL(static_cast<int>(SectionAPhase::Done), static_cast<int>(o.phase));
    TEST_ASSERT_EQUAL(16, o.visited);
}

static void test_without_a_marker_the_run_ends_in_no_route()
{
    World w;
    photoWorld(w);
    w.arena.markerI = -1;
    w.start(2, 0, 1);
    SectionA ms;
    sectionAReset(ms);
    const SectionAParams p = stopAtMarker();
    const Outcome o = run(w, ms, p);
    TEST_ASSERT_EQUAL(static_cast<int>(SectionAPhase::Fault), static_cast<int>(o.phase));
    TEST_ASSERT_EQUAL(static_cast<int>(SectionAFault::NoRoute), static_cast<int>(o.fault));
    TEST_ASSERT_EQUAL_UINT8(p.rescans, ms.rescans);
}

// The exit looks like a wall in the front sensor's dead zone: silent. Down a straight corridor to
// it the way is open for the whole run, and the robot tells that from the sides: a side sensor
// turned to look, the sides it measured on the way in, and cells it has already been in.
static void corridorToTheExit(World &w)
{
    for (int j = 0; j < world::N; ++j) {
        w.arena.vw[0 * world::N + j] = true;
        w.arena.vw[world::N * world::N + j] = j != 3; // the marker tile's east edge is open
    }
    for (int i = 0; i < world::N; ++i) {
        w.arena.hw[0 * world::N + i] = true;
        w.arena.hw[world::N * world::N + i] = true;
        w.arena.hw[3 * world::N + i] = true; // the corridor is row 3
    }
    w.arena.markerI = 3;
    w.arena.markerJ = 3;
    w.arena.exitDir = 0; // east
    w.arena.build();
}

static void test_a_straight_corridor_to_the_exit_is_driven_down_from_every_start()
{
    for (int i = 0; i < 3; ++i) {
        for (int facing = 0; facing < 4; ++facing) {
            World w;
            corridorToTheExit(w);
            w.start(i, 3, facing);
            SectionA ms;
            sectionAReset(ms);
            const Outcome o = run(w, ms, stopAtMarker());
            TEST_ASSERT_EQUAL_MESSAGE(static_cast<int>(SectionAPhase::Done),
                                      static_cast<int>(o.phase), "did not reach the marker");
            TEST_ASSERT_TRUE_MESSAGE(o.tileOffsetMm < 40.0f, "stopped off the marker");
            TEST_ASSERT_TRUE_MESSAGE(w.minNoseMm > world::NOSE_MM - 1.0f, "hit a wall");
        }
    }
}

static void test_it_holds_still_once_done()
{
    World w;
    photoWorld(w);
    w.start(3, 3, 2);
    SectionA ms;
    sectionAReset(ms);
    const SectionAParams p = stopAtMarker();
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
    const Outcome o = run(w, ms, stopAtMarker());
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
    const Outcome o = run(w, ms, stopAtMarker());
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
            const Outcome o = run(w, ms, stopAtMarker());
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
    const Outcome o = run(w, ms, stopAtMarker());
    TEST_ASSERT_EQUAL(static_cast<int>(SectionAPhase::Done), static_cast<int>(o.phase));
    TEST_ASSERT_TRUE(w.minSideMm > world::BODY_HALF_MM - 15.0f);
}

// Wheels that need the measured duty to start and more to keep rolling than the model's
// default: a move that ends short stands still, and only the friction feedforward carries the
// profile's last millimetres.
static void test_every_start_finishes_on_wheels_that_roll_hard()
{
    for (int i = 0; i < world::N; ++i) {
        for (int j = 0; j < world::N; ++j) {
            if (i == 0 && j == world::N - 1) {
                continue;
            }
            for (int facing = 0; facing < 4; ++facing) {
                World w;
                photoWorld(w);
                w.sticking = 1.0f;
                w.sustain = 130.0;
                w.sustainRight = 1.25;
                w.rng.s = 777u + static_cast<uint32_t>(i * 31 + j * 7 + facing);
                w.start(i, j, facing);
                SectionA ms;
                sectionAReset(ms);
                const Outcome o = run(w, ms, stopAtMarker());
                TEST_ASSERT_EQUAL_MESSAGE(static_cast<int>(SectionAPhase::Done),
                                          static_cast<int>(o.phase), "did not finish");
                TEST_ASSERT_TRUE_MESSAGE(w.minNoseMm > world::NOSE_MM - 1.0f, "hit a wall");
            }
        }
    }
}

// The gyro's scale error turns every pivot short or long; the walls take it out while it stays
// under about 1%. The measured chip reads 0.1 to 0.2% high.
static void test_a_gyro_half_a_percent_off_still_finishes_from_every_start()
{
    for (const float scale : {0.995f, 1.005f}) {
        for (int i = 0; i < world::N; ++i) {
            for (int j = 0; j < world::N; ++j) {
                if (i == 0 && j == world::N - 1) {
                    continue;
                }
                World w;
                photoWorld(w);
                w.faults.gyroScale = scale;
                w.rng.s = 31u + static_cast<uint32_t>(i * 5 + j);
                w.start(i, j, (i + 2 * j) & 3);
                SectionA ms;
                sectionAReset(ms);
                const Outcome o = run(w, ms, stopAtMarker());
                TEST_ASSERT_EQUAL_MESSAGE(static_cast<int>(SectionAPhase::Done),
                                          static_cast<int>(o.phase), "did not finish");
                TEST_ASSERT_TRUE_MESSAGE(w.minNoseMm > world::NOSE_MM - 1.0f, "hit a wall");
                TEST_ASSERT_TRUE_MESSAGE(w.minSideMm > world::BODY_HALF_MM - 15.0f,
                                         "rubbed a wall");
            }
        }
    }
}

// ---- Out of the maze and onto the line --------------------------------------------------------
struct LineOutcome {
    SectionAPhase phase = SectionAPhase::WaitStart;
    SectionAFault fault = SectionAFault::None;
    bool reachedLine = false;
    float worstErrMm = 0.0f;      // from the centreline, once the follower has the line
    float worstStraightMm = 0.0f; // ... on the straight, before the bend
    float finalErrMm = 0.0f;
    bool onRampSeen = false;
};

// Run until the follower has had `lineTicks` ticks (or the run ends).
static LineOutcome runToLine(World &w, SectionA &ms, const SectionAParams &p, uint32_t lineTicks)
{
    LineOutcome o;
    uint32_t onLine = 0;
    for (uint32_t t = 0; t < RUN_TICKS && onLine < lineTicks; ++t) {
        const Perception pc = w.sense();
        const DriveCommand cmd = sectionAStep(ms, p, pc);
        const float fix = sectionATakeHeadingFix(ms);
        if (fix != 0.0f) {
            headingSet(w.heading, w.heading.deg - fix);
        }
        w.drive(cmd);
        if (ms.phase == SectionAPhase::Fault || ms.phase == SectionAPhase::Done) {
            break;
        }
        if (ms.phase == SectionAPhase::Line) {
            o.reachedLine = true;
            ++onLine;
            const float e = w.lineErrorMm();
            o.worstErrMm = e > o.worstErrMm ? e : o.worstErrMm;
            if (w.lineErrorMm() < 60.0f && w.lateAlongMm() < world::LINE_STRAIGHT_TO_MM - 60.0f) {
                o.worstStraightMm = e > o.worstStraightMm ? e : o.worstStraightMm;
            }
            o.onRampSeen = o.onRampSeen || pc.inclineCounts > RAMP_INCLINE_COUNTS;
        }
    }
    o.phase = ms.phase;
    o.fault = ms.fault;
    o.finalErrMm = w.lineErrorMm();
    return o;
}

static void test_the_marker_leads_over_the_gap_and_onto_the_line()
{
    for (int facing = 0; facing < 4; ++facing) {
        World w;
        photoWorld(w);
        w.start(2, 1, facing);
        SectionA ms;
        sectionAReset(ms);
        // 8 s of following: the straight, the ramp and the bend to the left after it.
        const LineOutcome o = runToLine(w, ms, sectionAParams(), 800);
        TEST_ASSERT_TRUE_MESSAGE(o.reachedLine, "never reached the line");
        TEST_ASSERT_EQUAL(static_cast<int>(SectionAPhase::Line), static_cast<int>(o.phase));
        TEST_ASSERT_TRUE_MESSAGE(o.worstStraightMm < 15.0f, "wandered on the straight");
        TEST_ASSERT_TRUE_MESSAGE(o.worstErrMm < 45.0f, "lost the line at the bend");
        TEST_ASSERT_TRUE(o.onRampSeen);
        TEST_ASSERT_TRUE(w.minNoseMm > world::NOSE_MM - 1.0f);
    }
}

static void test_every_start_reaches_the_line_and_follows_it_round_the_bend()
{
    for (int i = 0; i < world::N; ++i) {
        for (int j = 0; j < world::N; ++j) {
            if (i == 0 && j == world::N - 1) {
                continue;
            }
            World w;
            photoWorld(w);
            w.rng.s = 55u + static_cast<uint32_t>(i * 9 + j);
            w.start(i, j, (i + j) & 3);
            SectionA ms;
            sectionAReset(ms);
            const LineOutcome o = runToLine(w, ms, sectionAParams(), 1000);
            TEST_ASSERT_EQUAL_MESSAGE(static_cast<int>(SectionAPhase::Line),
                                      static_cast<int>(o.phase), "not following the line");
            TEST_ASSERT_TRUE_MESSAGE(o.worstStraightMm < 15.0f, "wandered on the straight");
            TEST_ASSERT_TRUE_MESSAGE(o.worstErrMm < 45.0f, "lost the line at the bend");
            TEST_ASSERT_TRUE_MESSAGE(w.minNoseMm > world::NOSE_MM - 1.0f, "hit a wall");
        }
    }
}

// The marker's tile is entered facing a wall and the way out is to the side: it stops, reads both
// sides (the other open side leads to a maze tile, the exit to nothing), turns and drives out.
static void test_an_exit_to_the_side_is_turned_to_and_driven_out()
{
    for (int exitSide = 0; exitSide < 2; ++exitSide) {
        World w;
        // Row 3 is a corridor along the top of the arena. The marker tile (3,3) is at its end: an
        // open edge north (exitSide 0) or the tile below it being open and the exit north.
        for (int j = 0; j < world::N; ++j) {
            w.arena.vw[0 * world::N + j] = true;
            w.arena.vw[world::N * world::N + j] = true;
        }
        for (int i = 0; i < world::N; ++i) {
            w.arena.hw[0 * world::N + i] = true;
            w.arena.hw[world::N * world::N + i] = i != 3; // the marker tile's north edge is open
            w.arena.hw[3 * world::N + i] = i == 3 && exitSide == 1 ? false : true;
        }
        w.arena.markerI = 3;
        w.arena.markerJ = 3;
        w.arena.exitDir = 1; // north
        w.arena.build();
        w.start(0, 3, 0); // along the corridor; the tile ahead ends in a wall to the east
        SectionA ms;
        sectionAReset(ms);
        const LineOutcome o = runToLine(w, ms, sectionAParams(), 500);
        TEST_ASSERT_EQUAL_MESSAGE(static_cast<int>(SectionAPhase::Line), static_cast<int>(o.phase),
                                  "did not leave by the side");
        TEST_ASSERT_TRUE(ms.exiting); // it turned
        TEST_ASSERT_TRUE(o.worstStraightMm < 15.0f);
        TEST_ASSERT_TRUE(w.minNoseMm > world::NOSE_MM - 1.0f);
    }
}

static void test_a_line_that_never_comes_is_no_line()
{
    World w;
    photoWorld(w);
    w.arena.markerI = 0;
    w.arena.markerJ = 3;
    w.start(0, 2, 1);
    SectionAParams p = sectionAParams();
    p.noLineMm = 90.0f; // the gap is 180 mm out: the follower is never reached
    SectionA ms;
    sectionAReset(ms);
    const LineOutcome o = runToLine(w, ms, p, 100);
    TEST_ASSERT_EQUAL(static_cast<int>(SectionAPhase::Fault), static_cast<int>(o.phase));
    TEST_ASSERT_EQUAL(static_cast<int>(SectionAFault::NoLine), static_cast<int>(o.fault));
}

static void test_a_line_lost_is_searched_for_and_then_the_motors_are_cut()
{
    World w;
    photoWorld(w);
    w.start(0, 2, 1);
    SectionA ms;
    sectionAReset(ms);
    LineOutcome o = runToLine(w, ms, sectionAParams(), 100);
    TEST_ASSERT_EQUAL(static_cast<int>(SectionAPhase::Line), static_cast<int>(o.phase));
    // The line is gone: nothing but white board.
    const SectionAParams p = sectionAParams();
    w.lineGone = true;
    for (uint32_t t = 0; t < 1000 && ms.phase == SectionAPhase::Line; ++t) {
        const DriveCommand cmd = sectionAStep(ms, p, w.sense());
        w.drive(cmd);
    }
    TEST_ASSERT_EQUAL(static_cast<int>(SectionAPhase::Fault), static_cast<int>(ms.phase));
    TEST_ASSERT_EQUAL(static_cast<int>(SectionAFault::LineLost), static_cast<int>(ms.fault));
    DriveCommand last = sectionAStep(ms, p, w.sense());
    TEST_ASSERT_FALSE(last.enable);
}

// ---- Faults --------------------------------------------------------------------------------
static Outcome runWith(const SectionAParams &p, float sticking = 1.0f, int obstacleFrom = 0,
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
    SectionAParams p = stopAtMarker();
    p.step.plan.accelMmS2 = 0.0f; // the profile never leaves zero
    const Outcome o = runWith(p);
    TEST_ASSERT_EQUAL(static_cast<int>(SectionAPhase::Fault), static_cast<int>(o.phase));
    TEST_ASSERT_EQUAL(static_cast<int>(SectionAFault::StuckStep), static_cast<int>(o.fault));
}

static void test_a_turn_that_cannot_move_is_retried_then_a_stuck_turn()
{
    SectionAParams p = stopAtMarker();
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
    const Outcome o = runWith(stopAtMarker(), 100.0f);
    TEST_ASSERT_EQUAL(static_cast<int>(SectionAPhase::Fault), static_cast<int>(o.phase));
    TEST_ASSERT_EQUAL(static_cast<int>(SectionAFault::Stall), static_cast<int>(o.fault));
}

static void test_too_many_cut_short_steps_end_the_run()
{
    SectionAParams p = stopAtMarker();
    p.maxBlocked = 0;
    const Outcome o = runWith(p, 1.0f, 3500, 3750);
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
    const SectionAParams p = stopAtMarker();
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
    const DriveCommand cmd = sectionAStep(ms, stopAtMarker(), w.sense());
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
    RUN_TEST(test_every_start_tile_and_heading_reaches_the_marker);
    RUN_TEST(test_a_walls_only_run_explores_every_tile_and_stops);
    RUN_TEST(test_without_a_marker_the_run_ends_in_no_route);
    RUN_TEST(test_a_straight_corridor_to_the_exit_is_driven_down_from_every_start);
    RUN_TEST(test_it_holds_still_once_done);
    RUN_TEST(test_a_wall_in_the_front_sensors_dead_zone_is_taken_as_a_wall);
    RUN_TEST(test_a_stop_no_wall_explains_is_resumed_to_the_centre);
    RUN_TEST(test_dropped_readings_and_stray_echoes_cost_time_not_the_finish);
    RUN_TEST(test_gyro_scale_and_drift_are_taken_out_by_the_walls);
    RUN_TEST(test_every_start_finishes_on_wheels_that_roll_hard);
    RUN_TEST(test_a_gyro_half_a_percent_off_still_finishes_from_every_start);
    RUN_TEST(test_a_step_that_cannot_start_is_a_stuck_step);
    RUN_TEST(test_a_turn_that_cannot_move_is_retried_then_a_stuck_turn);
    RUN_TEST(test_wheels_that_never_break_away_stall_the_motors);
    RUN_TEST(test_too_many_cut_short_steps_end_the_run);
    RUN_TEST(test_nothing_moves_until_the_start_button);
    RUN_TEST(test_a_failure_set_by_the_caller_cuts_the_motors);
    RUN_TEST(test_the_sweep_goes_the_way_of_the_turn);
    RUN_TEST(test_map_directions_run_clockwise_and_the_gyro_counter_clockwise);
    RUN_TEST(test_the_default_parameters_are_the_spec);
    RUN_TEST(test_the_marker_leads_over_the_gap_and_onto_the_line);
    RUN_TEST(test_every_start_reaches_the_line_and_follows_it_round_the_bend);
    RUN_TEST(test_a_line_that_never_comes_is_no_line);
    RUN_TEST(test_a_line_lost_is_searched_for_and_then_the_motors_are_cut);
    RUN_TEST(test_an_exit_to_the_side_is_turned_to_and_driven_out);
    return UNITY_END();
}
