// ============================================================================
//  test_section_a.cpp  --  the mission manager (Section A, the way out, the line) in a simulated
//  arena: the real planner, step, turn, perception and motion controller against ray-cast walls
//  and a plant with stiction. No hardware needed.
// ============================================================================

#include <initializer_list>
#include <unity.h>

#include "mission/MissionManager.h"
#include "../../support/MapIndex.h"
#include "../../support/world/Presets.h"
#include "spec/fixed/Hardware.h"
#include "spec/fixed/Rules.h"
#include "spec/tuning/CellExplorer.h"
#include "spec/tuning/Imu.h"
#include "spec/tuning/Motion.h"
#include "spec/tuning/Run.h"

using world::Arena;
using world::World;

constexpr int N = SECTION_A_SIZE_TILES;

constexpr uint32_t RUN_TICKS = RUN_LIMIT_MS / CONTROL_LOOP_PERIOD_MS;

struct Outcome {
    MissionPhase phase = MissionPhase::WaitStart;
    MissionFault fault = MissionFault::None;
    uint32_t ticks = 0;
    float tileOffsetMm = 1.0e9f; // from the white tile's centre
    int visited = 0;
};

// A manager ready to run: reset, and told the run has started.
static void begin(MissionManager &ms)
{
    missionReset(ms);
    ms.start.started = true;
}

static int visitedCells(const MissionManager &ms)
{
    int n = 0;
    for (uint8_t i = 0; i < maze::kCells; ++i) {
        n += maze::explorerVisited(ms.explore.map, i) ? 1 : 0;
    }
    return n;
}

// How far the robot is from the marker tile's centre.
static float markerOffsetMm(const World &w)
{
    const float dx = static_cast<float>(w.plant.x) - w.floor.frame.x;
    const float dy = static_cast<float>(w.plant.y) - w.floor.frame.y;
    return sqrtf(dx * dx + dy * dy);
}

static Outcome run(World &w, MissionManager &ms, const MissionParams &p,
                   uint32_t maxTicks = RUN_TICKS)
{
    Outcome out;
    for (; out.ticks < maxTicks; ++out.ticks) {
        const Perception pc = w.sense();
        const DriveCommand cmd = missionStep(ms, p, pc);
        const float fix = missionTakeHeadingFix(ms);
        if (fix != 0.0f) {
            headingSet(w.heading, w.heading.deg - fix);
        }
        w.drive(cmd);
        if (ms.phase == MissionPhase::Done || ms.phase == MissionPhase::Fault) {
            break;
        }
    }
    out.phase = ms.phase;
    out.fault = ms.fault;
    out.visited = visitedCells(ms);
    out.tileOffsetMm = markerOffsetMm(w);
    return out;
}

// Where the map says the robot is, against where it is.
static bool mapMatchesTruth(const World &w, const MissionManager &ms, int startI, int startJ,
                            int facing)
{
    return ms.explore.map.pos ==
           mapindex::of(maze::kSide, w.cellI(), w.cellJ(), startI, startJ, facing);
}

// The navigation tests end on the marker's tile; the line is tested on its own below.
static MissionParams stopAtMarker()
{
    MissionParams p = missionParams();
    p.parts.last = Part::AExplore; // the run ends on the marker's tile
    return p;
}

// ---- Photo maze ----------------------------------------------------------------------------
static void test_every_start_tile_and_heading_reaches_the_marker()
{
    int reentered = 0;
    int runs = 0;
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            if (i == 0 && j == N - 1) {
                continue; // the white tile itself
            }
            for (int facing = 0; facing < 4; ++facing) {
                World w;
                world::photoMaze(w);
                w.start(i, j, facing);
                MissionManager ms;
                begin(ms);
                const Outcome o = run(w, ms, stopAtMarker());
                TEST_ASSERT_EQUAL_MESSAGE(static_cast<int>(MissionPhase::Done),
                                          static_cast<int>(o.phase), "did not finish");
                TEST_ASSERT_TRUE_MESSAGE(o.tileOffsetMm < 40.0f, "stopped off the white tile");
                TEST_ASSERT_TRUE_MESSAGE(mapMatchesTruth(w, ms, i, j, facing),
                                         "map position wrong");
                TEST_ASSERT_TRUE_MESSAGE(w.metrics.minNoseMm > model::NOSE_MM - 1.0f, "hit a wall");
                TEST_ASSERT_TRUE_MESSAGE(w.metrics.minSideMm > model::BODY_HALF_WIDTH_MM - 15.0f,
                                         "rubbed a wall");
                reentered += w.metrics.startTileEntries > 0 ? 1 : 0;
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
    world::photoMaze(w);
    w.start(3, 0, 1);
    MissionManager ms;
    begin(ms);
    MissionParams p = stopAtMarker();
    p.wallsOnly = true;
    const Outcome o = run(w, ms, p);
    TEST_ASSERT_EQUAL(static_cast<int>(MissionPhase::Done), static_cast<int>(o.phase));
    TEST_ASSERT_EQUAL(16, o.visited);
}

static void test_without_a_marker_the_run_ends_in_no_route()
{
    World w;
    world::photoMaze(w);
    w.floor = world::Floor{};
    w.start(2, 0, 1);
    MissionManager ms;
    begin(ms);
    const MissionParams p = stopAtMarker();
    const Outcome o = run(w, ms, p);
    TEST_ASSERT_EQUAL(static_cast<int>(MissionPhase::Fault), static_cast<int>(o.phase));
    TEST_ASSERT_EQUAL(static_cast<int>(MissionFault::NoRoute), static_cast<int>(o.fault));
    TEST_ASSERT_EQUAL_UINT8(p.rescans, ms.explore.rescans);
}

// The exit looks like a wall in the front sensor's dead zone: silent. Down a straight corridor to
// it the way is open for the whole run, and the robot tells that from the sides: a side sensor
// turned to look, the sides it measured on the way in, and cells it has already been in.
static void corridorToTheExit(World &w)
{
    Arena &a = w.arena;
    for (int j = 0; j < N; ++j) {
        a.side(0, j, 2) = true;
        a.side(N - 1, j, 0) = j != 3; // the marker tile's east edge is open
    }
    for (int i = 0; i < N; ++i) {
        a.side(i, 0, 3) = true;
        a.side(i, N - 1, 1) = true;
        a.side(i, 3, 3) = true; // the corridor is row 3
    }
    a.build();
    w.floor = world::bridgeOutFloor(3, 3, 0);
}

static void test_a_straight_corridor_to_the_exit_is_driven_down_from_every_start()
{
    for (int i = 0; i < 3; ++i) {
        for (int facing = 0; facing < 4; ++facing) {
            World w;
            corridorToTheExit(w);
            w.start(i, 3, facing);
            MissionManager ms;
            begin(ms);
            const Outcome o = run(w, ms, stopAtMarker());
            TEST_ASSERT_EQUAL_MESSAGE(static_cast<int>(MissionPhase::Done),
                                      static_cast<int>(o.phase), "did not reach the marker");
            TEST_ASSERT_TRUE_MESSAGE(o.tileOffsetMm < 40.0f, "stopped off the marker");
            TEST_ASSERT_TRUE_MESSAGE(w.metrics.minNoseMm > model::NOSE_MM - 1.0f, "hit a wall");
        }
    }
}

// The explorer on its own, without a mission around it: its target into the motion controller, the
// controller's command into the world. Runs until the explorer says something other than Running.
static ExploreStatus driveExplorer(World &w, CellExplorer &ce, MotionControl &mc,
                                   const MissionParams &p)
{
    ExploreStatus st = ExploreStatus::Running;
    for (uint32_t t = 0; t < RUN_TICKS && st == ExploreStatus::Running; ++t) {
        const Perception pc = w.sense();
        MotionTarget target;
        target.distRefMm = pc.motion.distMm;
        target.headingRefDeg = cellExplorerHeadingRef(ce);
        target.newMove = true;
        st = cellExplorerStep(ce, p, ExploreInput{}, pc, target);
        const float fix = ce.headingFixDeg;
        ce.headingFixDeg = 0.0f;
        if (fix != 0.0f) {
            headingSet(w.heading, w.heading.deg - fix);
        }
        const MotionOutput out = motionControlStep(mc, p.control, target, pc.motion, p.dtMs);
        DriveCommand cmd;
        cmd.enable = true;
        cmd.left = out.left;
        cmd.right = out.right;
        w.drive(cmd);
    }
    return st;
}

// Explore everything, then drive to the marker tile over checked edges and stand on it.
static void test_after_exploring_it_drives_to_the_marker_tile_through_known_cells()
{
    const struct {
        int i, j, facing;
    } starts[] = {{3, 0, 1}, {2, 1, 2}, {0, 0, 0}, {3, 3, 3}};
    for (const auto &st : starts) {
        World w;
        world::photoMaze(w);
        w.start(st.i, st.j, st.facing);
        MissionParams p = missionParams();
        p.wallsOnly = true;
        CellExplorer ce;
        cellExplorerReset(ce);
        MotionControl mc;
        motionControlReset(mc);
        cellExplorerStart(ce, w.sense().motion);
        TEST_ASSERT_EQUAL_MESSAGE(static_cast<int>(ExploreStatus::Explored),
                                  static_cast<int>(driveExplorer(w, ce, mc, p)),
                                  "did not explore it all");

        const int entriesBefore = w.metrics.startTileEntries;
        cellExplorerDriveTo(
            ce, static_cast<maze::Explorer::Index>(mapindex::of(
                    maze::kSide, w.floor.originI(), w.floor.originJ(), st.i, st.j, st.facing)));
        TEST_ASSERT_EQUAL_MESSAGE(static_cast<int>(ExploreStatus::AtTarget),
                                  static_cast<int>(driveExplorer(w, ce, mc, p)), "did not arrive");
        TEST_ASSERT_TRUE_MESSAGE(markerOffsetMm(w) < 40.0f, "stopped off the marker tile");
        TEST_ASSERT_TRUE_MESSAGE(w.metrics.minNoseMm > model::NOSE_MM - 1.0f, "hit a wall");
        TEST_ASSERT_EQUAL_MESSAGE(entriesBefore, w.metrics.startTileEntries,
                                  "crossed the start tile");

        // Forget that any edge was looked at (the walls stay): there is a way to the start tile,
        // but not over edges that were looked at.
        for (uint8_t &c : ce.map.cell) {
            c = static_cast<uint8_t>(c & 0x0F);
        }
        cellExplorerDriveTo(ce, maze::kStartCell);
        TEST_ASSERT_EQUAL_MESSAGE(static_cast<int>(ExploreStatus::Fault),
                                  static_cast<int>(driveExplorer(w, ce, mc, p)),
                                  "a target over unchecked edges did not fault");
        TEST_ASSERT_EQUAL(static_cast<int>(MissionFault::TargetUnreachable),
                          static_cast<int>(ce.fault));
    }
}

static void test_it_holds_still_once_done()
{
    World w;
    world::photoMaze(w);
    w.start(3, 3, 2);
    MissionManager ms;
    begin(ms);
    const MissionParams p = stopAtMarker();
    const Outcome o = run(w, ms, p);
    TEST_ASSERT_EQUAL(static_cast<int>(MissionPhase::Done), static_cast<int>(o.phase));
    const double x0 = w.plant.x;
    const double y0 = w.plant.y;
    DriveCommand last;
    for (int i = 0; i < 200; ++i) {
        last = missionStep(ms, p, w.sense());
        w.drive(last);
    }
    TEST_ASSERT_TRUE(last.enable); // holds the wheels
    TEST_ASSERT_FLOAT_WITHIN(3.0f, 0.0f, static_cast<float>(w.plant.x - x0));
    TEST_ASSERT_FLOAT_WITHIN(3.0f, 0.0f, static_cast<float>(w.plant.y - y0));
}

// ---- Edge cases ----------------------------------------------------------------------------
// 12 mm nearer the wall than the centre: the front sensor is blind there and never hears it.
static void test_a_wall_in_the_front_sensors_dead_zone_is_taken_as_a_wall()
{
    World w;
    world::photoMaze(w);
    w.start(1, 0, 2, -12.0f); // facing west at the wall between (0, 0) and (1, 0)
    MissionManager ms;
    begin(ms);
    const Outcome o = run(w, ms, stopAtMarker());
    TEST_ASSERT_EQUAL(static_cast<int>(MissionPhase::Done), static_cast<int>(o.phase));
    TEST_ASSERT_TRUE(w.metrics.minNoseMm > model::NOSE_MM - 1.0f);
    TEST_ASSERT_EQUAL_UINT16(0, ms.explore.blocked); // taken as a wall without driving at it
}

// Something held in front of the robot for half a second while it drives: it stops short of its
// centre with no wall to explain it, and carries on once the way is clear.
static void test_a_stop_no_wall_explains_is_resumed_to_the_centre()
{
    World w;
    world::photoMaze(w);
    w.faults.obstacleFromMs = 3500;
    w.faults.obstacleToMs = 3750;
    w.start(3, 0, 1);
    MissionManager ms;
    begin(ms);
    const Outcome o = run(w, ms, stopAtMarker());
    TEST_ASSERT_EQUAL(static_cast<int>(MissionPhase::Done), static_cast<int>(o.phase));
    TEST_ASSERT_TRUE(ms.explore.blocked > 0);
    TEST_ASSERT_TRUE(ms.explore.resumed > 0);
    TEST_ASSERT_TRUE(ms.explore.resumes == 0); // reset once the centre was reached
    TEST_ASSERT_TRUE(o.tileOffsetMm < 40.0f);
    TEST_ASSERT_TRUE(mapMatchesTruth(w, ms, 3, 0, 1));
    TEST_ASSERT_TRUE(w.metrics.minNoseMm > model::NOSE_MM - 1.0f);
}

static void test_dropped_readings_and_stray_echoes_cost_time_not_the_finish()
{
    for (int i : {0, 3}) {
        for (int j : {0, 1, 2}) {
            World w;
            world::photoMaze(w);
            w.faults.dropout = 0.30f;
            w.faults.strayNear = 0.05f;
            w.seed(99u + static_cast<uint32_t>(i * 7 + j));
            w.start(i, j, (i + j) & 3);
            MissionManager ms;
            begin(ms);
            const Outcome o = run(w, ms, stopAtMarker());
            TEST_ASSERT_EQUAL(static_cast<int>(MissionPhase::Done), static_cast<int>(o.phase));
            TEST_ASSERT_TRUE(o.tileOffsetMm < 40.0f);
            TEST_ASSERT_TRUE(w.metrics.minNoseMm > model::NOSE_MM - 1.0f);
            TEST_ASSERT_TRUE(mapMatchesTruth(w, ms, i, j, (i + j) & 3));
        }
    }
}

static void test_gyro_scale_and_drift_are_taken_out_by_the_walls()
{
    World w;
    world::photoMaze(w);
    w.faults.gyroScale = 1.02f;
    w.faults.gyroDriftDps = 0.15f;
    w.start(3, 0, 1);
    MissionManager ms;
    begin(ms);
    const Outcome o = run(w, ms, stopAtMarker());
    TEST_ASSERT_EQUAL(static_cast<int>(MissionPhase::Done), static_cast<int>(o.phase));
    TEST_ASSERT_TRUE(w.metrics.minSideMm > model::BODY_HALF_WIDTH_MM - 15.0f);
}

// Wheels that need the measured duty to start and more to keep rolling than the model's
// default: a move that ends short stands still, and only the friction feedforward carries the
// profile's last millimetres.
static void test_every_start_finishes_on_wheels_that_roll_hard()
{
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            if (i == 0 && j == N - 1) {
                continue;
            }
            for (int facing = 0; facing < 4; ++facing) {
                World w;
                world::photoMaze(w);
                w.plant.sticking = 1.0f;
                w.plant.sustain = 130.0;
                w.plant.sustainRight = 1.25;
                w.seed(777u + static_cast<uint32_t>(i * 31 + j * 7 + facing));
                w.start(i, j, facing);
                MissionManager ms;
                begin(ms);
                const Outcome o = run(w, ms, stopAtMarker());
                TEST_ASSERT_EQUAL_MESSAGE(static_cast<int>(MissionPhase::Done),
                                          static_cast<int>(o.phase), "did not finish");
                TEST_ASSERT_TRUE_MESSAGE(w.metrics.minNoseMm > model::NOSE_MM - 1.0f, "hit a wall");
            }
        }
    }
}

// The gyro's scale error turns every pivot short or long; the walls take it out while it stays
// under about 1%. The measured chip reads 0.1 to 0.2% high.
static void test_a_gyro_half_a_percent_off_still_finishes_from_every_start()
{
    for (const float scale : {0.995f, 1.005f}) {
        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < N; ++j) {
                if (i == 0 && j == N - 1) {
                    continue;
                }
                World w;
                world::photoMaze(w);
                w.faults.gyroScale = scale;
                w.seed(31u + static_cast<uint32_t>(i * 5 + j));
                w.start(i, j, (i + 2 * j) & 3);
                MissionManager ms;
                begin(ms);
                const Outcome o = run(w, ms, stopAtMarker());
                TEST_ASSERT_EQUAL_MESSAGE(static_cast<int>(MissionPhase::Done),
                                          static_cast<int>(o.phase), "did not finish");
                TEST_ASSERT_TRUE_MESSAGE(w.metrics.minNoseMm > model::NOSE_MM - 1.0f, "hit a wall");
                TEST_ASSERT_TRUE_MESSAGE(w.metrics.minSideMm > model::BODY_HALF_WIDTH_MM - 15.0f,
                                         "rubbed a wall");
            }
        }
    }
}

// ---- Out of the maze and onto the line --------------------------------------------------------
struct LineOutcome {
    MissionPhase phase = MissionPhase::WaitStart;
    MissionFault fault = MissionFault::None;
    bool reachedLine = false;
    float worstErrMm = 0.0f;      // from the centreline, once the follower has the line
    float worstStraightMm = 0.0f; // ... on the straight, before the bend
    float finalErrMm = 0.0f;
    bool onRampSeen = false;
};

// Run until the follower has had `lineTicks` ticks (or the run ends).
static LineOutcome runToLine(World &w, MissionManager &ms, const MissionParams &p,
                             uint32_t lineTicks)
{
    LineOutcome o;
    uint32_t onLine = 0;
    for (uint32_t t = 0; t < RUN_TICKS && onLine < lineTicks; ++t) {
        const Perception pc = w.sense();
        const DriveCommand cmd = missionStep(ms, p, pc);
        const float fix = missionTakeHeadingFix(ms);
        if (fix != 0.0f) {
            headingSet(w.heading, w.heading.deg - fix);
        }
        w.drive(cmd);
        if (ms.phase == MissionPhase::Fault || ms.phase == MissionPhase::Done) {
            break;
        }
        if (ms.phase == MissionPhase::Line) {
            o.reachedLine = true;
            ++onLine;
            const float e = w.lineErrorMm();
            o.worstErrMm = e > o.worstErrMm ? e : o.worstErrMm;
            if (w.lineErrorMm() < 60.0f &&
                w.arrayAlongMm() < world::BRIDGE_OUT_STRAIGHT_TO_MM - 60.0f) {
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

// The gap before the ramp reads as air on a bench and dark on the real course (915-1010): the
// follower takes over at the first line view past it either way, and only the air gap is seen as a
// gap. It does so with three times the IR noise measured on the arena as well.
static void test_the_marker_leads_over_the_gap_and_onto_the_line()
{
    const struct {
        world::Gap gap;
        float noise; // times the measured IR noise
    } cases[] = {{world::Gap::Air, 1.0f},
                 {world::Gap::Dark, 1.0f},
                 {world::Gap::Air, 3.0f},
                 {world::Gap::Dark, 3.0f}};
    for (const auto &c : cases) {
        for (int facing = 0; facing < 4; ++facing) {
            World w;
            world::photoMaze(w, c.gap);
            w.faults.irNoise = c.noise;
            w.start(2, 1, facing);
            MissionManager ms;
            begin(ms);
            // 8 s of following: the straight, the ramp and the bend to the left after it.
            const LineOutcome o = runToLine(w, ms, missionParams(), 800);
            TEST_ASSERT_TRUE_MESSAGE(o.reachedLine, "never reached the line");
            TEST_ASSERT_EQUAL(static_cast<int>(MissionPhase::Line), static_cast<int>(o.phase));
            TEST_ASSERT_EQUAL(c.gap == world::Gap::Air, ms.marker.gapSeen);
            TEST_ASSERT_TRUE_MESSAGE(o.worstStraightMm < 15.0f, "wandered on the straight");
            TEST_ASSERT_TRUE_MESSAGE(o.worstErrMm < 45.0f, "lost the line at the bend");
            TEST_ASSERT_TRUE(o.onRampSeen);
            TEST_ASSERT_TRUE(w.metrics.minNoseMm > model::NOSE_MM - 1.0f);
        }
    }
}

static void test_every_start_reaches_the_line_and_follows_it_round_the_bend()
{
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            if (i == 0 && j == N - 1) {
                continue;
            }
            World w;
            world::photoMaze(w);
            w.seed(55u + static_cast<uint32_t>(i * 9 + j));
            w.start(i, j, (i + j) & 3);
            MissionManager ms;
            begin(ms);
            const LineOutcome o = runToLine(w, ms, missionParams(), 1000);
            TEST_ASSERT_EQUAL_MESSAGE(static_cast<int>(MissionPhase::Line),
                                      static_cast<int>(o.phase), "not following the line");
            TEST_ASSERT_TRUE_MESSAGE(o.worstStraightMm < 15.0f, "wandered on the straight");
            TEST_ASSERT_TRUE_MESSAGE(o.worstErrMm < 45.0f, "lost the line at the bend");
            TEST_ASSERT_TRUE_MESSAGE(w.metrics.minNoseMm > model::NOSE_MM - 1.0f, "hit a wall");
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
        Arena &a = w.arena;
        for (int j = 0; j < N; ++j) {
            a.side(0, j, 2) = true;
            a.side(N - 1, j, 0) = true;
        }
        for (int i = 0; i < N; ++i) {
            a.side(i, 0, 3) = true;
            a.side(i, N - 1, 1) = i != 3; // the marker tile's north edge is open
            a.side(i, 3, 3) = i == 3 && exitSide == 1 ? false : true;
        }
        a.build();
        w.floor = world::bridgeOutFloor(3, 3, 1);
        w.start(0, 3, 0); // along the corridor; the tile ahead ends in a wall to the east
        MissionManager ms;
        begin(ms);
        const LineOutcome o = runToLine(w, ms, missionParams(), 500);
        TEST_ASSERT_EQUAL_MESSAGE(static_cast<int>(MissionPhase::Line), static_cast<int>(o.phase),
                                  "did not leave by the side");
        TEST_ASSERT_TRUE(ms.marker.sideExit); // it turned
        TEST_ASSERT_TRUE(o.worstStraightMm < 15.0f);
        TEST_ASSERT_TRUE(w.metrics.minNoseMm > model::NOSE_MM - 1.0f);
    }
}

static void test_a_line_that_never_comes_is_no_line()
{
    World w;
    world::photoMaze(w);
    w.start(0, 2, 1);
    MissionParams p = missionParams();
    p.noLineMm = 90.0f; // the gap is 180 mm out: the follower is never reached
    MissionManager ms;
    begin(ms);
    const LineOutcome o = runToLine(w, ms, p, 100);
    TEST_ASSERT_EQUAL(static_cast<int>(MissionPhase::Fault), static_cast<int>(o.phase));
    TEST_ASSERT_EQUAL(static_cast<int>(MissionFault::NoLine), static_cast<int>(o.fault));
}

static void test_a_line_lost_is_searched_for_and_then_the_motors_are_cut()
{
    World w;
    world::photoMaze(w);
    w.start(0, 2, 1);
    MissionManager ms;
    begin(ms);
    LineOutcome o = runToLine(w, ms, missionParams(), 100);
    TEST_ASSERT_EQUAL(static_cast<int>(MissionPhase::Line), static_cast<int>(o.phase));
    // The line is gone: nothing but white board.
    const MissionParams p = missionParams();
    w.floor.erase(world::Surface::Line);
    for (uint32_t t = 0; t < 1000 && ms.phase == MissionPhase::Line; ++t) {
        const DriveCommand cmd = missionStep(ms, p, w.sense());
        w.drive(cmd);
    }
    TEST_ASSERT_EQUAL(static_cast<int>(MissionPhase::Fault), static_cast<int>(ms.phase));
    TEST_ASSERT_EQUAL(static_cast<int>(MissionFault::LineLost), static_cast<int>(ms.fault));
    DriveCommand last = missionStep(ms, p, w.sense());
    TEST_ASSERT_FALSE(last.enable);
}

// ---- Faults --------------------------------------------------------------------------------
static Outcome runWith(const MissionParams &p, float sticking = 1.0f, int obstacleFrom = 0,
                       int obstacleTo = 0)
{
    World w;
    world::photoMaze(w);
    w.plant.sticking = sticking;
    w.faults.obstacleFromMs = static_cast<uint32_t>(obstacleFrom);
    w.faults.obstacleToMs = static_cast<uint32_t>(obstacleTo);
    w.start(3, 0, 1);
    MissionManager ms;
    begin(ms);
    return run(w, ms, p);
}

static void test_a_step_that_cannot_start_is_a_stuck_step()
{
    MissionParams p = stopAtMarker();
    p.step.plan.accelMmS2 = 0.0f; // the profile never leaves zero
    const Outcome o = runWith(p);
    TEST_ASSERT_EQUAL(static_cast<int>(MissionPhase::Fault), static_cast<int>(o.phase));
    TEST_ASSERT_EQUAL(static_cast<int>(MissionFault::StuckStep), static_cast<int>(o.fault));
}

static void test_a_turn_that_cannot_move_is_retried_then_a_stuck_turn()
{
    MissionParams p = stopAtMarker();
    p.step.plan.turnAccelDps2 = 0.0f;
    // Facing south on (3, 0) the first plan is a turn.
    World w;
    world::photoMaze(w);
    w.start(3, 0, 3);
    MissionManager ms;
    begin(ms);
    const Outcome o = run(w, ms, p);
    TEST_ASSERT_EQUAL(static_cast<int>(MissionPhase::Fault), static_cast<int>(o.phase));
    TEST_ASSERT_EQUAL(static_cast<int>(MissionFault::StuckTurn), static_cast<int>(o.fault));
    TEST_ASSERT_EQUAL_UINT8(p.turnRetries + 1, ms.explore.turnTries);
}

static void test_wheels_that_never_break_away_stall_the_motors()
{
    const Outcome o = runWith(stopAtMarker(), 100.0f);
    TEST_ASSERT_EQUAL(static_cast<int>(MissionPhase::Fault), static_cast<int>(o.phase));
    TEST_ASSERT_EQUAL(static_cast<int>(MissionFault::Stall), static_cast<int>(o.fault));
}

static void test_too_many_cut_short_steps_end_the_run()
{
    MissionParams p = stopAtMarker();
    p.maxBlocked = 0;
    const Outcome o = runWith(p, 1.0f, 3500, 3750);
    TEST_ASSERT_EQUAL(static_cast<int>(MissionPhase::Fault), static_cast<int>(o.phase));
    TEST_ASSERT_EQUAL(static_cast<int>(MissionFault::TooManyStops), static_cast<int>(o.fault));
}

// ---- The state machine ----------------------------------------------------------------------
static void test_nothing_moves_until_the_start_button()
{
    World w;
    world::photoMaze(w);
    w.start(1, 1, 0);
    MissionManager ms;
    missionReset(ms);
    const MissionParams p = stopAtMarker();
    DriveCommand cmd;
    for (int i = 0; i < 50; ++i) {
        cmd = missionStep(ms, p, w.sense());
        w.drive(cmd);
    }
    TEST_ASSERT_EQUAL(static_cast<int>(MissionPhase::WaitStart), static_cast<int>(ms.phase));
    TEST_ASSERT_FALSE(cmd.enable);
    TEST_ASSERT_EQUAL_INT16(0, cmd.left);
}

static void test_a_failure_set_by_the_caller_cuts_the_motors()
{
    MissionManager ms;
    begin(ms);
    missionFail(ms, MissionFault::NoImu);
    World w;
    world::photoMaze(w);
    w.start(1, 1, 0);
    const DriveCommand cmd = missionStep(ms, stopAtMarker(), w.sense());
    TEST_ASSERT_FALSE(cmd.enable);
    TEST_ASSERT_EQUAL(static_cast<int>(MissionFault::NoImu), static_cast<int>(ms.fault));
}

static void test_the_sweep_goes_the_way_of_the_turn()
{
    using cell_explorer_detail::sweepTo;
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
    MissionManager ms;
    ms.explore.h0 = 10.0f;
    using cell_explorer_detail::gridHeading;
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f, gridHeading(ms.explore.h0, maze::Dir::North));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -80.0f, gridHeading(ms.explore.h0, maze::Dir::East));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -170.0f, gridHeading(ms.explore.h0, maze::Dir::South));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, gridHeading(ms.explore.h0, maze::Dir::West));
}

static void test_the_default_parameters_are_the_spec()
{
    const MissionParams p = missionParams();
    TEST_ASSERT_EQUAL_FLOAT(ACCEL_MM_S2, p.step.plan.accelMmS2);
    TEST_ASSERT_EQUAL_FLOAT(KP_TURN, p.control.kpTurn);
    TEST_ASSERT_EQUAL_UINT16(MOTOR_PWM_TOP, p.control.pwmTop);
    TEST_ASSERT_EQUAL_UINT16(SECTION_A_SENSE_MS, p.senseMs);
    TEST_ASSERT_EQUAL_UINT8(SECTION_A_RESCANS, p.rescans);
}

// ---- Dead ends -------------------------------------------------------------------------------
// The photo maze has three: the two closed boxes and the bottom-left tile.

static int wallsAround(const Arena &a, int i, int j)
{
    int k = 0;
    for (int d = 0; d < 4; ++d) {
        k += a.side(i, j, d) ? 1 : 0;
    }
    return k;
}

// A back-out ends this close to the centre of the tile it reverses into. It is one cell from a stop
// placed by the dead end's end wall; a step's own stops land up to ~14 mm from a centre here.
constexpr float BACK_END_TOLERANCE_MM = 15.0f;

struct DeadEndRun {
    Outcome out;
    int halfTurnsInDeadEnds = 0; // half turns begun in a dead end, after leaving the start tile
    float worstBackEndMm = 0.0f; // a back-out's end from the centre of its tile
    bool mapRightAfterEveryBack = true;
};

// A run, watching what the explorer does in dead ends.
static DeadEndRun runWatchingDeadEnds(World &w, MissionManager &ms, const MissionParams &p, int si,
                                      int sj, int facing)
{
    DeadEndRun r;
    ExplorePhase was = ms.explore.phase;
    for (; r.out.ticks < RUN_TICKS; ++r.out.ticks) {
        const DriveCommand cmd = missionStep(ms, p, w.sense());
        const float fix = missionTakeHeadingFix(ms);
        if (fix != 0.0f) {
            headingSet(w.heading, w.heading.deg - fix);
        }
        w.drive(cmd);
        const ExplorePhase now = ms.explore.phase;
        if (now != was) {
            if (was == ExplorePhase::Back) {
                const float dx = static_cast<float>(w.plant.x) - world::PITCH * w.cellI();
                const float dy = static_cast<float>(w.plant.y) - world::PITCH * w.cellJ();
                const float off = sqrtf(dx * dx + dy * dy);
                r.worstBackEndMm = off > r.worstBackEndMm ? off : r.worstBackEndMm;
                r.mapRightAfterEveryBack =
                    r.mapRightAfterEveryBack && mapMatchesTruth(w, ms, si, sj, facing);
            }
            if (now == ExplorePhase::Turn && absf(ms.explore.turn.deltaDeg) > 135.0f &&
                w.metrics.leftStart && wallsAround(w.arena, w.cellI(), w.cellJ()) == 3) {
                ++r.halfTurnsInDeadEnds;
            }
            was = now;
        }
        if (ms.phase == MissionPhase::Done || ms.phase == MissionPhase::Fault) {
            break;
        }
    }
    r.out.phase = ms.phase;
    r.out.fault = ms.fault;
    r.out.visited = visitedCells(ms);
    return r;
}

// Every tile, from the bottom-right corner: the run enters all three dead ends.
static DeadEndRun exploreEveryTile(bool backOut, MissionManager &ms)
{
    World w;
    world::photoMaze(w);
    w.start(3, 0, 1);
    begin(ms);
    MissionParams p = stopAtMarker();
    p.wallsOnly = true;
    p.backOut = backOut;
    return runWatchingDeadEnds(w, ms, p, 3, 0, 1);
}

static void test_a_dead_end_is_reversed_out_of_not_turned_round_in()
{
    MissionManager ms;
    const DeadEndRun r = exploreEveryTile(true, ms);
    TEST_ASSERT_EQUAL(static_cast<int>(MissionPhase::Done), static_cast<int>(r.out.phase));
    TEST_ASSERT_EQUAL(16, r.out.visited);
    TEST_ASSERT_EQUAL_MESSAGE(0, r.halfTurnsInDeadEnds, "turned round in a dead end");
    // The search ends on the last new tile, which may be a dead end: two at least are backed out
    // of.
    TEST_ASSERT_GREATER_OR_EQUAL_UINT16(2, ms.explore.backs);
    TEST_ASSERT_TRUE_MESSAGE(r.mapRightAfterEveryBack, "map position wrong after backing out");
    TEST_ASSERT_TRUE_MESSAGE(r.worstBackEndMm < BACK_END_TOLERANCE_MM,
                             "backed out to off the centre");
}

// Turning round in the dead end costs a half turn, a look and a drive back; reversing costs the
// drive back alone.
static void test_backing_out_is_quicker_than_turning_round()
{
    MissionManager on;
    const DeadEndRun backing = exploreEveryTile(true, on);
    MissionManager off;
    const DeadEndRun turning = exploreEveryTile(false, off);
    TEST_ASSERT_EQUAL(16, turning.out.visited);
    TEST_ASSERT_GREATER_THAN_MESSAGE(0, turning.halfTurnsInDeadEnds,
                                     "the watch saw no half turn with backing out off");
    TEST_ASSERT_EQUAL_UINT16(0, off.explore.backs);
    // At least a second for each dead end backed out of.
    TEST_ASSERT_LESS_THAN_UINT32(turning.out.ticks - on.explore.backs * 100u, backing.out.ticks);
}

static void test_no_start_turns_round_in_a_dead_end_it_drove_into()
{
    int backs = 0;
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            if (i == 0 && j == N - 1) {
                continue; // the white tile itself
            }
            for (int facing = 0; facing < 4; ++facing) {
                World w;
                world::photoMaze(w);
                w.start(i, j, facing);
                MissionManager ms;
                begin(ms);
                const DeadEndRun r = runWatchingDeadEnds(w, ms, stopAtMarker(), i, j, facing);
                TEST_ASSERT_EQUAL_MESSAGE(static_cast<int>(MissionPhase::Done),
                                          static_cast<int>(r.out.phase), "did not finish");
                TEST_ASSERT_EQUAL_MESSAGE(0, r.halfTurnsInDeadEnds, "turned round in a dead end");
                TEST_ASSERT_TRUE_MESSAGE(r.mapRightAfterEveryBack,
                                         "map position wrong after backing out");
                TEST_ASSERT_TRUE_MESSAGE(r.worstBackEndMm < BACK_END_TOLERANCE_MM,
                                         "backed out to off the centre");
                backs += ms.explore.backs;
            }
        }
    }
    TEST_ASSERT_GREATER_THAN_MESSAGE(0, backs, "no start went into a dead end");
}

// A run is a stretch of the parts (Parts.h): Section A alone, the line alone with the robot placed
// on it, or a selection that cannot run, which is refused before the robot moves.
static void test_a_run_can_be_any_stretch_of_the_parts()
{
    struct PartCase {
        Part first;
        Part last;
        bool onLine; // start on the line board instead of in the maze
        bool wallsOnly;
        MissionPhase endPhase;
        MissionFault fault;
    };
    const PartCase cases[] = {
        // Section A alone: the run ends where the line is in view past the gap.
        {Part::AExplore, Part::AExit, false, false, MissionPhase::Done, MissionFault::None},
        // The line alone, the robot standing on it: still following it.
        {Part::Line, Part::Line, true, false, MissionPhase::Line, MissionFault::None},
        // The way out of the marker tile needs the map the exploring built.
        {Part::AExit, Part::Line, false, false, MissionPhase::Fault, MissionFault::BadConfig},
        // Not built: Section A's second look needs the exploring to go on after the marker.
        {Part::AProve, Part::Line, false, false, MissionPhase::Fault, MissionFault::BadConfig},
        // The last part is before the first.
        {Part::Line, Part::AExplore, false, false, MissionPhase::Fault, MissionFault::BadConfig},
        // No line calibration: nothing to follow.
        {Part::Line, Part::Line, false, true, MissionPhase::Fault, MissionFault::BadConfig},
    };
    for (const PartCase &c : cases) {
        World w;
        world::photoMaze(w);
        if (c.onLine) {
            // 214 mm out of the marker tile along the exit: the array is on the line, past the gap.
            w.start(w.floor.originI(), w.floor.originJ(), w.floor.frame.dir, 0.0f, 214.0f, 0.0f);
        } else {
            w.start(2, 1, 0);
        }
        MissionManager ms;
        begin(ms);
        MissionParams p = missionParams();
        p.parts.first = c.first;
        p.parts.last = c.last;
        p.wallsOnly = c.wallsOnly;
        bool sawLine = false;
        const uint32_t maxTicks = c.onLine ? 300 : RUN_TICKS;
        for (uint32_t t = 0; t < maxTicks; ++t) {
            const DriveCommand cmd = missionStep(ms, p, w.sense());
            const float fix = missionTakeHeadingFix(ms);
            if (fix != 0.0f) {
                headingSet(w.heading, w.heading.deg - fix);
            }
            w.drive(cmd);
            sawLine = sawLine || ms.phase == MissionPhase::Line;
            if (ms.phase == MissionPhase::Done || ms.phase == MissionPhase::Fault) {
                break;
            }
        }
        TEST_ASSERT_EQUAL_MESSAGE(static_cast<int>(c.endPhase), static_cast<int>(ms.phase),
                                  "the run ended in the wrong phase");
        TEST_ASSERT_EQUAL_MESSAGE(static_cast<int>(c.fault), static_cast<int>(ms.fault),
                                  "wrong fault");
        if (c.last == Part::AExit && c.first == Part::AExplore) {
            TEST_ASSERT_FALSE_MESSAGE(sawLine, "the line part ran in a run that ends before it");
        }
        if (c.onLine) {
            TEST_ASSERT_TRUE_MESSAGE(w.lineErrorMm() < 45.0f, "lost the line");
        }
    }
}

void tearDown(void)
{
    seeds::report();
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_every_start_tile_and_heading_reaches_the_marker);
    RUN_TEST(test_a_walls_only_run_explores_every_tile_and_stops);
    RUN_TEST(test_without_a_marker_the_run_ends_in_no_route);
    RUN_TEST(test_a_straight_corridor_to_the_exit_is_driven_down_from_every_start);
    RUN_TEST(test_after_exploring_it_drives_to_the_marker_tile_through_known_cells);
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
    RUN_TEST(test_a_run_can_be_any_stretch_of_the_parts);
    RUN_TEST(test_a_dead_end_is_reversed_out_of_not_turned_round_in);
    RUN_TEST(test_backing_out_is_quicker_than_turning_round);
    RUN_TEST(test_no_start_turns_round_in_a_dead_end_it_drove_into);
    return UNITY_END();
}
