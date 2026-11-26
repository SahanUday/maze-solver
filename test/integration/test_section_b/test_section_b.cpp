// ============================================================================
//  test_section_b.cpp  --  Section B's mission (BEntry, BExplore, BProve) in the host world's
//  Section B preset: a 9x9 maze from the seeded generator, the entry corner tile with marker B on
//  it, and a white finish tile the robot does not know about. The stop source plugged in reads that
//  tile; the real detector is #69. The entry is put at every corner and either wall opening.
// ============================================================================

#include <stdio.h>
#include <initializer_list>
#include <unity.h>

#include "../../support/MapIndex.h"
#include "../../support/MazeGen.h"
#include "../../support/Seed.h"
#include "../../support/world/Presets.h"
#include "mission/MissionManager.h"
#include "spec/fixed/Hardware.h"
#include "spec/fixed/Rules.h"
#include "spec/tuning/Run.h"

using world::Entry;
using world::World;

constexpr int N = SECTION_B_SIZE_TILES;
constexpr uint32_t RUN_TICKS = RUN_LIMIT_MS / CONTROL_LOOP_PERIOD_MS;

void tearDown(void)
{
    seeds::report();
}

// The stop source of these tests: the robot's centre is over the world's finish tile.
static bool onFinishTile(const void *ctx, const Perception &)
{
    const World &w = *static_cast<const World *>(ctx);
    return w.floor.surfaceAt(static_cast<float>(w.plant.x), static_cast<float>(w.plant.y)) ==
           world::Surface::Finish;
}

static MissionParams paramsFor(const World &w, Part first, Part last)
{
    MissionParams p = missionParams();
    p.parts.first = first;
    p.parts.last = last;
    p.finish.told = onFinishTile;
    p.finish.ctx = &w;
    return p;
}

static void begin(MissionManager &ms)
{
    missionReset(ms);
    ms.start.started = true;
}

struct Outcome {
    MissionPhase phase = MissionPhase::WaitStart;
    MissionFault fault = MissionFault::None;
    uint32_t ticks = 0;
    bool leftArena = false; // the robot's centre went outside the maze, onto the board
};

// The robot's centre outside the maze's tiles: out through the bridge's opening.
static bool outsideTheMaze(const World &w, int size)
{
    const double lo = -world::PITCH / 2 - 20.0;
    const double hi = world::PITCH * (size - 1) + world::PITCH / 2 + 20.0;
    return w.plant.x < lo || w.plant.x > hi || w.plant.y < lo || w.plant.y > hi;
}

static Outcome run(World &w, MissionManager &ms, const MissionParams &p,
                   uint32_t maxTicks = RUN_TICKS, int size = N)
{
    Outcome out;
    for (; out.ticks < maxTicks; ++out.ticks) {
        const DriveCommand cmd = missionStep(ms, p, w.sense());
        const float fix = missionTakeHeadingFix(ms);
        if (fix != 0.0f) {
            headingSet(w.heading, w.heading.deg - fix);
        }
        w.drive(cmd);
        out.leftArena = out.leftArena || outsideTheMaze(w, size);
        if (ms.phase == MissionPhase::Done || ms.phase == MissionPhase::Fault) {
            break;
        }
    }
    out.phase = ms.phase;
    out.fault = ms.fault;
    return out;
}

// The white tile of the preset, in arena tile numbers.
static bool findFinish(const World &w, int &fi, int &fj, int size = N)
{
    for (int j = 0; j < size; ++j) {
        for (int i = 0; i < size; ++i) {
            if (w.floor.surfaceAt(world::PITCH * i, world::PITCH * j) == world::Surface::Finish) {
                fi = i;
                fj = j;
                return true;
            }
        }
    }
    return false;
}

static float offsetFromTileMm(const World &w, int i, int j)
{
    const float dx = static_cast<float>(w.plant.x) - world::PITCH * i;
    const float dy = static_cast<float>(w.plant.y) - world::PITCH * j;
    return sqrtf(dx * dx + dy * dy);
}

static void test_it_explores_from_every_entry_to_the_finish_and_stops_on_it()
{
    for (const uint32_t seed : {1u, 2u}) {
        for (int corner = 0; corner < 4; ++corner) {
            for (int opening = 0; opening < 2; ++opening) {
                World w;
                world::bridgeIn(w, Entry{N, corner, opening, seed});
                world::startOnEntryTile(w);
                const int ei = w.floor.originI();
                const int ej = w.floor.originJ();
                const int facing = (w.floor.frame.dir + 2) & 3;
                int fi = 0;
                int fj = 0;
                TEST_ASSERT_TRUE(findFinish(w, fi, fj));

                MissionManager ms;
                begin(ms);
                const MissionParams p = paramsFor(w, Part::BEntry, Part::BExplore);
                const Outcome o = run(w, ms, p);
                char msg[80];
                snprintf(msg, sizeof msg, "corner %d opening %d seed %u", corner, opening,
                         static_cast<unsigned>(seed));
                TEST_ASSERT_EQUAL_MESSAGE(static_cast<int>(MissionFault::None),
                                          static_cast<int>(o.fault), msg);
                TEST_ASSERT_EQUAL_MESSAGE(static_cast<int>(MissionPhase::Done),
                                          static_cast<int>(o.phase), msg);
                TEST_ASSERT_FALSE_MESSAGE(o.leftArena, msg);
                TEST_ASSERT_EQUAL_MESSAGE(fi, w.cellI(), msg);
                TEST_ASSERT_EQUAL_MESSAGE(fj, w.cellJ(), msg);
                TEST_ASSERT_TRUE_MESSAGE(offsetFromTileMm(w, fi, fj) < 40.0f, msg);
                TEST_ASSERT_TRUE_MESSAGE(ms.sectionB.finishKnown, msg);
                TEST_ASSERT_EQUAL_MESSAGE(mapindex::of(SectionBMap::kSide, fi, fj, ei, ej, facing),
                                          ms.sectionB.finish, msg);
                TEST_ASSERT_TRUE_MESSAGE(w.metrics.minNoseMm > model::NOSE_MM - 1.0f, msg);
            }
        }
    }
}

// After the finish the robot looks at the edges that could still shorten the path, and stops with
// the path it holds proved the shortest: the same length as the generator's own shortest path.
static void test_it_proves_the_shortest_path_from_every_entry()
{
    for (const uint32_t seed : {1u, 2u}) {
        for (int corner = 0; corner < 4; ++corner) {
            for (int opening = 0; opening < 2; ++opening) {
                World w;
                world::bridgeIn(w, Entry{N, corner, opening, seed});
                world::startOnEntryTile(w);
                const int ei = w.floor.originI();
                const int ej = w.floor.originJ();
                int fi = 0;
                int fj = 0;
                TEST_ASSERT_TRUE(findFinish(w, fi, fj));
                const mazegen::RandomMaze m =
                    mazegen::randomMaze(static_cast<uint8_t>(N), seed, static_cast<uint8_t>(N));

                MissionManager ms;
                begin(ms);
                const MissionParams p = paramsFor(w, Part::BEntry, Part::BProve);
                const Outcome o =
                    run(w, ms, p, 2 * RUN_TICKS); // a test cap, not a firmware limit (ADR 0022)
                char msg[96];
                snprintf(msg, sizeof msg, "corner %d opening %d seed %u fault %d looks %d", corner,
                         opening, static_cast<unsigned>(seed), static_cast<int>(o.fault),
                         static_cast<int>(ms.sectionB.looks));
                TEST_ASSERT_EQUAL_MESSAGE(static_cast<int>(MissionFault::None),
                                          static_cast<int>(o.fault), msg);
                TEST_ASSERT_EQUAL_MESSAGE(static_cast<int>(MissionPhase::Done),
                                          static_cast<int>(o.phase), msg);
                TEST_ASSERT_FALSE_MESSAGE(o.leftArena, msg);
                TEST_ASSERT_EQUAL_MESSAGE(
                    mazegen::trueDistance(m, (N - 1 - ej) * N + ei, (N - 1 - fj) * N + fi),
                    ms.sectionB.proved, msg);
                TEST_ASSERT_TRUE_MESSAGE(w.metrics.minNoseMm > model::NOSE_MM - 1.0f, msg);
            }
        }
    }
}

// The robot placed on a tile of Section B with no entry before it, facing the finish tile one step
// away: BExplore begins the map where it stands and finds the finish. Without BEntry the map has no
// closed edge, so the explorer cannot tell the bridge's opening from the maze's: from a start that
// leads elsewhere first it can drive out through it before the finish is found. The 9x9 fold (#75)
// is what bounds that; this test does not claim more than the near start.
static void test_it_explores_on_its_own_from_a_tile_facing_the_finish()
{
    for (const uint32_t seed : {1u, 2u, 3u}) {
        World w;
        world::bridgeIn(w, Entry{N, 0, 0, seed});
        int fi = 0;
        int fj = 0;
        TEST_ASSERT_TRUE(findFinish(w, fi, fj));
        // A tile with no wall between it and the finish tile, and the way it faces the finish.
        const mazegen::RandomMaze m =
            mazegen::randomMaze(static_cast<uint8_t>(N), seed, static_cast<uint8_t>(N));
        const int finishTile = (N - 1 - fj) * N + fi;
        int si = -1;
        int sj = -1;
        int facing = 0;
        for (uint8_t d = 0; d < 4 && si < 0; ++d) {
            const maze::Dir dir = static_cast<maze::Dir>(d);
            const int other = mazegen::mazeNeighbour(m, finishTile, dir);
            if (other >= 0 && (m.wall[finishTile] & maze::wallBit(dir)) == 0) {
                si = other % N;
                sj = N - 1 - other / N;
                facing = (world::WORLD_SIDE[d] + 2) & 3; // from there, toward the finish
            }
        }
        TEST_ASSERT_TRUE(si >= 0);
        w.start(si, sj, facing);
        MissionManager ms;
        begin(ms);
        const MissionParams p = paramsFor(w, Part::BExplore, Part::BExplore);
        const Outcome o = run(w, ms, p);
        TEST_ASSERT_EQUAL_MESSAGE(static_cast<int>(MissionPhase::Done), static_cast<int>(o.phase),
                                  "did not finish");
        TEST_ASSERT_FALSE(o.leftArena);
        TEST_ASSERT_EQUAL(fi, w.cellI());
        TEST_ASSERT_EQUAL(fj, w.cellJ());
        TEST_ASSERT_EQUAL(mapindex::of(SectionBMap::kSide, fi, fj, si, sj, facing),
                          ms.sectionB.finish);
    }
}

// A small maze to explore to its end: the whole of it, from the entry tile.
static void bridgeInSmall(World &w, int corner, int opening)
{
    world::bridgeIn(w, Entry{3, corner, opening, 1u});
    world::startOnEntryTile(w);
}

static int tilesVisited(const MissionManager &ms)
{
    int n = 0;
    for (uint16_t i = 0; i < SectionBMap::kCells; ++i) {
        n += maze::explorerVisited(ms.exploreB.map, static_cast<SectionBMap::Index>(i)) ? 1 : 0;
    }
    return n;
}

// A gyro that reads 1% high is taken out by the walls in Section B too.
static void test_a_gyro_scale_error_is_taken_out_by_the_walls_in_section_b()
{
    World w;
    world::bridgeIn(w, Entry{N, 2, 0, 1u});
    world::startOnEntryTile(w);
    w.faults.gyroScale = 1.01f;
    MissionManager ms;
    begin(ms);
    const MissionParams p = paramsFor(w, Part::BEntry, Part::BExplore);
    const Outcome o = run(w, ms, p);
    TEST_ASSERT_EQUAL(static_cast<int>(MissionPhase::Done), static_cast<int>(o.phase));
}

// Nothing plugged in to say "finish": the search ends when every tile has been entered, no finish
// is known, and the part after it has nothing to prove.
static void test_with_no_finish_source_it_explores_everything_and_proves_nothing()
{
    for (int corner = 0; corner < 4; ++corner) {
        World w;
        bridgeInSmall(w, corner, 0);
        MissionManager ms;
        begin(ms);
        MissionParams p = missionParams();
        p.parts.first = Part::BEntry;
        p.parts.last = Part::BProve;
        const Outcome o = run(w, ms, p, RUN_TICKS, 3);
        TEST_ASSERT_EQUAL(static_cast<int>(MissionPhase::Done), static_cast<int>(o.phase));
        TEST_ASSERT_EQUAL(static_cast<int>(MissionFault::None), static_cast<int>(o.fault));
        TEST_ASSERT_EQUAL(9, tilesVisited(ms));
        TEST_ASSERT_FALSE(ms.sectionB.finishKnown);
        TEST_ASSERT_EQUAL(0, ms.sectionB.looks);
        TEST_ASSERT_FALSE(o.leftArena);
    }
}

static bool neverTold(const void *, const Perception &)
{
    return false;
}

// A finish that is never told is a search that ends in no route, after the rescans.
static void test_a_finish_that_is_never_told_ends_in_no_route()
{
    World w;
    bridgeInSmall(w, 1, 1);
    MissionManager ms;
    begin(ms);
    MissionParams p = paramsFor(w, Part::BEntry, Part::BProve);
    p.finish.told = neverTold;
    const Outcome o = run(w, ms, p, RUN_TICKS, 3);
    TEST_ASSERT_EQUAL(static_cast<int>(MissionPhase::Fault), static_cast<int>(o.phase));
    TEST_ASSERT_EQUAL(static_cast<int>(MissionFault::NoRoute), static_cast<int>(o.fault));
    TEST_ASSERT_EQUAL_UINT8(p.rescans, ms.exploreB.rescans);
    TEST_ASSERT_FALSE(o.leftArena);
}

// Facing the corner tile's other outer wall, the first plan is a turn; one that cannot move is
// tried again and then fails.
static void test_a_turn_that_cannot_move_is_a_stuck_turn_in_section_b_too()
{
    World w;
    world::bridgeIn(w, Entry{N, 2, 0, 1u});
    const int wallDir = world::CORNER_OUTER_SIDES[2][1]; // the side that stays a wall
    w.start(w.floor.originI(), w.floor.originJ(), wallDir);
    MissionManager ms;
    begin(ms);
    MissionParams p = paramsFor(w, Part::BExplore, Part::BExplore);
    p.step.plan.turnAccelDps2 = 0.0f;
    const Outcome o = run(w, ms, p);
    TEST_ASSERT_EQUAL(static_cast<int>(MissionPhase::Fault), static_cast<int>(o.phase));
    TEST_ASSERT_EQUAL(static_cast<int>(MissionFault::StuckTurn), static_cast<int>(o.fault));
    TEST_ASSERT_EQUAL_UINT8(p.turnRetries + 1, ms.exploreB.turnTries);
}

// What each Section B part is, for the manager.
static void test_the_section_b_parts_are_built_and_only_the_proof_carries_on_a_map()
{
    for (const Part part : {Part::BEntry, Part::BExplore, Part::BProve}) {
        TEST_ASSERT_TRUE(partBuilt(part));
    }
    TEST_ASSERT_TRUE(partStartsAlone(Part::BEntry));
    TEST_ASSERT_TRUE(partStartsAlone(Part::BExplore));
    TEST_ASSERT_FALSE(partStartsAlone(Part::BProve));
    TEST_ASSERT_FALSE(partUsesMap(Part::BEntry));
    TEST_ASSERT_TRUE(partUsesMap(Part::BExplore));
    TEST_ASSERT_TRUE(partUsesMap(Part::BProve));

    // BProve cannot be the first part: it carries on the map of the part before it.
    World w;
    world::bridgeIn(w, Entry{N, 0, 0, 1u});
    world::startOnEntryTile(w);
    MissionManager ms;
    begin(ms);
    const MissionParams p = paramsFor(w, Part::BProve, Part::BProve);
    w.drive(missionStep(ms, p, w.sense()));
    TEST_ASSERT_EQUAL(static_cast<int>(MissionPhase::Fault), static_cast<int>(ms.phase));
    TEST_ASSERT_EQUAL(static_cast<int>(MissionFault::BadConfig), static_cast<int>(ms.fault));
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_it_explores_from_every_entry_to_the_finish_and_stops_on_it);
    RUN_TEST(test_it_proves_the_shortest_path_from_every_entry);
    RUN_TEST(test_it_explores_on_its_own_from_a_tile_facing_the_finish);
    RUN_TEST(test_a_gyro_scale_error_is_taken_out_by_the_walls_in_section_b);
    RUN_TEST(test_with_no_finish_source_it_explores_everything_and_proves_nothing);
    RUN_TEST(test_a_finish_that_is_never_told_ends_in_no_route);
    RUN_TEST(test_a_turn_that_cannot_move_is_a_stuck_turn_in_section_b_too);
    RUN_TEST(test_the_section_b_parts_are_built_and_only_the_proof_carries_on_a_map);
    return UNITY_END();
}
