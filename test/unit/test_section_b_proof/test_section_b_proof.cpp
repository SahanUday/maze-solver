// ============================================================================
//  test_section_b_proof.cpp  --  the logic of Section B's proof loop and stop source (SectionB.h,
//  FinishSource.h) on a map made by hand, with no world: what it decides, tick by tick. The robot's
//  driving is tested in integration/test_section_b.
// ============================================================================

#include <initializer_list>
#include <unity.h>

#include "mission/SectionB.h"

using maze::Dir;

constexpr SectionBMap::Index kStart = SectionBMap::kStart;
constexpr SectionBMap::Index kNorthOfStart =
    static_cast<SectionBMap::Index>(kStart - SectionBMap::kSide);

static SectionBMap::Index north(SectionBMap::Index tile)
{
    return static_cast<SectionBMap::Index>(tile - SectionBMap::kSide);
}

// The tile one step north of the start, to prove a path to.
static SectionB lookingFor(SectionBMap::Index finish)
{
    SectionB sb;
    sb.finishKnown = true;
    sb.finish = finish;
    return sb;
}

static BStatus runUntilSettled(SectionB &sb, CellExplorerB &ce, const CellExplorerParams &p,
                               uint32_t maxTicks = 5000)
{
    const Perception pc; // nothing heard: every wall reading is invalid
    BStatus st = BStatus::Running;
    for (uint32_t t = 0; t < maxTicks && st == BStatus::Running; ++t) {
        MotionTarget target;
        st = sectionBProveStep(sb, ce, p, pc, target);
    }
    return st;
}

static void test_a_path_over_checked_open_edges_is_proved_at_once()
{
    CellExplorerB ce;
    cellExplorerReset(ce);
    maze::markOpen(ce.map, kStart, Dir::North);
    SectionB sb = lookingFor(kNorthOfStart);
    const CellExplorerParams p;
    TEST_ASSERT_EQUAL(static_cast<int>(BStatus::Done),
                      static_cast<int>(runUntilSettled(sb, ce, p)));
    TEST_ASSERT_EQUAL(1, sb.proved);
    TEST_ASSERT_EQUAL(0, sb.looks);
}

static void test_a_finish_walled_off_has_no_route()
{
    CellExplorerB ce;
    cellExplorerReset(ce);
    for (uint8_t d = 0; d < 4; ++d) {
        maze::markWall(ce.map, kStart, static_cast<Dir>(d));
    }
    SectionB sb = lookingFor(kNorthOfStart);
    const CellExplorerParams p;
    TEST_ASSERT_EQUAL(static_cast<int>(BStatus::Fault),
                      static_cast<int>(runUntilSettled(sb, ce, p)));
    TEST_ASSERT_EQUAL(static_cast<int>(MissionFault::NoRoute), static_cast<int>(sb.fault));
}

static void test_without_a_finish_there_is_nothing_to_prove()
{
    CellExplorerB ce;
    cellExplorerReset(ce);
    SectionB sb;
    const CellExplorerParams p;
    TEST_ASSERT_EQUAL(static_cast<int>(BStatus::Done),
                      static_cast<int>(runUntilSettled(sb, ce, p)));
    TEST_ASSERT_EQUAL(0, sb.looks);
}

// The edge that could shorten the path is looked at, and nothing answers: after `frontTries` looks
// it is taken as a wall (as the explorer takes a front that stays silent), so the loop ends instead
// of looking for ever. The other three edges of the start tile are walls, so with that one a wall
// the finish has no route.
static void test_an_edge_nothing_answers_for_becomes_a_wall_after_the_tries()
{
    CellExplorerB ce;
    cellExplorerReset(ce);
    const Perception pc;
    cellExplorerStart(ce, pc.motion); // North is the heading, on the start tile
    for (const Dir d : {Dir::East, Dir::South, Dir::West}) {
        maze::markWall(ce.map, kStart, d);
    }
    SectionB sb = lookingFor(kNorthOfStart);
    const CellExplorerParams p;
    TEST_ASSERT_EQUAL(static_cast<int>(BStatus::Fault),
                      static_cast<int>(runUntilSettled(sb, ce, p)));
    TEST_ASSERT_EQUAL(static_cast<int>(MissionFault::NoRoute), static_cast<int>(sb.fault));
    TEST_ASSERT_EQUAL(p.frontTries, sb.looks);
    TEST_ASSERT_TRUE((ce.map.cell[kStart] & maze::wallBit(Dir::North)) != 0);
    TEST_ASSERT_TRUE((ce.map.cell[kStart] & maze::checkedBit(Dir::North)) != 0);
}

static bool neverTold(const void *, const Perception &)
{
    return false;
}

// Entering closes the edge behind the robot, and puts it back after the explorer's rescan forgot
// every wall.
static void test_the_edge_the_robot_came_in_by_is_closed_and_closed_again_after_a_rescan()
{
    CellExplorerB ce;
    cellExplorerReset(ce);
    SectionB sb;
    const Perception pc;
    sectionBEnter(sb, ce, pc.motion, true);
    TEST_ASSERT_TRUE((ce.map.cell[kStart] & maze::wallBit(Dir::South)) != 0);
    TEST_ASSERT_TRUE((ce.map.cell[kStart] & maze::checkedBit(Dir::South)) != 0);

    // Walled in on the other sides too: nothing to explore, no finish told, so the explorer forgets
    // the walls and tries again. The way in is a wall still; the others are forgotten.
    for (const Dir d : {Dir::North, Dir::East, Dir::West}) {
        maze::markWall(ce.map, kStart, d);
    }
    const FinishSource never = {neverTold, nullptr};
    const CellExplorerParams p;
    for (uint32_t t = 0; t < 2000 && ce.rescans == 0; ++t) {
        MotionTarget target;
        sectionBExploreStep(sb, ce, p, never, pc, target);
    }
    TEST_ASSERT_EQUAL(1, ce.rescans);
    TEST_ASSERT_TRUE((ce.map.cell[kStart] & maze::wallBit(Dir::South)) != 0);
    TEST_ASSERT_TRUE((ce.map.cell[kStart] & maze::wallBit(Dir::North)) == 0);

    // Placed alone, nothing is closed.
    CellExplorerB alone;
    cellExplorerReset(alone);
    SectionB sa;
    sectionBEnter(sa, alone, pc.motion, false);
    TEST_ASSERT_TRUE((alone.map.cell[kStart] & maze::wallBit(Dir::South)) == 0);
}

static bool toldIfCtxSet(const void *ctx, const Perception &)
{
    return ctx != nullptr;
}

static void test_a_finish_source_is_asked_with_its_context_and_none_is_never_told()
{
    const Perception pc;
    FinishSource none;
    TEST_ASSERT_FALSE(finishPlugged(none));
    TEST_ASSERT_FALSE(finishTold(none, pc));

    int flag = 1;
    FinishSource some = {toldIfCtxSet, &flag};
    TEST_ASSERT_TRUE(finishPlugged(some));
    TEST_ASSERT_TRUE(finishTold(some, pc));
    some.ctx = nullptr;
    TEST_ASSERT_FALSE(finishTold(some, pc));
}

// The edge to look at is between the entry tile and the tile north of it, and the robot is on the
// far side with no checked way round: it looks from its own side.
static void test_an_edge_out_of_reach_from_one_side_is_looked_at_from_the_other()
{
    CellExplorerB ce;
    cellExplorerReset(ce);
    const Perception pc;
    cellExplorerStart(ce, pc.motion);
    const SectionBMap::Index two =
        static_cast<SectionBMap::Index>(kNorthOfStart - SectionBMap::kSide);
    maze::markOpen(ce.map, kNorthOfStart, Dir::North);
    ce.map.pos = two;
    SectionB sb = lookingFor(two);
    const CellExplorerParams p;
    const BStatus st = runUntilSettled(sb, ce, p, 200);
    TEST_ASSERT_NOT_EQUAL(static_cast<int>(MissionFault::TargetUnreachable),
                          static_cast<int>(sb.fault));
    TEST_ASSERT_NOT_EQUAL(static_cast<int>(BStatus::Fault), static_cast<int>(st));
    TEST_ASSERT_EQUAL(kNorthOfStart, sb.lookTile);
}

// No checked way to either side of the edge from where the robot is: the proof ends, unproved.
static void test_an_edge_out_of_reach_from_both_sides_ends_the_proof_unproved()
{
    CellExplorerB ce;
    cellExplorerReset(ce);
    const Perception pc;
    cellExplorerStart(ce, pc.motion);
    ce.map.pos = north(north(north(kStart))); // nothing checked between here and the entry tile
    SectionB sb = lookingFor(kNorthOfStart);
    const CellExplorerParams p;
    TEST_ASSERT_EQUAL(static_cast<int>(BStatus::Done),
                      static_cast<int>(runUntilSettled(sb, ce, p, 200)));
    TEST_ASSERT_EQUAL(0, sb.proved);
    TEST_ASSERT_EQUAL(0, sb.looks);
}

// A wall seen on the way cuts the route to the edge: the proof chooses again, with no fault.
static void test_a_wall_found_on_the_way_makes_the_proof_choose_again()
{
    CellExplorerB ce;
    cellExplorerReset(ce);
    Perception pc;
    cellExplorerStart(ce, pc.motion);
    maze::markOpen(ce.map, kStart, Dir::North);
    maze::markOpen(ce.map, kNorthOfStart, Dir::North);
    SectionB sb = lookingFor(north(north(kNorthOfStart)));
    const CellExplorerParams p;
    pc.walls.frontValid = true; // the way marked open turns out to have a wall
    pc.walls.front = true;
    for (uint32_t t = 0; t < 100; ++t) {
        MotionTarget target;
        TEST_ASSERT_NOT_EQUAL(static_cast<int>(BStatus::Fault),
                              static_cast<int>(sectionBProveStep(sb, ce, p, pc, target)));
    }
    TEST_ASSERT_TRUE((ce.map.cell[kStart] & maze::wallBit(Dir::North)) != 0);
    TEST_ASSERT_EQUAL(kStart, sb.lookTile);
}

// Looking from the other side, a silent edge is the one taken as a wall.
static void test_a_silent_edge_seen_from_the_other_side_becomes_that_wall()
{
    CellExplorerB ce;
    cellExplorerReset(ce);
    const Perception pc;
    cellExplorerStart(ce, pc.motion);
    ce.map.pos = kNorthOfStart;
    ce.map.heading = Dir::South; // facing the edge to the entry tile
    SectionB sb = lookingFor(kNorthOfStart);
    const CellExplorerParams p;
    runUntilSettled(sb, ce, p, 2000);
    TEST_ASSERT_TRUE((ce.map.cell[kStart] & maze::wallBit(Dir::North)) != 0);
    TEST_ASSERT_TRUE((ce.map.cell[kNorthOfStart] & maze::wallBit(Dir::North)) == 0);
    TEST_ASSERT_EQUAL(p.frontTries, sb.looks);
}

// On the tile to look from, the robot turns to the edge only if the arrival did not check it.
static void test_the_proof_turns_only_to_an_edge_not_yet_checked()
{
    for (const bool heard : {false, true}) {
        CellExplorerB ce;
        cellExplorerReset(ce);
        Perception pc;
        cellExplorerStart(ce, pc.motion);
        ce.map.heading = Dir::East; // the edge to look at, North, is on the left
        for (const Dir d : {Dir::East, Dir::South, Dir::West}) {
            maze::markWall(ce.map, kStart, d);
        }
        pc.walls.leftValid = heard; // heard open: checked without turning
        SectionB sb = lookingFor(kNorthOfStart);
        const CellExplorerParams p;
        for (uint32_t t = 0; t < 40; ++t) {
            MotionTarget target;
            sectionBProveStep(sb, ce, p, pc, target);
        }
        TEST_ASSERT_EQUAL(heard ? 0 : 1, ce.phase == ExplorePhase::Turn ? 1 : 0);
    }
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_a_path_over_checked_open_edges_is_proved_at_once);
    RUN_TEST(test_a_finish_walled_off_has_no_route);
    RUN_TEST(test_without_a_finish_there_is_nothing_to_prove);
    RUN_TEST(test_an_edge_nothing_answers_for_becomes_a_wall_after_the_tries);
    RUN_TEST(test_the_edge_the_robot_came_in_by_is_closed_and_closed_again_after_a_rescan);
    RUN_TEST(test_an_edge_out_of_reach_from_one_side_is_looked_at_from_the_other);
    RUN_TEST(test_a_finish_source_is_asked_with_its_context_and_none_is_never_told);
    RUN_TEST(test_an_edge_out_of_reach_from_both_sides_ends_the_proof_unproved);
    RUN_TEST(test_a_wall_found_on_the_way_makes_the_proof_choose_again);
    RUN_TEST(test_a_silent_edge_seen_from_the_other_side_becomes_that_wall);
    RUN_TEST(test_the_proof_turns_only_to_an_edge_not_yet_checked);
    return UNITY_END();
}
