// ============================================================================
//  test_maze_explorer.cpp  --  MazeMap and Explorer against ground-truth 4x4 mazes, started from
//  every cell and every heading. No hardware needed.
//
//  The truth mazes use absolute coordinates (ax, ay) with abs dir 0 = +y. The robot never sees
//  them: it works in its own relative frame, whose "north" is whichever absolute direction it
//  was facing at the start, so each run also checks that nothing depends on the real layout.
// ============================================================================

#include <unity.h>

#include "Explorer.h"

using namespace maze;

constexpr int kSide = 4;

struct Link {
    int8_t ax, ay;
    char to; // 'N' (+y) or 'E' (+x)
};

struct Truth {
    uint8_t open[kSide][kSide] = {}; // bit per abs dir
    int8_t bridgeX = 0, bridgeY = 0;
    int8_t markerX = 0, markerY = 0;

    void link(int8_t ax, int8_t ay, char to)
    {
        const uint8_t d = to == 'N' ? 0 : 3;
        const int8_t bx = static_cast<int8_t>(ax + kDx[d]);
        const int8_t by = static_cast<int8_t>(ay + kDy[d]);
        open[ay][ax] = static_cast<uint8_t>(open[ay][ax] | (1u << d));
        open[by][bx] = static_cast<uint8_t>(open[by][bx] | (1u << ((d + 2) & 3)));
    }

    bool wall(int8_t ax, int8_t ay, uint8_t absDir) const
    {
        return (open[ay][ax] & (1u << absDir)) == 0;
    }
};

// Ring round the border plus four inner links: every cell connected, with loops.
static Truth ringMaze()
{
    Truth t;
    for (int8_t i = 0; i < 3; ++i) {
        t.link(i, 0, 'E');
        t.link(i, 3, 'E');
        t.link(0, i, 'N');
        t.link(3, i, 'N');
    }
    t.link(1, 0, 'N');
    t.link(1, 1, 'N');
    t.link(2, 2, 'N');
    t.link(2, 1, 'N');
    return t;
}

// One long snake: a perfect maze, so one route and plenty of dead ends.
static Truth snakeMaze()
{
    Truth t;
    for (int8_t i = 0; i < 3; ++i) {
        t.link(i, 0, 'E');
        t.link(i, 1, 'E');
        t.link(i, 2, 'E');
        t.link(i, 3, 'E');
    }
    t.link(3, 0, 'N'); // rows 0 and 1 meet at the right edge
    t.link(0, 1, 'N'); // rows 1 and 2 at the left edge
    t.link(3, 2, 'N'); // rows 2 and 3 at the right edge
    return t;
}

// An open 4x4 room: no interior walls at all.
static Truth openRoom()
{
    Truth t;
    for (int8_t y = 0; y < kSide; ++y) {
        for (int8_t x = 0; x < kSide; ++x) {
            if (x < kSide - 1) {
                t.link(x, y, 'E');
            }
            if (y < kSide - 1) {
                t.link(x, y, 'N');
            }
        }
    }
    return t;
}

struct Run {
    Truth truth;
    MazeMap map;
    int8_t ax = 0, ay = 0; // true position
    int8_t rx = kStartX, ry = kStartY;
    Dir facing = Dir::North; // in the robot's frame
    uint8_t f0 = 0;          // absolute direction of the robot's North
    int steps = 0;
    int startReentries = 0;
    int visitedCount = 0;
    bool found = false;
    int8_t startAx = 0, startAy = 0;
    int8_t hand = 1;

    uint8_t absDir(Dir relative) const
    {
        return static_cast<uint8_t>((dirIndex(relative) + f0) & 3);
    }

    void observeHere()
    {
        const bool front = truth.wall(ax, ay, absDir(facing));
        const bool left = truth.wall(ax, ay, absDir(dirRotate(facing, 1)));
        const bool right = truth.wall(ax, ay, absDir(dirRotate(facing, -1)));
        mapObserve(map, rx, ry, facing, front, left, right);
        if (ax == truth.markerX && ay == truth.markerY) {
            mapMarkMarker(map, rx, ry);
        }
    }

    void begin(int8_t sx, int8_t sy, uint8_t startFacingAbs)
    {
        mapReset(map);
        ax = startAx = sx;
        ay = startAy = sy;
        f0 = startFacingAbs;
        rx = kStartX;
        ry = kStartY;
        facing = Dir::North;
        observeHere();
        // The turn-round at the start shows the wall behind.
        const Dir was = facing;
        facing = dirOpposite(facing);
        observeHere();
        facing = was;
    }

    void stepForward()
    {
        const uint8_t a = absDir(facing);
        ax = static_cast<int8_t>(ax + kDx[a]);
        ay = static_cast<int8_t>(ay + kDy[a]);
        rx = static_cast<int8_t>(rx + kDx[dirIndex(facing)]);
        ry = static_cast<int8_t>(ry + kDy[dirIndex(facing)]);
        // The way we came through is open.
        mapSetEdge(map, rx, ry, dirOpposite(facing), false);
        observeHere();
        if (ax == startAx && ay == startAy) {
            ++startReentries;
        }
    }

    // Returns false when the explorer has nothing left to do.
    bool step()
    {
        Move mv = Move::None;
        int8_t cx = 0, cy = 0;
        bool have = false;
        if (map.hasMarker && bridgeCandidate(map, facing, cx, cy)) {
            have = goToMove(map, rx, ry, facing, hand, cx, cy, mv);
        } else {
            have = exploreMove(map, rx, ry, facing, hand, mv);
        }
        if (!have) {
            return false;
        }
        switch (mv) {
            case Move::Left:
                facing = dirRotate(facing, 1);
                break;
            case Move::Right:
                facing = dirRotate(facing, -1);
                break;
            case Move::Back:
                facing = dirOpposite(facing);
                break;
            default:
                break;
        }
        stepForward();
        ++steps;
        return true;
    }

    // Explore until the bridge cell is entered, or the explorer gives up, or `maxSteps`.
    void runToBridge(int maxSteps = 150)
    {
        while (steps < maxSteps) {
            if (ax == truth.bridgeX && ay == truth.bridgeY) {
                found = true;
                return;
            }
            if (!step()) {
                return;
            }
        }
    }

    void runToEnd(int maxSteps = 300)
    {
        while (steps < maxSteps && step()) {
        }
        visitedCount = 0;
        for (const auto &row : map.cell) {
            for (const auto &c : row) {
                visitedCount += c.visited ? 1 : 0;
            }
        }
    }
};

// ---- MazeMap -----------------------------------------------------------------------

static void test_direction_arithmetic()
{
    TEST_ASSERT_EQUAL(1, dirIndex(dirRotate(Dir::North, 1)));  // left of north is west
    TEST_ASSERT_EQUAL(3, dirIndex(dirRotate(Dir::North, -1))); // right of north is east
    TEST_ASSERT_EQUAL(0, dirIndex(dirRotate(Dir::East, 1)));
    TEST_ASSERT_EQUAL(2, dirIndex(dirOpposite(Dir::North)));
    TEST_ASSERT_EQUAL(3, dirIndex(dirOpposite(Dir::West)));
}

static void test_edges_are_stored_on_both_sides()
{
    MazeMap m;
    mapSetEdge(m, 3, 3, Dir::North, true);
    TEST_ASSERT_TRUE(m.cell[3][3].wall & 1);
    TEST_ASSERT_TRUE(m.cell[4][3].wall & (1u << 2)); // the cell above sees it as its south wall
    mapSetEdge(m, 3, 3, Dir::East, false);
    TEST_ASSERT_TRUE(mapOpen(m, 3, 3, Dir::East));
    TEST_ASSERT_TRUE(mapOpen(m, 4, 3, Dir::West));
    TEST_ASSERT_FALSE(mapOpen(m, 3, 3, Dir::North));
    TEST_ASSERT_FALSE(mapOpen(m, 3, 3, Dir::South)); // never observed: not known open
}

static void test_a_wall_can_be_corrected()
{
    MazeMap m;
    mapSetEdge(m, 3, 3, Dir::North, true);
    mapSetEdge(m, 3, 3, Dir::North, false);
    TEST_ASSERT_TRUE(mapOpen(m, 3, 3, Dir::North));
}

static void test_edges_at_the_array_border_do_not_overrun()
{
    MazeMap m;
    mapSetEdge(m, 0, 0, Dir::West, true);
    mapSetEdge(m, 0, 0, Dir::South, true);
    mapSetEdge(m, kMapSize - 1, kMapSize - 1, Dir::North, true);
    mapSetEdge(m, kMapSize - 1, kMapSize - 1, Dir::East, true);
    mapSetEdge(m, -1, 0, Dir::North, true); // outside: ignored
    mapVisit(m, kMapSize, 0);
    TEST_ASSERT_TRUE(m.cell[0][0].wall & (1u << 1));
    TEST_ASSERT_EQUAL(-1, m.maxVx);
}

static void test_observe_maps_left_and_right_by_facing()
{
    MazeMap m;
    // Facing west: ahead is west, left is south, right is north.
    mapObserve(m, 3, 3, Dir::West, true, false, true);
    TEST_ASSERT_FALSE(mapOpen(m, 3, 3, Dir::West));
    TEST_ASSERT_TRUE(mapOpen(m, 3, 3, Dir::South));
    TEST_ASSERT_FALSE(mapOpen(m, 3, 3, Dir::North));
    TEST_ASSERT_TRUE(m.cell[3][3].visited);
}

static void test_bounds_before_and_after_the_extent_is_known()
{
    MazeMap m;
    TEST_ASSERT_TRUE(mapInBounds(m, 0, 0)); // nothing visited yet: anywhere in the array
    TEST_ASSERT_FALSE(mapInBounds(m, -1, 0));
    mapVisit(m, 3, 3);
    TEST_ASSERT_TRUE(mapInBounds(m, 0, 3));  // 3 away: the far edge of a 4-wide section
    TEST_ASSERT_FALSE(mapInBounds(m, 7, 3)); // outside the array
    mapVisit(m, 3, 5);
    mapVisit(m, 3, 6); // y now spans 3..6: pinned
    TEST_ASSERT_TRUE(mapBoundsFixedY(m));
    TEST_ASSERT_FALSE(mapBoundsFixedX(m));
    TEST_ASSERT_FALSE(mapInBounds(m, 3, 2)); // below the pinned extent
}

static void test_reset_clears_the_marker_and_extent()
{
    MazeMap m;
    mapVisit(m, 2, 2);
    mapMarkMarker(m, 2, 2);
    mapReset(m);
    TEST_ASSERT_FALSE(m.hasMarker);
    TEST_ASSERT_EQUAL(-1, m.maxVx);
    TEST_ASSERT_FALSE(m.cell[2][2].visited);
}

// ---- Explorer ----------------------------------------------------------------------

static void test_explores_every_cell_of_an_open_room_without_re_entering_the_start()
{
    Run r;
    r.truth = openRoom();
    r.truth.markerX = -1; // no marker
    r.begin(1, 1, 0);
    r.runToEnd();
    TEST_ASSERT_EQUAL(16, r.visitedCount);
    TEST_ASSERT_EQUAL(0, r.startReentries);
}

static void test_explores_a_perfect_maze_completely()
{
    Run r;
    r.truth = snakeMaze();
    r.truth.markerX = -1;
    r.begin(0, 0, 0);
    r.runToEnd();
    TEST_ASSERT_EQUAL(16, r.visitedCount);
}

static void test_gives_up_when_everything_is_explored()
{
    Run r;
    r.truth = ringMaze();
    r.truth.markerX = -1;
    r.begin(0, 0, 0);
    r.runToEnd();
    Move mv = Move::None;
    TEST_ASSERT_FALSE(exploreMove(r.map, r.rx, r.ry, r.facing, 1, mv));
}

static void sweepAllStartsAndHeadings(const Truth &base, int8_t bridgeX, int8_t bridgeY,
                                      int8_t markerX, int8_t markerY)
{
    int failures = 0;
    for (int8_t sy = 0; sy < kSide; ++sy) {
        for (int8_t sx = 0; sx < kSide; ++sx) {
            if ((sx == bridgeX && sy == bridgeY) || (sx == markerX && sy == markerY)) {
                continue; // the start tile is neither
            }
            for (uint8_t f = 0; f < 4; ++f) {
                for (int8_t hand = -1; hand <= 1; hand += 2) {
                    Run r;
                    r.truth = base;
                    r.truth.bridgeX = bridgeX;
                    r.truth.bridgeY = bridgeY;
                    r.truth.markerX = markerX;
                    r.truth.markerY = markerY;
                    r.hand = hand;
                    r.begin(sx, sy, f);
                    r.runToBridge();
                    failures += r.found ? 0 : 1;
                }
            }
        }
    }
    TEST_ASSERT_EQUAL(0, failures);
}

static void test_finds_the_bridge_in_the_ring_maze_from_everywhere()
{
    sweepAllStartsAndHeadings(ringMaze(), 3, 3, 3, 2);
    sweepAllStartsAndHeadings(ringMaze(), 3, 3, 2, 3);
    sweepAllStartsAndHeadings(ringMaze(), 0, 0, 1, 0);
}

static void test_finds_the_bridge_in_the_snake_maze_from_everywhere()
{
    sweepAllStartsAndHeadings(snakeMaze(), 3, 3, 3, 2);
    sweepAllStartsAndHeadings(snakeMaze(), 0, 3, 1, 3);
    sweepAllStartsAndHeadings(snakeMaze(), 0, 0, 1, 0);
}

static void test_finds_the_bridge_in_the_open_room_from_everywhere()
{
    sweepAllStartsAndHeadings(openRoom(), 0, 3, 0, 2);
    sweepAllStartsAndHeadings(openRoom(), 3, 0, 2, 0);
}

static void test_after_the_marker_it_heads_straight_for_the_bridge_cell()
{
    Run r;
    r.truth = openRoom();
    r.truth.markerX = 2;
    r.truth.markerY = 3;
    r.truth.bridgeX = 3;
    r.truth.bridgeY = 3;
    r.begin(0, 3, 3); // start top-left, facing east: marker is straight ahead
    r.runToBridge();
    TEST_ASSERT_TRUE(r.found);
    TEST_ASSERT_TRUE(r.steps <= 5); // 3 steps along the top row, give or take a first look
}

static void test_candidate_prefers_the_cell_ahead()
{
    MazeMap m;
    mapObserve(m, 3, 3, Dir::North, false, false, false); // all sides open
    mapSetEdge(m, 3, 3, Dir::South, false);
    mapMarkMarker(m, 3, 3);
    int8_t cx = 0, cy = 0;
    TEST_ASSERT_TRUE(bridgeCandidate(m, Dir::North, cx, cy));
    TEST_ASSERT_EQUAL(3, cx);
    TEST_ASSERT_EQUAL(4, cy);
    TEST_ASSERT_TRUE(bridgeCandidate(m, Dir::East, cx, cy));
    TEST_ASSERT_EQUAL(4, cx);
    TEST_ASSERT_EQUAL(3, cy);
}

static void test_candidate_skips_walls_and_visited_cells()
{
    MazeMap m;
    mapObserve(m, 3, 3, Dir::North, true, false, true); // wall ahead and right, left open
    mapSetEdge(m, 3, 3, Dir::South, false);
    mapMarkMarker(m, 3, 3);
    mapVisit(m, 2, 3); // west neighbour already entered: not it
    int8_t cx = 0, cy = 0;
    TEST_ASSERT_TRUE(bridgeCandidate(m, Dir::North, cx, cy));
    TEST_ASSERT_EQUAL(3, cx);
    TEST_ASSERT_EQUAL(2, cy); // only the way back south is left
}

static void test_candidate_must_sit_on_the_edge_of_a_pinned_axis()
{
    MazeMap m;
    // Visited column x = 3 over y = 3..6: y is pinned. The marker is at (3, 4); its open
    // neighbours (3, 5) is interior in y, so not a corner of the section.
    for (int8_t y = 3; y <= 6; ++y) {
        mapVisit(m, 3, y);
    }
    mapObserve(m, 3, 4, Dir::North, false, false, true);
    mapSetEdge(m, 3, 4, Dir::South, false);
    mapMarkMarker(m, 3, 4);
    mapSetEdge(m, 3, 4, Dir::West, false);
    mapSetEdge(m, 3, 4, Dir::East, true);
    m.cell[5][3].visited = false; // (3,5) not entered yet
    int8_t cx = 0, cy = 0;
    // Candidates: (3,5) ahead is interior; (2,4) west is interior in y (4 is not 3 or 6).
    TEST_ASSERT_FALSE(bridgeCandidate(m, Dir::North, cx, cy));
}

static void test_no_candidate_without_a_marker()
{
    MazeMap m;
    int8_t cx = 0, cy = 0;
    TEST_ASSERT_FALSE(bridgeCandidate(m, Dir::North, cx, cy));
}

static void test_goto_the_cell_you_are_in_is_not_a_move()
{
    MazeMap m;
    mapObserve(m, 3, 3, Dir::North, false, true, true);
    Move mv = Move::None;
    TEST_ASSERT_FALSE(goToMove(m, 3, 3, Dir::North, 1, 3, 3, mv));
    TEST_ASSERT_FALSE(goToMove(m, 3, 3, Dir::North, 1, -1, 0, mv));
}

static void test_goto_reports_relative_moves()
{
    MazeMap m;
    mapObserve(m, 3, 3, Dir::North, false, false, false);
    mapSetEdge(m, 3, 3, Dir::South, false);
    Move mv = Move::None;
    TEST_ASSERT_TRUE(goToMove(m, 3, 3, Dir::North, 1, 3, 4, mv));
    TEST_ASSERT_EQUAL(static_cast<int>(Move::Forward), static_cast<int>(mv));
    TEST_ASSERT_TRUE(goToMove(m, 3, 3, Dir::North, 1, 2, 3, mv));
    TEST_ASSERT_EQUAL(static_cast<int>(Move::Left), static_cast<int>(mv));
    TEST_ASSERT_TRUE(goToMove(m, 3, 3, Dir::North, 1, 4, 3, mv));
    TEST_ASSERT_EQUAL(static_cast<int>(Move::Right), static_cast<int>(mv));
    TEST_ASSERT_TRUE(goToMove(m, 3, 3, Dir::North, 1, 3, 2, mv));
    TEST_ASSERT_EQUAL(static_cast<int>(Move::Back), static_cast<int>(mv));
}

static void test_goto_cannot_pass_a_wall()
{
    MazeMap m;
    mapObserve(m, 3, 3, Dir::North, true, true, true);
    Move mv = Move::None;
    TEST_ASSERT_FALSE(goToMove(m, 3, 3, Dir::North, 1, 3, 4, mv));
}

static void test_hand_picks_the_side_on_a_tie()
{
    // Target (3,2) is directly behind (3,3) but walled off; the ways round it, west via (2,3),
    // (2,2) and east via (4,3), (4,2), are the same length.
    MazeMap m;
    mapObserve(m, 3, 3, Dir::North, false, false, false);
    mapSetEdge(m, 3, 3, Dir::South, true);
    mapObserve(m, 2, 3, Dir::North, false, true, false);
    mapSetEdge(m, 2, 3, Dir::South, false);
    mapObserve(m, 2, 2, Dir::North, false, true, false);
    mapObserve(m, 4, 3, Dir::North, false, false, true);
    mapSetEdge(m, 4, 3, Dir::South, false);
    mapObserve(m, 4, 2, Dir::North, false, false, true);
    Move mv = Move::None;
    TEST_ASSERT_TRUE(goToMove(m, 3, 3, Dir::North, 1, 3, 2, mv));
    TEST_ASSERT_EQUAL(static_cast<int>(Move::Left), static_cast<int>(mv));
    TEST_ASSERT_TRUE(goToMove(m, 3, 3, Dir::North, -1, 3, 2, mv));
    TEST_ASSERT_EQUAL(static_cast<int>(Move::Right), static_cast<int>(mv));
    // Already facing one of the ways: no turn beats either hand.
    TEST_ASSERT_TRUE(goToMove(m, 3, 3, Dir::West, -1, 3, 2, mv));
    TEST_ASSERT_EQUAL(static_cast<int>(Move::Forward), static_cast<int>(mv));
    TEST_ASSERT_TRUE(goToMove(m, 3, 3, Dir::East, 1, 3, 2, mv));
    TEST_ASSERT_EQUAL(static_cast<int>(Move::Forward), static_cast<int>(mv));
}

static void test_passes_through_the_start_when_there_is_no_other_way()
{
    // A corridor: (3,3) start, (3,4) beyond it, (3,2) the other side. From (3,2) the only way to
    // the unexplored (3,4) is through the start.
    MazeMap m;
    mapObserve(m, 3, 3, Dir::North, false, true, true);
    mapSetEdge(m, 3, 3, Dir::South, false);
    mapObserve(m, 3, 2, Dir::South, true, true, true);
    mapSetEdge(m, 3, 2, Dir::North, false);
    Move mv = Move::None;
    TEST_ASSERT_TRUE(exploreMove(m, 3, 2, Dir::South, 1, mv));
    TEST_ASSERT_EQUAL(static_cast<int>(Move::Back), static_cast<int>(mv)); // north, via the start
}

// ---- The exit, found from the walls ----------------------------------------------------------

static void test_the_exit_is_the_open_side_that_leaves_a_pinned_extent()
{
    MazeMap m;
    // Column x = 3 over y = 3..6 and row y = 6 over x = 3..6 are visited: the 4x4 is pinned. The
    // cell (6, 6) has its north side open, and north of it is outside.
    for (int8_t y = 3; y <= 6; ++y) {
        for (int8_t x = 3; x <= 6; ++x) {
            mapVisit(m, x, y);
        }
    }
    mapObserve(m, 6, 6, Dir::North, false, true, true); // walls left and right, front open
    int8_t ex = 0, ey = 0;
    Dir ed = Dir::South;
    TEST_ASSERT_TRUE(exitCell(m, ex, ey, ed));
    TEST_ASSERT_EQUAL(6, ex);
    TEST_ASSERT_EQUAL(6, ey);
    TEST_ASSERT_EQUAL(static_cast<int>(Dir::North), static_cast<int>(ed));
}

static void test_no_exit_while_the_extent_is_not_pinned()
{
    MazeMap m;
    // Only part of the section has been entered: an open side may lead to a cell still inside it.
    mapVisit(m, 3, 3);
    mapVisit(m, 3, 4);
    mapObserve(m, 3, 4, Dir::North, false, true, true);
    int8_t ex = 0, ey = 0;
    Dir ed = Dir::South;
    TEST_ASSERT_FALSE(exitCell(m, ex, ey, ed));
}

static void test_a_wall_is_not_an_exit()
{
    MazeMap m;
    for (int8_t y = 3; y <= 6; ++y) {
        for (int8_t x = 3; x <= 6; ++x) {
            mapVisit(m, x, y);
        }
    }
    mapObserve(m, 6, 6, Dir::North, true, true, true); // walled all round
    int8_t ex = 0, ey = 0;
    Dir ed = Dir::South;
    TEST_ASSERT_FALSE(exitCell(m, ex, ey, ed));
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_direction_arithmetic);
    RUN_TEST(test_edges_are_stored_on_both_sides);
    RUN_TEST(test_a_wall_can_be_corrected);
    RUN_TEST(test_edges_at_the_array_border_do_not_overrun);
    RUN_TEST(test_observe_maps_left_and_right_by_facing);
    RUN_TEST(test_bounds_before_and_after_the_extent_is_known);
    RUN_TEST(test_reset_clears_the_marker_and_extent);
    RUN_TEST(test_explores_every_cell_of_an_open_room_without_re_entering_the_start);
    RUN_TEST(test_explores_a_perfect_maze_completely);
    RUN_TEST(test_gives_up_when_everything_is_explored);
    RUN_TEST(test_finds_the_bridge_in_the_ring_maze_from_everywhere);
    RUN_TEST(test_finds_the_bridge_in_the_snake_maze_from_everywhere);
    RUN_TEST(test_finds_the_bridge_in_the_open_room_from_everywhere);
    RUN_TEST(test_after_the_marker_it_heads_straight_for_the_bridge_cell);
    RUN_TEST(test_candidate_prefers_the_cell_ahead);
    RUN_TEST(test_candidate_skips_walls_and_visited_cells);
    RUN_TEST(test_candidate_must_sit_on_the_edge_of_a_pinned_axis);
    RUN_TEST(test_no_candidate_without_a_marker);
    RUN_TEST(test_goto_the_cell_you_are_in_is_not_a_move);
    RUN_TEST(test_goto_reports_relative_moves);
    RUN_TEST(test_goto_cannot_pass_a_wall);
    RUN_TEST(test_hand_picks_the_side_on_a_tie);
    RUN_TEST(test_passes_through_the_start_when_there_is_no_other_way);
    RUN_TEST(test_the_exit_is_the_open_side_that_leaves_a_pinned_extent);
    RUN_TEST(test_no_exit_while_the_extent_is_not_pinned);
    RUN_TEST(test_a_wall_is_not_an_exit);
    return UNITY_END();
}
