// ============================================================================
//  test_maze.cpp  --  exploration logic on the host, and a simulator that runs it in the
//  Section A maze of the team's photo from every start.
//
//  Ground truth, 4x4, row 0 at the top, '|' and '-' are interior walls:
//
//        c0  c1  c2  c3
//      +---+---+---+---+
//  r0  | F | . | . | . |      F = the white rectangle
//      +   +---+---+   +      (the outer wall is not drawn)
//  r1  | . . | . . . . |
//      +---+   +---+   +
//  r2  | . | . | . | . |
//      +   +   +   +   +
//  r3  | . | . . . . . |
//
//  Read from the photo of the arena; the edges are in kWallEdges, W1..W7 left to right.
// ============================================================================

#include <initializer_list>
#include <stdint.h>
#include <unity.h>

#include "Maze.h"
#include "Proof.h"
#include "spec/Competition.h"

using namespace maze;

// ---- Ground truth -----------------------------------------------------------------------
constexpr int kN = SECTION_A_SIZE_TILES;
constexpr int kFinishRow = 0;
constexpr int kFinishCol = 0;

// Row and column step of each Dir.
constexpr int kDRow[4] = {-1, 0, 1, 0};
constexpr int kDCol[4] = {0, 1, 0, -1};

struct Edge {
    uint8_t row, col;
    Dir dir; // the wall on this side of (row, col)
};

constexpr Edge kWallEdges[] = {
    {0, 0, Dir::East},  // W1
    {1, 0, Dir::South}, // W2
    {3, 0, Dir::East},  // W3
    {0, 2, Dir::South}, // W4
    {1, 1, Dir::East},  // W5, rows 1 and 2
    {2, 1, Dir::East},  //
    {1, 2, Dir::South}, // W6
    {2, 2, Dir::East},  // W7
};

static bool truthWall(int row, int col, Dir d)
{
    const int r = row + kDRow[static_cast<uint8_t>(d)];
    const int c = col + kDCol[static_cast<uint8_t>(d)];
    if (r < 0 || c < 0 || r >= kN || c >= kN) {
        return true;
    }
    for (const Edge &e : kWallEdges) {
        const bool direct = e.row == row && e.col == col && e.dir == d;
        const bool reverse = e.row == r && e.col == c && e.dir == opposite(d);
        if (direct || reverse) {
            return true;
        }
    }
    return false;
}

// ---- Simulator ---------------------------------------------------------------------------
struct SimResult {
    bool reachedFinish = false;
    bool hitWall = false;
    uint16_t moves = 0;
    uint16_t turns = 0;
    uint16_t startReentries = 0; // times it came back to the start tile before the finish
    bool mapWrong = false;       // a checked edge says something else than the maze
};

struct Sim {
    int row, col;
    Dir heading;
    uint32_t rng;
    uint8_t missPct; // chance that a side sensor hears nothing

    bool miss()
    {
        rng = rng * 1664525u + 1013904223u;
        return ((rng >> 16) % 100) < missPct;
    }
};

// The front is always heard; a side may be silent, and then nothing is recorded for it.
static void observe(Explorer &e, Sim &s)
{
    const bool front = truthWall(s.row, s.col, s.heading);
    const bool leftHeard = !s.miss();
    const bool rightHeard = !s.miss();
    const bool left = truthWall(s.row, s.col, turnLeft(s.heading));
    const bool right = truthWall(s.row, s.col, turnRight(s.heading));
    explorerObserve(e, front, leftHeard && left, rightHeard && right);
    explorerObserveOpen(e, !front, leftHeard && !left, rightHeard && !right);
}

// A checked edge must say what the maze has there, and a wall must be checked. The map's North is
// the start heading, so a map cell is turned by `offset` quarters to reach the maze's.
static bool mapDisagrees(const Explorer &e, int startRow, int startCol, uint8_t offset)
{
    for (uint8_t i = 0; i < kCells; ++i) {
        const int mapRow = i / kSide - kSide / 2;
        const int mapCol = i % kSide - kSide / 2;
        const int row =
            startRow + mapRow * kDRow[(2 + offset) & 3] + mapCol * kDRow[(1 + offset) & 3];
        const int col =
            startCol + mapRow * kDCol[(2 + offset) & 3] + mapCol * kDCol[(1 + offset) & 3];
        const bool inside = row >= 0 && col >= 0 && row < kN && col < kN;
        for (uint8_t d = 0; d < 4; ++d) {
            const Dir dir = static_cast<Dir>(d);
            const bool wall = (e.cell[i] & wallBit(dir)) != 0;
            const bool checked = (e.cell[i] & checkedBit(dir)) != 0;
            if (wall && !checked) {
                return true;
            }
            if (checked && inside &&
                wall != truthWall(row, col, static_cast<Dir>((d + offset) & 3))) {
                return true;
            }
        }
    }
    return false;
}

static SimResult run(int startRow, int startCol, Dir startHeading, uint8_t missPct, uint32_t seed)
{
    Explorer e;
    explorerInit(e);
    Sim s{startRow, startCol, startHeading, seed, missPct};
    SimResult out;
    // The map's North is the start heading; `offset` converts to the truth's.
    const uint8_t offset = static_cast<uint8_t>(startHeading);
    auto toTruth = [offset](Dir mapDir) {
        return static_cast<Dir>((static_cast<uint8_t>(mapDir) + offset) & 3);
    };
    auto finish = [&]() {
        out.mapWrong = mapDisagrees(e, startRow, startCol, offset);
        return out;
    };

    for (uint16_t guard = 0; guard < 2000; ++guard) {
        if (s.row == kFinishRow && s.col == kFinishCol) {
            out.reachedFinish = true;
            return finish();
        }
        s.heading = toTruth(e.heading);
        observe(e, s);
        Dir step;
        if (!explorerPlan(e, step)) {
            return finish();
        }
        if (step != e.heading) {
            explorerFace(e, step);
            s.heading = toTruth(e.heading);
            ++out.turns;
            observe(e, s); // the new front
            continue;
        }
        if (truthWall(s.row, s.col, toTruth(e.heading))) {
            out.hitWall = true;
            return finish();
        }
        if (!explorerAdvance(e)) {
            return finish();
        }
        s.row += kDRow[static_cast<uint8_t>(toTruth(e.heading))];
        s.col += kDCol[static_cast<uint8_t>(toTruth(e.heading))];
        ++out.moves;
        if (s.row == startRow && s.col == startCol) {
            ++out.startReentries;
        }
    }
    return finish();
}

// ---- Unit tests ----------------------------------------------------------------------------
static void test_the_spec_is_native_safe()
{
    TEST_ASSERT_EQUAL(250, TILE_PITCH_MM);
}

static void test_turns()
{
    TEST_ASSERT_EQUAL(static_cast<int>(Dir::East), static_cast<int>(turnRight(Dir::North)));
    TEST_ASSERT_EQUAL(static_cast<int>(Dir::West), static_cast<int>(turnLeft(Dir::North)));
    TEST_ASSERT_EQUAL(static_cast<int>(Dir::North), static_cast<int>(turnRight(Dir::West)));
    TEST_ASSERT_EQUAL(static_cast<int>(Dir::South), static_cast<int>(opposite(Dir::North)));
}

// A wall and an edge heard open are both checked, on both cells that share the edge; only the
// wall is a wall.
static void test_an_edge_looked_at_is_recorded_on_both_sides_of_the_edge()
{
    for (const bool wall : {true, false}) {
        Explorer e;
        explorerInit(e);
        explorerObserve(e, wall, false, false); // the front to the North; the sides not heard
        explorerObserveOpen(e, !wall, false, false);
        uint8_t north = 0;
        TEST_ASSERT_TRUE(Explorer::neighbour(e.pos, Dir::North, north));
        TEST_ASSERT_BITS_HIGH(checkedBit(Dir::North), e.cell[e.pos]);
        TEST_ASSERT_BITS_HIGH(checkedBit(Dir::South), e.cell[north]);
        TEST_ASSERT_EQUAL(wall, (e.cell[e.pos] & wallBit(Dir::North)) != 0);
        TEST_ASSERT_EQUAL(wall, (e.cell[north] & wallBit(Dir::South)) != 0);
        TEST_ASSERT_BITS_LOW(checkedBit(Dir::East) | checkedBit(Dir::West), e.cell[e.pos]);
        TEST_ASSERT_TRUE(explorerVisited(e, e.pos));
        TEST_ASSERT_FALSE(explorerVisited(e, north));
    }
}

static void test_side_walls_follow_the_heading()
{
    Explorer e;
    explorerInit(e);
    explorerFace(e, Dir::East);
    explorerObserve(e, false, true, false); // left of East is North
    TEST_ASSERT_BITS_HIGH(wallBit(Dir::North), e.cell[e.pos]);
    TEST_ASSERT_BITS_LOW(wallBit(Dir::South), e.cell[e.pos]);
}

static void test_a_wall_is_never_erased()
{
    Explorer e;
    explorerInit(e);
    explorerObserve(e, true, false, false);
    explorerObserve(e, false, false, false);
    TEST_ASSERT_BITS_HIGH(wallBit(Dir::North), e.cell[e.pos]);
}

static void test_plan_goes_straight_when_everything_is_open()
{
    Explorer e;
    explorerInit(e);
    explorerObserve(e, false, false, false);
    Dir step = Dir::South;
    TEST_ASSERT_TRUE(explorerPlan(e, step));
    TEST_ASSERT_EQUAL(static_cast<int>(Dir::North), static_cast<int>(step));
}

static void test_plan_goes_around_a_known_wall()
{
    Explorer e;
    explorerInit(e);
    explorerObserve(e, true, true, false); // North and West closed
    Dir step = Dir::South;
    TEST_ASSERT_TRUE(explorerPlan(e, step));
    TEST_ASSERT_EQUAL(static_cast<int>(Dir::East), static_cast<int>(step)); // right before back
}

static void test_plan_returns_through_visited_cells_to_the_nearest_unvisited()
{
    Explorer e;
    explorerInit(e);
    // A dead end one tile north; the tile beside the start is unexplored.
    explorerObserve(e, false, true, true);
    explorerAdvance(e);
    explorerObserve(e, true, true, true); // dead end one tile north
    Dir step = Dir::North;
    TEST_ASSERT_TRUE(explorerPlan(e, step));
    TEST_ASSERT_EQUAL(static_cast<int>(Dir::South), static_cast<int>(step));
}

static void test_plan_reports_a_sealed_cell()
{
    Explorer e;
    explorerInit(e);
    for (uint8_t i = 0; i < 4; ++i) {
        explorerObserve(e, true, false, false);
        explorerFace(e, turnRight(e.heading));
    }
    Dir step;
    TEST_ASSERT_FALSE(explorerPlan(e, step));
}

static void test_rescan_forgets_the_walls_and_keeps_the_pose()
{
    Explorer e;
    explorerInit(e);
    for (uint8_t i = 0; i < 4; ++i) {
        explorerObserve(e, true, false, false);
        explorerFace(e, turnRight(e.heading));
    }
    Dir step;
    TEST_ASSERT_FALSE(explorerPlan(e, step));
    explorerFace(e, Dir::East);
    explorerRescan(e);
    TEST_ASSERT_EQUAL_UINT8(kStartCell, e.pos);
    TEST_ASSERT_EQUAL(static_cast<int>(Dir::East), static_cast<int>(e.heading));
    TEST_ASSERT_EQUAL_UINT8(0, e.cell[e.pos]); // walls and checked edges are gone
    TEST_ASSERT_FALSE(explorerVisited(e, e.pos));
    explorerObserve(e, false, false, false);
    TEST_ASSERT_TRUE(explorerPlan(e, step));
    TEST_ASSERT_EQUAL(static_cast<int>(Dir::East), static_cast<int>(step));
}

static void test_retreat_undoes_an_advance()
{
    Explorer e;
    explorerInit(e);
    explorerObserve(e, false, false, false);
    explorerFace(e, Dir::East);
    TEST_ASSERT_TRUE(explorerAdvance(e));
    TEST_ASSERT_TRUE(explorerRetreat(e));
    TEST_ASSERT_EQUAL_UINT8(kStartCell, e.pos);
    TEST_ASSERT_EQUAL(static_cast<int>(Dir::East), static_cast<int>(e.heading));
}

static void test_retreat_stops_at_the_map_edge()
{
    Explorer e;
    explorerInit(e);
    e.pos = 0;              // a corner
    e.heading = Dir::South; // back is North, off the map
    TEST_ASSERT_FALSE(explorerRetreat(e));
    TEST_ASSERT_EQUAL_UINT8(0, e.pos);
}

static void test_advance_refuses_a_known_wall()
{
    Explorer e;
    explorerInit(e);
    explorerObserve(e, true, false, false);
    const uint8_t before = e.pos;
    TEST_ASSERT_FALSE(explorerAdvance(e));
    TEST_ASSERT_EQUAL_UINT8(before, e.pos);
}

static void test_plan_keeps_out_of_the_start_tile_when_another_way_exists()
{
    // S is the start, E1 east of it, NE north of E1, N north of S, with a wall between S and N.
    // Unvisited: W (two steps, through S) and NW (three steps, by NE and N). The nearer is through
    // the start; the planner takes the other.
    Explorer e;
    explorerInit(e);
    uint8_t east1 = 0;
    uint8_t northEast = 0;
    uint8_t north = 0;
    TEST_ASSERT_TRUE(Explorer::neighbour(kStartCell, Dir::East, east1));
    TEST_ASSERT_TRUE(Explorer::neighbour(east1, Dir::North, northEast));
    TEST_ASSERT_TRUE(Explorer::neighbour(kStartCell, Dir::North, north));
    for (const uint8_t idx : {kStartCell, east1, northEast, north}) {
        markVisited(e, idx);
    }
    markWall(e, kStartCell, Dir::South);
    markWall(e, kStartCell, Dir::North);
    markWall(e, east1, Dir::East);
    markWall(e, east1, Dir::South);
    markWall(e, northEast, Dir::North);
    markWall(e, northEast, Dir::East);
    markWall(e, north, Dir::North);
    e.pos = east1;
    e.heading = Dir::West;
    Dir step = Dir::West;
    TEST_ASSERT_TRUE(explorerPlan(e, step));
    TEST_ASSERT_EQUAL(static_cast<int>(Dir::North), static_cast<int>(step));
}

static void test_plan_crosses_the_start_tile_when_it_is_the_only_way()
{
    Explorer e;
    explorerInit(e);
    // A corridor with walls east and west of the start; north is a dead end.
    explorerObserve(e, false, true, true);
    explorerAdvance(e);
    explorerObserve(e, true, true, true);
    Dir step = Dir::North;
    TEST_ASSERT_TRUE(explorerPlan(e, step));
    TEST_ASSERT_EQUAL(static_cast<int>(Dir::South), static_cast<int>(step)); // through the start
}

// ---- Section A -------------------------------------------------------------------------------
static void test_truth_maze_is_one_connected_region()
{
    // Every tile must reach the finish, or a start in the cut-off part could never succeed.
    bool reached[kN][kN] = {};
    int stackR[kN * kN];
    int stackC[kN * kN];
    int top = 0;
    stackR[top] = kFinishRow;
    stackC[top++] = kFinishCol;
    reached[kFinishRow][kFinishCol] = true;
    int count = 1;
    while (top > 0) {
        --top;
        const int r = stackR[top];
        const int c = stackC[top];
        for (uint8_t d = 0; d < 4; ++d) {
            const Dir dir = static_cast<Dir>(d);
            if (truthWall(r, c, dir)) {
                continue;
            }
            const int nr = r + (dir == Dir::South) - (dir == Dir::North);
            const int nc = c + (dir == Dir::East) - (dir == Dir::West);
            if (!reached[nr][nc]) {
                reached[nr][nc] = true;
                ++count;
                stackR[top] = nr;
                stackC[top++] = nc;
            }
        }
    }
    TEST_ASSERT_EQUAL(kN * kN, count);
}

static void test_every_start_tile_and_heading_reaches_the_white_tile()
{
    uint16_t worstMoves = 0;
    for (int r = 0; r < kN; ++r) {
        for (int c = 0; c < kN; ++c) {
            for (uint8_t h = 0; h < 4; ++h) {
                const SimResult res = run(r, c, static_cast<Dir>(h), 0, 1);
                TEST_ASSERT_TRUE_MESSAGE(res.reachedFinish, "did not reach the finish");
                TEST_ASSERT_FALSE_MESSAGE(res.hitWall, "drove into a wall");
                TEST_ASSERT_FALSE_MESSAGE(res.mapWrong, "a checked edge disagrees with the maze");
                if (res.moves > worstMoves) {
                    worstMoves = res.moves;
                }
            }
        }
    }
    // 31 is the worst measured; 40 leaves room for a planner tweak, not for a loop.
    TEST_ASSERT_LESS_OR_EQUAL_UINT16(40, worstMoves);
}

static void test_missed_side_readings_cost_time_not_the_finish()
{
    for (int r = 0; r < kN; ++r) {
        for (int c = 0; c < kN; ++c) {
            for (uint8_t h = 0; h < 4; ++h) {
                for (uint32_t seed = 1; seed <= 5; ++seed) {
                    const SimResult res = run(r, c, static_cast<Dir>(h), 40, seed);
                    TEST_ASSERT_TRUE_MESSAGE(res.reachedFinish, "did not reach the finish");
                    TEST_ASSERT_FALSE_MESSAGE(res.hitWall, "drove into a wall");
                    TEST_ASSERT_FALSE_MESSAGE(res.mapWrong,
                                              "a checked edge disagrees with the maze");
                }
            }
        }
    }
}

static void test_the_start_tile_is_re_entered_only_where_it_cannot_be_avoided()
{
    // The start tile is a chokepoint for some starts and is crossed there: 10 of the 60 runs,
    // down from 30 without the avoidance.
    int reentered = 0;
    for (int r = 0; r < kN; ++r) {
        for (int c = 0; c < kN; ++c) {
            for (uint8_t h = 0; h < 4; ++h) {
                if (r == kFinishRow && c == kFinishCol) {
                    continue;
                }
                const SimResult res = run(r, c, static_cast<Dir>(h), 0, 1);
                reentered += res.startReentries > 0 ? 1 : 0;
            }
        }
    }
    TEST_ASSERT_LESS_OR_EQUAL_INT(10, reentered);
}

static void test_starting_on_the_finish_does_not_move()
{
    const SimResult res = run(kFinishRow, kFinishCol, Dir::South, 0, 1);
    TEST_ASSERT_TRUE(res.reachedFinish);
    TEST_ASSERT_EQUAL_UINT16(0, res.moves);
}

static void test_the_cell_ahead_is_visited_once_the_robot_has_been_in_it()
{
    maze::Explorer e;
    maze::explorerInit(e);
    TEST_ASSERT_FALSE(maze::explorerAheadVisited(e));
    maze::explorerObserve(e, false, true, true); // the start cell, a corridor to the north
    TEST_ASSERT_TRUE(maze::explorerAdvance(e));
    maze::explorerObserve(e, false, true, true);
    TEST_ASSERT_FALSE(maze::explorerAheadVisited(e)); // the cell to the north is not
    maze::explorerFace(e, maze::Dir::South);
    TEST_ASSERT_TRUE(maze::explorerAheadVisited(e)); // the start cell behind it is
}

static void test_the_section_b_map_plans_a_path_longer_than_62_tiles()
{
    // A 17 x 17 map (more than 255 cells, so its index is 16 bits) with four wall lines across it,
    // gaps at alternating ends: the way from the south-east corner to the north-west one is a
    // serpentine of 96 tiles. Every cell but the north-west corner is visited.
    using B = ExplorerT<SECTION_B_SIZE_TILES>;
    B e;
    explorerInit(e);
    for (uint16_t i = 1; i < B::kCells; ++i) {
        markVisited(e, static_cast<B::Index>(i));
    }
    const uint8_t gapCol[4] = {0, 16, 0, 16};
    const uint8_t lineRow[4] = {14, 10, 6, 2}; // the wall is south of this row
    for (uint8_t k = 0; k < 4; ++k) {
        for (uint8_t col = 0; col < B::kSide; ++col) {
            if (col != gapCol[k]) {
                markWall(e, static_cast<B::Index>(lineRow[k] * B::kSide + col), Dir::South);
            }
        }
    }
    e.pos = static_cast<B::Index>(B::kCells - 1);
    e.heading = Dir::East;
    uint16_t moves = 0;
    while (e.pos != 0 && moves < 300) {
        Dir step = Dir::North;
        TEST_ASSERT_TRUE(explorerPlan(e, step));
        explorerFace(e, step);
        TEST_ASSERT_TRUE(explorerAdvance(e));
        ++moves;
    }
    TEST_ASSERT_EQUAL_UINT16(96, moves);
}

// ---- The proof ---------------------------------------------------------------------------
// S to G. Known open: S-c, c-d, d-e, e-f and f-G, so 5 steps are known; every other edge is
// unchecked, so 3 (S-a-b-G) are possible. a-b turns out to be a wall.
//   S a b G
//   c d e f
static void test_the_proof_names_the_first_unchecked_edge_until_nothing_can_be_shorter()
{
    Explorer e;
    explorerInit(e);
    const uint8_t s = kStartCell;
    uint8_t a = 0;
    uint8_t c = 0;
    uint8_t d = 0;
    uint8_t x = 0;
    uint8_t f = 0;
    uint8_t g = 0;
    TEST_ASSERT_TRUE(Explorer::neighbour(s, Dir::East, a));
    TEST_ASSERT_TRUE(Explorer::neighbour(s, Dir::South, c));
    TEST_ASSERT_TRUE(Explorer::neighbour(c, Dir::East, d));
    TEST_ASSERT_TRUE(Explorer::neighbour(d, Dir::East, x));
    TEST_ASSERT_TRUE(Explorer::neighbour(x, Dir::East, f));
    TEST_ASSERT_TRUE(Explorer::neighbour(f, Dir::North, g));
    markOpen(e, s, Dir::South);
    markOpen(e, c, Dir::East);
    markOpen(e, d, Dir::East);
    markOpen(e, x, Dir::East);
    markOpen(e, f, Dir::North);

    Proof p = explorerProof(e, s, g);
    TEST_ASSERT_EQUAL(static_cast<int>(ProofStatus::NotYet), static_cast<int>(p.status));
    TEST_ASSERT_EQUAL_UINT16(3, p.length);
    TEST_ASSERT_EQUAL_UINT8(s, p.lookTile);
    TEST_ASSERT_EQUAL(static_cast<int>(Dir::East), static_cast<int>(p.lookDir));

    markOpen(e, s, Dir::East); // S-a looked at: open
    p = explorerProof(e, s, g);
    TEST_ASSERT_EQUAL(static_cast<int>(ProofStatus::NotYet), static_cast<int>(p.status));
    TEST_ASSERT_EQUAL_UINT8(a, p.lookTile); // the unchecked edge nearest the start
    TEST_ASSERT_EQUAL(static_cast<int>(Dir::East), static_cast<int>(p.lookDir));

    markWall(e, a, Dir::East); // a-b is a wall: nothing left can beat 5
    p = explorerProof(e, s, g);
    TEST_ASSERT_EQUAL(static_cast<int>(ProofStatus::Proved), static_cast<int>(p.status));
    TEST_ASSERT_EQUAL_UINT16(5, p.length);

    for (uint8_t k = 0; k < 4; ++k) { // a sealed start: no route even with the rest open
        markWall(e, s, static_cast<Dir>(k));
    }
    TEST_ASSERT_EQUAL(static_cast<int>(ProofStatus::NoRoute),
                      static_cast<int>(explorerProof(e, s, g).status));
}

// Random mazes with loops, on the maps of Section A (4x4 in 7x7) and Section B (9x9 in 17x17): from
// a start tile and a goal, look at whatever edge the proof names until it says "proved". The length
// must be the true shortest, which a search over the maze itself finds.
struct RandomMaze {
    uint8_t n;
    uint8_t wall[81]; // wallBit() of each tile, row-major; the outer edges are walls
};

static uint32_t nextRandom(uint32_t &s)
{
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s << 5;
    return s;
}

// The tile next to `tile` in direction `d`, or -1 outside the maze.
static int mazeNeighbour(const RandomMaze &m, int tile, Dir d)
{
    const int r = tile / m.n + kDRow[static_cast<uint8_t>(d)];
    const int c = tile % m.n + kDCol[static_cast<uint8_t>(d)];
    return (r < 0 || c < 0 || r >= m.n || c >= m.n) ? -1 : r * m.n + c;
}

// Knock out the wall between `tile` and its neighbour `next` in direction `d`.
static void openWall(RandomMaze &m, int tile, Dir d, int next)
{
    m.wall[tile] = static_cast<uint8_t>(m.wall[tile] & ~wallBit(d));
    m.wall[next] = static_cast<uint8_t>(m.wall[next] & ~wallBit(opposite(d)));
}

// A maze with one way between any two tiles (depth-first), then `loops` more walls knocked out.
static RandomMaze randomMaze(uint8_t n, uint32_t seed, uint8_t loops)
{
    RandomMaze m{n, {}};
    for (int i = 0; i < n * n; ++i) {
        m.wall[i] = 0x0F;
    }
    uint32_t rng = seed * 2654435761u + 1u;
    bool seen[81] = {};
    uint8_t stack[81];
    uint8_t top = 0;
    stack[top++] = 0;
    seen[0] = true;
    while (top > 0) {
        const int cur = stack[top - 1];
        Dir options[4];
        uint8_t count = 0;
        for (uint8_t k = 0; k < 4; ++k) {
            const int next = mazeNeighbour(m, cur, static_cast<Dir>(k));
            if (next >= 0 && !seen[next]) {
                options[count++] = static_cast<Dir>(k);
            }
        }
        if (count == 0) {
            --top;
            continue;
        }
        const Dir d = options[nextRandom(rng) % count];
        const int next = mazeNeighbour(m, cur, d);
        openWall(m, cur, d, next);
        seen[next] = true;
        stack[top++] = static_cast<uint8_t>(next);
    }
    for (uint8_t k = 0; k < loops; ++k) {
        const int tile = static_cast<int>(nextRandom(rng) % (n * n));
        const Dir d = static_cast<Dir>(nextRandom(rng) & 3);
        const int next = mazeNeighbour(m, tile, d);
        if (next >= 0) {
            openWall(m, tile, d, next);
        }
    }
    return m;
}

// Steps from `from` to `goal` by a search over the maze itself.
static uint16_t trueDistance(const RandomMaze &m, int from, int goal)
{
    int dist[81];
    int queue[81];
    for (int i = 0; i < m.n * m.n; ++i) {
        dist[i] = -1;
    }
    int head = 0;
    int tail = 0;
    dist[from] = 0;
    queue[tail++] = from;
    while (head < tail) {
        const int cur = queue[head++];
        for (uint8_t k = 0; k < 4; ++k) {
            const Dir d = static_cast<Dir>(k);
            const int next = mazeNeighbour(m, cur, d);
            if ((m.wall[cur] & wallBit(d)) == 0 && dist[next] < 0) {
                dist[next] = dist[cur] + 1;
                queue[tail++] = next;
            }
        }
    }
    return static_cast<uint16_t>(dist[goal]);
}

template <uint8_t N> static void proveRandomMazes(uint32_t firstSeed, uint32_t lastSeed)
{
    using E = ExplorerT<N>;
    for (uint32_t seed = firstSeed; seed <= lastSeed; ++seed) {
        const RandomMaze m = randomMaze(N, seed, N);
        uint32_t rng = seed + 7u;
        const int start = static_cast<int>(nextRandom(rng) % (N * N));
        int goal = static_cast<int>(nextRandom(rng) % (N * N));
        goal = goal == start ? (goal + 1) % (N * N) : goal;
        const uint16_t truth = trueDistance(m, start, goal);
        E e;
        explorerInit(e);
        // Map cell of maze tile (r, c): as many rows and columns from the start cell.
        const int row0 = start / N;
        const int col0 = start % N;
        const typename E::Index goalCell = static_cast<typename E::Index>(
            E::kStart + (goal / N - row0) * E::kSide + (goal % N - col0));
        bool proved = false;
        for (uint16_t looks = 0; looks < 4 * E::kCells && !proved; ++looks) {
            const Proof p = explorerProof(e, E::kStart, goalCell);
            TEST_ASSERT_NOT_EQUAL(static_cast<int>(ProofStatus::NoRoute),
                                  static_cast<int>(p.status));
            TEST_ASSERT_LESS_OR_EQUAL_UINT16_MESSAGE(truth, p.length, "a bound above the truth");
            if (p.status == ProofStatus::Proved) {
                TEST_ASSERT_EQUAL_UINT16_MESSAGE(truth, p.length,
                                                 "proved a path that is not shortest");
                proved = true;
                continue;
            }
            TEST_ASSERT_BITS_LOW(checkedBit(p.lookDir), e.cell[p.lookTile]); // a new edge
            const int r = row0 + p.lookTile / E::kSide - E::kSide / 2;
            const int c = col0 + p.lookTile % E::kSide - E::kSide / 2;
            if ((m.wall[r * N + c] & wallBit(p.lookDir)) != 0) {
                markWall(e, p.lookTile, p.lookDir);
            } else {
                markOpen(e, p.lookTile, p.lookDir);
            }
        }
        TEST_ASSERT_TRUE_MESSAGE(proved, "the proof did not end");
    }
}

static void test_the_proof_finds_the_true_shortest_path_on_random_mazes()
{
    proveRandomMazes<SECTION_A_SIZE_TILES>(1, 20);
    proveRandomMazes<SECTION_B_SIZE_TILES>(1, 3);
}

int main()
{
    UNITY_BEGIN();
    UNITY_BEGIN();
    RUN_TEST(test_the_spec_is_native_safe);
    RUN_TEST(test_turns);
    RUN_TEST(test_an_edge_looked_at_is_recorded_on_both_sides_of_the_edge);
    RUN_TEST(test_side_walls_follow_the_heading);
    RUN_TEST(test_a_wall_is_never_erased);
    RUN_TEST(test_plan_goes_straight_when_everything_is_open);
    RUN_TEST(test_plan_goes_around_a_known_wall);
    RUN_TEST(test_plan_returns_through_visited_cells_to_the_nearest_unvisited);
    RUN_TEST(test_plan_keeps_out_of_the_start_tile_when_another_way_exists);
    RUN_TEST(test_plan_crosses_the_start_tile_when_it_is_the_only_way);
    RUN_TEST(test_plan_reports_a_sealed_cell);
    RUN_TEST(test_rescan_forgets_the_walls_and_keeps_the_pose);
    RUN_TEST(test_retreat_undoes_an_advance);
    RUN_TEST(test_retreat_stops_at_the_map_edge);
    RUN_TEST(test_advance_refuses_a_known_wall);
    RUN_TEST(test_truth_maze_is_one_connected_region);
    RUN_TEST(test_every_start_tile_and_heading_reaches_the_white_tile);
    RUN_TEST(test_missed_side_readings_cost_time_not_the_finish);
    RUN_TEST(test_the_start_tile_is_re_entered_only_where_it_cannot_be_avoided);
    RUN_TEST(test_starting_on_the_finish_does_not_move);
    RUN_TEST(test_the_cell_ahead_is_visited_once_the_robot_has_been_in_it);
    RUN_TEST(test_the_section_b_map_plans_a_path_longer_than_62_tiles);
    RUN_TEST(test_the_proof_names_the_first_unchecked_edge_until_nothing_can_be_shorter);
    RUN_TEST(test_the_proof_finds_the_true_shortest_path_on_random_mazes);
    return UNITY_END();
}
