// ============================================================================
//  test_host_world.cpp  --  the host world itself: what it draws and how it shortcuts. The mission
//  tests trust it, so these check it first. No hardware needed.
// ============================================================================

#include <initializer_list>
#include <unity.h>

#include "../../support/world/Presets.h"
#include "spec/tuning/Imu.h"

using namespace world;

// An arena of n x n tiles with about `density` of its inner edges walled.
static Arena randomArena(int n, float density, Lcg &rng)
{
    Arena a;
    a.reset(n);
    for (int j = 0; j < n; ++j) {
        for (int i = 0; i < n; ++i) {
            a.side(i, j, 0) = rng.u() < density;
            a.side(i, j, 1) = rng.u() < density;
        }
    }
    a.closeOuterWalls();
    a.build();
    return a;
}

// The brute-force ray is the reference: where noWallWithin() says no wall is near, none is.
static void test_the_clearance_shortcut_never_hides_a_wall_a_ray_would_hit()
{
    const float reach = model::NOSE_MM;
    int skipped = 0;
    int looked = 0;
    for (uint32_t master = 1; master <= 4; ++master) {
        seeds::note(master);
        Lcg rng{seeds::derive(master, seeds::MAZE)};
        const Arena a = randomArena(MAX_TILES, 0.35f, rng);
        for (int walk = 0; walk < 25; ++walk) {
            ClearanceCache cached;
            float x = rng.u() * (MAX_TILES - 1) * PITCH;
            float y = rng.u() * (MAX_TILES - 1) * PITCH;
            for (int step = 0; step < 300; ++step) {
                x += (rng.u() - 0.5f) * 2.0f;
                y += (rng.u() - 0.5f) * 2.0f;
                if (!cached.noWallWithin(a, x, y, reach)) {
                    ++looked;
                    continue;
                }
                ++skipped;
                for (int k = 0; k < 6; ++k) {
                    const float angle = rng.u() * 6.2831853f;
                    TEST_ASSERT_TRUE(a.ray(x, y, cosf(angle), sinf(angle)) > reach);
                }
            }
        }
    }
    TEST_ASSERT_TRUE_MESSAGE(skipped > 5000, "the shortcut was hardly used");
    TEST_ASSERT_TRUE_MESSAGE(looked > 500, "no point was ever near a wall");
}

// Tiles reachable from tile (i, j) through the arena's open sides.
static int reachableTiles(const Arena &a, int i, int j)
{
    bool seen[MAX_TILES * MAX_TILES] = {};
    int queue[MAX_TILES * MAX_TILES];
    int head = 0;
    int tail = 0;
    queue[tail++] = j * a.n + i;
    seen[j * a.n + i] = true;
    static const int di[4] = {1, 0, -1, 0};
    static const int dj[4] = {0, 1, 0, -1};
    while (head < tail) {
        const int ci = queue[head] % a.n;
        const int cj = queue[head] / a.n;
        ++head;
        for (int d = 0; d < 4; ++d) {
            const int ni = ci + di[d];
            const int nj = cj + dj[d];
            const bool inside = ni >= 0 && nj >= 0 && ni < a.n && nj < a.n;
            if (inside && !a.side(ci, cj, d) && !seen[nj * a.n + ni]) {
                seen[nj * a.n + ni] = true;
                queue[tail++] = nj * a.n + ni;
            }
        }
    }
    return tail;
}

static Surface surfaceAtFrame(const World &w, float along, float lat)
{
    float x;
    float y;
    w.floor.frame.from(along, lat, x, y);
    return w.floor.surfaceAt(x, y);
}

// From any corner and either opening: the board leads to the entry tile, which is a corner tile
// with one way out, marker B on it and the rest of the maze behind it, and the finish elsewhere.
static void test_the_board_meets_the_entry_tile_from_any_corner_and_opening()
{
    const uint32_t master = 3u;
    seeds::note(master);
    for (const int size : {1, static_cast<int>(SECTION_B_SIZE_TILES)}) {
        for (int corner = 0; corner < 4; ++corner) {
            for (int opening = 0; opening < 2; ++opening) {
                World w;
                bridgeIn(w, {size, corner, opening, seeds::derive(master, seeds::MAZE)});
                const int ei = w.floor.originI();
                const int ej = w.floor.originJ();
                const int out = CORNER_OUTER_SIDES[corner][opening];
                const int other = CORNER_OUTER_SIDES[corner][1 - opening];
                TEST_ASSERT_EQUAL(out, w.floor.frame.dir);
                TEST_ASSERT_FALSE(w.arena.side(ei, ej, out));
                TEST_ASSERT_TRUE(w.arena.side(ei, ej, other));
                TEST_ASSERT_EQUAL(size * size, reachableTiles(w.arena, ei, ej));

                // Marker B's white frame around its black box, then the board with its line.
                TEST_ASSERT_EQUAL(static_cast<int>(Surface::MazeFloor),
                                  static_cast<int>(surfaceAtFrame(w, 0.0f, 0.0f)));
                TEST_ASSERT_EQUAL(
                    static_cast<int>(Surface::Marker),
                    static_cast<int>(surfaceAtFrame(w, MARKER_B_BOX_MM / 2 + 5, 0.0f)));
                for (const Point &p : BRIDGE_IN_LINE) {
                    TEST_ASSERT_EQUAL(static_cast<int>(Surface::Line),
                                      static_cast<int>(surfaceAtFrame(w, p.along, p.lat)));
                }
                TEST_ASSERT_EQUAL(static_cast<int>(Surface::Board),
                                  static_cast<int>(surfaceAtFrame(w, 1000.0f, -300.0f)));
                TEST_ASSERT_EQUAL(
                    static_cast<int>(Surface::Air),
                    static_cast<int>(surfaceAtFrame(w, BRIDGE_IN_CREST_MM - 5, 0.0f)));

                // One finish tile, and not the entry.
                int finishes = 0;
                for (int j = 0; j < size; ++j) {
                    for (int i = 0; i < size; ++i) {
                        const bool white =
                            w.floor.surfaceAt(PITCH * i, PITCH * j) == Surface::Finish;
                        finishes += white ? 1 : 0;
                        TEST_ASSERT_FALSE(white && i == ei && j == ej);
                    }
                }
                TEST_ASSERT_EQUAL(size > 1 ? 1 : 0, finishes);
            }
        }
    }
}

// From every entry: placed on the line and set going, the robot climbs the ramp and its array lifts
// at the crest without leaving the line; placed on the entry tile, it stands on it facing in.
static void test_a_robot_can_be_placed_on_the_line_and_on_the_entry_tile_and_run()
{
    const float speedMmS = 200.0f;
    DriveCommand cmd;
    cmd.enable = true;
    cmd.left =
        static_cast<int16_t>(model::KEEP_ROLLING_DUTY + speedMmS * model::WHEEL_DUTY_PER_MM_S);
    cmd.right = static_cast<int16_t>(
        model::KEEP_ROLLING_DUTY + speedMmS * model::WHEEL_DUTY_PER_MM_S / model::RIGHT_WHEEL_GAIN);
    for (int corner = 0; corner < 4; ++corner) {
        for (int opening = 0; opening < 2; ++opening) {
            World w;
            bridgeIn(w, {1, corner, opening, 1u});
            startOnLine(w);
            const float startAlong = w.arrayAlongMm();
            bool climbed = false;
            bool lifted = false;
            float worstMm = 0.0f;
            for (int t = 0; t < 350; ++t) {
                const Perception pc = w.sense();
                climbed = climbed || pc.inclineCounts > RAMP_INCLINE_COUNTS;
                lifted = lifted || pc.floor == FloorClass::Air;
                worstMm = w.lineErrorMm() > worstMm ? w.lineErrorMm() : worstMm;
                w.drive(cmd);
            }
            TEST_ASSERT_TRUE(startAlong - w.arrayAlongMm() > 500.0f); // 3.5 s toward the tile
            TEST_ASSERT_TRUE(climbed);
            TEST_ASSERT_TRUE(lifted);
            TEST_ASSERT_TRUE(worstMm < 20.0f); // still on the first straight

            startOnEntryTile(w);
            TEST_ASSERT_EQUAL(w.floor.originI(), w.cellI());
            TEST_ASSERT_EQUAL(w.floor.originJ(), w.cellJ());
            TEST_ASSERT_FLOAT_WITHIN(0.01f, 90.0f * ((w.floor.frame.dir + 2) & 3),
                                     static_cast<float>(w.plant.yawDeg));
        }
    }
}

// Each surface, drawn alone, reads its measured level, and the firmware's air rule calls only the
// array in the air "air": not the bare maze floor, the white, the line or the dark gap.
static void test_each_surface_reads_its_level_and_only_air_is_air()
{
    const Surface surfaces[] = {Surface::MazeFloor, Surface::Marker, Surface::Board, Surface::Line,
                                Surface::GapDark,   Surface::Air,    Surface::Finish};
    const int ticks = 400;
    for (const Surface s : surfaces) {
        World w;
        w.floor.addBand(-FAR_MM, FAR_MM, s);
        w.start(1, 1, 0);
        float sum[IR_CHANNEL_COUNT] = {};
        int airTicks = 0;
        for (int t = 0; t < ticks; ++t) {
            airTicks += w.sense().floor == FloorClass::Air ? 1 : 0;
            for (uint8_t ch = 0; ch < IR_CHANNEL_COUNT; ++ch) {
                sum[ch] += w.state.irRaw[ch];
            }
        }
        for (uint8_t ch = 0; ch < IR_CHANNEL_COUNT; ++ch) {
            TEST_ASSERT_FLOAT_WITHIN(2.0f, irLevel(s).on(ch), sum[ch] / ticks);
        }
        TEST_ASSERT_EQUAL(s == Surface::Air ? ticks : 0, airTicks);
    }
}

// One master seed replays a run exactly, another changes it, and the streams it feeds are apart.
static void test_one_master_seed_replays_a_run_and_another_changes_it()
{
    uint32_t sums[3] = {};
    const uint32_t masters[3] = {5u, 5u, 6u};
    for (int k = 0; k < 3; ++k) {
        World w;
        photoMaze(w);
        w.seed(masters[k]);
        w.start(1, 1, 0);
        for (int t = 0; t < 200; ++t) {
            w.sense();
            w.drive(DriveCommand{});
            for (uint8_t ch = 0; ch < IR_CHANNEL_COUNT; ++ch) {
                sums[k] = sums[k] * 31u + w.state.irRaw[ch];
            }
            sums[k] = sums[k] * 31u + w.state.distFrontMm + w.state.distLeftMm +
                      w.state.distRightMm + static_cast<uint32_t>(w.state.headingDeg * 100.0f);
        }
    }
    TEST_ASSERT_EQUAL_UINT32(sums[0], sums[1]);
    TEST_ASSERT_TRUE(sums[0] != sums[2]);
    TEST_ASSERT_TRUE(seeds::derive(5u, seeds::NOISE) != seeds::derive(5u, seeds::IR));
    TEST_ASSERT_TRUE(seeds::derive(5u, seeds::MAZE) != seeds::derive(6u, seeds::MAZE));
}

// A failing run prints the master seed that replays it.
void tearDown(void)
{
    seeds::report();
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_the_clearance_shortcut_never_hides_a_wall_a_ray_would_hit);
    RUN_TEST(test_the_board_meets_the_entry_tile_from_any_corner_and_opening);
    RUN_TEST(test_a_robot_can_be_placed_on_the_line_and_on_the_entry_tile_and_run);
    RUN_TEST(test_each_surface_reads_its_level_and_only_air_is_air);
    RUN_TEST(test_one_master_seed_replays_a_run_and_another_changes_it);
    return UNITY_END();
}
