// ============================================================================
//  test_world.cpp  --  the host world itself: what it draws and how it shortcuts. The mission tests
//  trust it, so these check it first. No hardware needed.
// ============================================================================

#include <unity.h>

#include "../../support/world/Presets.h"

using namespace world;

// An arena of n x n tiles with about `density` of its inner edges walled.
static Arena randomArena(int n, float density, Lcg &rng)
{
    Arena a;
    a.clear(n);
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

// The brute-force ray is the reference: where beyond() says no wall is near, none is.
static void test_beyond_never_hides_a_wall_a_ray_would_hit()
{
    Lcg rng;
    const float reach = model::NOSE_MM;
    int skipped = 0;
    int looked = 0;
    for (int arena = 0; arena < 4; ++arena) {
        const Arena a = randomArena(MAX_TILES, 0.35f, rng);
        for (int walk = 0; walk < 25; ++walk) {
            Clearance cached;
            float x = rng.u() * (MAX_TILES - 1) * PITCH;
            float y = rng.u() * (MAX_TILES - 1) * PITCH;
            for (int step = 0; step < 300; ++step) {
                x += (rng.u() - 0.5f) * 2.0f;
                y += (rng.u() - 0.5f) * 2.0f;
                if (!cached.beyond(a, x, y, reach)) {
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

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_beyond_never_hides_a_wall_a_ray_would_hit);
    return UNITY_END();
}
