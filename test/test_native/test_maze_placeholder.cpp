// ============================================================================
//  test_maze_placeholder.cpp  --  proves the native (host PC) build works.
//  Not a real maze-logic test - there isn't any yet.
// ============================================================================

#include <unity.h>

#include "Maze.h"
#include "RobotSpec.h"

static void test_placeholder_builds_and_links()
{
    TEST_ASSERT_EQUAL(1, maze::kPlaceholderOk);
}

static void test_robotspec_is_native_safe()
{
    TEST_ASSERT_EQUAL(250, TILE_PITCH_MM);
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_placeholder_builds_and_links);
    RUN_TEST(test_robotspec_is_native_safe);
    return UNITY_END();
}
