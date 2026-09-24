// ============================================================================
//  test_maze_placeholder.cpp  --  proves the native (host PC) build works.
//  Not a real maze-logic test - there isn't any yet.
// ============================================================================

#include <unity.h>

#include "Maze.h"

static void test_placeholder_builds_and_links()
{
    TEST_ASSERT_EQUAL(1, maze::kPlaceholderOk);
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_placeholder_builds_and_links);
    return UNITY_END();
}
