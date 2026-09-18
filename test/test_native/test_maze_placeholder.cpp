// ============================================================================
//  test_maze_placeholder.cpp  --  proves the native (host PC) build works.
//
//  Not a real test of maze logic - there isn't any yet. This exists so
//  SW-01's "build compiles for both the board and the host PC" outcome is
//  checked by something other than eyeballing it, and so SIM-01 has a
//  working `pio test -e native` to add real tests into.
// ============================================================================

#include <unity.h>

#include "Maze.h"

static void test_placeholder_builds_and_links() {
    TEST_ASSERT_EQUAL(1, maze::kPlaceholderOk);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_placeholder_builds_and_links);
    return UNITY_END();
}
