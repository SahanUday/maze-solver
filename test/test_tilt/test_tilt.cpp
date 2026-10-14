// ============================================================================
//  test_tilt.cpp  --  tiltDeg() tests. No hardware needed.
// ============================================================================

#include <unity.h>

#include "Tilt.h"

static ImuRaw accel(int16_t x, int16_t y, int16_t z)
{
    ImuRaw r;
    r.accel[0] = x;
    r.accel[1] = y;
    r.accel[2] = z;
    return r;
}

static void test_flat_is_zero()
{
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, tiltDeg(accel(0, 0, 16384)));
}

static void test_thirty_five_degrees_either_way()
{
    // 35 deg ramp: gravity splits into sin/cos components along the board.
    const int16_t along = static_cast<int16_t>(16384 * 0.573576f);
    const int16_t up = static_cast<int16_t>(16384 * 0.819152f);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 35.0f, tiltDeg(accel(along, 0, up)));
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 35.0f, tiltDeg(accel(-along, 0, up)));
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 35.0f, tiltDeg(accel(0, along, up)));
}

static void test_axis_choice_does_not_matter()
{
    TEST_ASSERT_FLOAT_WITHIN(0.01f, tiltDeg(accel(5000, 0, 15000)), tiltDeg(accel(0, 5000, 15000)));
}

static void test_on_its_side_is_ninety()
{
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 90.0f, tiltDeg(accel(16384, 0, 0)));
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_flat_is_zero);
    RUN_TEST(test_thirty_five_degrees_either_way);
    RUN_TEST(test_axis_choice_does_not_matter);
    RUN_TEST(test_on_its_side_is_ninety);
    return UNITY_END();
}
