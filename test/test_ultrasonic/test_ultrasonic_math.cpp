// ============================================================================
//  test_ultrasonic_math.cpp  --  UltrasonicMath.h against synthetic echo
//  durations. No hardware: the input is a tick count.
// ============================================================================

#include <unity.h>

#include "UltrasonicMath.h"

static void test_zero_ticks_is_invalid()
{
    const UltrasonicReading r = ultrasonicReadingFromTicks(0);
    TEST_ASSERT_FALSE(r.valid);
    TEST_ASSERT_EQUAL_UINT16(0, r.distanceMm);
}

static void test_exact_conversion()
{
    // 4000 ticks * 343 / 4000 = 343mm exactly, no truncation.
    const UltrasonicReading r = ultrasonicReadingFromTicks(4000);
    TEST_ASSERT_TRUE(r.valid);
    TEST_ASSERT_EQUAL_UINT16(343, r.distanceMm);
}

static void test_wall_height_distance()
{
    // 100mm round trip is 1166 ticks; truncation gives 99.
    const UltrasonicReading r = ultrasonicReadingFromTicks(1166);
    TEST_ASSERT_TRUE(r.valid);
    TEST_ASSERT_EQUAL_UINT16(99, r.distanceMm);
}

static void test_corridor_width_distance()
{
    // USABLE_CORRIDOR_MM = 235 -> 2740 ticks -> 234 after truncation.
    const UltrasonicReading r = ultrasonicReadingFromTicks(2740);
    TEST_ASSERT_TRUE(r.valid);
    TEST_ASSERT_EQUAL_UINT16(234, r.distanceMm);
}

static void test_longest_arena_sightline_is_in_range()
{
    // 9 tiles x 250mm = 2250mm, the furthest the arena can present.
    const UltrasonicReading r = ultrasonicReadingFromTicks(26239);
    TEST_ASSERT_TRUE(r.valid);
    TEST_ASSERT_EQUAL_UINT16(2249, r.distanceMm);
}

static void test_dead_zone_is_rejected()
{
    // 100 ticks = ~8mm, inside US_MIN_RANGE_MM.
    const UltrasonicReading r = ultrasonicReadingFromTicks(100);
    TEST_ASSERT_FALSE(r.valid);
    TEST_ASSERT_EQUAL_UINT16(0, r.distanceMm);
}

static void test_deadline_boundary_is_accepted()
{
    const UltrasonicReading r = ultrasonicReadingFromTicks(US_ECHO_TIMEOUT_TICKS);
    TEST_ASSERT_TRUE(r.valid);
}

static void test_one_tick_past_deadline_is_rejected()
{
    const UltrasonicReading r = ultrasonicReadingFromTicks(US_ECHO_TIMEOUT_TICKS + 1);
    TEST_ASSERT_FALSE(r.valid);
    // A caller ignoring `valid` must not read a plausible distance.
    TEST_ASSERT_EQUAL_UINT16(0, r.distanceMm);
}

static void test_timeout_never_reports_max_range()
{
    // A silent sensor must not look like "clear corridor ahead".
    const UltrasonicReading r = ultrasonicReadingFromTicks(60000);
    TEST_ASSERT_FALSE(r.valid);
    TEST_ASSERT_NOT_EQUAL_UINT16(US_RANGE_CAP_MM, r.distanceMm);
}

static void test_distance_is_monotonic_in_ticks()
{
    uint16_t previous = 0;
    for (uint16_t ticks = 1000; ticks < US_ECHO_TIMEOUT_TICKS; ticks = (uint16_t)(ticks + 1000)) {
        const UltrasonicReading r = ultrasonicReadingFromTicks(ticks);
        TEST_ASSERT_TRUE(r.valid);
        TEST_ASSERT_GREATER_THAN_UINT16(previous, r.distanceMm);
        previous = r.distanceMm;
    }
}

static void test_no_overflow_at_the_deadline()
{
    // ticks * 343 must be widened: 29154 * 343 overflows 16 bits.
    const UltrasonicReading r = ultrasonicReadingFromTicks(US_ECHO_TIMEOUT_TICKS);
    TEST_ASSERT_TRUE(r.valid);
    TEST_ASSERT_UINT16_WITHIN(5, US_RANGE_CAP_MM, r.distanceMm);
}

// ---- per-sensor mounting offset -------------------------------------------

static void test_positive_offset_shifts_result()
{
    // 4000 ticks = 343mm raw; +20 puts the reference point behind the face.
    const UltrasonicReading r = ultrasonicReadingFromTicks(4000, 20);
    TEST_ASSERT_TRUE(r.valid);
    TEST_ASSERT_EQUAL_UINT16(363, r.distanceMm);
}

static void test_negative_offset_shifts_result()
{
    const UltrasonicReading r = ultrasonicReadingFromTicks(4000, -20);
    TEST_ASSERT_TRUE(r.valid);
    TEST_ASSERT_EQUAL_UINT16(323, r.distanceMm);
}

static void test_zero_offset_matches_no_offset()
{
    const UltrasonicReading a = ultrasonicReadingFromTicks(2740);
    const UltrasonicReading b = ultrasonicReadingFromTicks(2740, 0);
    TEST_ASSERT_EQUAL_UINT16(a.distanceMm, b.distanceMm);
    TEST_ASSERT_EQUAL(a.valid, b.valid);
}

static void test_negative_offset_past_zero_is_invalid()
{
    // 4000 ticks = 343mm raw; -400 would underflow to a huge uint16.
    const UltrasonicReading r = ultrasonicReadingFromTicks(4000, -400);
    TEST_ASSERT_FALSE(r.valid);
    TEST_ASSERT_EQUAL_UINT16(0, r.distanceMm);
}

static void test_offset_cannot_rescue_a_dead_zone_reading()
{
    // 100 ticks = ~8mm raw, inside US_MIN_RANGE_MM. The dead zone is a
    // property of the sensor, so a generous offset must not make it valid.
    const UltrasonicReading r = ultrasonicReadingFromTicks(100, 50);
    TEST_ASSERT_FALSE(r.valid);
}

static void test_offset_does_not_move_the_deadline()
{
    // Past the deadline stays invalid whatever the offset.
    const UltrasonicReading r = ultrasonicReadingFromTicks(US_ECHO_TIMEOUT_TICKS + 1, -2000);
    TEST_ASSERT_FALSE(r.valid);
}

static void test_large_offset_does_not_overflow()
{
    const UltrasonicReading r = ultrasonicReadingFromTicks(US_ECHO_TIMEOUT_TICKS, 30000);
    TEST_ASSERT_TRUE(r.valid);
    TEST_ASSERT_UINT16_WITHIN(5, 32499, r.distanceMm);
}

// Unity links against these unconditionally.
extern "C" void setUp(void) {}

extern "C" void tearDown(void) {}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_zero_ticks_is_invalid);
    RUN_TEST(test_exact_conversion);
    RUN_TEST(test_wall_height_distance);
    RUN_TEST(test_corridor_width_distance);
    RUN_TEST(test_longest_arena_sightline_is_in_range);
    RUN_TEST(test_dead_zone_is_rejected);
    RUN_TEST(test_deadline_boundary_is_accepted);
    RUN_TEST(test_one_tick_past_deadline_is_rejected);
    RUN_TEST(test_timeout_never_reports_max_range);
    RUN_TEST(test_distance_is_monotonic_in_ticks);
    RUN_TEST(test_no_overflow_at_the_deadline);
    RUN_TEST(test_positive_offset_shifts_result);
    RUN_TEST(test_negative_offset_shifts_result);
    RUN_TEST(test_zero_offset_matches_no_offset);
    RUN_TEST(test_negative_offset_past_zero_is_invalid);
    RUN_TEST(test_offset_cannot_rescue_a_dead_zone_reading);
    RUN_TEST(test_offset_does_not_move_the_deadline);
    RUN_TEST(test_large_offset_does_not_overflow);
    return UNITY_END();
}
