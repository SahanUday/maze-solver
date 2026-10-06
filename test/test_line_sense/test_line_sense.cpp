// ============================================================================
//  test_line_sense.cpp  --  calibration, normalization and mask tests. No
//  hardware needed. The fixtures are counts measured on the robot: white =
//  average of three runs over white paper, black = black paper.
// ============================================================================

#include <unity.h>

#include "LineSense.h"

static const uint16_t kWhite[IR_CHANNEL_COUNT] = {983, 973, 976, 976, 974, 986, 982, 990};
static const uint16_t kBlack[IR_CHANNEL_COUNT] = {1019, 1019, 1019, 1019, 1019, 1019, 1019, 1019};

static void copyCounts(uint16_t (&to)[IR_CHANNEL_COUNT], const uint16_t (&from)[IR_CHANNEL_COUNT])
{
    for (uint8_t ch = 0; ch < IR_CHANNEL_COUNT; ++ch) {
        to[ch] = from[ch];
    }
}

// Calibrated on the fixtures, every channel usable.
static LineCalibration fixtureCalibration()
{
    LineCalibration cal;
    lineCalibrationReset(cal);
    lineCalibrationAccumulate(cal, kWhite);
    lineCalibrationAccumulate(cal, kBlack);
    lineCalibrationFinish(cal);
    return cal;
}

static void test_reset_sees_nothing_and_scales_nothing()
{
    LineCalibration cal;
    lineCalibrationReset(cal);
    for (uint8_t ch = 0; ch < IR_CHANNEL_COUNT; ++ch) {
        TEST_ASSERT_EQUAL_UINT16(ADC_FULL_SCALE_COUNTS, cal.white[ch]);
        TEST_ASSERT_EQUAL_UINT16(0, cal.black[ch]);
        TEST_ASSERT_EQUAL_UINT32(0, cal.scale[ch]);
    }
}

static void test_accumulate_keeps_the_lowest_and_highest_count_per_channel()
{
    LineCalibration cal;
    lineCalibrationReset(cal);
    uint16_t a[IR_CHANNEL_COUNT] = {500, 500, 500, 500, 500, 500, 500, 500};
    uint16_t b[IR_CHANNEL_COUNT] = {400, 600, 500, 500, 500, 500, 500, 700};
    uint16_t c[IR_CHANNEL_COUNT] = {450, 550, 500, 500, 500, 500, 500, 650};
    lineCalibrationAccumulate(cal, a);
    lineCalibrationAccumulate(cal, b);
    lineCalibrationAccumulate(cal, c);
    TEST_ASSERT_EQUAL_UINT16(400, cal.white[0]);
    TEST_ASSERT_EQUAL_UINT16(500, cal.black[0]);
    TEST_ASSERT_EQUAL_UINT16(500, cal.white[1]);
    TEST_ASSERT_EQUAL_UINT16(600, cal.black[1]);
    TEST_ASSERT_EQUAL_UINT16(500, cal.white[7]);
    TEST_ASSERT_EQUAL_UINT16(700, cal.black[7]);
}

static void test_finish_marks_every_fixture_channel_usable()
{
    LineCalibration cal;
    lineCalibrationReset(cal);
    lineCalibrationAccumulate(cal, kWhite);
    lineCalibrationAccumulate(cal, kBlack);
    TEST_ASSERT_EQUAL_UINT8(0xFF, lineCalibrationFinish(cal));
    for (uint8_t ch = 0; ch < IR_CHANNEL_COUNT; ++ch) {
        TEST_ASSERT_TRUE(cal.scale[ch] != 0);
    }
}

static void test_a_range_under_the_minimum_is_unusable_the_minimum_itself_is_not()
{
    LineCalibration cal;
    lineCalibrationReset(cal);
    uint16_t low[IR_CHANNEL_COUNT], high[IR_CHANNEL_COUNT];
    for (uint8_t ch = 0; ch < IR_CHANNEL_COUNT; ++ch) {
        low[ch] = 1000;
        high[ch] = 1000; // no range at all
    }
    high[0] = 1000 + LINE_CAL_MIN_SPAN_COUNTS - 1; // one short of the minimum
    high[1] = 1000 + LINE_CAL_MIN_SPAN_COUNTS;     // exactly the minimum
    lineCalibrationAccumulate(cal, low);
    lineCalibrationAccumulate(cal, high);
    TEST_ASSERT_EQUAL_UINT8(0b00000010, lineCalibrationFinish(cal));
    TEST_ASSERT_EQUAL_UINT32(0, cal.scale[0]);
    TEST_ASSERT_TRUE(cal.scale[1] != 0);
    TEST_ASSERT_EQUAL_UINT32(0, cal.scale[2]);
}

static void test_a_channel_never_sampled_is_unusable()
{
    LineCalibration cal;
    lineCalibrationReset(cal);
    TEST_ASSERT_EQUAL_UINT8(0, lineCalibrationFinish(cal));
}

static void test_white_normalizes_to_0_and_black_to_255_on_every_channel()
{
    const LineCalibration cal = fixtureCalibration();
    uint8_t norm[IR_CHANNEL_COUNT];
    lineNormalize(cal, kWhite, norm);
    for (uint8_t ch = 0; ch < IR_CHANNEL_COUNT; ++ch) {
        TEST_ASSERT_EQUAL_UINT8(0, norm[ch]);
    }
    lineNormalize(cal, kBlack, norm);
    for (uint8_t ch = 0; ch < IR_CHANNEL_COUNT; ++ch) {
        TEST_ASSERT_EQUAL_UINT8(255, norm[ch]);
    }
}

static void test_the_endpoints_are_exact_for_any_range_from_the_minimum_to_full_scale()
{
    for (uint16_t range = LINE_CAL_MIN_SPAN_COUNTS; range <= ADC_FULL_SCALE_COUNTS; ++range) {
        LineCalibration cal;
        lineCalibrationReset(cal);
        uint16_t white[IR_CHANNEL_COUNT], black[IR_CHANNEL_COUNT];
        for (uint8_t ch = 0; ch < IR_CHANNEL_COUNT; ++ch) {
            white[ch] = 0;
            black[ch] = range;
        }
        lineCalibrationAccumulate(cal, white);
        lineCalibrationAccumulate(cal, black);
        lineCalibrationFinish(cal);
        uint8_t norm[IR_CHANNEL_COUNT];
        lineNormalize(cal, white, norm);
        TEST_ASSERT_EQUAL_UINT8(0, norm[0]);
        lineNormalize(cal, black, norm);
        TEST_ASSERT_EQUAL_UINT8(255, norm[0]);
    }
}

static void test_the_middle_of_the_range_is_the_middle_of_the_scale()
{
    const LineCalibration cal = fixtureCalibration();
    uint16_t mid[IR_CHANNEL_COUNT];
    for (uint8_t ch = 0; ch < IR_CHANNEL_COUNT; ++ch) {
        mid[ch] = static_cast<uint16_t>((kWhite[ch] + kBlack[ch]) / 2);
    }
    uint8_t norm[IR_CHANNEL_COUNT];
    lineNormalize(cal, mid, norm);
    for (uint8_t ch = 0; ch < IR_CHANNEL_COUNT; ++ch) {
        TEST_ASSERT_UINT8_WITHIN(8, 127, norm[ch]); // one count of a 29-count range is ~9 units
    }
}

static void test_counts_outside_the_calibrated_range_clamp()
{
    const LineCalibration cal = fixtureCalibration();
    uint16_t below[IR_CHANNEL_COUNT], above[IR_CHANNEL_COUNT];
    for (uint8_t ch = 0; ch < IR_CHANNEL_COUNT; ++ch) {
        below[ch] = 0;
        above[ch] = ADC_FULL_SCALE_COUNTS;
    }
    uint8_t norm[IR_CHANNEL_COUNT];
    lineNormalize(cal, below, norm);
    TEST_ASSERT_EQUAL_UINT8(0, norm[3]);
    lineNormalize(cal, above, norm);
    TEST_ASSERT_EQUAL_UINT8(255, norm[3]);
}

static void test_normalizing_never_decreases_as_the_count_rises()
{
    const LineCalibration cal = fixtureCalibration();
    uint8_t previous = 0;
    for (uint16_t count = 0; count <= ADC_FULL_SCALE_COUNTS; ++count) {
        uint16_t counts[IR_CHANNEL_COUNT];
        for (uint8_t ch = 0; ch < IR_CHANNEL_COUNT; ++ch) {
            counts[ch] = count;
        }
        uint8_t norm[IR_CHANNEL_COUNT];
        lineNormalize(cal, counts, norm);
        TEST_ASSERT_TRUE(norm[5] >= previous);
        previous = norm[5];
    }
}

static void test_an_unusable_channel_reads_zero_whatever_the_count()
{
    LineCalibration cal;
    lineCalibrationReset(cal);
    lineCalibrationAccumulate(cal, kWhite);
    lineCalibrationFinish(cal); // never saw black: no range anywhere
    uint8_t norm[IR_CHANNEL_COUNT];
    lineNormalize(cal, kBlack, norm);
    for (uint8_t ch = 0; ch < IR_CHANNEL_COUNT; ++ch) {
        TEST_ASSERT_EQUAL_UINT8(0, norm[ch]);
    }
}

static void test_mask_sets_at_on_clears_at_off_and_holds_between()
{
    constexpr uint8_t on = (255u * LINE_MASK_ON_PCT) / 100;
    constexpr uint8_t off = (255u * LINE_MASK_OFF_PCT) / 100;
    uint8_t norm[IR_CHANNEL_COUNT] = {0};

    norm[0] = on;
    TEST_ASSERT_EQUAL_UINT8(0b1, lineMaskUpdate(norm, 0));
    norm[0] = on - 1; // dead band: keeps what it had
    TEST_ASSERT_EQUAL_UINT8(0b0, lineMaskUpdate(norm, 0));
    TEST_ASSERT_EQUAL_UINT8(0b1, lineMaskUpdate(norm, 0b1));
    norm[0] = off + 1;
    TEST_ASSERT_EQUAL_UINT8(0b1, lineMaskUpdate(norm, 0b1));
    norm[0] = off;
    TEST_ASSERT_EQUAL_UINT8(0b0, lineMaskUpdate(norm, 0b1));
}

static void test_a_channel_rising_then_falling_flips_at_the_two_thresholds_not_one()
{
    constexpr uint8_t on = (255u * LINE_MASK_ON_PCT) / 100;
    constexpr uint8_t off = (255u * LINE_MASK_OFF_PCT) / 100;
    uint8_t norm[IR_CHANNEL_COUNT] = {0};
    uint8_t mask = 0;
    uint8_t flipsUp = 0, flipsDown = 0;
    for (uint16_t v = 0; v <= 255; ++v) { // rising
        norm[2] = static_cast<uint8_t>(v);
        const uint8_t next = lineMaskUpdate(norm, mask);
        if (!(mask & 0b100) && (next & 0b100)) {
            TEST_ASSERT_EQUAL_UINT8(on, v);
            ++flipsUp;
        }
        mask = next;
    }
    for (int16_t v = 255; v >= 0; --v) { // falling
        norm[2] = static_cast<uint8_t>(v);
        const uint8_t next = lineMaskUpdate(norm, mask);
        if ((mask & 0b100) && !(next & 0b100)) {
            TEST_ASSERT_EQUAL_UINT8(off, v);
            ++flipsDown;
        }
        mask = next;
    }
    TEST_ASSERT_EQUAL_UINT8(1, flipsUp);
    TEST_ASSERT_EQUAL_UINT8(1, flipsDown);
}

static void test_mask_channels_are_independent()
{
    uint8_t norm[IR_CHANNEL_COUNT] = {0, 255, 0, 255, 0, 255, 0, 255};
    TEST_ASSERT_EQUAL_UINT8(0b10101010, lineMaskUpdate(norm, 0));
    TEST_ASSERT_EQUAL_UINT8(0b10101010,
                            lineMaskUpdate(norm, 0b01010101)); // the other half is cleared
}

// A line under D4-D6: calibrate on white then black, then a sweep with the middle three black.
static void test_a_line_under_the_middle_three_channels_gives_that_mask()
{
    const LineCalibration cal = fixtureCalibration();
    uint16_t counts[IR_CHANNEL_COUNT];
    copyCounts(counts, kWhite);
    counts[3] = kBlack[3];
    counts[4] = kBlack[4];
    counts[5] = kBlack[5];
    uint8_t norm[IR_CHANNEL_COUNT];
    lineNormalize(cal, counts, norm);
    TEST_ASSERT_EQUAL_UINT8(0b00111000, lineMaskUpdate(norm, 0));
    TEST_ASSERT_EQUAL_UINT8(0, norm[0]);
    TEST_ASSERT_EQUAL_UINT8(255, norm[4]);
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_reset_sees_nothing_and_scales_nothing);
    RUN_TEST(test_accumulate_keeps_the_lowest_and_highest_count_per_channel);
    RUN_TEST(test_finish_marks_every_fixture_channel_usable);
    RUN_TEST(test_a_range_under_the_minimum_is_unusable_the_minimum_itself_is_not);
    RUN_TEST(test_a_channel_never_sampled_is_unusable);
    RUN_TEST(test_white_normalizes_to_0_and_black_to_255_on_every_channel);
    RUN_TEST(test_the_endpoints_are_exact_for_any_range_from_the_minimum_to_full_scale);
    RUN_TEST(test_the_middle_of_the_range_is_the_middle_of_the_scale);
    RUN_TEST(test_counts_outside_the_calibrated_range_clamp);
    RUN_TEST(test_normalizing_never_decreases_as_the_count_rises);
    RUN_TEST(test_an_unusable_channel_reads_zero_whatever_the_count);
    RUN_TEST(test_mask_sets_at_on_clears_at_off_and_holds_between);
    RUN_TEST(test_a_channel_rising_then_falling_flips_at_the_two_thresholds_not_one);
    RUN_TEST(test_mask_channels_are_independent);
    RUN_TEST(test_a_line_under_the_middle_three_channels_gives_that_mask);
    return UNITY_END();
}
