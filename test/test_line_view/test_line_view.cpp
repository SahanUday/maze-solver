// ============================================================================
//  test_line_view.cpp  --  lineView(): one sweep of the array to where the line is. No hardware.
// ============================================================================

#include <unity.h>

#include "LineView.h"

constexpr float PITCH = 10.0f;

struct Sweep {
    uint8_t norm[IR_CHANNEL_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t mask = 0;
};

static Sweep sweep(uint8_t mask, uint8_t level = 255)
{
    Sweep s;
    s.mask = mask;
    for (uint8_t ch = 0; ch < IR_CHANNEL_COUNT; ++ch) {
        s.norm[ch] = (mask & (1u << ch)) ? level : 0;
    }
    return s;
}

static void test_centred_line_has_zero_error()
{
    const Sweep s = sweep(0b00011000); // channels 3 and 4
    const LineView v = lineView(s.norm, s.mask, 0xFF, PITCH);
    TEST_ASSERT_TRUE(v.onLine);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, v.errMm);
}

static void test_line_to_the_left_is_positive()
{
    const Sweep s = sweep(0b00000001); // channel 0 = D1 = leftmost
    const LineView v = lineView(s.norm, s.mask, 0xFF, PITCH);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 35.0f, v.errMm);
}

static void test_line_to_the_right_is_negative()
{
    const Sweep s = sweep(0b10000000);
    const LineView v = lineView(s.norm, s.mask, 0xFF, PITCH);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, -35.0f, v.errMm);
}

static void test_centroid_weights_by_darkness()
{
    Sweep s = sweep(0b00011000);
    s.norm[3] = 255; // position +5
    s.norm[4] = 85;  // position -5
    const LineView v = lineView(s.norm, s.mask, 0xFF, PITCH);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 2.5f, v.errMm); // (255*5 - 85*5) / 340
}

static void test_all_black_is_the_floor()
{
    const Sweep s = sweep(0xFF);
    const LineView v = lineView(s.norm, s.mask, 0xFF, PITCH);
    TEST_ASSERT_TRUE(v.allBlack);
    TEST_ASSERT_FALSE(v.onLine);
    TEST_ASSERT_FALSE(v.allWhite);
}

static void test_all_white()
{
    const Sweep s = sweep(0x00);
    const LineView v = lineView(s.norm, s.mask, 0xFF, PITCH);
    TEST_ASSERT_TRUE(v.allWhite);
    TEST_ASSERT_FALSE(v.onLine);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, v.errMm);
}

static void test_unusable_channels_are_ignored()
{
    // Channel 7 is dead (reads white always). The other seven all black still means floor.
    const Sweep s = sweep(0b01111111);
    const LineView v = lineView(s.norm, s.mask, 0b01111111, PITCH);
    TEST_ASSERT_TRUE(v.allBlack);
    // A mask bit on an unusable channel is not believed.
    const Sweep t = sweep(0b10000000);
    TEST_ASSERT_TRUE(lineView(t.norm, t.mask, 0b01111111, PITCH).allWhite);
}

static void test_no_usable_channels_never_reads_as_floor()
{
    const Sweep s = sweep(0xFF);
    const LineView v = lineView(s.norm, s.mask, 0x00, PITCH);
    TEST_ASSERT_FALSE(v.allBlack);
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_centred_line_has_zero_error);
    RUN_TEST(test_line_to_the_left_is_positive);
    RUN_TEST(test_line_to_the_right_is_negative);
    RUN_TEST(test_centroid_weights_by_darkness);
    RUN_TEST(test_all_black_is_the_floor);
    RUN_TEST(test_all_white);
    RUN_TEST(test_unusable_channels_are_ignored);
    RUN_TEST(test_no_usable_channels_never_reads_as_floor);
    return UNITY_END();
}
