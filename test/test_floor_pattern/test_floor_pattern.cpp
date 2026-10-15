// ============================================================================
//  test_floor_pattern.cpp  --  classifyFloor() and the marker / gap detector. No hardware needed.
//  The proportions are the ones measured on the robot's marker: white ~36, black ~118, white ~37
//  mm.
// ============================================================================

#include <unity.h>

#include "FloorPattern.h"

static float g_reportMm = 0.0f; // 0 = report at the frame's minimum width

static PatternParams params()
{
    PatternParams p;
    p.reportMm = g_reportMm;
    p.frameMinMm = 25.0f;
    p.frameMaxMm = 55.0f;
    p.centreMinMm = 95.0f;
    p.centreMaxMm = 140.0f;
    p.gapMinMm = 12.0f;
    p.glitchMm = 4.0f;
    return p;
}

struct Seg {
    FloorClass cls;
    float lengthMm;
};

struct Result {
    int markers = 0;
    int gaps = 0;
    float markerCentreMm = 0.0f;
    float markerAtMm = 0.0f;
    float gapAtMm = 0.0f;
};

// Drive the detector along `segs`, one sweep per millimetre.
static Result run(const Seg *segs, int n, const PatternParams &p = params())
{
    FloorPattern f;
    floorPatternReset(f);
    Result r;
    float pos = 0.0f;
    for (int i = 0; i < n; ++i) {
        for (float x = 0.0f; x < segs[i].lengthMm; x += 1.0f, pos += 1.0f) {
            float centre = 0.0f;
            const PatternEvent e = floorPatternStep(f, segs[i].cls, pos, p, centre);
            if (e == PatternEvent::Marker) {
                ++r.markers;
                r.markerCentreMm = centre;
                r.markerAtMm = pos;
            } else if (e == PatternEvent::Gap) {
                ++r.gaps;
                r.gapAtMm = pos;
            }
        }
    }
    return r;
}

constexpr FloorClass B = FloorClass::Black;
constexpr FloorClass W = FloorClass::White;
constexpr FloorClass A = FloorClass::Air;
constexpr FloorClass X = FloorClass::Mixed;

// ---- classifyFloor ---------------------------------------------------------------

static LineView viewOf(bool black, bool white)
{
    LineView v;
    v.allBlack = black;
    v.allWhite = white;
    v.onLine = !black && !white;
    return v;
}

static void test_black_white_and_mixed_follow_the_line_view()
{
    const uint16_t raw[IR_CHANNEL_COUNT] = {900, 900, 900, 900, 900, 900, 900, 900};
    TEST_ASSERT_EQUAL(static_cast<int>(B),
                      static_cast<int>(classifyFloor(viewOf(true, false), raw, 1010, 7)));
    TEST_ASSERT_EQUAL(static_cast<int>(W),
                      static_cast<int>(classifyFloor(viewOf(false, true), raw, 1010, 7)));
    TEST_ASSERT_EQUAL(static_cast<int>(X),
                      static_cast<int>(classifyFloor(viewOf(false, false), raw, 1010, 7)));
}

static void test_the_whole_array_saturated_is_air()
{
    const uint16_t raw[IR_CHANNEL_COUNT] = {1020, 1020, 1016, 1020, 1020, 1020, 1012, 1020};
    TEST_ASSERT_EQUAL(static_cast<int>(A),
                      static_cast<int>(classifyFloor(viewOf(true, false), raw, 1010, 7)));
}

static void test_the_tape_alone_is_not_air()
{
    // The black tape reads ~1020 on a few channels; the rest are the floor's ~1000.
    const uint16_t raw[IR_CHANNEL_COUNT] = {1002, 990, 1010, 1022, 1021, 1015, 992, 1004};
    TEST_ASSERT_EQUAL(static_cast<int>(B),
                      static_cast<int>(classifyFloor(viewOf(true, false), raw, 1010, 7)));
}

static void test_one_low_channel_does_not_hide_air()
{
    const uint16_t raw[IR_CHANNEL_COUNT] = {1020, 1020, 1020, 1020, 1020, 960, 1020, 1020};
    TEST_ASSERT_EQUAL(static_cast<int>(A),
                      static_cast<int>(classifyFloor(viewOf(false, false), raw, 1010, 7)));
}

static void test_air_can_be_switched_off()
{
    const uint16_t raw[IR_CHANNEL_COUNT] = {1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023};
    TEST_ASSERT_EQUAL(static_cast<int>(B),
                      static_cast<int>(classifyFloor(viewOf(true, false), raw, 0, 7)));
}

// ---- the marker ------------------------------------------------------------------

static void test_the_measured_marker_is_found_once_with_its_centre()
{
    const Seg path[] = {{B, 100}, {W, 36}, {B, 118}, {W, 37}, {B, 30}};
    const Result r = run(path, 5);
    TEST_ASSERT_EQUAL(1, r.markers);
    // The black box ran from 136 to 254: its middle is 195.
    TEST_ASSERT_FLOAT_WITHIN(1.5f, 195.0f, r.markerCentreMm);
    // Reported as the far frame reaches its minimum width (25 mm into it).
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 254.0f + 25.0f, r.markerAtMm);
}

static void test_a_lower_report_length_finds_the_marker_while_the_far_frame_is_short()
{
    g_reportMm = 4.0f;
    // The robot stops on its odometer's cell centre with the array only ~6 mm into the far frame.
    const Seg path[] = {{B, 100}, {W, 36}, {B, 118}, {W, 6}};
    const Result r = run(path, 4);
    TEST_ASSERT_EQUAL(1, r.markers);
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 254.0f + 4.0f, r.markerAtMm);
    // Below the report length it is not yet a marker.
    const Seg shortPath[] = {{B, 100}, {W, 36}, {B, 118}, {W, 2}};
    TEST_ASSERT_EQUAL(0, run(shortPath, 4).markers);
    g_reportMm = 0.0f;
}

static void test_the_marker_is_found_whichever_way_it_is_crossed()
{
    const Seg path[] = {{B, 80}, {W, 37}, {B, 118}, {W, 36}, {B, 50}};
    TEST_ASSERT_EQUAL(1, run(path, 5).markers);
}

static void test_a_long_white_start_tile_is_not_a_marker()
{
    const Seg path[] = {{W, 250}, {B, 118}, {W, 37}, {B, 30}};
    TEST_ASSERT_EQUAL(0, run(path, 4).markers);
}

static void test_frames_that_are_too_narrow_or_too_wide_are_not_a_marker()
{
    const Seg narrow[] = {{B, 100}, {W, 10}, {B, 118}, {W, 10}, {B, 30}};
    TEST_ASSERT_EQUAL(0, run(narrow, 5).markers);
    const Seg wide[] = {{B, 100}, {W, 90}, {B, 118}, {W, 90}, {B, 30}};
    TEST_ASSERT_EQUAL(0, run(wide, 5).markers);
}

static void test_a_centre_box_of_the_wrong_size_is_not_a_marker()
{
    const Seg small[] = {{B, 100}, {W, 36}, {B, 50}, {W, 37}, {B, 30}};
    TEST_ASSERT_EQUAL(0, run(small, 5).markers);
    const Seg big[] = {{B, 100}, {W, 36}, {B, 220}, {W, 37}, {B, 30}};
    TEST_ASSERT_EQUAL(0, run(big, 5).markers);
}

static void test_a_short_glitch_inside_the_black_box_does_not_break_the_marker()
{
    const Seg path[] = {{B, 100}, {W, 36}, {B, 60}, {W, 3}, {B, 55}, {W, 37}, {B, 30}};
    TEST_ASSERT_EQUAL(1, run(path, 7).markers);
}

static void test_a_glitch_in_the_frame_does_not_split_it()
{
    const Seg path[] = {{B, 100}, {W, 18}, {B, 3}, {W, 15}, {B, 118}, {W, 37}, {B, 30}};
    TEST_ASSERT_EQUAL(1, run(path, 7).markers);
}

static void test_edge_sweeps_do_not_change_the_run()
{
    // The edges are Mixed for a few mm each; they must neither end a run nor start one.
    const Seg path[] = {{B, 100}, {X, 4}, {W, 32}, {X, 4}, {B, 114}, {X, 4}, {W, 33}, {B, 30}};
    TEST_ASSERT_EQUAL(1, run(path, 8).markers);
}

static void test_two_markers_are_two_events()
{
    const Seg path[] = {{B, 100}, {W, 36},  {B, 118}, {W, 37}, {B, 200},
                        {W, 36},  {B, 118}, {W, 37},  {B, 30}};
    TEST_ASSERT_EQUAL(2, run(path, 9).markers);
}

// ---- the gap ---------------------------------------------------------------------

static void test_a_gap_is_reported_once_after_its_minimum_length()
{
    const Seg path[] = {{B, 100}, {A, 21}, {W, 40}};
    const Result r = run(path, 3);
    TEST_ASSERT_EQUAL(1, r.gaps);
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 100.0f + 12.0f, r.gapAtMm);
}

static void test_air_shorter_than_the_minimum_is_not_a_gap()
{
    const Seg path[] = {{B, 100}, {A, 8}, {B, 40}};
    TEST_ASSERT_EQUAL(0, run(path, 3).gaps);
}

static void test_a_marker_the_strip_and_the_gap_in_the_measured_order()
{
    const Seg path[] = {{B, 105}, {W, 36}, {B, 118}, {W, 37}, {B, 20}, {A, 21}, {B, 2}, {W, 60}};
    const Result r = run(path, 8);
    TEST_ASSERT_EQUAL(1, r.markers);
    TEST_ASSERT_EQUAL(1, r.gaps);
    TEST_ASSERT_TRUE(r.gapAtMm > r.markerAtMm);
}

static void test_reset_forgets_the_path()
{
    FloorPattern f;
    floorPatternReset(f);
    float centre = 0.0f;
    const PatternParams p = params();
    for (int i = 0; i < 100; ++i) {
        floorPatternStep(f, W, static_cast<float>(i), p, centre);
    }
    floorPatternReset(f);
    TEST_ASSERT_FALSE(f.started);
    TEST_ASSERT_EQUAL(0, f.doneCount);
}

// ---- the handover to the line follower, after the gap --------------------------------------

static LineView lineViewOn()
{
    LineView v;
    v.onLine = true;
    return v;
}

static LineView lineViewBlack()
{
    LineView v;
    v.allBlack = true;
    return v;
}

static void test_no_handover_before_the_gap_even_on_a_line()
{
    BridgeHandover b;
    bridgeHandoverReset(b);
    for (float pos = 0.0f; pos < 100.0f; pos += 1.0f) {
        TEST_ASSERT_FALSE(bridgeHandoverStep(b, false, FloorClass::Mixed, lineViewOn(), pos, 4.0f));
    }
}

static void test_no_handover_while_over_the_gap()
{
    BridgeHandover b;
    bridgeHandoverReset(b);
    for (float pos = 0.0f; pos < 30.0f; pos += 1.0f) {
        TEST_ASSERT_FALSE(bridgeHandoverStep(b, true, FloorClass::Air, lineViewOn(), pos, 4.0f));
    }
}

static void test_the_dark_patch_after_the_gap_does_not_hand_over_and_the_line_after_it_does()
{
    BridgeHandover b;
    bridgeHandoverReset(b);
    bridgeHandoverStep(b, true, FloorClass::Air, lineViewBlack(), 100.0f, 4.0f);
    // 8 mm of bare dark floor after the air: not a line.
    for (float pos = 101.0f; pos <= 108.0f; pos += 1.0f) {
        TEST_ASSERT_FALSE(
            bridgeHandoverStep(b, true, FloorClass::Black, lineViewBlack(), pos, 4.0f));
    }
    TEST_ASSERT_TRUE(bridgeHandoverStep(b, true, FloorClass::Mixed, lineViewOn(), 109.0f, 4.0f));
}

static void test_a_line_right_at_the_trailing_edge_waits_for_the_minimum_distance()
{
    BridgeHandover b;
    bridgeHandoverReset(b);
    bridgeHandoverStep(b, true, FloorClass::Air, lineViewBlack(), 100.0f, 4.0f);
    TEST_ASSERT_FALSE(bridgeHandoverStep(b, true, FloorClass::Mixed, lineViewOn(), 101.0f, 4.0f));
    TEST_ASSERT_FALSE(bridgeHandoverStep(b, true, FloorClass::Mixed, lineViewOn(), 104.0f, 4.0f));
    TEST_ASSERT_TRUE(bridgeHandoverStep(b, true, FloorClass::Mixed, lineViewOn(), 105.0f, 4.0f));
}

static void test_air_again_restarts_the_wait()
{
    BridgeHandover b;
    bridgeHandoverReset(b);
    bridgeHandoverStep(b, true, FloorClass::Mixed, lineViewBlack(), 100.0f, 4.0f);
    bridgeHandoverStep(b, true, FloorClass::Air, lineViewBlack(), 103.0f, 4.0f);
    TEST_ASSERT_FALSE(bridgeHandoverStep(b, true, FloorClass::Mixed, lineViewOn(), 106.0f, 4.0f));
}

void setUp() {}
void tearDown() {}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_black_white_and_mixed_follow_the_line_view);
    RUN_TEST(test_the_whole_array_saturated_is_air);
    RUN_TEST(test_the_tape_alone_is_not_air);
    RUN_TEST(test_one_low_channel_does_not_hide_air);
    RUN_TEST(test_air_can_be_switched_off);
    RUN_TEST(test_the_measured_marker_is_found_once_with_its_centre);
    RUN_TEST(test_a_lower_report_length_finds_the_marker_while_the_far_frame_is_short);
    RUN_TEST(test_the_marker_is_found_whichever_way_it_is_crossed);
    RUN_TEST(test_a_long_white_start_tile_is_not_a_marker);
    RUN_TEST(test_frames_that_are_too_narrow_or_too_wide_are_not_a_marker);
    RUN_TEST(test_a_centre_box_of_the_wrong_size_is_not_a_marker);
    RUN_TEST(test_a_short_glitch_inside_the_black_box_does_not_break_the_marker);
    RUN_TEST(test_a_glitch_in_the_frame_does_not_split_it);
    RUN_TEST(test_edge_sweeps_do_not_change_the_run);
    RUN_TEST(test_two_markers_are_two_events);
    RUN_TEST(test_a_gap_is_reported_once_after_its_minimum_length);
    RUN_TEST(test_air_shorter_than_the_minimum_is_not_a_gap);
    RUN_TEST(test_a_marker_the_strip_and_the_gap_in_the_measured_order);
    RUN_TEST(test_reset_forgets_the_path);
    RUN_TEST(test_no_handover_before_the_gap_even_on_a_line);
    RUN_TEST(test_no_handover_while_over_the_gap);
    RUN_TEST(test_the_dark_patch_after_the_gap_does_not_hand_over_and_the_line_after_it_does);
    RUN_TEST(test_a_line_right_at_the_trailing_edge_waits_for_the_minimum_distance);
    RUN_TEST(test_air_again_restarts_the_wait);
    return UNITY_END();
}
