// ============================================================================
//  test_line_follow.cpp  --  lineView(), FloorDetector and LineFollow. No hardware needed.
// ============================================================================

#include <unity.h>

#include "LineFollow.h"
#include "sim.h"

constexpr uint16_t DT_MS = 10;
constexpr float DT_S = 0.01f;
constexpr float PITCH = 10.0f;

static int asInt(LineStatus s)
{
    return static_cast<int>(s);
}

// ---- lineView ------------------------------------------------------------------

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

// ---- FloorDetector --------------------------------------------------------------

static FloorParams floorParams()
{
    FloorParams p;
    p.markerMinMm = 60.0f;
    p.markerMaxMm = 120.0f;
    p.lineEnterMm = 150.0f;
    p.gapMm = 8.0f;
    return p;
}

// Drive a pattern along the odometer: `black` for each 1 mm step.
struct Track {
    FloorDetector d;
    FloorParams p = floorParams();
    float pos = 0.0f;
    FloorEvent last = FloorEvent::None;
    float center = -1.0f;
    int markers = 0;
    int zones = 0;

    void run(bool black, float mm)
    {
        for (float i = 0; i < mm; i += 1.0f) {
            pos += 1.0f;
            float c = 0.0f;
            const FloorEvent e = floorDetectorStep(d, black, pos, p, c);
            if (e == FloorEvent::Marker) {
                ++markers;
                center = c;
            }
            if (e == FloorEvent::LineZone) {
                ++zones;
            }
            if (e != FloorEvent::None) {
                last = e;
            }
        }
    }
};

static void test_marker_is_a_short_run()
{
    Track t;
    t.run(true, 50);
    t.run(false, 90);
    t.run(true, 50);
    TEST_ASSERT_EQUAL(1, t.markers);
    TEST_ASSERT_EQUAL(0, t.zones);
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 50.0f + 45.0f, t.center);
}

static void test_a_long_run_is_the_line_zone_reported_once()
{
    Track t;
    t.run(true, 50);
    t.run(false, 500);
    TEST_ASSERT_EQUAL(1, t.zones);
    TEST_ASSERT_EQUAL(0, t.markers);
}

static void test_zone_can_be_found_again_after_the_floor_returns()
{
    Track t;
    t.run(true, 50);
    t.run(false, 200);
    t.run(true, 50);
    t.run(false, 200);
    TEST_ASSERT_EQUAL(2, t.zones);
}

static void test_runs_outside_the_marker_window_are_ignored()
{
    Track t;
    t.run(true, 50);
    t.run(false, 20); // too short: a speck
    t.run(true, 50);
    t.run(false, 130); // between marker max and line enter
    t.run(true, 50);
    TEST_ASSERT_EQUAL(0, t.markers);
    TEST_ASSERT_EQUAL(0, t.zones);
}

static void test_a_run_in_progress_at_power_up_is_not_measured()
{
    // Starting on the white START tile: the run began before we were watching.
    Track t;
    t.run(false, 100);
    t.run(true, 50);
    TEST_ASSERT_EQUAL(0, t.markers);
    // Floor seen, so it is armed now.
    t.run(false, 90);
    t.run(true, 50);
    TEST_ASSERT_EQUAL(1, t.markers);
}

static void test_a_glitch_inside_a_run_does_not_split_it()
{
    Track t;
    t.run(true, 50);
    t.run(false, 40);
    t.run(true, 3); // shorter than gapMm
    t.run(false, 40);
    t.run(true, 50);
    TEST_ASSERT_EQUAL(1, t.markers); // one run of ~83 mm, not two of 40
}

static void test_run_start_is_reported_when_white_begins()
{
    FloorDetector d;
    const FloorParams p = floorParams();
    float c = 0.0f;
    TEST_ASSERT_EQUAL(static_cast<int>(FloorEvent::None),
                      static_cast<int>(floorDetectorStep(d, true, 0.0f, p, c)));
    TEST_ASSERT_EQUAL(static_cast<int>(FloorEvent::RunStart),
                      static_cast<int>(floorDetectorStep(d, false, 10.0f, p, c)));
    TEST_ASSERT_EQUAL(static_cast<int>(FloorEvent::None),
                      static_cast<int>(floorDetectorStep(d, false, 20.0f, p, c)));
}

static void test_a_line_crossing_a_white_tile_does_not_split_it()
{
    // Enter a white tile across the line: 110 mm of white, the 30 mm line (all-black under the
    // array), 110 mm more. With a gap wider than the line this is one run, a line zone; without
    // it the second half would look like a marker.
    Track t;
    t.p.gapMm = 45.0f;
    t.run(true, 50);
    t.run(false, 110);
    t.run(true, 30);
    t.run(false, 110);
    t.run(true, 100);
    TEST_ASSERT_EQUAL(1, t.zones);
    TEST_ASSERT_EQUAL(0, t.markers);
}

static void test_the_marker_still_ends_at_the_floor_between_it_and_the_bridge_tile()
{
    // Marker (90 mm), 80 mm of floor, then the bridge tile: two runs, one marker, one zone.
    Track t;
    t.p.gapMm = 45.0f;
    t.run(true, 50);
    t.run(false, 90);
    t.run(true, 80);
    t.run(false, 250);
    TEST_ASSERT_EQUAL(1, t.markers);
    TEST_ASSERT_EQUAL(1, t.zones);
}

static void test_reset_forgets_the_floor()
{
    Track t;
    t.run(true, 10);
    floorDetectorReset(t.d);
    TEST_ASSERT_EQUAL(static_cast<int>(FloorDetector::State::WaitFloor),
                      static_cast<int>(t.d.state));
}

// ---- LineFollow, closed loop ----------------------------------------------------

struct LineWorld {
    float lineEndX = 600.0f; // the line (y = 0) runs from x = 0 to here; bare floor beyond
    float lineHalfMm = 15.0f;
    float tilt = 0.0f;
    bool lift = false;          // array lifted: reads black whatever is below
    float arrayAheadMm = 60.0f; // the array sits this far ahead of the axle

    // The sweep the array would read with the robot's axle at (x, y) heading `headingDeg`: the
    // array is ahead of the axle, so a heading error shows up as a sideways offset.
    void read(float x, float y, float headingDeg, uint8_t (&norm)[IR_CHANNEL_COUNT],
              uint8_t &mask) const
    {
        mask = 0;
        const float h = headingDeg * 0.0174533f;
        const float ay = y + arrayAheadMm * sinf(h);
        const float ax = x + arrayAheadMm * cosf(h);
        for (uint8_t ch = 0; ch < IR_CHANNEL_COUNT; ++ch) {
            const float pos = ay + (3.5f - static_cast<float>(ch)) * PITCH;
            const bool black = lift || ax > lineEndX || (pos >= -lineHalfMm && pos <= lineHalfMm);
            norm[ch] = black ? 255 : 0;
            if (black) {
                mask = static_cast<uint8_t>(mask | (1u << ch));
            }
        }
    }
};

static LineParams lineParams()
{
    LineParams p;
    p.kp = 1.0f; // the array sits ahead of the axle, which already damps the loop
    p.kd = 0.0f;
    p.steerMaxDeg = 25.0f;
    p.speedMmS = 200.0f;
    p.accelMmS2 = 1500.0f;
    p.slowdownErrMm = 30.0f;
    p.slowdownPct = 50;
    p.searchRateDps = 90.0f;
    p.lostTimeoutMs = 0;
    p.endZoneMm = 40.0f;
    p.arrayAheadMm = 60.0f;
    p.exitMm = 80.0f;
    p.rampTiltDeg = 15.0f;
    return p;
}

struct LineRig {
    Sim sim;
    MotionControl mc;
    MotionControlParams cp = simControlParams();
    LineParams p = lineParams();
    LineWorld world;
    LineFollow lf;

    void begin(bool alreadyOnLine = true)
    {
        lineFollowBegin(lf, sim.feedback(), alreadyOnLine);
    }

    LineStatus tick()
    {
        uint8_t norm[IR_CHANNEL_COUNT];
        uint8_t mask = 0;
        world.read(sim.distMm, sim.yMm, sim.headingDeg, norm, mask);
        const LineView v = lineView(norm, mask, 0xFF, PITCH);
        MotionTarget t;
        const LineStatus st = lineFollowStep(lf, p, sim.feedback(), v, world.tilt, false, DT_MS, t);
        sim.step(motionControlStep(mc, cp, t, sim.feedback(), DT_MS), DT_S);
        return st;
    }
};

static void test_converges_onto_the_line()
{
    LineRig r;
    r.sim.yMm = 20.0f; // line is to the right of the array centre (line y=0, robot y=+20)
    r.begin();
    for (int i = 0; i < 200; ++i) {
        r.tick();
    }
    TEST_ASSERT_FLOAT_WITHIN(8.0f, 0.0f, r.sim.yMm);
    TEST_ASSERT_TRUE(r.sim.speedMmS > 150.0f);
}

static void test_converges_from_the_other_side_too()
{
    LineRig r;
    r.sim.yMm = -20.0f;
    r.begin();
    for (int i = 0; i < 200; ++i) {
        r.tick();
    }
    TEST_ASSERT_FLOAT_WITHIN(8.0f, 0.0f, r.sim.yMm);
}

static void test_slows_down_for_a_large_error()
{
    LineRig r;
    r.sim.yMm = 35.0f; // channel 7 still reads the line: error -35 mm, past slowdownErrMm
    r.begin();
    float peak = 0.0f;
    for (int i = 0; i < 10; ++i) { // before the error has had time to shrink
        r.tick();
        peak = r.sim.speedMmS > peak ? r.sim.speedMmS : peak;
    }
    TEST_ASSERT_TRUE(peak < 120.0f); // 200 * 0.5 = 100 at full slowdown
}

static void test_stops_after_the_zone_ends()
{
    LineRig r;
    r.world.lineEndX = 300.0f;
    r.begin();
    LineStatus st = LineStatus::Following;
    for (int i = 0; i < 600 && st != LineStatus::Ended; ++i) {
        st = r.tick();
    }
    TEST_ASSERT_EQUAL(asInt(LineStatus::Ended), asInt(st));
    // The line ends at lineEndX under the array; the axle drives on exitMm past that point, into
    // Section B, and stops there. (Not at the end of the line: the axle is arrayAheadMm behind
    // the array, so stopping when the array sees floor would leave the robot on the line.)
    const float stopAt = r.world.lineEndX + r.p.exitMm;
    TEST_ASSERT_FLOAT_WITHIN(12.0f, stopAt, r.sim.distMm);
    TEST_ASSERT_TRUE(r.sim.distMm > r.world.lineEndX + r.p.exitMm * 0.8f);
    TEST_ASSERT_TRUE(r.sim.speedMmS < 15.0f);
    // Stays ended.
    TEST_ASSERT_EQUAL(asInt(LineStatus::Ended), asInt(r.tick()));
}

static void test_exit_is_cut_short_by_something_close_ahead()
{
    LineRig r;
    r.world.lineEndX = 300.0f;
    r.begin();
    LineStatus st = LineStatus::Following;
    for (int i = 0; i < 600 && st != LineStatus::Ended; ++i) {
        uint8_t norm[IR_CHANNEL_COUNT];
        uint8_t mask = 0;
        r.world.read(r.sim.distMm, r.sim.yMm, r.sim.headingDeg, norm, mask);
        const LineView v = lineView(norm, mask, 0xFF, PITCH);
        // A wall appears right after the line ends.
        const bool blocked = r.sim.distMm > r.world.lineEndX + 20.0f;
        MotionTarget t;
        st = lineFollowStep(r.lf, r.p, r.sim.feedback(), v, 0.0f, blocked, DT_MS, t);
        r.sim.step(motionControlStep(r.mc, r.cp, t, r.sim.feedback(), DT_MS), DT_S);
    }
    TEST_ASSERT_EQUAL(asInt(LineStatus::Ended), asInt(st));
    TEST_ASSERT_TRUE(r.sim.distMm < r.world.lineEndX + r.p.exitMm - 15.0f); // stopped short
}

static void test_bare_floor_before_the_line_is_not_the_end()
{
    LineRig r;
    r.world.lineEndX = -1.0f; // everything is floor: never any line
    r.begin(false);
    LineStatus st = LineStatus::Following;
    for (int i = 0; i < 300; ++i) {
        st = r.tick();
    }
    TEST_ASSERT_EQUAL(asInt(LineStatus::Following), asInt(st));
    TEST_ASSERT_TRUE(r.sim.distMm > 300.0f); // kept driving
}

static void test_ramp_lift_does_not_end_the_zone()
{
    LineRig r;
    r.begin();
    r.world.lift = true; // array lifted at the crest: reads black
    r.world.tilt = 30.0f;
    LineStatus st = LineStatus::Following;
    for (int i = 0; i < 300; ++i) {
        st = r.tick();
    }
    TEST_ASSERT_EQUAL(asInt(LineStatus::Following), asInt(st));
    // Back on the level and still black: now it is the real end.
    r.world.tilt = 0.0f;
    for (int i = 0; i < 300 && st != LineStatus::Ended; ++i) {
        st = r.tick();
    }
    TEST_ASSERT_EQUAL(asInt(LineStatus::Ended), asInt(st));
}

static void test_lost_line_searches_toward_the_last_side()
{
    LineRig r;
    r.begin();
    // See the line on the left for a moment...
    {
        uint8_t norm[IR_CHANNEL_COUNT] = {255, 255, 0, 0, 0, 0, 0, 0};
        const LineView v = lineView(norm, 0b00000011, 0xFF, PITCH);
        MotionTarget t;
        lineFollowStep(r.lf, r.p, r.sim.feedback(), v, 0.0f, false, DT_MS, t);
    }
    // ...then nothing.
    uint8_t white[IR_CHANNEL_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0};
    const LineView lost = lineView(white, 0x00, 0xFF, PITCH);
    MotionTarget t;
    const LineStatus st = lineFollowStep(r.lf, r.p, r.sim.feedback(), lost, 0.0f, false, DT_MS, t);
    TEST_ASSERT_EQUAL(asInt(LineStatus::Searching), asInt(st));
    TEST_ASSERT_TRUE(t.yawRateRefDps > 0.0f); // CCW = toward the left
    TEST_ASSERT_TRUE(t.headingRefDeg > 0.0f);
}

static void test_lost_on_the_right_spins_clockwise()
{
    LineRig r;
    r.begin();
    uint8_t norm[IR_CHANNEL_COUNT] = {0, 0, 0, 0, 0, 0, 255, 255};
    MotionTarget t;
    lineFollowStep(r.lf, r.p, r.sim.feedback(), lineView(norm, 0b11000000, 0xFF, PITCH), 0.0f,
                   false, DT_MS, t);
    uint8_t white[IR_CHANNEL_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0};
    lineFollowStep(r.lf, r.p, r.sim.feedback(), lineView(white, 0x00, 0xFF, PITCH), 0.0f, false,
                   DT_MS, t);
    TEST_ASSERT_TRUE(t.yawRateRefDps < 0.0f);
}

static void test_search_resumes_when_the_line_returns()
{
    LineRig r;
    r.begin();
    uint8_t white[IR_CHANNEL_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0};
    MotionTarget t;
    TEST_ASSERT_EQUAL(
        asInt(LineStatus::Searching),
        asInt(lineFollowStep(r.lf, r.p, r.sim.feedback(), lineView(white, 0x00, 0xFF, PITCH), 0.0f,
                             false, DT_MS, t)));
    uint8_t on[IR_CHANNEL_COUNT] = {0, 0, 0, 255, 255, 0, 0, 0};
    TEST_ASSERT_EQUAL(
        asInt(LineStatus::Following),
        asInt(lineFollowStep(r.lf, r.p, r.sim.feedback(), lineView(on, 0b00011000, 0xFF, PITCH),
                             0.0f, false, DT_MS, t)));
}

static void test_search_gives_up_after_the_timeout()
{
    LineRig r;
    r.p.lostTimeoutMs = 100;
    r.begin();
    uint8_t white[IR_CHANNEL_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0};
    MotionTarget t;
    LineStatus st = LineStatus::Following;
    for (int i = 0; i < 20 && st != LineStatus::Lost; ++i) {
        st = lineFollowStep(r.lf, r.p, r.sim.feedback(), lineView(white, 0x00, 0xFF, PITCH), 0.0f,
                            false, DT_MS, t);
    }
    TEST_ASSERT_EQUAL(asInt(LineStatus::Lost), asInt(st));
    // Stays lost.
    TEST_ASSERT_EQUAL(
        asInt(LineStatus::Lost),
        asInt(lineFollowStep(r.lf, r.p, r.sim.feedback(), lineView(white, 0x00, 0xFF, PITCH), 0.0f,
                             false, DT_MS, t)));
}

static void test_all_white_on_the_ramp_is_not_a_lost_line()
{
    LineRig r;
    r.begin();
    uint8_t white[IR_CHANNEL_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0};
    MotionTarget t;
    const LineStatus st = lineFollowStep(
        r.lf, r.p, r.sim.feedback(), lineView(white, 0x00, 0xFF, PITCH), 40.0f, false, DT_MS, t);
    TEST_ASSERT_EQUAL(asInt(LineStatus::Following), asInt(st));
}

static void test_begin_carries_the_current_speed()
{
    LineRig r;
    r.sim.speedMmS = 120.0f;
    r.begin();
    TEST_ASSERT_EQUAL_FLOAT(120.0f, r.lf.refSpeedMmS);
}

static void test_derivative_term_acts_on_a_changing_error()
{
    LineRig r;
    r.p.kp = 0.0f;
    r.p.kd = 0.01f;
    r.begin();
    uint8_t a[IR_CHANNEL_COUNT] = {0, 0, 0, 255, 255, 0, 0, 0};
    uint8_t b[IR_CHANNEL_COUNT] = {0, 0, 255, 255, 0, 0, 0, 0};
    MotionTarget t;
    lineFollowStep(r.lf, r.p, r.sim.feedback(), lineView(a, 0b00011000, 0xFF, PITCH), 0.0f, false,
                   DT_MS, t);
    lineFollowStep(r.lf, r.p, r.sim.feedback(), lineView(b, 0b00001100, 0xFF, PITCH), 0.0f, false,
                   DT_MS, t);
    TEST_ASSERT_TRUE(t.headingRefDeg > 0.0f); // error grew to the left
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
    RUN_TEST(test_marker_is_a_short_run);
    RUN_TEST(test_a_long_run_is_the_line_zone_reported_once);
    RUN_TEST(test_zone_can_be_found_again_after_the_floor_returns);
    RUN_TEST(test_runs_outside_the_marker_window_are_ignored);
    RUN_TEST(test_a_run_in_progress_at_power_up_is_not_measured);
    RUN_TEST(test_a_glitch_inside_a_run_does_not_split_it);
    RUN_TEST(test_run_start_is_reported_when_white_begins);
    RUN_TEST(test_a_line_crossing_a_white_tile_does_not_split_it);
    RUN_TEST(test_the_marker_still_ends_at_the_floor_between_it_and_the_bridge_tile);
    RUN_TEST(test_reset_forgets_the_floor);
    RUN_TEST(test_converges_onto_the_line);
    RUN_TEST(test_converges_from_the_other_side_too);
    RUN_TEST(test_slows_down_for_a_large_error);
    RUN_TEST(test_stops_after_the_zone_ends);
    RUN_TEST(test_exit_is_cut_short_by_something_close_ahead);
    RUN_TEST(test_bare_floor_before_the_line_is_not_the_end);
    RUN_TEST(test_ramp_lift_does_not_end_the_zone);
    RUN_TEST(test_lost_line_searches_toward_the_last_side);
    RUN_TEST(test_lost_on_the_right_spins_clockwise);
    RUN_TEST(test_search_resumes_when_the_line_returns);
    RUN_TEST(test_search_gives_up_after_the_timeout);
    RUN_TEST(test_all_white_on_the_ramp_is_not_a_lost_line);
    RUN_TEST(test_begin_carries_the_current_speed);
    RUN_TEST(test_derivative_term_acts_on_a_changing_error);
    return UNITY_END();
}
