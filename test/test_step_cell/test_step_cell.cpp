// ============================================================================
//  test_step_cell.cpp  --  StepCell in a simulated corridor. No hardware needed.
//
//  The corridor runs along +x. Walls, if present, sit at +-HALF mm from the centre line; the
//  front wall, if present, at wallX. Sensors read distances to them from the robot.
// ============================================================================

#include <unity.h>

#include "StepCell.h"
#include "sim.h"

constexpr uint16_t DT_MS = 10;
constexpr float DT_S = 0.01f;
constexpr float HALF = 120.0f;

struct World {
    bool leftWall = true;
    bool rightWall = true;
    float wallX = -1.0f;        // front wall position along the track; < 0 = none (open ahead)
    bool echo = true;           // false: nothing hears anything
    float frontJitterMm = 0.0f; // the front reading alternates +- this, a new value every 60 ms
};

struct Rig {
    Sim sim;
    MotionControl mc;
    MotionControlParams cp = simControlParams();
    StepCellParams p;
    World world;
    StepCell sc;
    int ticks = 0;

    Rig()
    {
        p.plan = simPlanParams();
        p.plan.moveTimeoutMs = 5000;
        p.wall.sideMm = 200;
        p.wall.frontMm = 280;
        p.wall.centerDistMm = 120;
        p.wall.kp = 0.3f;
        p.wall.steerMaxDeg = 15.0f;
        p.pitchMm = 250.0f;
        p.cruiseMmS = 300.0f;
        p.sampleLeadMm = 60.0f;
        p.sampleWindowMm = 30.0f;
        p.frontStopMm = 125.0f;
    }

    WallView view() const
    {
        RobotState s;
        s.usValid = world.echo ? 0x07 : 0x00;
        const float jitter = ((ticks / 6) % 2 != 0) ? world.frontJitterMm : -world.frontJitterMm;
        s.distFrontMm = static_cast<uint16_t>(
            world.wallX >= 0.0f ? world.wallX - sim.distMm + jitter : 2000.0f);
        s.distLeftMm = static_cast<uint16_t>(world.leftWall ? HALF - sim.yMm : 400.0f);
        s.distRightMm = static_cast<uint16_t>(world.rightWall ? HALF + sim.yMm : 400.0f);
        return wallView(s, p.wall);
    }

    void begin(float headingRef = 0.0f)
    {
        stepCellBegin(sc, p, sim.feedback(), headingRef);
    }

    // One tick; returns the move status.
    MoveStatus tick()
    {
        MotionTarget t;
        const MoveStatus st = stepCellStep(sc, p, sim.feedback(), view(), DT_MS, t);
        ++ticks;
        sim.step(motionControlStep(mc, cp, t, sim.feedback(), DT_MS), DT_S);
        return st;
    }
};

static int asInt(MoveStatus s)
{
    return static_cast<int>(s);
}

static void test_stops_on_the_next_centre_when_nobody_answers()
{
    Rig r;
    r.world.wallX = 375.0f; // wall ahead of the NEXT cell too: nothing to anchor on this one
    r.world.wallX = -1.0f;
    r.begin();
    MoveStatus st = MoveStatus::Running;
    for (int i = 0; i < 600 && st == MoveStatus::Running; ++i) {
        st = r.tick();
    }
    TEST_ASSERT_EQUAL(asInt(MoveStatus::Done), asInt(st));
    TEST_ASSERT_FLOAT_WITHIN(3.0f, 250.0f, r.sim.distMm);
}

static void test_walls_are_ready_before_the_centre()
{
    Rig r;
    r.world.rightWall = false;
    r.world.wallX = 250.0f + 125.0f; // front wall of the next cell
    r.begin();
    float readyAt = -1.0f;
    int readyCount = 0;
    for (int i = 0; i < 600; ++i) {
        const bool before = r.sc.wallsReady;
        if (r.tick() != MoveStatus::Running) {
            break;
        }
        if (r.sc.wallsReady && !before) {
            readyAt = r.sim.distMm;
            ++readyCount;
        }
    }
    TEST_ASSERT_EQUAL(1, readyCount);
    TEST_ASSERT_FLOAT_WITHIN(15.0f, 250.0f - 60.0f, readyAt);
    TEST_ASSERT_TRUE(r.sc.walls.front);
    TEST_ASSERT_TRUE(r.sc.walls.left);
    TEST_ASSERT_FALSE(r.sc.walls.right);
    TEST_ASSERT_FALSE(r.sc.walls.unreliable);
}

static void test_open_front_is_reported_open()
{
    Rig r;
    r.begin();
    for (int i = 0; i < 600 && !r.sc.wallsReady; ++i) {
        r.tick();
    }
    TEST_ASSERT_TRUE(r.sc.wallsReady);
    TEST_ASSERT_FALSE(r.sc.walls.front);
}

static void test_continue_drives_through_without_stopping()
{
    Rig r;
    r.begin();
    float minSpeedNearCentre = 1e9f;
    bool continued = false;
    MoveStatus st = MoveStatus::Running;
    for (int i = 0; i < 1200 && st == MoveStatus::Running; ++i) {
        if (r.sc.wallsReady && !continued) {
            stepCellContinue(r.sc, r.p); // front is open: go on
            continued = true;
        }
        st = r.tick();
        if (r.sim.distMm > 220.0f && r.sim.distMm < 280.0f) {
            minSpeedNearCentre =
                r.sim.speedMmS < minSpeedNearCentre ? r.sim.speedMmS : minSpeedNearCentre;
        }
    }
    TEST_ASSERT_TRUE(continued);
    TEST_ASSERT_EQUAL(asInt(MoveStatus::Done), asInt(st));
    TEST_ASSERT_FLOAT_WITHIN(3.0f, 500.0f, r.sim.distMm);
    TEST_ASSERT_TRUE(minSpeedNearCentre > 150.0f); // rolled through the first centre
}

static void test_continue_samples_the_next_cell_again()
{
    Rig r;
    r.begin();
    int ready = 0;
    bool continued = false;
    MoveStatus st = MoveStatus::Running;
    for (int i = 0; i < 1200 && st == MoveStatus::Running; ++i) {
        const bool before = r.sc.wallsReady;
        if (r.sc.wallsReady && !continued) {
            stepCellContinue(r.sc, r.p);
            continued = true;
        }
        st = r.tick();
        if (r.sc.wallsReady && !before) {
            ++ready;
        }
    }
    TEST_ASSERT_EQUAL(2, ready);
}

static void test_front_wall_anchors_the_stop_and_cancels_drift()
{
    Rig r;
    // The odometer is 10 mm short: the real wall is 10 mm further than the model thinks, so
    // the centre by the wall is at 260, not 250.
    r.world.wallX = 260.0f + 125.0f;
    r.begin();
    MoveStatus st = MoveStatus::Running;
    for (int i = 0; i < 600 && st == MoveStatus::Running; ++i) {
        st = r.tick();
    }
    TEST_ASSERT_EQUAL(asInt(MoveStatus::Done), asInt(st));
    TEST_ASSERT_FLOAT_WITHIN(3.0f, 260.0f, r.sim.distMm);
}

// The front sensor jitters (110 and 119 mm alternated at a dead end on the robot, 9 mm apart).
// Retargeting to each reading kept the profile restarting, so the move never settled. With the
// robot standing at the wall the stop target must hold still.
static void test_the_stop_target_does_not_follow_a_jittery_front_reading()
{
    Rig r;
    r.world.wallX = 250.0f + 125.0f;
    r.world.frontJitterMm = 4.5f;
    r.begin();
    r.sim.distMm = 247.0f; // standing at the wall
    float lo = 1e9f;
    float hi = -1e9f;
    for (int i = 0; i < 300; ++i) {
        MotionTarget t;
        stepCellStep(r.sc, r.p, r.sim.feedback(), r.view(), DT_MS, t);
        ++r.ticks;
        if (i >= 60) { // after the first few readings
            lo = r.sc.straight.targetMm < lo ? r.sc.straight.targetMm : lo;
            hi = r.sc.straight.targetMm > hi ? r.sc.straight.targetMm : hi;
        }
    }
    TEST_ASSERT_TRUE(hi - lo < 4.0f); // unfiltered it swings by 9 mm
}

static void test_absurd_front_correction_is_ignored()
{
    Rig r;
    // A wall 200 mm "too far": a reading like that is not a correction, it is something else.
    r.world.wallX = 250.0f + 125.0f + 200.0f;
    r.begin();
    MoveStatus st = MoveStatus::Running;
    for (int i = 0; i < 600 && st == MoveStatus::Running; ++i) {
        st = r.tick();
    }
    TEST_ASSERT_EQUAL(asInt(MoveStatus::Done), asInt(st));
    TEST_ASSERT_FLOAT_WITHIN(3.0f, 250.0f, r.sim.distMm);
}

static void test_front_wall_closer_than_the_stop_distance_stops_now()
{
    Rig r;
    // A wall appears only 140 mm ahead of the start, well before the centre: do not drive into it.
    r.world.wallX = 140.0f + 125.0f; // front reads 265 at the start; the stop point is 125 away
    r.p.sampleLeadMm = 0.0f;
    r.p.sampleWindowMm = 0.0f;
    r.begin();
    MoveStatus st = MoveStatus::Running;
    for (int i = 0; i < 600 && st == MoveStatus::Running; ++i) {
        st = r.tick();
    }
    TEST_ASSERT_TRUE(r.sim.distMm < 250.0f);
}

static void test_centres_between_two_walls()
{
    Rig r;
    r.sim.yMm = 25.0f; // started off to the left
    r.p.pitchMm = 1000.0f;
    r.begin();
    for (int i = 0; i < 400; ++i) {
        r.tick();
    }
    TEST_ASSERT_FLOAT_WITHIN(6.0f, 0.0f, r.sim.yMm);
}

static void test_follows_a_single_wall()
{
    Rig r;
    r.world.rightWall = false;
    r.sim.yMm = -20.0f; // too close to... no: left wall reads 120 + 20 = 140, steer left
    r.p.pitchMm = 1000.0f;
    r.begin();
    for (int i = 0; i < 400; ++i) {
        r.tick();
    }
    TEST_ASSERT_FLOAT_WITHIN(6.0f, 0.0f, r.sim.yMm);
}

static void test_no_walls_holds_the_gyro_heading()
{
    Rig r;
    r.world.leftWall = false;
    r.world.rightWall = false;
    r.sim.headingDeg = 5.0f;
    r.begin(0.0f);
    for (int i = 0; i < 100; ++i) {
        r.tick();
    }
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, r.sim.headingDeg);
}

static void test_silent_sensors_read_as_walls_and_unreliable()
{
    Rig r;
    r.world.echo = false;
    r.begin();
    for (int i = 0; i < 600 && !r.sc.wallsReady; ++i) {
        r.tick();
    }
    TEST_ASSERT_TRUE(r.sc.wallsReady);
    TEST_ASSERT_TRUE(r.sc.walls.front);
    TEST_ASSERT_TRUE(r.sc.walls.left);
    TEST_ASSERT_TRUE(r.sc.walls.right);
    TEST_ASSERT_TRUE(r.sc.walls.unreliable);
}

static void test_verdict_follows_the_majority()
{
    uint8_t mostlyOpen[3] = {1, 5, 0};
    uint8_t mostlyWall[3] = {5, 1, 0};
    uint8_t tie[3] = {2, 2, 0};
    uint8_t onlyOpen[3] = {0, 3, 0};
    uint8_t nothing[3] = {0, 0, 0};
    bool unreliable = false;
    TEST_ASSERT_FALSE(step_detail::verdict(mostlyOpen, unreliable));
    TEST_ASSERT_TRUE(step_detail::verdict(mostlyWall, unreliable));
    TEST_ASSERT_TRUE(step_detail::verdict(tie, unreliable)); // a tie errs toward wall
    TEST_ASSERT_FALSE(step_detail::verdict(onlyOpen, unreliable));
    TEST_ASSERT_TRUE(step_detail::verdict(nothing, unreliable));
    TEST_ASSERT_FALSE(unreliable);
    uint8_t someSilent[3] = {0, 3, 1};
    step_detail::verdict(someSilent, unreliable);
    TEST_ASSERT_TRUE(unreliable);
}

static void test_vote_counters_saturate()
{
    uint8_t v[3] = {255, 0, 0};
    step_detail::vote(v, true, true);
    TEST_ASSERT_EQUAL_UINT8(255, v[0]);
}

static void test_stuck_move_times_out()
{
    Rig r;
    r.p.plan.accelMmS2 = 0.0f;
    r.p.plan.moveTimeoutMs = 200;
    r.begin();
    MoveStatus st = MoveStatus::Running;
    for (int i = 0; i < 100 && st == MoveStatus::Running; ++i) {
        st = r.tick();
    }
    TEST_ASSERT_EQUAL(asInt(MoveStatus::Timeout), asInt(st));
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_stops_on_the_next_centre_when_nobody_answers);
    RUN_TEST(test_walls_are_ready_before_the_centre);
    RUN_TEST(test_open_front_is_reported_open);
    RUN_TEST(test_continue_drives_through_without_stopping);
    RUN_TEST(test_continue_samples_the_next_cell_again);
    RUN_TEST(test_front_wall_anchors_the_stop_and_cancels_drift);
    RUN_TEST(test_the_stop_target_does_not_follow_a_jittery_front_reading);
    RUN_TEST(test_absurd_front_correction_is_ignored);
    RUN_TEST(test_front_wall_closer_than_the_stop_distance_stops_now);
    RUN_TEST(test_centres_between_two_walls);
    RUN_TEST(test_follows_a_single_wall);
    RUN_TEST(test_no_walls_holds_the_gyro_heading);
    RUN_TEST(test_silent_sensors_read_as_walls_and_unreliable);
    RUN_TEST(test_verdict_follows_the_majority);
    RUN_TEST(test_vote_counters_saturate);
    RUN_TEST(test_stuck_move_times_out);
    return UNITY_END();
}
