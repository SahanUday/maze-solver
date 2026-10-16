// ============================================================================
//  test_step_cell.cpp  --  StepCell in a simulated corridor. No hardware needed.
//
//  Corridor along +x: side walls at +-HALF mm of the centre line, a front wall at wallX.
// ============================================================================

#include <initializer_list>
#include <unity.h>

#include "mission/StepCell.h"
#include "../support/sim.h"

constexpr uint16_t DT_MS = 10;
constexpr float DT_S = 0.01f;
constexpr float HALF = 120.0f;

struct World {
    bool leftWall = true;
    bool rightWall = true;
    float wallX = -1.0f;        // front wall along the track; < 0 = open ahead
    bool echo = true;           // false: no sensor hears anything
    float frontJitterMm = 0.0f; // the front alternates +- this, a new value every 60 ms
    int glitchTick = -1;        // the front reads glitchMm on this one tick
    float frontDeadMm = 0.0f;   // no front echo when the wall is nearer than this (the dead zone)
    int echoFromTick = 0;       // no front echo before this tick
    uint16_t glitchMm = 40;
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
        p.plan.patienceMs = 5000;
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
        p.snapMm = 100.0f;
    }

    WallView view() const
    {
        RobotState s;
        s.usValid = world.echo ? 0x07 : 0x00;
        if (world.wallX >= 0.0f && world.wallX - sim.distMm < world.frontDeadMm) {
            s.usValid &= static_cast<uint8_t>(~0x01);
        }
        if (ticks < world.echoFromTick) {
            s.usValid &= static_cast<uint8_t>(~0x01);
        }
        const float jitter = ((ticks / 6) % 2 != 0) ? world.frontJitterMm : -world.frontJitterMm;
        s.distFrontMm = static_cast<uint16_t>(
            world.wallX >= 0.0f ? world.wallX - sim.distMm + jitter : 2000.0f);
        if (ticks == world.glitchTick) {
            s.distFrontMm = world.glitchMm;
        }
        s.distLeftMm = static_cast<uint16_t>(world.leftWall ? HALF - sim.yMm : 400.0f);
        s.distRightMm = static_cast<uint16_t>(world.rightWall ? HALF + sim.yMm : 400.0f);
        return wallView(s, p.wall);
    }

    void begin(float headingRef = 0.0f)
    {
        stepCellBegin(sc, p, sim.feedback(), headingRef);
    }

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
    r.begin(); // open ahead: nothing to anchor on, the profile ends on the pitch
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
    TEST_ASSERT_TRUE(r.sc.walls.frontHeard);
    TEST_ASSERT_TRUE(r.sc.walls.leftHeard);
    TEST_ASSERT_TRUE(r.sc.walls.rightHeard);
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
    // The odometer reads 10 mm short: the centre by the wall is at 260, not 250.
    r.world.wallX = 260.0f + 125.0f;
    r.begin();
    MoveStatus st = MoveStatus::Running;
    for (int i = 0; i < 600 && st == MoveStatus::Running; ++i) {
        st = r.tick();
    }
    TEST_ASSERT_EQUAL(asInt(MoveStatus::Done), asInt(st));
    TEST_ASSERT_FLOAT_WITHIN(3.0f, 260.0f, r.sim.distMm);
}

// 110 and 119 mm alternated at a dead end on the robot: the stop target must hold still.
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
    // 200 mm "too far" is not a correction.
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
    // A wall 140 mm ahead, well before the centre: stop short of it.
    r.world.wallX = 140.0f + 125.0f; // front reads 265 at the start; the stop point is 125 away
    r.p.sampleLeadMm = 0.0f;
    r.p.sampleWindowMm = 0.0f;
    r.begin();
    MoveStatus st = MoveStatus::Running;
    float last = 0.0f;
    float backtrack = 0.0f;
    for (int i = 0; i < 600 && st == MoveStatus::Running; ++i) {
        st = r.tick();
        backtrack = last - r.sim.distMm > backtrack ? last - r.sim.distMm : backtrack;
        last = r.sim.distMm;
    }
    TEST_ASSERT_EQUAL(asInt(MoveStatus::Done), asInt(st));
    TEST_ASSERT_TRUE(backtrack < 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(6.0f, 140.0f, r.sim.distMm);
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
    r.sim.yMm = -20.0f; // right of centre: the left wall reads 140, steer left
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
    TEST_ASSERT_FALSE(r.sc.walls.frontHeard);
    TEST_ASSERT_FALSE(r.sc.walls.leftHeard);
    TEST_ASSERT_FALSE(r.sc.walls.rightHeard);
}

static void test_only_the_silent_sensor_is_not_heard()
{
    Rig r;
    r.world.wallX = 250.0f + 125.0f;
    r.world.frontDeadMm = 1000.0f; // the front hears nothing at any distance
    r.p.blindMs = 0;               // go on by the odometer
    r.begin();
    for (int i = 0; i < 600 && !r.sc.wallsReady; ++i) {
        r.tick();
    }
    TEST_ASSERT_TRUE(r.sc.wallsReady);
    TEST_ASSERT_FALSE(r.sc.walls.frontHeard);
    TEST_ASSERT_TRUE(r.sc.walls.front); // reported as a wall, the safe answer
    TEST_ASSERT_TRUE(r.sc.walls.leftHeard);
    TEST_ASSERT_TRUE(r.sc.walls.rightHeard);
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
    TEST_ASSERT_FALSE(unreliable); // a missed echo among heard ones is not a silent sensor
    uint8_t allSilent[3] = {0, 0, 3};
    step_detail::verdict(allSilent, unreliable);
    TEST_ASSERT_TRUE(unreliable);
}

static void test_vote_counters_saturate()
{
    uint8_t v[3] = {255, 0, 0};
    step_detail::vote(v, true, true);
    TEST_ASSERT_EQUAL_UINT8(255, v[0]);
}

static void test_the_wall_bias_survives_into_the_next_cell()
{
    // The heading reference has not changed, so the next cell keeps the bias.
    Rig r;
    r.sc.steer.biasDeg = 2.0f;
    r.sc.steer.anchored = true;
    r.sc.steer.anchorDeg = 0.0f;
    r.begin();
    r.tick();
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 2.0f, r.sc.steer.biasDeg);
}

static void test_a_turn_before_the_cell_drops_the_wall_bias()
{
    Rig r;
    r.sc.steer.biasDeg = 2.0f;
    r.sc.steer.anchored = true;
    r.sc.steer.anchorDeg = 0.0f;
    r.sim.headingDeg = 90.0f; // a quarter turn has just ended
    r.begin(90.0f);
    r.tick();
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, r.sc.steer.biasDeg);
}

// One stray near echo in an open cell neither shortens the stop nor reverses the robot.
static void test_one_stray_near_echo_does_not_move_the_stop()
{
    for (int glitch : {40, 55, 70, 85, 100}) {
        Rig r;
        r.world.glitchTick = glitch;
        r.begin();
        MoveStatus st = MoveStatus::Running;
        float last = 0.0f;
        float backtrack = 0.0f;
        for (int i = 0; i < 600 && st == MoveStatus::Running; ++i) {
            st = r.tick();
            backtrack = last - r.sim.distMm > backtrack ? last - r.sim.distMm : backtrack;
            last = r.sim.distMm;
        }
        TEST_ASSERT_EQUAL(asInt(MoveStatus::Done), asInt(st));
        TEST_ASSERT_FLOAT_WITHIN(3.0f, 250.0f, r.sim.distMm);
        TEST_ASSERT_TRUE(backtrack < 1.0f);
        TEST_ASSERT_FALSE(r.sc.blocked);
    }
}

static void test_a_wall_that_stays_is_believed_after_the_second_echo()
{
    Rig r;
    r.world.wallX = 180.0f; // appears inside the stop distance well before the centre
    r.begin();
    MoveStatus st = MoveStatus::Running;
    for (int i = 0; i < 600 && st == MoveStatus::Running; ++i) {
        st = r.tick();
    }
    TEST_ASSERT_EQUAL(asInt(MoveStatus::Done), asInt(st));
    TEST_ASSERT_TRUE(r.sc.blocked);
    TEST_ASSERT_FLOAT_WITHIN(8.0f, 55.0f, r.sim.distMm); // the stop point, 125 mm from the wall
    TEST_ASSERT_TRUE(r.sim.distMm < 125.0f);
}

static void test_a_wall_inside_the_stop_distance_at_the_start_is_not_driven_into()
{
    Rig r;
    r.world.wallX = 100.0f; // already nearer than the stop point
    r.begin();
    MoveStatus st = MoveStatus::Running;
    for (int i = 0; i < 600 && st == MoveStatus::Running; ++i) {
        st = r.tick();
    }
    TEST_ASSERT_EQUAL(asInt(MoveStatus::Done), asInt(st));
    TEST_ASSERT_TRUE(r.sc.blocked);
    TEST_ASSERT_TRUE(r.sim.distMm < 5.0f);
}

static void test_a_small_correction_is_not_a_blocked_move()
{
    Rig r;
    r.world.wallX = 260.0f + 125.0f;
    r.begin();
    for (int i = 0; i < 600 && r.tick() == MoveStatus::Running;) {
        ++i;
    }
    TEST_ASSERT_FALSE(r.sc.blocked);
}

static void test_continue_clears_the_cell_it_leaves()
{
    Rig r;
    r.world.rightWall = false;
    r.begin();
    for (int i = 0; i < 600 && !r.sc.wallsReady; ++i) {
        r.tick();
    }
    TEST_ASSERT_TRUE(r.sc.wallsReady);
    TEST_ASSERT_TRUE(r.sc.walls.left);
    stepCellContinue(r.sc, r.p);
    TEST_ASSERT_FALSE(r.sc.wallsReady);
    TEST_ASSERT_FALSE(r.sc.walls.left);
    TEST_ASSERT_FALSE(r.sc.blocked);
}

static void test_the_params_default_to_the_spec()
{
    const StepCellParams d;
    TEST_ASSERT_EQUAL_FLOAT(TILE_PITCH_MM, d.pitchMm);
    TEST_ASSERT_EQUAL_FLOAT(SEARCH_SPEED_MM_S, d.cruiseMmS);
    TEST_ASSERT_EQUAL_FLOAT(WALL_SAMPLE_LEAD_MM, d.sampleLeadMm);
    TEST_ASSERT_EQUAL_FLOAT(WALL_SAMPLE_WINDOW_MM, d.sampleWindowMm);
    TEST_ASSERT_EQUAL_FLOAT(FRONT_STOP_MM, d.frontStopMm);
    TEST_ASSERT_EQUAL_FLOAT(FRONT_AGREE_MM, d.frontAgreeMm);
    TEST_ASSERT_EQUAL_FLOAT(FRONT_ANCHOR_FILTER, d.anchorFilter);
    TEST_ASSERT_EQUAL_FLOAT(US_SLOT_MS * US_SENSOR_COUNT, d.echoMs);
}

// The safety stop starts a braking distance and an echo early, so a wall that the verdict missed is
// stopped for at the stop point, not past it.
static void test_a_wall_the_verdict_missed_is_stopped_for_at_the_stop_point()
{
    Rig r;
    r.world.wallX = 250.0f + 125.0f;
    r.begin();
    MoveStatus st = MoveStatus::Running;
    bool answered = false;
    for (int i = 0; i < 1200 && st == MoveStatus::Running; ++i) {
        if (r.sc.wallsReady && !answered) {
            stepCellContinue(r.sc, r.p); // the caller drives on regardless of the wall
            answered = true;
        }
        st = r.tick();
    }
    TEST_ASSERT_EQUAL(asInt(MoveStatus::Done), asInt(st));
    TEST_ASSERT_FLOAT_WITHIN(8.0f, 125.0f, 375.0f - r.sim.distMm);
}

// The wall stays where it was seen when the sensor goes quiet: the dead zone must not make it
// vanish.
static void test_a_wall_is_still_stopped_for_when_the_sensor_goes_silent()
{
    Rig r;
    r.world.wallX = 250.0f + 125.0f;
    r.world.frontDeadMm = 190.0f; // blind from before the safety trigger
    r.begin();
    MoveStatus st = MoveStatus::Running;
    bool answered = false;
    for (int i = 0; i < 1200 && st == MoveStatus::Running; ++i) {
        if (r.sc.wallsReady && !answered) {
            stepCellContinue(r.sc, r.p); // drives on regardless of the wall
            answered = true;
        }
        st = r.tick();
    }
    TEST_ASSERT_EQUAL(asInt(MoveStatus::Done), asInt(st));
    TEST_ASSERT_FLOAT_WITHIN(12.0f, 125.0f, 375.0f - r.sim.distMm);
}

static void test_a_move_that_begins_in_the_dead_zone_does_not_drive_into_the_wall()
{
    Rig r;
    r.world.wallX = 100.0f;
    r.world.frontDeadMm = 140.0f;
    r.begin();
    MoveStatus st = MoveStatus::Running;
    for (int i = 0; i < 600 && st == MoveStatus::Running; ++i) {
        st = r.tick();
    }
    TEST_ASSERT_EQUAL(asInt(MoveStatus::Done), asInt(st));
    TEST_ASSERT_TRUE(r.sc.blocked);
    TEST_ASSERT_TRUE(r.sim.distMm < 3.0f);
}

static void test_a_move_waits_for_its_first_echo_then_goes()
{
    Rig r;
    r.world.echoFromTick = 15; // the sensor answers 150 ms late
    r.begin();
    MoveStatus st = MoveStatus::Running;
    for (int i = 0; i < 14; ++i) {
        r.tick();
    }
    TEST_ASSERT_TRUE(r.sim.distMm < 0.5f); // held
    for (int i = 0; i < 600 && st == MoveStatus::Running; ++i) {
        st = r.tick();
    }
    TEST_ASSERT_EQUAL(asInt(MoveStatus::Done), asInt(st));
    TEST_ASSERT_FALSE(r.sc.blocked);
    TEST_ASSERT_FLOAT_WITHIN(3.0f, 250.0f, r.sim.distMm);
}

static void test_a_dead_front_sensor_blocks_unless_waiting_is_turned_off()
{
    Rig r;
    r.world.echoFromTick = 100000;
    r.p.blindMs = 0;
    r.begin();
    MoveStatus st = MoveStatus::Running;
    for (int i = 0; i < 600 && st == MoveStatus::Running; ++i) {
        st = r.tick();
    }
    TEST_ASSERT_FALSE(r.sc.blocked); // the caller chose to go blind: it does, by odometry
    TEST_ASSERT_FLOAT_WITHIN(3.0f, 250.0f, r.sim.distMm);
}

// A way known to be open (a side sensor measured it) does not wait for a front echo that will
// never come: the silence is an exit with nothing beyond, not a wall in the dead zone.
static void test_a_known_open_way_does_not_wait_for_a_front_echo()
{
    Rig r;
    r.world.echoFromTick = 100000; // nothing is ever heard in front
    r.begin();
    r.sc.openAhead = true;
    MoveStatus st = MoveStatus::Running;
    for (int i = 0; i < 600 && st == MoveStatus::Running; ++i) {
        st = r.tick();
    }
    TEST_ASSERT_EQUAL(asInt(MoveStatus::Done), asInt(st));
    TEST_ASSERT_FALSE(r.sc.blocked);
    TEST_ASSERT_FLOAT_WITHIN(3.0f, 250.0f, r.sim.distMm);
}

static void test_without_that_knowledge_a_silent_front_blocks_the_move()
{
    Rig r;
    r.world.echoFromTick = 100000;
    r.begin();
    MoveStatus st = MoveStatus::Running;
    for (int i = 0; i < 600 && st == MoveStatus::Running; ++i) {
        st = r.tick();
    }
    TEST_ASSERT_TRUE(r.sc.blocked);
}

// Centred between both walls the bias the steer learned is the heading frame's error: handed back.
static void test_the_learned_bias_is_handed_back_as_a_heading_fix()
{
    Rig r;
    r.sim.gyroOffsetDeg = 5.0f; // the gyro reads 5 degrees more than the true heading
    r.sim.headingDeg = 5.0f;
    r.p.pitchMm = 3000.0f;
    r.begin();
    float taken = 0.0f;
    for (int i = 0; i < 1500; ++i) {
        r.tick();
        const float fix = stepCellTakeHeadingFix(r.sc);
        r.sim.headingDeg -= fix;
        r.sim.gyroOffsetDeg -= fix;
        taken += fix;
    }
    TEST_ASSERT_TRUE(taken > 3.0f);
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 0.0f, r.sim.gyroOffsetDeg);
    TEST_ASSERT_FLOAT_WITHIN(6.0f, 0.0f, r.sim.yMm);
}

static void test_one_wall_teaches_the_heading_frame_too()
{
    Rig r;
    r.world.rightWall = false;
    r.sim.gyroOffsetDeg = 5.0f;
    r.sim.headingDeg = 5.0f;
    r.p.pitchMm = 3000.0f;
    r.begin();
    for (int i = 0; i < 1500; ++i) {
        r.tick();
        const float fix = stepCellTakeHeadingFix(r.sc);
        r.sim.headingDeg -= fix;
        r.sim.gyroOffsetDeg -= fix;
    }
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 0.0f, r.sim.gyroOffsetDeg);
}

static void test_no_wall_teaches_nothing()
{
    Rig r;
    r.world.leftWall = false;
    r.world.rightWall = false;
    r.sim.gyroOffsetDeg = 5.0f;
    r.sim.headingDeg = 5.0f;
    r.p.pitchMm = 3000.0f;
    r.begin();
    float taken = 0.0f;
    for (int i = 0; i < 1500; ++i) {
        r.tick();
        taken += stepCellTakeHeadingFix(r.sc);
    }
    TEST_ASSERT_EQUAL_FLOAT(0.0f, taken);
}

static void test_a_frame_error_too_big_to_be_the_gyro_is_left_alone()
{
    Rig r;
    r.sim.gyroOffsetDeg = 9.0f; // over snapMaxDeg: the corridor is not square to the grid
    r.sim.headingDeg = 9.0f;
    r.p.pitchMm = 3000.0f;
    r.begin();
    float taken = 0.0f;
    for (int i = 0; i < 1500; ++i) {
        r.tick();
        taken += stepCellTakeHeadingFix(r.sc);
    }
    TEST_ASSERT_EQUAL_FLOAT(0.0f, taken);
}

static void test_cells_covered_and_the_remaining_distance_come_from_the_odometer()
{
    Rig r;
    r.begin();
    r.sim.distMm = 100.0f;
    TEST_ASSERT_EQUAL(0, stepCellCells(r.sc, r.p, r.sim.feedback()));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 150.0f, stepCellRemainingMm(r.sc, r.sim.feedback()));
    r.sim.distMm = 130.0f;
    TEST_ASSERT_EQUAL(1, stepCellCells(r.sc, r.p, r.sim.feedback()));
    stepCellContinue(r.sc, r.p);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 370.0f, stepCellRemainingMm(r.sc, r.sim.feedback()));
    r.sim.distMm = -130.0f; // backwards counts too
    TEST_ASSERT_EQUAL(-1, stepCellCells(r.sc, r.p, r.sim.feedback()));
}

static void test_a_blocked_move_can_resume_to_its_planned_centre()
{
    Rig r;
    r.begin();
    r.sim.distMm = 100.0f; // stopped short by a stray pair of echoes
    stepCellResume(r.sc, r.p, r.sim.feedback(), 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 250.0f, r.sc.straight.targetMm);
    TEST_ASSERT_FALSE(r.sc.blocked);
    r.sim.distMm = 249.0f;
    TEST_ASSERT_EQUAL(1,
                      stepCellCells(r.sc, r.p, r.sim.feedback())); // counted from the first begin
}

static void test_a_move_that_cannot_start_is_stuck()
{
    Rig r;
    r.p.plan.accelMmS2 = 0.0f;
    r.p.plan.patienceMs = 200;
    r.begin();
    MoveStatus st = MoveStatus::Running;
    for (int i = 0; i < 100 && st == MoveStatus::Running; ++i) {
        st = r.tick();
    }
    TEST_ASSERT_EQUAL(asInt(MoveStatus::Stuck), asInt(st));
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
    RUN_TEST(test_only_the_silent_sensor_is_not_heard);
    RUN_TEST(test_verdict_follows_the_majority);
    RUN_TEST(test_vote_counters_saturate);
    RUN_TEST(test_the_wall_bias_survives_into_the_next_cell);
    RUN_TEST(test_a_turn_before_the_cell_drops_the_wall_bias);
    RUN_TEST(test_one_stray_near_echo_does_not_move_the_stop);
    RUN_TEST(test_a_wall_that_stays_is_believed_after_the_second_echo);
    RUN_TEST(test_a_wall_inside_the_stop_distance_at_the_start_is_not_driven_into);
    RUN_TEST(test_a_small_correction_is_not_a_blocked_move);
    RUN_TEST(test_continue_clears_the_cell_it_leaves);
    RUN_TEST(test_the_params_default_to_the_spec);
    RUN_TEST(test_a_wall_the_verdict_missed_is_stopped_for_at_the_stop_point);
    RUN_TEST(test_a_wall_is_still_stopped_for_when_the_sensor_goes_silent);
    RUN_TEST(test_a_move_that_begins_in_the_dead_zone_does_not_drive_into_the_wall);
    RUN_TEST(test_a_move_waits_for_its_first_echo_then_goes);
    RUN_TEST(test_a_dead_front_sensor_blocks_unless_waiting_is_turned_off);
    RUN_TEST(test_the_learned_bias_is_handed_back_as_a_heading_fix);
    RUN_TEST(test_one_wall_teaches_the_heading_frame_too);
    RUN_TEST(test_no_wall_teaches_nothing);
    RUN_TEST(test_a_frame_error_too_big_to_be_the_gyro_is_left_alone);
    RUN_TEST(test_cells_covered_and_the_remaining_distance_come_from_the_odometer);
    RUN_TEST(test_a_blocked_move_can_resume_to_its_planned_centre);
    RUN_TEST(test_a_move_that_cannot_start_is_stuck);
    RUN_TEST(test_a_known_open_way_does_not_wait_for_a_front_echo);
    RUN_TEST(test_without_that_knowledge_a_silent_front_blocks_the_move);
    return UNITY_END();
}
