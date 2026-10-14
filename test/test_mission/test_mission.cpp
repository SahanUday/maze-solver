// ============================================================================
//  test_mission.cpp  --  the whole control stack against a simulated arena: a 4x4 maze with
//  walls, ultrasonic rays (one sensor read per 20 ms slot, like the real driver), encoders, an
//  IR array over a painted floor (start tile, approach marker, bridge tile with its line), and
//  a plant with a little wheel lag. The mission runs through RobotState -> Perception ->
//  missionStep -> DriveCommand exactly as main.cpp does. No hardware needed.
//
//  Coordinates: absolute mm, +y = abs North. Tile (ax, ay) spans [250 ax, 250 (ax + 1)] etc.
//  An absolute direction index runs counter-clockwise from North, like maze::Dir.
// ============================================================================

#include <math.h>
#include <unity.h>

#include "Mission.h"
#include "sim.h"

constexpr float TILE = 250.0f;
constexpr float RAD = 0.0174533f;
constexpr uint16_t DT_MS = 10;
constexpr float DT_S = 0.01f;
constexpr float IR_AHEAD = 70.0f;
constexpr float IR_PITCH = 9.0f;
constexpr float MM_PER_COUNT = 0.2f;

struct Wall {
    float x0, y0, x1, y1; // axis-aligned segment
};

// ---- Arena -------------------------------------------------------------------------

struct Arena {
    uint8_t open[4][4] = {}; // bit per abs dir: a passage out of that cell
    int8_t startX = 0, startY = 0;
    int8_t markerX = 0, markerY = 0;
    int8_t bridgeX = 0, bridgeY = 0;
    uint8_t bridgeDir = 0; // abs dir the bridge leaves in (through the outer wall)
    float lineLength = 600.0f;

    void link(int8_t ax, int8_t ay, char to)
    {
        const uint8_t d = to == 'N' ? 0 : 3;
        const int8_t bx = static_cast<int8_t>(ax + maze::kDx[d]);
        const int8_t by = static_cast<int8_t>(ay + maze::kDy[d]);
        open[ay][ax] = static_cast<uint8_t>(open[ay][ax] | (1u << d));
        open[by][bx] = static_cast<uint8_t>(open[by][bx] | (1u << ((d + 2) & 3)));
    }

    bool isOpen(int8_t ax, int8_t ay, uint8_t d) const
    {
        const bool bridgeSide = ax == bridgeX && ay == bridgeY && d == bridgeDir;
        return bridgeSide || (open[ay][ax] & (1u << d)) != 0;
    }

    // Distance along a ray from (x, y) in direction (dx, dy) to the nearest wall, or `cap`.
    float ray(float x, float y, float dx, float dy, float cap) const
    {
        float best = cap;
        for (int8_t ay = 0; ay < 4; ++ay) {
            for (int8_t ax = 0; ax < 4; ++ax) {
                for (uint8_t d = 0; d < 4; ++d) {
                    if (isOpen(ax, ay, d)) {
                        continue;
                    }
                    // The wall on side d of this tile.
                    Wall w;
                    const float x0 = TILE * ax, y0 = TILE * ay;
                    if (d == 0) {
                        w = {x0, y0 + TILE, x0 + TILE, y0 + TILE};
                    } else if (d == 2) {
                        w = {x0, y0, x0 + TILE, y0};
                    } else if (d == 1) {
                        w = {x0, y0, x0, y0 + TILE};
                    } else {
                        w = {x0 + TILE, y0, x0 + TILE, y0 + TILE};
                    }
                    float t = -1.0f;
                    if (w.y0 == w.y1 && dy != 0.0f) { // horizontal wall
                        t = (w.y0 - y) / dy;
                        const float hx = x + t * dx;
                        if (hx < w.x0 || hx > w.x1) {
                            t = -1.0f;
                        }
                    } else if (w.x0 == w.x1 && dx != 0.0f) { // vertical wall
                        t = (w.x0 - x) / dx;
                        const float hy = y + t * dy;
                        if (hy < w.y0 || hy > w.y1) {
                            t = -1.0f;
                        }
                    }
                    if (t > 0.0f && t < best) {
                        best = t;
                    }
                }
            }
        }
        return best;
    }

    // Is the floor black at (x, y)?
    bool floorBlack(float x, float y) const
    {
        const float sx = TILE * (startX + 0.5f), sy = TILE * (startY + 0.5f);
        if (fabsf(x - sx) <= TILE / 2 && fabsf(y - sy) <= TILE / 2) {
            return false; // the START tile is plain white
        }
        const float mx = TILE * (markerX + 0.5f), my = TILE * (markerY + 0.5f);
        if (fabsf(x - mx) <= 45.0f && fabsf(y - my) <= 45.0f) {
            return fabsf(x - mx) <= 15.0f && fabsf(y - my) <= 15.0f; // white 90, black 30
        }
        // Bridge tile and the corridor beyond: white with a black line along the bridge axis.
        const float bx = TILE * (bridgeX + 0.5f), by = TILE * (bridgeY + 0.5f);
        const float ux = static_cast<float>(maze::kDx[bridgeDir]);
        const float uy = static_cast<float>(maze::kDy[bridgeDir]);
        const float along = (x - bx) * ux + (y - by) * uy;
        const float lateral = fabsf((x - bx) * (-uy) + (y - by) * ux);
        if (along >= -TILE / 2 && along <= TILE / 2 + lineLength && lateral <= TILE / 2) {
            return lateral <= 15.0f;
        }
        return true;
    }
};

static Arena ringArena()
{
    Arena a;
    for (int8_t i = 0; i < 3; ++i) {
        a.link(i, 0, 'E');
        a.link(i, 3, 'E');
        a.link(0, i, 'N');
        a.link(3, i, 'N');
    }
    a.link(1, 0, 'N');
    a.link(1, 1, 'N');
    a.link(2, 2, 'N');
    a.link(2, 1, 'N');
    return a;
}

// ---- The world around the robot ------------------------------------------------------

struct Robot {
    Arena arena;
    Sim sim; // heading, speed, wheel lag; its y/dist are not used for position
    float x = 0.0f, y = 0.0f;
    float f0Deg = 0.0f; // abs angle of the robot's start heading, CCW from abs North
    float posL = 0.0f, posR = 0.0f;
    int tick = 0;
    uint16_t us[3] = {2000, 2000, 2000};
    LineCalibration cal{};

    // Place the robot on the centre of a tile facing abs direction `absDir`.
    void place(int8_t ax, int8_t ay, uint8_t absDir)
    {
        arena.startX = ax;
        arena.startY = ay;
        x = TILE * (ax + 0.5f);
        y = TILE * (ay + 0.5f);
        f0Deg = 90.0f * absDir;
        sim = Sim{};
        posL = posR = 0.0f;
        tick = 0;
        lineCalibrationReset(cal);
        uint16_t white[IR_CHANNEL_COUNT], black[IR_CHANNEL_COUNT];
        for (uint8_t ch = 0; ch < IR_CHANNEL_COUNT; ++ch) {
            white[ch] = 150;
            black[ch] = 1000;
        }
        lineCalibrationAccumulate(cal, white);
        lineCalibrationAccumulate(cal, black);
        lineCalibrationFinish(cal);
    }

    float theta() const
    {
        return (f0Deg + sim.headingDeg) * RAD; // CCW from abs North
    }

    // The sensors as the drivers would leave them in RobotState this tick.
    RobotState state(bool startPressed)
    {
        // One ultrasonic per 20 ms slot, round robin, holding the others' last readings.
        if (tick % 2 == 0) {
            const int which = (tick / 2) % 3; // 0 front, 1 left, 2 right
            const float th = theta() + (which == 1 ? 1.5708f : (which == 2 ? -1.5708f : 0.0f));
            const float d = arena.ray(x, y, -sinf(th), cosf(th), 2500.0f);
            us[which] = static_cast<uint16_t>(d);
        }
        RobotState s;
        s.timestampMs = static_cast<uint32_t>(tick) * DT_MS;
        s.encoderCountL = static_cast<int32_t>(posL / MM_PER_COUNT);
        s.encoderCountR = static_cast<int32_t>(posR / MM_PER_COUNT);
        s.headingDeg = sim.headingDeg;
        s.distFrontMm = us[0];
        s.distLeftMm = us[1];
        s.distRightMm = us[2];
        s.usValid = 0x07;
        // IR array: ahead of the axle, across the heading. Channel 0 is the leftmost.
        const float th = theta();
        const float ax = x + IR_AHEAD * -sinf(th);
        const float ay = y + IR_AHEAD * cosf(th);
        for (uint8_t ch = 0; ch < IR_CHANNEL_COUNT; ++ch) {
            const float lat = (3.5f - static_cast<float>(ch)) * IR_PITCH; // + = left
            const float px = ax + lat * -cosf(th);
            const float py = ay + lat * -sinf(th);
            s.irRaw[ch] = arena.floorBlack(px, py) ? 1000 : 150;
        }
        s.startButtonPressed = startPressed;
        return s;
    }

    void advance(const DriveCommand &cmd)
    {
        MotionOutput o;
        o.left = cmd.enable ? cmd.left : 0;
        o.right = cmd.enable ? cmd.right : 0;
        sim.step(o, DT_S);
        posL += sim.vL * DT_S;
        posR += sim.vR * DT_S;
        const float th = theta();
        x += sim.speedMmS * DT_S * -sinf(th);
        y += sim.speedMmS * DT_S * cosf(th);
        ++tick;
    }
};

static MissionParams missionParams()
{
    MissionParams p;
    p.control = simControlParams();
    p.control.stallDuty = 300;
    p.control.stallTimeMs = 600;
    p.control.stallMinSpeedMmS = 5.0f;
    p.control.stallMinYawDps = 5.0f;

    p.step.plan = simPlanParams();
    p.step.plan.moveTimeoutMs = 6000;
    p.step.wall.sideMm = 200;
    p.step.wall.frontMm = 300;
    p.step.wall.centerDistMm = 125;
    p.step.wall.kp = 0.3f;
    p.step.wall.steerMaxDeg = 15.0f;
    p.step.pitchMm = TILE;
    p.step.cruiseMmS = 200.0f;
    p.step.sampleLeadMm = 50.0f;
    p.step.sampleWindowMm = 30.0f;
    p.step.frontStopMm = 125.0f;

    p.line.kp = 1.0f;
    p.line.kd = 0.0f;
    p.line.steerMaxDeg = 25.0f;
    p.line.speedMmS = 150.0f;
    p.line.accelMmS2 = 1000.0f;
    p.line.slowdownErrMm = 30.0f;
    p.line.slowdownPct = 60;
    p.line.searchRateDps = 90.0f;
    p.line.lostTimeoutMs = 3000;
    p.line.endZoneMm = 40.0f;
    p.line.arrayAheadMm = IR_AHEAD;
    p.line.exitMm = 100.0f;
    p.line.rampTiltDeg = 15.0f;

    p.floor.markerMinMm = 60.0f;
    p.floor.markerMaxMm = 120.0f;
    p.floor.lineEnterMm = 160.0f;
    p.floor.gapMm = 45.0f;

    p.senseMs = 100;
    p.leaveMarginMm = 40.0f;
    p.irAheadMm = IR_AHEAD;
    p.dtMs = DT_MS;
    return p;
}

static PerceiveParams perceiveParams(const MissionParams &mp)
{
    PerceiveParams p;
    p.mmPerCount = MM_PER_COUNT;
    p.dtMs = DT_MS;
    p.wall = mp.step.wall;
    p.irPitchMm = IR_PITCH;
    return p;
}

struct Outcome {
    Phase phase = Phase::WaitStart;
    FaultCode fault = FaultCode::None;
    int ticks = 0;
    bool touchedWall = false;
    int startReentries = 0;
};

static bool nearWall(const Robot &r)
{
    // Is the robot's centre within 100 mm of a wall? The robot is 185 mm wide, so a centre closer
    // than half of that plus a few mm means the body is touching it.
    const float th = r.theta();
    for (int k = 0; k < 4; ++k) {
        const float a = th + 1.5708f * k;
        if (r.arena.ray(r.x, r.y, -sinf(a), cosf(a), 500.0f) < 100.0f) {
            return true;
        }
    }
    return false;
}

static Outcome runMission(Robot &r, const MissionParams &mp, int maxTicks = 9000)
{
    Mission ms;
    missionReset(ms);
    Perceiver pc;
    perceiverReset(pc);
    perceiverSetCalibration(pc, r.cal);
    const PerceiveParams pp = perceiveParams(mp);
    Perception per;

    Outcome out;
    const int8_t startAx = r.arena.startX;
    const int8_t startAy = r.arena.startY;
    bool wasOnStart = true;
    for (int i = 0; i < maxTicks; ++i) {
        const RobotState s = r.state(i >= 5); // press START on tick 5
        perceive(pc, s, pp, per);
        const DriveCommand cmd = missionStep(ms, mp, per);
        r.advance(cmd);
        out.ticks = i;

        if (i % 3 == 0 && ms.phase == Phase::Explore && nearWall(r)) {
            out.touchedWall = true;
        }
        const bool onStart = static_cast<int8_t>(r.x / TILE) == startAx &&
                             static_cast<int8_t>(r.y / TILE) == startAy;
        if (onStart && !wasOnStart) {
            ++out.startReentries;
        }
        wasOnStart = onStart;

        if (ms.phase == Phase::Done || ms.phase == Phase::Fault) {
            break;
        }
    }
    out.phase = ms.phase;
    out.fault = ms.fault;
    return out;
}

static int asInt(Phase p)
{
    return static_cast<int>(p);
}

static void makeBridge(Arena &a, int8_t bx, int8_t by, uint8_t outward)
{
    a.bridgeX = bx;
    a.bridgeY = by;
    a.bridgeDir = outward;
    // The marker is the tile just before the bridge tile along the bridge axis.
    const uint8_t back = static_cast<uint8_t>((outward + 2) & 3);
    a.markerX = static_cast<int8_t>(bx + maze::kDx[back]);
    a.markerY = static_cast<int8_t>(by + maze::kDy[back]);
    // ...and the two are connected.
    if (!a.isOpen(bx, by, back)) {
        a.link(a.markerX < bx || a.markerY < by ? a.markerX : bx,
               a.markerX < bx || a.markerY < by ? a.markerY : by, a.markerX != bx ? 'E' : 'N');
    }
}

// ---- The run, end to end ----------------------------------------------------------------

static void test_finds_the_bridge_and_follows_the_line_to_its_end()
{
    Robot r;
    r.arena = ringArena();
    makeBridge(r.arena, 3, 3, 0); // the bridge leaves north through the top-right tile
    r.place(0, 0, 0);
    const Outcome o = runMission(r, missionParams());
    TEST_ASSERT_EQUAL(asInt(Phase::Done), asInt(o.phase));
    TEST_ASSERT_EQUAL(0, static_cast<int>(o.fault));
    TEST_ASSERT_TRUE(r.y > TILE * 4.0f);               // out along the line, past the bridge tile
    TEST_ASSERT_FLOAT_WITHIN(25.0f, TILE * 3.5f, r.x); // still on the line
    TEST_ASSERT_FALSE(o.touchedWall);
}

// Every corner, both outward directions, from many starts and headings, with either hand.
static void sweep(Arena (*make)())
{
    int failures = 0;
    int touched = 0;
    int runs = 0;
    const int8_t corners[4][2] = {{0, 0}, {3, 0}, {0, 3}, {3, 3}};
    for (int8_t c = 0; c < 4; ++c) {
        for (uint8_t out = 0; out < 4; ++out) {
            const int8_t bx = corners[c][0], by = corners[c][1];
            // The bridge leaves through an OUTER wall of the corner tile.
            const int8_t ox = static_cast<int8_t>(bx + maze::kDx[out]);
            const int8_t oy = static_cast<int8_t>(by + maze::kDy[out]);
            if (ox >= 0 && ox < 4 && oy >= 0 && oy < 4) {
                continue;
            }
            int idx = 0;
            for (int8_t sy = 0; sy < 4; ++sy) {
                for (int8_t sx = 0; sx < 4; ++sx, ++idx) {
                    Robot r;
                    r.arena = make();
                    makeBridge(r.arena, bx, by, out);
                    if ((sx == r.arena.bridgeX && sy == r.arena.bridgeY) ||
                        (sx == r.arena.markerX && sy == r.arena.markerY)) {
                        continue;
                    }
                    r.place(sx, sy, static_cast<uint8_t>((idx + c + out) & 3));
                    MissionParams mp = missionParams();
                    mp.hand = (idx & 1) ? 1 : -1;
                    const Outcome o = runMission(r, mp);
                    ++runs;
                    failures += o.phase == Phase::Done ? 0 : 1;
                    touched += o.touchedWall ? 1 : 0;
                }
            }
        }
    }
    TEST_ASSERT_TRUE(runs > 60);
    TEST_ASSERT_EQUAL(0, failures);
    TEST_ASSERT_EQUAL(0, touched);
}

static Arena makeRing()
{
    return ringArena();
}

static Arena makeSnake()
{
    Arena a;
    for (int8_t i = 0; i < 3; ++i) {
        for (int8_t row = 0; row < 4; ++row) {
            a.link(i, row, 'E');
        }
    }
    a.link(3, 0, 'N');
    a.link(0, 1, 'N');
    a.link(3, 2, 'N');
    return a;
}

static void test_ring_arena_from_every_start_and_heading()
{
    sweep(makeRing);
}

static void test_snake_arena_from_every_start_and_heading()
{
    sweep(makeSnake);
}

// ---- Walls only: no floor sensing -------------------------------------------------------------

static MissionParams wallsOnlyParams()
{
    MissionParams mp = missionParams();
    mp.wallsOnly = true;
    return mp;
}

// The arena still has the marker and the bridge on its floor; with floor sensing off the robot
// must not react to them, explore every cell, and stop inside the maze.
static void test_walls_only_explores_and_stops_inside_the_maze_from_every_start()
{
    int failures = 0;
    int left = 0;
    int runs = 0;
    for (int8_t sy = 0; sy < 4; ++sy) {
        for (int8_t sx = 0; sx < 4; ++sx) {
            for (int8_t hand = -1; hand <= 1; hand += 2) {
                Robot r;
                r.arena = ringArena();
                makeBridge(r.arena, 3, 3, 0);
                if ((sx == r.arena.bridgeX && sy == r.arena.bridgeY) ||
                    (sx == r.arena.markerX && sy == r.arena.markerY)) {
                    continue;
                }
                r.place(sx, sy, static_cast<uint8_t>((sx + sy) & 3));
                MissionParams mp = wallsOnlyParams();
                mp.hand = hand;
                const Outcome o = runMission(r, mp);
                ++runs;
                failures += (o.phase == Phase::Done && o.fault == FaultCode::None) ? 0 : 1;
                left +=
                    (r.x > 0.0f && r.x < TILE * 4.0f && r.y > 0.0f && r.y < TILE * 4.0f) ? 0 : 1;
            }
        }
    }
    TEST_ASSERT_TRUE(runs > 20);
    TEST_ASSERT_EQUAL(0, failures);
    TEST_ASSERT_EQUAL(0, left);
}

static void test_walls_only_never_follows_the_line()
{
    Robot r;
    r.arena = ringArena();
    makeBridge(r.arena, 3, 3, 0);
    r.place(0, 0, 0);
    const Outcome o = runMission(r, wallsOnlyParams());
    TEST_ASSERT_EQUAL(asInt(Phase::Done), asInt(o.phase));
    TEST_ASSERT_TRUE(r.y < TILE * 4.0f); // the full-sensing run ends beyond y = 4 tiles
    TEST_ASSERT_FALSE(o.touchedWall);
}

static void test_walls_only_with_nothing_to_explore_is_done_not_a_fault()
{
    Robot r;
    r.arena = Arena{}; // no passages at all
    makeBridge(r.arena, 3, 3, 0);
    r.place(0, 0, 0);
    const Outcome o = runMission(r, wallsOnlyParams(), 1500);
    TEST_ASSERT_EQUAL(asInt(Phase::Done), asInt(o.phase));
    TEST_ASSERT_EQUAL(static_cast<int>(FaultCode::None), static_cast<int>(o.fault));
}

// ---- Faults and refusals ----------------------------------------------------------------------

static void test_waits_for_the_start_button()
{
    Robot r;
    r.arena = ringArena();
    makeBridge(r.arena, 3, 3, 0);
    r.place(0, 0, 0);
    const MissionParams mp = missionParams();
    Mission ms;
    missionReset(ms);
    Perceiver pc;
    perceiverReset(pc);
    perceiverSetCalibration(pc, r.cal);
    Perception per;
    for (int i = 0; i < 200; ++i) {
        perceive(pc, r.state(false), perceiveParams(mp), per);
        const DriveCommand c = missionStep(ms, mp, per);
        TEST_ASSERT_FALSE(c.enable);
        r.advance(c);
    }
    TEST_ASSERT_EQUAL(asInt(Phase::WaitStart), asInt(ms.phase));
}

static void test_a_missing_precondition_keeps_the_motors_off()
{
    Robot r;
    r.arena = ringArena();
    makeBridge(r.arena, 3, 3, 0);
    r.place(0, 0, 0);
    const MissionParams mp = missionParams();
    Mission ms;
    missionReset(ms);
    missionFail(ms, FaultCode::NoImu);
    Perceiver pc;
    perceiverReset(pc);
    perceiverSetCalibration(pc, r.cal);
    Perception per;
    for (int i = 0; i < 100; ++i) {
        perceive(pc, r.state(true), perceiveParams(mp), per); // START pressed
        const DriveCommand c = missionStep(ms, mp, per);
        TEST_ASSERT_FALSE(c.enable);
        TEST_ASSERT_EQUAL_INT16(0, c.left);
    }
    TEST_ASSERT_EQUAL(asInt(Phase::Fault), asInt(ms.phase));
    TEST_ASSERT_EQUAL(static_cast<int>(FaultCode::NoImu), static_cast<int>(ms.fault));
}

static void test_a_boxed_in_start_is_no_route()
{
    Robot r;
    r.arena = Arena{}; // no passages at all
    makeBridge(r.arena, 3, 3, 0);
    r.place(0, 0, 0);
    const Outcome o = runMission(r, missionParams(), 1500);
    TEST_ASSERT_EQUAL(asInt(Phase::Fault), asInt(o.phase));
    TEST_ASSERT_EQUAL(static_cast<int>(FaultCode::NoRoute), static_cast<int>(o.fault));
}

static void test_a_blocked_robot_stalls_out_instead_of_pushing_on()
{
    Robot r;
    r.arena = ringArena();
    makeBridge(r.arena, 3, 3, 0);
    r.place(0, 0, 0);
    const MissionParams mp = missionParams();
    Mission ms;
    missionReset(ms);
    Perceiver pc;
    perceiverReset(pc);
    perceiverSetCalibration(pc, r.cal);
    Perception per;
    bool stalled = false;
    for (int i = 0; i < 3000 && !stalled; ++i) {
        perceive(pc, r.state(i >= 5), perceiveParams(mp), per);
        const DriveCommand c = missionStep(ms, mp, per);
        if (ms.phase == Phase::Fault) {
            stalled = ms.fault == FaultCode::Stall;
            TEST_ASSERT_FALSE(c.enable);
            break;
        }
        // Wedged: whatever the motors do, the robot does not move or turn.
        r.tick++;
    }
    TEST_ASSERT_TRUE(stalled);
}

static void test_a_move_that_never_finishes_times_out()
{
    Robot r;
    r.arena = ringArena();
    makeBridge(r.arena, 3, 3, 0);
    r.place(0, 0, 0);
    MissionParams mp = missionParams();
    mp.control.stallDuty = 0; // no stall protection: only the move timeout is left
    mp.step.plan.moveTimeoutMs = 500;
    Mission ms;
    missionReset(ms);
    Perceiver pc;
    perceiverReset(pc);
    perceiverSetCalibration(pc, r.cal);
    Perception per;
    for (int i = 0; i < 3000 && ms.phase != Phase::Fault; ++i) {
        perceive(pc, r.state(i >= 5), perceiveParams(mp), per);
        missionStep(ms, mp, per);
        r.tick++; // wedged
    }
    TEST_ASSERT_EQUAL(asInt(Phase::Fault), asInt(ms.phase));
    TEST_ASSERT_EQUAL(static_cast<int>(FaultCode::MoveTimeout), static_cast<int>(ms.fault));
}

// Run until `stop(ms)` says so, with the robot physically wedged from the tick `wedgeFrom(ms)`.
template <typename Wedge>
static Mission runWedged(Robot &r, const MissionParams &mp, Wedge wedge, int maxTicks = 6000)
{
    Mission ms;
    missionReset(ms);
    Perceiver pc;
    perceiverReset(pc);
    perceiverSetCalibration(pc, r.cal);
    Perception per;
    for (int i = 0; i < maxTicks && ms.phase != Phase::Fault; ++i) {
        perceive(pc, r.state(i >= 5), perceiveParams(mp), per);
        const DriveCommand c = missionStep(ms, mp, per);
        if (wedge(ms)) {
            r.tick++; // wedged: the motors do nothing
        } else {
            r.advance(c);
        }
    }
    return ms;
}

static Robot wedgeRobot()
{
    Robot r;
    r.arena = ringArena();
    makeBridge(r.arena, 3, 3, 0);
    r.place(0, 0, 0);
    return r;
}

static MissionParams noStallParams()
{
    MissionParams mp = missionParams();
    mp.control.stallDuty = 0; // leave only the move timeout
    mp.step.plan.moveTimeoutMs = 400;
    return mp;
}

static void test_the_turn_round_times_out_when_the_robot_cannot_turn()
{
    Robot r = wedgeRobot();
    const Mission ms = runWedged(r, noStallParams(),
                                 [](const Mission &m) { return m.phase == Phase::TurnAround; });
    TEST_ASSERT_EQUAL(static_cast<int>(FaultCode::MoveTimeout), static_cast<int>(ms.fault));
    TEST_ASSERT_EQUAL(asInt(Phase::Fault), asInt(ms.phase));
}

static void test_an_explore_turn_times_out_when_the_robot_cannot_turn()
{
    Robot r = wedgeRobot();
    const Mission ms = runWedged(r, noStallParams(), [](const Mission &m) {
        return m.phase == Phase::Explore && m.sub == Mission::Sub::Turning;
    });
    TEST_ASSERT_EQUAL(static_cast<int>(FaultCode::MoveTimeout), static_cast<int>(ms.fault));
}

static void test_an_explore_step_times_out_when_the_robot_cannot_move()
{
    Robot r = wedgeRobot();
    const Mission ms = runWedged(r, noStallParams(), [](const Mission &m) {
        return m.phase == Phase::Explore && m.sub == Mission::Sub::Stepping;
    });
    TEST_ASSERT_EQUAL(static_cast<int>(FaultCode::MoveTimeout), static_cast<int>(ms.fault));
}

static void test_a_step_that_never_reads_its_walls_cannot_decide_so_it_is_no_route()
{
    Robot r = wedgeRobot();
    MissionParams mp = missionParams();
    mp.step.sampleLeadMm = -5.0f; // the window never opens: nothing to decide on
    const Mission ms = runWedged(r, mp, [](const Mission &) { return false; });
    TEST_ASSERT_EQUAL(static_cast<int>(FaultCode::NoRoute), static_cast<int>(ms.fault));
}

static void test_done_holds_the_wheels_and_stays_done()
{
    Robot r;
    r.arena = ringArena();
    makeBridge(r.arena, 3, 3, 0);
    r.place(0, 0, 0);
    const MissionParams mp = missionParams();
    Mission ms;
    missionReset(ms);
    Perceiver pc;
    perceiverReset(pc);
    perceiverSetCalibration(pc, r.cal);
    Perception per;
    int i = 0;
    for (; i < 9000 && ms.phase != Phase::Done; ++i) {
        perceive(pc, r.state(i >= 5), perceiveParams(mp), per);
        r.advance(missionStep(ms, mp, per));
    }
    TEST_ASSERT_EQUAL(asInt(Phase::Done), asInt(ms.phase));
    const float y = r.y;
    for (int k = 0; k < 100; ++k) {
        perceive(pc, r.state(true), perceiveParams(mp), per);
        const DriveCommand c = missionStep(ms, mp, per);
        TEST_ASSERT_TRUE(c.enable); // driver enabled, duty zero: brake, not coast
        r.advance(c);
    }
    TEST_ASSERT_EQUAL(asInt(Phase::Done), asInt(ms.phase));
    TEST_ASSERT_FLOAT_WITHIN(5.0f, y, r.y);
}

static void test_a_lost_line_is_a_fault_not_a_spin_for_ever()
{
    // The START tile is plain white: no line anywhere under the array, however it turns.
    Robot r;
    r.arena = ringArena();
    makeBridge(r.arena, 3, 3, 0);
    r.place(1, 1, 0);
    MissionParams mp = missionParams();
    mp.line.lostTimeoutMs = 400;
    Mission ms;
    missionReset(ms);
    ms.phase = Phase::LineFollow;
    lineFollowBegin(ms.line, MotionFeedback{}, true);
    Perceiver pc;
    perceiverReset(pc);
    perceiverSetCalibration(pc, r.cal);
    Perception per;
    for (int i = 0; i < 300 && ms.phase == Phase::LineFollow; ++i) {
        perceive(pc, r.state(true), perceiveParams(mp), per);
        r.advance(missionStep(ms, mp, per));
    }
    TEST_ASSERT_EQUAL(asInt(Phase::Fault), asInt(ms.phase));
    TEST_ASSERT_EQUAL(static_cast<int>(FaultCode::LineLost), static_cast<int>(ms.fault));
}

// ---- The marker's cell -------------------------------------------------------------------------

static Mission markerSetup()
{
    Mission ms;
    missionReset(ms);
    ms.facing = maze::Dir::North;
    // The run of white began on a step from (3,3) toward (3,4), whose centre is at 1000 mm.
    ms.haveRun = true;
    ms.runFromX = 3;
    ms.runFromY = 3;
    ms.runToX = 3;
    ms.runToY = 4;
    ms.runFacing = maze::Dir::North;
    ms.runTargetMm = 1000.0f;
    return ms;
}

static void test_marker_seen_as_the_array_crosses_the_target_centre_is_in_the_target_cell()
{
    Mission ms = markerSetup();
    MissionParams mp = missionParams();
    // The array passes the target's centre when the axle is irAhead short of it.
    mission_detail::onMarker(ms, mp, 1000.0f - IR_AHEAD);
    TEST_ASSERT_TRUE(ms.map.hasMarker);
    TEST_ASSERT_EQUAL(3, ms.map.markerX);
    TEST_ASSERT_EQUAL(4, ms.map.markerY);
}

static void test_marker_seen_a_cell_behind_the_target_is_the_previous_cell()
{
    Mission ms = markerSetup();
    MissionParams mp = missionParams();
    // The run began on a step whose target is (3,4); the array saw the marker's centre one pitch
    // short of that centre.
    mission_detail::onMarker(ms, mp, 1000.0f - TILE - IR_AHEAD + 10.0f);
    TEST_ASSERT_EQUAL(3, ms.map.markerX);
    TEST_ASSERT_EQUAL(3, ms.map.markerY);
}

static void test_marker_one_cell_ahead_of_the_target()
{
    Mission ms = markerSetup();
    mission_detail::onMarker(ms, missionParams(), 1000.0f + TILE - IR_AHEAD);
    TEST_ASSERT_EQUAL(3, ms.map.markerX);
    TEST_ASSERT_EQUAL(5, ms.map.markerY);
}

static void test_a_marker_run_with_no_snapshot_is_ignored()
{
    Mission ms = markerSetup();
    ms.haveRun = false; // began near the start tile
    mission_detail::onMarker(ms, missionParams(), 1000.0f - IR_AHEAD);
    TEST_ASSERT_FALSE(ms.map.hasMarker);
}

static void test_run_start_heading_for_the_start_tile_is_not_kept()
{
    Mission ms;
    missionReset(ms); // cell and target are both the start cell
    mission_detail::onRunStart(ms);
    TEST_ASSERT_FALSE(ms.haveRun);
    // Leaving the start for the next cell: a tile beside the start tile may be the marker.
    ms.tx = maze::kStartX;
    ms.ty = static_cast<int8_t>(maze::kStartY + 1);
    mission_detail::onRunStart(ms);
    TEST_ASSERT_TRUE(ms.haveRun);
    ms.cx = 2;
    ms.cy = 3;
    ms.tx = maze::kStartX; // heading for the start tile
    ms.ty = maze::kStartY;
    mission_detail::onRunStart(ms);
    TEST_ASSERT_FALSE(ms.haveRun);
}

static void test_line_zone_is_followed_only_when_entered_from_the_marker()
{
    Mission ms;
    missionReset(ms);
    maze::mapMarkMarker(ms.map, 3, 4);
    ms.haveRun = true;
    ms.runFromX = 3;
    ms.runFromY = 4;
    ms.runToX = 3;
    ms.runToY = 5;
    TEST_ASSERT_TRUE(mission_detail::onLineZone(ms));
    // The same white tile met from the side: remembered, not followed.
    ms.haveRun = true;
    ms.runFromX = 2;
    ms.runFromY = 5;
    TEST_ASSERT_FALSE(mission_detail::onLineZone(ms));
    TEST_ASSERT_TRUE(ms.map.cell[5][3].line);
    // With no marker known, nothing is followed.
    Mission none;
    missionReset(none);
    none.haveRun = true;
    none.runToX = 4;
    none.runToY = 4;
    TEST_ASSERT_FALSE(mission_detail::onLineZone(none));
    TEST_ASSERT_TRUE(none.map.cell[4][4].line);
    // And a run with no snapshot (near the start tile) is nothing at all.
    none.haveRun = false;
    TEST_ASSERT_FALSE(mission_detail::onLineZone(none));
}

static void test_planning_goes_to_the_marker_cell_then_steps_into_the_bridge_tile()
{
    using namespace maze;
    Mission ms;
    missionReset(ms);
    MissionParams mp = missionParams();
    // A straight corridor north: (3,3) start, (3,4), marker (3,5), unvisited (3,6) beyond it.
    mapObserve(ms.map, 3, 3, Dir::North, false, true, true);
    mapSetEdge(ms.map, 3, 3, Dir::South, true);
    mapObserve(ms.map, 3, 4, Dir::North, false, true, true);
    mapObserve(ms.map, 3, 5, Dir::North, false, true, true);
    mapMarkMarker(ms.map, 3, 5);
    // At (3,4) facing north: head for the marker cell.
    ms.cx = 3;
    ms.cy = 4;
    ms.facing = Dir::North;
    TEST_ASSERT_EQUAL(static_cast<int>(Move::Forward),
                      static_cast<int>(mission_detail::plan(ms, mp)));
    // On the marker cell: straight into the neighbour beside it.
    ms.cx = 3;
    ms.cy = 5;
    TEST_ASSERT_EQUAL(static_cast<int>(Move::Forward),
                      static_cast<int>(mission_detail::plan(ms, mp)));
    ms.facing = Dir::South; // arrived the other way: turn back round to it
    TEST_ASSERT_EQUAL(static_cast<int>(Move::Back), static_cast<int>(mission_detail::plan(ms, mp)));
}

static void test_the_start_tile_is_never_taken_for_the_marker_or_the_line()
{
    // The robot starts ON the white start tile and leaves: nothing seen there may count.
    Robot r;
    r.arena = ringArena();
    makeBridge(r.arena, 3, 3, 0);
    r.place(1, 1, 0); // start tile is interior, far from the bridge
    const Outcome o = runMission(r, missionParams());
    TEST_ASSERT_EQUAL(asInt(Phase::Done), asInt(o.phase));
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_finds_the_bridge_and_follows_the_line_to_its_end);
    RUN_TEST(test_ring_arena_from_every_start_and_heading);
    RUN_TEST(test_snake_arena_from_every_start_and_heading);
    RUN_TEST(test_walls_only_explores_and_stops_inside_the_maze_from_every_start);
    RUN_TEST(test_walls_only_never_follows_the_line);
    RUN_TEST(test_walls_only_with_nothing_to_explore_is_done_not_a_fault);
    RUN_TEST(test_waits_for_the_start_button);
    RUN_TEST(test_a_missing_precondition_keeps_the_motors_off);
    RUN_TEST(test_a_boxed_in_start_is_no_route);
    RUN_TEST(test_a_blocked_robot_stalls_out_instead_of_pushing_on);
    RUN_TEST(test_a_move_that_never_finishes_times_out);
    RUN_TEST(test_the_turn_round_times_out_when_the_robot_cannot_turn);
    RUN_TEST(test_an_explore_turn_times_out_when_the_robot_cannot_turn);
    RUN_TEST(test_an_explore_step_times_out_when_the_robot_cannot_move);
    RUN_TEST(test_a_step_that_never_reads_its_walls_cannot_decide_so_it_is_no_route);
    RUN_TEST(test_done_holds_the_wheels_and_stays_done);
    RUN_TEST(test_a_lost_line_is_a_fault_not_a_spin_for_ever);
    RUN_TEST(test_marker_seen_as_the_array_crosses_the_target_centre_is_in_the_target_cell);
    RUN_TEST(test_marker_seen_a_cell_behind_the_target_is_the_previous_cell);
    RUN_TEST(test_marker_one_cell_ahead_of_the_target);
    RUN_TEST(test_a_marker_run_with_no_snapshot_is_ignored);
    RUN_TEST(test_run_start_heading_for_the_start_tile_is_not_kept);
    RUN_TEST(test_line_zone_is_followed_only_when_entered_from_the_marker);
    RUN_TEST(test_planning_goes_to_the_marker_cell_then_steps_into_the_bridge_tile);
    RUN_TEST(test_the_start_tile_is_never_taken_for_the_marker_or_the_line);
    return UNITY_END();
}
