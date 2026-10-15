// ============================================================================
//  test_mission.cpp  --  the whole control stack against a simulated arena: a 4x4 maze with
//  walls, ultrasonic rays (one sensor read per 20 ms slot, like the real driver), encoders, an
//  IR array over a painted floor (start tile, the approach marker, the strip and gap after it, the
//  ramp's line), and
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
constexpr float IR_AHEAD = IR_ARRAY_FORWARD_OFFSET_MM; // as on the robot
constexpr float IR_PITCH = IR_SENSOR_PITCH_MM;
constexpr float MM_PER_COUNT = 0.2f;

struct Wall {
    float x0, y0, x1, y1; // axis-aligned segment
};

// ---- Arena -------------------------------------------------------------------------

struct Arena {
    uint8_t open[4][4] = {}; // bit per abs dir: a passage out of that cell
    int8_t startX = 0, startY = 0;
    int8_t markerX = 0, markerY = 0; // the marker cell: two openings, the one it is entered by and
    int8_t bridgeX = 0, bridgeY = 0; // the one straight across, which leaves the maze
    uint8_t bridgeDir = 0;           // abs dir the exit leaves in (through the outer wall)
    float lineLength = 600.0f;
    bool startWhite = true;    // the start tile is white (false: black like the rest)
    bool exitOpen = true;      // false: a wall where the exit should be
    bool ramp = true;          // false: nothing but black floor beyond the marker
    bool markerPainted = true; // false: the marker's frames and box are not on the floor

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
        const bool bridgeSide = exitOpen && ax == bridgeX && ay == bridgeY && d == bridgeDir;
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

    // What the IR array reads at (x, y), in raw counts. Measured on the robot: the marker is a
    // white frame (36 mm), a black box (118 mm, with a short bar of tape that reads a little
    // higher), a white frame (37 mm), then a 20 mm black strip, ~22 mm of nothing under the array
    // (the gap, every channel at the top of the scale) and the ramp: white with a 24 mm black line.
    uint16_t floorRaw(float x, float y) const
    {
        const uint16_t white = 150, black = 1000, tape = 1015, air = 1023;
        const float sx = TILE * (startX + 0.5f), sy = TILE * (startY + 0.5f);
        if (startWhite && fabsf(x - sx) <= TILE / 2 && fabsf(y - sy) <= TILE / 2) {
            return white;
        }
        const float mx = TILE * (markerX + 0.5f), my = TILE * (markerY + 0.5f);
        const float ux = static_cast<float>(maze::kDx[bridgeDir]);
        const float uy = static_cast<float>(maze::kDy[bridgeDir]);
        const float along = (x - mx) * ux + (y - my) * uy;
        const float lateral = fabsf((x - mx) * (-uy) + (y - my) * ux);
        if (markerPainted && lateral <= 95.5f && along >= -95.5f && along <= 96.5f) {
            if (along < -59.5f || along > 59.5f) {
                return white;
            }
            return (lateral <= 14.0f && along >= -20.0f && along <= 5.0f) ? tape : black;
        }
        if (ramp && lateral <= 150.0f && along > 96.5f) {
            if (along <= 116.5f) {
                return black; // the strip
            }
            if (along <= 138.5f) {
                return air; // the gap
            }
            if (along <= 138.5f + lineLength) {
                return lateral <= 12.0f ? black : white; // the ramp and its line
            }
            if (along <= 138.5f + lineLength + 400.0f) {
                return white; // the line is over: plain white floor beyond it
            }
        }
        return black;
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
            s.irRaw[ch] = arena.floorRaw(px, py);
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
    p.line.searchRateDps = 0.0f; // the line just ends: straight on, then stop
    p.line.lostTimeoutMs = 3000;
    p.line.searchSpeedMmS = 100.0f;
    p.line.endLostMm = 60.0f;
    p.line.rampTiltDeg = 15.0f;

    p.pattern.frameMinMm = MARKER_FRAME_MIN_MM;
    p.pattern.frameMaxMm = MARKER_FRAME_MAX_MM;
    p.pattern.reportMm = MARKER_FRAME_REPORT_MM;
    p.pattern.centreMinMm = MARKER_CENTRE_MIN_MM;
    p.pattern.centreMaxMm = MARKER_CENTRE_MAX_MM;
    p.pattern.gapMinMm = GAP_MIN_MM;
    p.pattern.glitchMm = FLOOR_GLITCH_MM;
    p.bridgeMinLineMm = BRIDGE_HANDOVER_MIN_MM;
    p.bridgeMaxMm = BRIDGE_MAX_MM;

    p.senseMs = 100;
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
    p.irAirCounts = IR_AIR_COUNTS;
    p.irAirChannels = IR_AIR_CHANNELS;
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

// The marker cell (mx, my) has exactly two openings: the one it is entered by and the one straight
// across, which leaves the maze through the outer wall. Its other two sides are walls.
static void makeBridge(Arena &a, int8_t mx, int8_t my, uint8_t outward)
{
    a.bridgeX = a.markerX = mx;
    a.bridgeY = a.markerY = my;
    a.bridgeDir = outward;
    const uint8_t back = static_cast<uint8_t>((outward + 2) & 3);
    for (uint8_t d = 0; d < 4; ++d) {
        if (d == outward) {
            continue;
        }
        const int8_t nx = static_cast<int8_t>(mx + maze::kDx[d]);
        const int8_t ny = static_cast<int8_t>(my + maze::kDy[d]);
        const bool inside = nx >= 0 && nx < 4 && ny >= 0 && ny < 4;
        const uint8_t bit = static_cast<uint8_t>(1u << d);
        const uint8_t nbit = static_cast<uint8_t>(1u << ((d + 2) & 3));
        if (d == back) { // the way in
            a.open[my][mx] = static_cast<uint8_t>(a.open[my][mx] | bit);
            if (inside) {
                a.open[ny][nx] = static_cast<uint8_t>(a.open[ny][nx] | nbit);
            }
        } else { // the sides: walls
            a.open[my][mx] = static_cast<uint8_t>(a.open[my][mx] & ~bit);
            if (inside) {
                a.open[ny][nx] = static_cast<uint8_t>(a.open[ny][nx] & ~nbit);
            }
        }
    }
}

// Can every cell be reached from (sx, sy) through the maze's own openings?
static bool allReachable(const Arena &a, int8_t sx, int8_t sy)
{
    bool seen[4][4] = {};
    int8_t stackX[16], stackY[16];
    int n = 0;
    stackX[n] = sx;
    stackY[n++] = sy;
    seen[sy][sx] = true;
    int count = 1;
    while (n > 0) {
        --n;
        const int8_t x = stackX[n], y = stackY[n];
        for (uint8_t d = 0; d < 4; ++d) {
            if ((a.open[y][x] & (1u << d)) == 0) {
                continue;
            }
            const int8_t nx = static_cast<int8_t>(x + maze::kDx[d]);
            const int8_t ny = static_cast<int8_t>(y + maze::kDy[d]);
            if (nx < 0 || nx > 3 || ny < 0 || ny > 3 || seen[ny][nx]) {
                continue;
            }
            seen[ny][nx] = true;
            stackX[n] = nx;
            stackY[n++] = ny;
            ++count;
        }
    }
    return count == 16;
}

// ---- The run, end to end ----------------------------------------------------------------

static void test_finds_the_bridge_and_follows_the_line_to_its_end()
{
    Robot r;
    r.arena = ringArena();
    makeBridge(r.arena, 3, 3, 0); // the marker cell is the top-right corner; the exit is north
    r.place(0, 0, 0);
    const Outcome o = runMission(r, missionParams());
    TEST_ASSERT_EQUAL(asInt(Phase::Done), asInt(o.phase));
    TEST_ASSERT_EQUAL(0, static_cast<int>(o.fault));
    TEST_ASSERT_TRUE(r.y > TILE * 4.0f);               // out along the line, past the bridge tile
    TEST_ASSERT_FLOAT_WITHIN(25.0f, TILE * 3.5f, r.x); // still on the line
    TEST_ASSERT_FALSE(o.touchedWall);
}

// Every corner, both outward directions, from many starts and headings, with either hand.
// `minRuns`: a marker cell with only two openings splits some mazes, and those placements are
// skipped. `painted` false: the marker is not seen at all, so the exit has to come from the map.
static void sweep(Arena (*make)(), int minRuns, bool painted = true)
{
    int failures = 0;
    int touched = 0;
    int runs = 0;
    for (int8_t by = 0; by < 4; ++by) {
        for (int8_t bx = 0; bx < 4; ++bx) {
            for (uint8_t out = 0; out < 4; ++out) {
                // The exit leaves through an OUTER wall of the marker cell.
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
                        if ((sx == bx && sy == by) || !allReachable(r.arena, sx, sy)) {
                            continue;
                        }
                        r.arena.startWhite = (idx & 2) == 0; // a white or a black start tile
                        r.arena.markerPainted = painted;
                        r.place(sx, sy, static_cast<uint8_t>((idx + bx + by + out) & 3));
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
    }
    TEST_ASSERT_TRUE(runs >= minRuns);
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
    sweep(makeRing, 150);
}

// The robot stopped short of seeing the far frame, or the marker is faded: the walls alone give the
// exit once everything is explored, and the robot goes there and takes it.
static void test_without_a_marker_the_exit_comes_from_the_map()
{
    sweep(makeRing, 150, false);
}

static void test_snake_arena_from_every_start_and_heading()
{
    sweep(makeSnake, 40);
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
                // Walls only cannot tell the open exit from an unexplored cell inside the maze (the
                // bounds are not known yet), so this test walls it.
                r.arena.exitOpen = false;
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

// ---- Turn sweeps ------------------------------------------------------------------------------

// A turn lands on the grid heading whatever the robot's heading error was at the start. A nominal
// 90 would have added that error (seen on the robot: 5.8 deg off after the second turn).
static void test_sweep_ends_exactly_on_the_grid_heading()
{
    using mission_detail::sweepTo;
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 86.9f,
                             sweepTo(-176.9f, -90.0f, 1.0f)); // 3.1 deg short of south
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 87.0f, sweepTo(3.0f, 90.0f, 1.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -87.0f, sweepTo(-3.0f, -90.0f, -1.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 93.0f, sweepTo(-3.0f, 90.0f, 1.0f));
}

// A half turn goes the way the hand says, even when the start error would make the short way round
// the other one.
static void test_half_turn_sweep_follows_the_requested_direction()
{
    using mission_detail::sweepTo;
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 178.0f, sweepTo(2.0f, -180.0f, 1.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -182.0f, sweepTo(2.0f, -180.0f, -1.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 182.0f, sweepTo(-2.0f, -180.0f, 1.0f));
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

// Exploring never needs the map's marker: the pattern finds it, and the way out is straight on.
static void test_no_ramp_after_the_marker_is_no_bridge()
{
    Robot r;
    r.arena = ringArena();
    makeBridge(r.arena, 3, 3, 0);
    r.arena.ramp = false; // the marker, then plain black floor: no strip, no gap, no line
    r.place(0, 0, 0);
    const Outcome o = runMission(r, missionParams());
    TEST_ASSERT_EQUAL(asInt(Phase::Fault), asInt(o.phase));
    TEST_ASSERT_EQUAL(static_cast<int>(FaultCode::NoBridge), static_cast<int>(o.fault));
    // It did not drive far out looking for the line: the marker centre plus the bridge limit.
    TEST_ASSERT_TRUE(r.y < TILE * 3.5f + BRIDGE_MAX_MM + 60.0f);
}

static void test_a_wall_where_the_exit_should_be_is_no_bridge()
{
    Robot r;
    r.arena = ringArena();
    makeBridge(r.arena, 3, 3, 0);
    r.arena.exitOpen = false; // the marker is there, but the way straight on is walled
    r.place(0, 0, 0);
    const Outcome o = runMission(r, missionParams());
    TEST_ASSERT_EQUAL(asInt(Phase::Fault), asInt(o.phase));
    TEST_ASSERT_EQUAL(static_cast<int>(FaultCode::NoBridge), static_cast<int>(o.fault));
    TEST_ASSERT_FALSE(o.touchedWall);
}

static void test_the_bridge_phase_is_entered_and_handed_over_to_the_line_follower()
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
    bool sawBridge = false, sawLine = false;
    for (int i = 0; i < 9000 && ms.phase != Phase::Done && ms.phase != Phase::Fault; ++i) {
        perceive(pc, r.state(i >= 5), perceiveParams(mp), per);
        r.advance(missionStep(ms, mp, per));
        sawBridge = sawBridge || ms.phase == Phase::Bridge;
        if (ms.phase == Phase::LineFollow) {
            TEST_ASSERT_TRUE(sawBridge); // the line follower only ever follows the bridge
            sawLine = true;
        }
    }
    TEST_ASSERT_TRUE(sawBridge);
    TEST_ASSERT_TRUE(sawLine);
    TEST_ASSERT_EQUAL(asInt(Phase::Done), asInt(ms.phase));
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
    RUN_TEST(test_without_a_marker_the_exit_comes_from_the_map);
    RUN_TEST(test_snake_arena_from_every_start_and_heading);
    RUN_TEST(test_walls_only_explores_and_stops_inside_the_maze_from_every_start);
    RUN_TEST(test_walls_only_never_follows_the_line);
    RUN_TEST(test_walls_only_with_nothing_to_explore_is_done_not_a_fault);
    RUN_TEST(test_sweep_ends_exactly_on_the_grid_heading);
    RUN_TEST(test_half_turn_sweep_follows_the_requested_direction);
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
    RUN_TEST(test_no_ramp_after_the_marker_is_no_bridge);
    RUN_TEST(test_a_wall_where_the_exit_should_be_is_no_bridge);
    RUN_TEST(test_the_bridge_phase_is_entered_and_handed_over_to_the_line_follower);
    RUN_TEST(test_the_start_tile_is_never_taken_for_the_marker_or_the_line);
    return UNITY_END();
}
