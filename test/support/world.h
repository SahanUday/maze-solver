// A small world for mission tests: a 4x4 arena of walls with ray-cast ultrasonics, a
// differential-drive plant with stiction, encoders, a gyro and an IR array under the robot, all
// assembled into the same RobotState the firmware reads and run through perceive().
#pragma once

#include <math.h>
#include <stdint.h>

#include "DriveCommand.h"
#include "Heading.h"
#include "LineSense.h"
#include "Perception.h"
#include "RobotSpec.h"
#include "RobotState.h"

namespace world
{

struct Lcg {
    uint32_t s = 12345u;
    float u() // [0, 1)
    {
        s = s * 1664525u + 1013904223u;
        return static_cast<float>(s >> 8) / 16777216.0f;
    }
    float g() // about N(0, 1)
    {
        return (u() + u() + u() + u() + u() + u() - 3.0f) * 1.4142f;
    }
};

constexpr int N = 4;
constexpr float PITCH = 250.0f;
constexpr float WALL_T = 17.0f;       // wall thickness
constexpr float NOSE_MM = 89.0f;      // axle to the front of the body (assumed)
constexpr float BODY_HALF_MM = 90.0f; // axle half width over the tyres
constexpr float ARRAY_AHEAD_MM = IR_ARRAY_FORWARD_OFFSET_MM; // IR array ahead of the axle
constexpr float MARKER_OUTER_MM = 191.0f; // white frame, 36.5 mm each side of the black box
constexpr float MARKER_BOX_MM = 118.0f;
constexpr int16_t WHITE_COUNTS = 500; // channel k reads this + 6k on white
constexpr int16_t BLACK_COUNTS = 990; // the maze floor reads 976-1013
constexpr int16_t AIR_COUNTS = 1020;  // nothing under the array
// Beyond the marker tile's open edge, along the exit direction from the tile's centre: the edge
// at 125 mm, ~20 mm of black strip, the ~37 mm gap, a dark patch, then the white ramp board with
// the black line down the middle: straight for 600 mm, then a bend to the left.
constexpr float STRIP_FROM_MM = 125.0f;
constexpr float GAP_FROM_MM = 145.0f;
constexpr float GAP_TO_MM = 182.0f;
constexpr float PATCH_TO_MM = 190.0f;
constexpr float LINE_STRAIGHT_TO_MM = 800.0f;
constexpr float BAR_LENGTH_MM = 1500.0f; // the bend's bar, to the left of the line's end
constexpr float LINE_HALF_MM = 12.0f;    // the line is 24 mm
constexpr float RAMP_SLOPE_DEG = 10.0f;  // the ramp's slope, from PATCH_TO_MM for 300 mm
constexpr float RAMP_LENGTH_MM = 300.0f;

struct Rect {
    float x0, x1, y0, y1;
};

// Cell (i, j): column i from the west, row j from the south; the centre of (0, 0) is the origin.
struct Arena {
    bool vw[(N + 1) * N] = {}; // vw[i * N + j]: a wall on the west side of column i
    bool hw[N * (N + 1)] = {}; // hw[j * N + i]: a wall on the south side of row j
    Rect rect[(N + 1) * N + N * (N + 1)];
    int rects = 0;
    int markerI = -1; // the tile with the approach marker; -1 = none
    int markerJ = -1;
    int exitDir = 1; // the marker tile's open side: 0 = east, 1 = north, 2 = west, 3 = south

    void build()
    {
        rects = 0;
        for (int i = 0; i <= N; ++i) {
            for (int j = 0; j < N; ++j) {
                if (vw[i * N + j]) {
                    const float bx = PITCH * i - PITCH / 2;
                    const float cy = PITCH * j;
                    rect[rects++] = {bx - WALL_T / 2, bx + WALL_T / 2, cy - PITCH / 2 - WALL_T / 2,
                                     cy + PITCH / 2 + WALL_T / 2};
                }
            }
        }
        for (int j = 0; j <= N; ++j) {
            for (int i = 0; i < N; ++i) {
                if (hw[j * N + i]) {
                    const float by = PITCH * j - PITCH / 2;
                    const float cx = PITCH * i;
                    rect[rects++] = {cx - PITCH / 2 - WALL_T / 2, cx + PITCH / 2 + WALL_T / 2,
                                     by - WALL_T / 2, by + WALL_T / 2};
                }
            }
        }
    }

    // The team's Section A maze (the photo): the marker tile is its top-left corner, closed on
    // both sides and open to the north, where the line leaves the maze.
    // `row` counts from the top of the photo.
    void photo()
    {
        for (int j = 0; j < N; ++j) {
            vw[0 * N + j] = true;
            vw[N * N + j] = true;
        }
        for (int i = 0; i < N; ++i) {
            hw[0 * N + i] = true;
            hw[N * N + i] = i != 0;
        }
        exitDir = 1;
        const struct {
            int row, col;
            char side; // E = its east side, S = its south side
        } edges[] = {{0, 0, 'E'}, {1, 0, 'S'}, {3, 0, 'E'}, {0, 2, 'S'},
                     {1, 1, 'E'}, {2, 1, 'E'}, {1, 2, 'S'}, {2, 2, 'E'}};
        for (const auto &e : edges) {
            const int j = N - 1 - e.row;
            if (e.side == 'E') {
                vw[(e.col + 1) * N + j] = true;
            } else {
                hw[j * N + e.col] = true;
            }
        }
        markerI = 0;
        markerJ = N - 1;
        build();
    }

    // Distance along a ray (unit direction) to the nearest wall face; a large number if none.
    float ray(float ox, float oy, float dx, float dy) const
    {
        float best = 1.0e9f;
        for (int k = 0; k < rects; ++k) {
            float t0 = 0.0f;
            float t1 = 1.0e9f;
            bool hit = true;
            const float o[2] = {ox, oy};
            const float d[2] = {dx, dy};
            const float lo[2] = {rect[k].x0, rect[k].y0};
            const float hi[2] = {rect[k].x1, rect[k].y1};
            for (int a = 0; a < 2 && hit; ++a) {
                if (fabsf(d[a]) < 1.0e-9f) {
                    hit = o[a] >= lo[a] && o[a] <= hi[a];
                    continue;
                }
                float ta = (lo[a] - o[a]) / d[a];
                float tb = (hi[a] - o[a]) / d[a];
                if (ta > tb) {
                    const float t = ta;
                    ta = tb;
                    tb = t;
                }
                t0 = ta > t0 ? ta : t0;
                t1 = tb < t1 ? tb : t1;
                hit = t0 <= t1;
            }
            if (hit && t0 > 0.0f && t0 < best) {
                best = t0;
            }
        }
        return best;
    }

    bool wall(int i, int j, int dir) const // dir: 0 = east, 1 = north, 2 = west, 3 = south
    {
        switch (dir) {
            case 0:
                return vw[(i + 1) * N + j];
            case 1:
                return hw[(j + 1) * N + i];
            case 2:
                return vw[i * N + j];
            default:
                return hw[j * N + i];
        }
    }
};

// Knobs a test turns.
struct Faults {
    float noiseMm = 3.0f;      // ultrasonic noise, 1 sigma
    float dropout = 0.0f;      // chance a reading gets no echo
    float strayNear = 0.0f;    // chance a wall reading is a false short echo (30 to 100 mm)
    float gyroScale = 1.0015f; // measured 1.001 to 1.002
    float gyroDriftDps = 0.0f;
    float odometerScale = 1.003f; // true mm per count over the spec's
    // Something close in front (a hand, a dropped part) between these times: the front reads 70 mm.
    uint32_t obstacleFromMs = 0;
    uint32_t obstacleToMs = 0;
};

struct World {
    Arena arena;
    Faults faults;
    Lcg rng;

    // Plant: wheel speeds (mm/s) and positions (mm), pose in the arena (mm, deg).
    double v[2] = {0, 0};
    double pos[2] = {0, 0};
    double x = 0, y = 0, yawDeg = 0;
    double yawRateDps = 0;
    // Wheels: a stopped one needs MOTOR_MIN_PWM_* x `sticking` duty to start (1 = the values
    // measured on the floor), a rolling one `sustain` x (`sustainRight` for the right) to go on.
    float sticking = 1.0f;
    double sustain = 110.0; // the wheels kept going down to duty 100-120 on the stand
    double sustainRight = 1.0;

    RobotState state;
    Perceiver pc;
    PerceiveParams pp;
    GyroBias bias;
    Heading heading;
    uint16_t held[3] = {600, 117, 117};
    uint8_t valid = 0x07;
    uint32_t tMs = 0;

    // Metrics.
    float minNoseMm = 1.0e9f;
    float minSideMm = 1.0e9f;
    int startTileEntries = 0;
    int startI = 0;
    int startJ = 0;
    bool leftStart = false;
    int lastI = 0;
    int lastJ = 0;

    int cellI() const
    {
        return static_cast<int>(lroundf(static_cast<float>(x) / PITCH));
    }
    int cellJ() const
    {
        return static_cast<int>(lroundf(static_cast<float>(y) / PITCH));
    }

    // Place the robot on tile (i, j) facing `facing` quarter turns counter-clockwise from east,
    // with `offX`, `offY` mm and `offYaw` deg of error, then do what the firmware does at boot.
    void start(int i, int j, int facing, float offX = 0.0f, float offY = 0.0f, float offYaw = 0.0f)
    {
        x = PITCH * i + offX;
        y = PITCH * j + offY;
        yawDeg = 90.0 * facing + offYaw;
        startI = lastI = i;
        startJ = lastJ = j;
        state.imu.accel[IMU_ACCEL_AXIS_FORWARD] = IMU_ACCEL_FORWARD_LEVEL_COUNTS;
        gyroBiasReset(bias);
        for (uint16_t k = 0; k < GYRO_BIAS_SAMPLES; ++k) {
            gyroBiasAccumulate(bias, gyroCounts(0.0));
        }
        gyroBiasFinish(bias);

        perceiverSetCalibration(pc, calibration());
        state.startButtonPressed = true;
    }

    // The IR array's calibration: the white and black of arrayCounts().
    static LineCalibration calibration()
    {
        LineCalibration cal;
        lineCalibrationReset(cal);
        uint16_t counts[IR_CHANNEL_COUNT];
        for (uint8_t ch = 0; ch < IR_CHANNEL_COUNT; ++ch) {
            counts[ch] = static_cast<uint16_t>(WHITE_COUNTS + 6 * ch);
        }
        lineCalibrationAccumulate(cal, counts);
        for (uint8_t ch = 0; ch < IR_CHANNEL_COUNT; ++ch) {
            counts[ch] = BLACK_COUNTS;
        }
        lineCalibrationAccumulate(cal, counts);
        lineCalibrationFinish(cal);
        return cal;
    }

    int16_t gyroCounts(double rateDps)
    {
        const double c = rateDps * GYRO_COUNTS_PER_DPS * faults.gyroScale + 3.0 +
                         faults.gyroDriftDps * GYRO_COUNTS_PER_DPS + rng.g() * 2.4;
        return static_cast<int16_t>(lround(c));
    }

    float rayFromAxle(float angleDeg) const
    {
        const float a = angleDeg * 0.0174533f;
        return arena.ray(static_cast<float>(x), static_cast<float>(y), cosf(a), sinf(a));
    }

    void sampleUltrasonic(int slot)
    {
        if (rng.u() < faults.dropout) {
            valid = static_cast<uint8_t>(valid & ~(1u << slot));
            return;
        }
        valid = static_cast<uint8_t>(valid | (1u << slot));
        const float yaw = static_cast<float>(yawDeg);
        const float angle = yaw + (slot == 0 ? 0.0f : (slot == 1 ? 90.0f : -90.0f));
        const float d = rayFromAxle(angle);
        if (slot == 0 && (d - US_OFFSET_FRONT_MM < 30.0f || d > 2500.0f)) {
            valid = static_cast<uint8_t>(valid & ~1u); // the sensor's dead zone, or out of range
            return;
        }
        float mm = (d > 2500.0f ? 2500.0f : d) + rng.g() * faults.noiseMm;
        if (slot == 0 && tMs >= faults.obstacleFromMs && tMs < faults.obstacleToMs) {
            mm = 70.0f + rng.g() * faults.noiseMm;
        }
        if (d < 400.0f && rng.u() < faults.strayNear) {
            mm = 30.0f + rng.u() * 70.0f;
        }
        held[slot] = static_cast<uint16_t>(mm < 0.0f ? 0.0f : mm);
    }

    // Distances from the marker tile's centre along the exit direction, and across it (left
    // positive).
    void exitFrame(float px, float py, float &along, float &lat) const
    {
        static const float ex[4] = {1.0f, 0.0f, -1.0f, 0.0f};
        static const float ey[4] = {0.0f, 1.0f, 0.0f, -1.0f};
        const float dx = px - PITCH * static_cast<float>(arena.markerI);
        const float dy = py - PITCH * static_cast<float>(arena.markerJ);
        along = dx * ex[exitDirOrNorth()] + dy * ey[exitDirOrNorth()];
        lat = -dx * ey[exitDirOrNorth()] + dy * ex[exitDirOrNorth()];
    }
    bool lineGone = false;  // the board has no line on it (a test takes it away)
    bool gapIsDark = false; // the gap reads dark like the strip, not as air (as on the real robot)
    int exitDirOrNorth() const
    {
        return arena.exitDir;
    }

    // What one channel reads under a point of the floor.
    uint16_t countsAt(float px, float py, uint8_t ch) const
    {
        const uint16_t white = static_cast<uint16_t>(WHITE_COUNTS + 6 * ch);
        if (arena.markerI < 0) {
            return BLACK_COUNTS;
        }
        float along = 0.0f;
        float lat = 0.0f;
        exitFrame(px, py, along, lat);
        if (along < STRIP_FROM_MM) {
            // The maze: black, and the marker's white frame around its black box.
            const float dx = fabsf(px - PITCH * static_cast<float>(arena.markerI));
            const float dy = fabsf(py - PITCH * static_cast<float>(arena.markerJ));
            const float o = MARKER_OUTER_MM / 2;
            const float b = MARKER_BOX_MM / 2;
            return dx < o && dy < o && !(dx < b && dy < b) ? white
                                                           : static_cast<uint16_t>(BLACK_COUNTS);
        }
        if (along < GAP_FROM_MM) {
            return BLACK_COUNTS;
        }
        if (along < GAP_TO_MM) {
            return gapIsDark ? static_cast<uint16_t>(BLACK_COUNTS)
                             : static_cast<uint16_t>(AIR_COUNTS);
        }
        if (along < PATCH_TO_MM) {
            return BLACK_COUNTS;
        }
        if (lineGone) {
            return white;
        }
        const bool straight = along <= LINE_STRAIGHT_TO_MM && fabsf(lat) < LINE_HALF_MM;
        const bool bend = fabsf(along - LINE_STRAIGHT_TO_MM) < LINE_HALF_MM &&
                          lat >= -LINE_HALF_MM && lat <= BAR_LENGTH_MM;
        return straight || bend ? static_cast<uint16_t>(BLACK_COUNTS) : white;
    }

    // The array centre's distance along the exit direction.
    float lateAlongMm() const
    {
        float along = 0.0f;
        float lat = 0.0f;
        const float a = static_cast<float>(yawDeg) * 0.0174533f;
        exitFrame(static_cast<float>(x) + ARRAY_AHEAD_MM * cosf(a),
                  static_cast<float>(y) + ARRAY_AHEAD_MM * sinf(a), along, lat);
        return along;
    }

    // How far the IR array's centre is from the line's centreline: the distance to the straight or
    // to the bend's bar.
    float lineErrorMm() const
    {
        float along = 0.0f;
        float lat = 0.0f;
        const float a = static_cast<float>(yawDeg) * 0.0174533f;
        exitFrame(static_cast<float>(x) + ARRAY_AHEAD_MM * cosf(a),
                  static_cast<float>(y) + ARRAY_AHEAD_MM * sinf(a), along, lat);
        const float c = along < PATCH_TO_MM
                            ? PATCH_TO_MM
                            : (along > LINE_STRAIGHT_TO_MM ? LINE_STRAIGHT_TO_MM : along);
        const float dA = sqrtf((along - c) * (along - c) + lat * lat);
        const float l = lat < 0.0f ? 0.0f : (lat > BAR_LENGTH_MM ? BAR_LENGTH_MM : lat);
        const float dB = sqrtf((along - LINE_STRAIGHT_TO_MM) * (along - LINE_STRAIGHT_TO_MM) +
                               (lat - l) * (lat - l));
        return dA < dB ? dA : dB;
    }

    // The eight channels of the array: ahead of the axle, D1 at the robot's right end.
    void arrayCounts(uint16_t (&out)[IR_CHANNEL_COUNT]) const
    {
        const float a = static_cast<float>(yawDeg) * 0.0174533f;
        const float cx = static_cast<float>(x) + ARRAY_AHEAD_MM * cosf(a);
        const float cy = static_cast<float>(y) + ARRAY_AHEAD_MM * sinf(a);
        for (uint8_t ch = 0; ch < IR_CHANNEL_COUNT; ++ch) {
            const float across = (static_cast<float>(ch) - 3.5f) * IR_SENSOR_PITCH_MM *
                                 (IR_D1_ON_RIGHT ? 1.0f : -1.0f); // positive to the left
            // A channel sees a patch of floor about a pitch wide: it reads the average, so a line
            // edge moves its reading smoothly.
            float sum = 0.0f;
            for (int k = -2; k <= 2; ++k) {
                const float o = across + static_cast<float>(k) * (IR_SENSOR_PITCH_MM * 0.2f);
                sum += static_cast<float>(countsAt(cx - o * sinf(a), cy + o * cosf(a), ch));
            }
            out[ch] = static_cast<uint16_t>(sum / 5.0f + 0.5f);
        }
    }

    // The forward accelerometer axis: level, or tilted by the ramp under the axle.
    int16_t forwardAccel() const
    {
        float along = 0.0f;
        float lat = 0.0f;
        exitFrame(static_cast<float>(x), static_cast<float>(y), along, lat);
        float counts = 0.0f;
        if (arena.markerI >= 0 && along >= PATCH_TO_MM && along <= PATCH_TO_MM + RAMP_LENGTH_MM) {
            counts = static_cast<float>(IMU_ACCEL_ONE_G_COUNTS) * sinf(RAMP_SLOPE_DEG * 0.0174533f);
        }
        counts = IMU_ACCEL_NOSE_UP_NEGATIVE ? -counts : counts;
        return static_cast<int16_t>(IMU_ACCEL_FORWARD_LEVEL_COUNTS + counts);
    }

    // What the robot senses this tick, as the firmware's Perception.
    Perception sense()
    {
        if (tMs % US_SLOT_MS == 0) {
            sampleUltrasonic(static_cast<int>((tMs / US_SLOT_MS) % US_SENSOR_COUNT));
        }
        const double mmPerCount = MM_PER_ENCODER_COUNT * faults.odometerScale;
        state.encoderCountL = static_cast<int32_t>(floor(pos[0] / mmPerCount));
        state.encoderCountR = static_cast<int32_t>(floor(pos[1] / mmPerCount));
        state.yawRateDps =
            headingUpdate(heading, bias, gyroCounts(yawRateDps), CONTROL_LOOP_PERIOD_MS);
        state.headingDeg = heading.deg;
        state.distFrontMm = held[0];
        state.distLeftMm = held[1];
        state.distRightMm = held[2];
        state.usValid = valid;
        arrayCounts(state.irRaw);
        state.imu.accel[IMU_ACCEL_AXIS_FORWARD] = forwardAccel();
        Perception out;
        perceive(pc, state, pp, out);
        return out;
    }

    // Apply one command for one control tick.
    void drive(const DriveCommand &cmd)
    {
        const int n = CONTROL_LOOP_PERIOD_MS;
        const double duty[2] = {cmd.enable ? static_cast<double>(cmd.left) : 0.0,
                                cmd.enable ? static_cast<double>(cmd.right) : 0.0};
        const double kL = FEEDFORWARD_PWM_PER_MM_S;
        const double k[2] = {kL, kL / 0.8}; // the right wheel is weaker
        const double start[2] = {MOTOR_MIN_PWM_L * sticking, MOTOR_MIN_PWM_R * sticking};
        for (int step = 0; step < n; ++step) {
            for (int w = 0; w < 2; ++w) {
                const double a = fabs(duty[w]);
                const double sgn = duty[w] >= 0 ? 1.0 : -1.0;
                double ss = 0.0;
                const double keep = sustain * (w == 1 ? sustainRight : 1.0);
                if (a >= keep && (fabs(v[w]) > 2.0 || a >= start[w])) {
                    ss = sgn * (a - keep) / k[w];
                }
                v[w] += (ss - v[w]) * (0.001 / 0.02);
                // The body is stopped by the wall in front of it.
                if (v[0] + v[1] > 0 && rayFromAxle(static_cast<float>(yawDeg)) <= NOSE_MM) {
                    v[w] = 0.0;
                }
                pos[w] += v[w] * 0.001;
            }
            yawRateDps = (v[1] - v[0]) / TRACK_WIDTH_MM * 57.29578;
            yawDeg += yawRateDps * 0.001;
            const double mid = 0.5 * (v[0] + v[1]);
            const double a = yawDeg * 0.0174533;
            x += mid * cos(a) * 0.001;
            y += mid * sin(a) * 0.001;
        }
        tMs += CONTROL_LOOP_PERIOD_MS;

        const float front = rayFromAxle(static_cast<float>(yawDeg));
        const float left = rayFromAxle(static_cast<float>(yawDeg) + 90.0f);
        const float right = rayFromAxle(static_cast<float>(yawDeg) - 90.0f);
        minNoseMm = front < minNoseMm ? front : minNoseMm;
        const float side = left < right ? left : right;
        minSideMm = side < minSideMm ? side : minSideMm;
        const int ci = cellI();
        const int cj = cellJ();
        if (ci != lastI || cj != lastJ) {
            if (ci == startI && cj == startJ && leftStart) {
                ++startTileEntries;
            }
            if (!(ci == startI && cj == startJ)) {
                leftStart = true;
            }
            lastI = ci;
            lastJ = cj;
        }
    }
};

} // namespace world
