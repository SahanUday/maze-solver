// The host world: an arena, a floor, a plant and its sensors, assembled into the RobotState the
// firmware reads and run through perceive(). Presets that fill one: Presets.h.
#pragma once

#include <initializer_list>
#include <math.h>
#include <stdint.h>

#include "../Fingerprint.h"
#include "../Seed.h"
#include "Arena.h"
#include "Floor.h"
#include "Plant.h"
#include "Rng.h"
#include "Sensors.h"
#include "core/RobotState.h"
#include "motion/DriveCommand.h"
#include "motion/Heading.h"
#include "sense/LineSense.h"
#include "sense/Perception.h"
#include "spec/fixed/Hardware.h"
#include "spec/tuning/Imu.h"

namespace world
{

// What a run did, for the asserts.
struct Metrics {
    float minNoseMm = FAR_MM; // nearest front wall
    float minSideMm = FAR_MM; // nearest side wall
    int startTileEntries = 0; // re-entries of the start tile after leaving it
    int startI = 0;
    int startJ = 0;
    bool leftStart = false;
    int lastI = 0;
    int lastJ = 0;

    void begin(int i, int j)
    {
        *this = Metrics{};
        startI = lastI = i;
        startJ = lastJ = j;
    }

    // A wall farther than this cannot lower either minimum.
    float horizonMm() const
    {
        return minNoseMm > minSideMm ? minNoseMm : minSideMm;
    }

    void track(int i, int j, float frontMm, float leftMm, float rightMm)
    {
        minNoseMm = frontMm < minNoseMm ? frontMm : minNoseMm;
        const float side = leftMm < rightMm ? leftMm : rightMm;
        minSideMm = side < minSideMm ? side : minSideMm;
        if (i != lastI || j != lastJ) {
            if (i == startI && j == startJ && leftStart) {
                ++startTileEntries;
            }
            if (!(i == startI && j == startJ)) {
                leftStart = true;
            }
            lastI = i;
            lastJ = j;
        }
    }
};

struct World {
    Arena arena;
    Floor floor;
    Plant plant;
    Faults faults;
    Lcg rng;
    Lcg irRng{777u};
    Ultrasonics ultrasonics;
    Metrics metrics;

    RobotState state;
    Perceiver pc;
    PerceiveParams pp;
    GyroBias bias;
    Heading heading;
    uint32_t tMs = 0;

    // Reseed every random stream from one master seed. An unseeded world runs on fixed streams.
    void seed(uint32_t master)
    {
        rng.s = seeds::derive(master, seeds::NOISE);
        irRng.s = seeds::derive(master, seeds::IR);
        seeds::note(master);
    }

    int cellI() const
    {
        return static_cast<int>(lroundf(static_cast<float>(plant.x) / PITCH));
    }
    int cellJ() const
    {
        return static_cast<int>(lroundf(static_cast<float>(plant.y) / PITCH));
    }

    // Place the robot on tile (i, j) facing `facing` quarter turns counter-clockwise from east, off
    // by `offX`, `offY` mm and `offYaw` deg, and boot it.
    void start(int i, int j, int facing, float offX = 0.0f, float offY = 0.0f, float offYaw = 0.0f)
    {
        place(PITCH * i + offX, PITCH * j + offY, 90.0 * facing + offYaw);
        metrics.begin(i, j);
        boot();
    }

    // The same, at a point of the arena (mm) facing `yawDeg`.
    void startAt(double x, double y, double yawDeg)
    {
        place(x, y, yawDeg);
        metrics.begin(cellI(), cellJ());
        boot();
    }

    // The IR calibration: the white of the marker, then the black of the maze floor.
    static LineCalibration calibration()
    {
        LineCalibration cal;
        lineCalibrationReset(cal);
        uint16_t counts[IR_CHANNEL_COUNT];
        for (const Surface s : {Surface::Marker, Surface::MazeFloor}) {
            for (uint8_t ch = 0; ch < IR_CHANNEL_COUNT; ++ch) {
                counts[ch] = static_cast<uint16_t>(irLevel(s).on(ch));
            }
            lineCalibrationAccumulate(cal, counts);
        }
        lineCalibrationFinish(cal);
        return cal;
    }

    // The IR array's distance from the line's centre.
    float lineErrorMm() const
    {
        const Xy c = plant.arrayCentre();
        return floor.lineErrorMm(c.x, c.y);
    }

    // The IR array's distance along the floor's frame.
    float arrayAlongMm() const
    {
        const Xy c = plant.arrayCentre();
        return floor.alongMm(c.x, c.y);
    }

    // What the robot senses this tick, as the firmware's Perception.
    Perception sense()
    {
        if (tMs % US_SLOT_MS == 0) {
            ultrasonics.sample(static_cast<int>((tMs / US_SLOT_MS) % US_SENSOR_COUNT), tMs, plant,
                               arena, faults, rng);
        }
        const double mmPerCount = MM_PER_ENCODER_COUNT * faults.odometerScale;
        state.encoderCountL = static_cast<int32_t>(::floor(plant.pos[0] / mmPerCount));
        state.encoderCountR = static_cast<int32_t>(::floor(plant.pos[1] / mmPerCount));
        state.yawRateDps = headingUpdate(heading, bias, gyroCounts(plant.yawRateDps, faults, rng),
                                         CONTROL_LOOP_PERIOD_MS);
        state.headingDeg = heading.deg;
        state.distFrontMm = ultrasonics.held[0];
        state.distLeftMm = ultrasonics.held[1];
        state.distRightMm = ultrasonics.held[2];
        state.usValid = ultrasonics.valid;
        irArray(floor, plant, faults, irRng, state.irRaw);
        state.imu.accel[IMU_ACCEL_AXIS_FORWARD] = forwardAccel(floor, plant);
        hashSensed(state);
        Perception out;
        perceive(pc, state, pp, out);
        return out;
    }

    // Apply one command for one control tick.
    void drive(const DriveCommand &cmd)
    {
        plant.step(cmd, arena);
        tMs += CONTROL_LOOP_PERIOD_MS;
        hashCommanded(cmd, plant.x, plant.y, plant.yawDeg, plant.v[0], plant.v[1]);

        float front = FAR_MM;
        float left = FAR_MM;
        float right = FAR_MM;
        if (!plant.clearance.noWallWithin(arena, static_cast<float>(plant.x),
                                          static_cast<float>(plant.y), metrics.horizonMm())) {
            const float yaw = static_cast<float>(plant.yawDeg);
            front = plant.ray(arena, yaw);
            left = plant.ray(arena, yaw + 90.0f);
            right = plant.ray(arena, yaw - 90.0f);
        }
        metrics.track(cellI(), cellJ(), front, left, right);
    }

  private:
    void place(double x, double y, double yawDeg)
    {
        plant.x = x;
        plant.y = y;
        plant.yawDeg = yawDeg;
        plant.clearance.forget();
    }

    // Power-up: the gyro's zero and the IR calibration.
    void boot()
    {
        state.imu.accel[IMU_ACCEL_AXIS_FORWARD] = IMU_ACCEL_FORWARD_LEVEL_COUNTS;
        gyroBiasReset(bias);
        for (uint16_t k = 0; k < GYRO_BIAS_SAMPLES; ++k) {
            gyroBiasAccumulate(bias, gyroCounts(0.0, faults, rng));
        }
        gyroBiasFinish(bias);
        perceiverSetCalibration(pc, calibration());
    }
};

} // namespace world
