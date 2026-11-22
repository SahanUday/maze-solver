// What the robot reads from the plant and the floor: ultrasonic echoes, gyro counts, the IR array
// and the forward accelerometer axis.
#pragma once

#include <math.h>
#include <stdint.h>

#include "../../model/Infrared.h"
#include "../../model/Sensors.h"
#include "Arena.h"
#include "Floor.h"
#include "Plant.h"
#include "Rng.h"
#include "Units.h"
#include "spec/fixed/Hardware.h"
#include "spec/measurements/Robot.h"

namespace world
{

// Knobs a test turns.
struct Faults {
    float noiseMm = model::US_NOISE_MM; // ultrasonic noise, 1 sigma
    float dropout = 0.0f;               // chance a reading gets no echo
    float strayNear = 0.0f; // chance a wall reading is a false short echo (30 to 100 mm)
    float gyroScale = model::GYRO_SCALE_ERROR;
    float gyroDriftDps = 0.0f;
    float odometerScale = model::ODOMETER_SCALE;
    // Something close in front (a hand, a dropped part) between these times: the front reads 70 mm.
    uint32_t obstacleFromMs = 0;
    uint32_t obstacleToMs = 0;
};

// The three modules' last readings, as the driver holds them.
struct Ultrasonics {
    static constexpr float RANGE_CAP_MM = US_RANGE_CAP_MM;

    uint16_t held[US_SENSOR_COUNT] = {600, 117, 117}; // front, left, right
    uint8_t valid = 0x07;                             // a bit per module that heard an echo

    void sample(int slot, uint32_t tMs, const Plant &plant, const Arena &arena,
                const Faults &faults, Lcg &rng)
    {
        if (rng.u() < faults.dropout) {
            valid = static_cast<uint8_t>(valid & ~(1u << slot));
            return;
        }
        valid = static_cast<uint8_t>(valid | (1u << slot));
        const float yaw = static_cast<float>(plant.yawDeg);
        const float angle = yaw + (slot == 0 ? 0.0f : (slot == 1 ? 90.0f : -90.0f));
        const float d = plant.ray(arena, angle);
        if (slot == 0 && (d - US_OFFSET_FRONT_MM < model::US_DEAD_ZONE_MM || d > RANGE_CAP_MM)) {
            valid = static_cast<uint8_t>(valid & ~1u); // the dead zone, or out of range
            return;
        }
        float mm = (d > RANGE_CAP_MM ? RANGE_CAP_MM : d) + rng.g() * faults.noiseMm;
        if (slot == 0 && tMs >= faults.obstacleFromMs && tMs < faults.obstacleToMs) {
            mm = 70.0f + rng.g() * faults.noiseMm;
        }
        if (d < 400.0f && rng.u() < faults.strayNear) {
            mm = 30.0f + rng.u() * 70.0f;
        }
        held[slot] = static_cast<uint16_t>(mm < 0.0f ? 0.0f : mm);
    }
};

inline int16_t gyroCounts(double rateDps, const Faults &faults, Lcg &rng)
{
    const double c = rateDps * GYRO_COUNTS_PER_DPS * faults.gyroScale + model::GYRO_OFFSET_COUNTS +
                     faults.gyroDriftDps * GYRO_COUNTS_PER_DPS + rng.g() * model::GYRO_NOISE_COUNTS;
    return static_cast<int16_t>(lround(c));
}

// What a channel reads over a surface.
inline uint16_t irCounts(Surface s, uint8_t channel)
{
    switch (s) {
        case Surface::Marker:
        case Surface::Board:
        case Surface::Finish:
            return model::WHITE.on(channel);
        case Surface::Air:
            return model::AIR.on(channel);
        default:
            return model::BLACK.on(channel);
    }
}

// The eight channels, ahead of the axle, D1 at the robot's right end.
inline void irArray(const Floor &floor, const Plant &plant, uint16_t (&out)[IR_CHANNEL_COUNT])
{
    const float a = plant.headingRad();
    float cx;
    float cy;
    plant.arrayCentre(cx, cy);
    for (uint8_t ch = 0; ch < IR_CHANNEL_COUNT; ++ch) {
        const float across = (static_cast<float>(ch) - 3.5f) * IR_SENSOR_PITCH_MM *
                             (IR_D1_ON_RIGHT ? 1.0f : -1.0f); // positive to the left
        // A channel sees a patch of floor about a pitch wide: it reads the average, so a line
        // edge moves its reading smoothly.
        float sum = 0.0f;
        for (int k = -2; k <= 2; ++k) {
            const float o = across + static_cast<float>(k) * (IR_SENSOR_PITCH_MM * 0.2f);
            const Surface s = floor.surfaceAt(cx - o * sinf(a), cy + o * cosf(a));
            sum += static_cast<float>(irCounts(s, ch));
        }
        out[ch] = static_cast<uint16_t>(sum / 5.0f + 0.5f);
    }
}

// The forward accelerometer axis: level, or tilted by the ramp under the axle.
inline int16_t forwardAccel(const Floor &floor, const Plant &plant)
{
    const float slope = floor.slopeDeg(static_cast<float>(plant.x), static_cast<float>(plant.y));
    float counts = static_cast<float>(IMU_ACCEL_ONE_G_COUNTS) * sinf(slope * DEG_TO_RAD_F);
    counts = IMU_ACCEL_NOSE_UP_NEGATIVE ? -counts : counts;
    return static_cast<int16_t>(IMU_ACCEL_FORWARD_LEVEL_COUNTS + counts);
}

} // namespace world
