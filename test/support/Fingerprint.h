// A running hash of what the world hands the firmware and takes back, so a refactor of the world
// can be shown to change no tick: build with -DWORLD_FINGERPRINT and compare the line printed at
// exit before and after. Without the flag the calls compile to nothing.
#pragma once

#include "core/RobotState.h"
#include "motion/DriveCommand.h"

#ifdef WORLD_FINGERPRINT

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "spec/measurements/Robot.h"

namespace world
{

struct Fingerprint {
    uint64_t hash = 14695981039346656037ull; // FNV-1a
    uint32_t ticks = 0;

    void add(uint64_t v)
    {
        hash = (hash ^ v) * 1099511628211ull;
    }
    void add(int64_t v)
    {
        add(static_cast<uint64_t>(v));
    }
    void add(double v)
    {
        uint64_t bits;
        memcpy(&bits, &v, sizeof bits);
        add(bits);
    }

    ~Fingerprint()
    {
        fprintf(stderr, "FINGERPRINT ticks=%u hash=%016llx\n", ticks,
                static_cast<unsigned long long>(hash));
    }
};

inline Fingerprint &fingerprint()
{
    static Fingerprint f;
    return f;
}

// What the robot senses this tick.
inline void hashSensed(const RobotState &s)
{
    Fingerprint &f = fingerprint();
    ++f.ticks;
    f.add(static_cast<int64_t>(s.encoderCountL));
    f.add(static_cast<int64_t>(s.encoderCountR));
    f.add(static_cast<double>(s.headingDeg));
    f.add(static_cast<double>(s.yawRateDps));
    f.add(static_cast<uint64_t>(s.distFrontMm));
    f.add(static_cast<uint64_t>(s.distLeftMm));
    f.add(static_cast<uint64_t>(s.distRightMm));
    f.add(static_cast<uint64_t>(s.usValid));
    for (uint16_t ir : s.irRaw) {
        f.add(static_cast<uint64_t>(ir));
    }
    f.add(static_cast<int64_t>(s.imu.accel[IMU_ACCEL_AXIS_FORWARD]));
}

// What the robot commanded, and where the plant is after it.
inline void hashCommanded(const DriveCommand &c, double x, double y, double yawDeg, double vL,
                          double vR)
{
    Fingerprint &f = fingerprint();
    f.add(static_cast<int64_t>(c.left));
    f.add(static_cast<int64_t>(c.right));
    f.add(static_cast<uint64_t>(c.enable));
    f.add(x);
    f.add(y);
    f.add(yawDeg);
    f.add(vL);
    f.add(vR);
}

} // namespace world

#else

namespace world
{

inline void hashSensed(const RobotState &) {}
inline void hashCommanded(const DriveCommand &, double, double, double, double, double) {}

} // namespace world

#endif
