// One master seed per run: every random stream and maze of the run derives from it, so one number
// replays it. A seeded test notes the seed; a suite with
//   void tearDown(void) { seeds::report(); }
// prints it beside a failure.
#pragma once

#include <stdint.h>
#include <unity.h>

namespace seeds
{

// The consumers of a master seed, each with a stream of its own.
enum Stream : uint32_t { NOISE = 1, IR = 2, MAZE = 3 };

// The seed of `stream` for a master seed (splitmix32).
inline uint32_t derive(uint32_t master, uint32_t stream)
{
    uint32_t z = master + stream * 0x9E3779B9u;
    z = (z ^ (z >> 16)) * 0x85EBCA6Bu;
    z = (z ^ (z >> 13)) * 0xC2B2AE35u;
    return z ^ (z >> 16);
}

struct Noted {
    uint32_t master = 0;
    bool set = false;
};

inline Noted &noted()
{
    static Noted n;
    return n;
}

inline void note(uint32_t master)
{
    noted() = {master, true};
}

// For tearDown: print the noted seed if the test failed.
inline void report()
{
    if (Unity.CurrentTestFailed && noted().set) {
        UnityPrint(" [seed ");
        UnityPrintNumberUnsigned(noted().master);
        UnityPrint("]");
    }
    noted() = {};
}

} // namespace seeds
