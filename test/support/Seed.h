// One master seed per run: every random stream and generated maze of the run derives from it, so a
// single number replays the run. A test that seeds a run notes the seed, and a suite that defines
//   void tearDown(void) { seeds::report(); }
// has it printed beside the failure.
#pragma once

#include <stdint.h>
#include <unity.h>

namespace seeds
{

// The consumers of a master seed; each draws from a stream of its own.
enum Stream : uint32_t { NOISE = 1, IR = 2, MAZE = 3 };

// The seed of `stream` for a master seed (a splitmix32 step: nearby masters give unrelated seeds).
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

// For tearDown: print the noted seed if the test failed, then forget it.
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
