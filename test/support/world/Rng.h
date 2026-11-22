// A seedable random source: the same seed gives the same run.
#pragma once

#include <stdint.h>

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

} // namespace world
