// The map cell of arena tile (i, j) when a start-relative map of `side` x `side` tiles began on
// tile (startI, startJ) facing `facing` (0 east, 1 north, 2 west, 3 south): the map's North is the
// start heading, and the start tile is its centre.
#pragma once

namespace mapindex
{

inline int of(int side, int i, int j, int startI, int startJ, int facing)
{
    static const int fwdI[4] = {1, 0, -1, 0};
    static const int fwdJ[4] = {0, 1, 0, -1};
    const int di = i - startI;
    const int dj = j - startJ;
    const int fwd = di * fwdI[facing] + dj * fwdJ[facing];
    const int right = di * fwdJ[facing] - dj * fwdI[facing]; // right of the heading
    return (side / 2 - fwd) * side + side / 2 + right;
}

} // namespace mapindex
