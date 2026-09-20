// ============================================================================
//  Maze.h  --  placeholder for the hardware-independent maze library.
//
//  This library must never include <Arduino.h>. That is the whole point of
//  splitting it out: it is what lets mapping/exploration/path-finding code
//  compile and run its unit tests on a laptop (env:native) as well as on
//  the Mega (env:mega), and what lets the SIM-01 simulator link against the
//  exact same code the robot runs.
//
//  Real maze types (wall map, cell graph, flood-fill) land here starting
//  with SW-03 / SIM-01. This file only proves the library builds and is
//  reachable from a native unit test (SW-01's "compiles on the host PC"
//  outcome) before that code exists.
// ============================================================================

#pragma once

namespace maze
{

constexpr int kPlaceholderOk = 1;

} // namespace maze
