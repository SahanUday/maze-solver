// ============================================================================
//  model/Ultrasonic.h  --  what the ultrasonic modules give the host world. Only the simulator
//  reads these. Tags: include/spec/README.md.
// ============================================================================

#pragma once

namespace model
{

// ---- Dead zone ------------------------------------------------ [MEASURED] --
// The front module hears nothing closer than this (bench, PR #17).
constexpr float US_DEAD_ZONE_MM = 30.0f;

// ---- Noise ------------------------------------------------ <<TBD HARDWARE>> --
// 1 sigma. The bench read 0 to 2.4 mm at 100 to 300 mm; log 1 m on a wall (#54).
constexpr float US_NOISE_MM = 3.0f;

} // namespace model
