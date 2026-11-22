// ============================================================================
//  model/Sensors.h  --  what the gyro and the ultrasonic modules give the host world. Only the
//  simulator reads these; the IR array is in Infrared.h. Tags: include/spec/README.md.
// ============================================================================

#pragma once

namespace model
{

// ---- Gyro scale ---------------------------------------------- [MEASURED] --
// Measured / true, 1.001 to 1.002 (docs/calibration).
constexpr float GYRO_SCALE_ERROR = 1.0015f;

// ---- Gyro offset and noise -------------------------------- <<TBD HARDWARE>> --
// Counts, before the bias capture at boot takes the offset out. Log 10 minutes at rest (#54).
constexpr double GYRO_OFFSET_COUNTS = 3.0;
constexpr double GYRO_NOISE_COUNTS = 2.4; // 1 sigma

// ---- Ultrasonic ----------------------------------------------- [MEASURED] --
// The front module hears nothing closer than this (bench, PR #17).
constexpr float US_DEAD_ZONE_MM = 30.0f;

// ---- Ultrasonic noise ------------------------------------- <<TBD HARDWARE>> --
// 1 sigma. The bench read 0 to 2.4 mm at 100 to 300 mm; log 1 m on a wall (#54).
constexpr float US_NOISE_MM = 3.0f;

} // namespace model
