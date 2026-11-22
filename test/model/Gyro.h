// ============================================================================
//  model/Gyro.h  --  what the gyro gives the host world. Only the simulator reads these. Tags:
//  include/spec/README.md.
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

} // namespace model
