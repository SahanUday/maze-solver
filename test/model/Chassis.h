// ============================================================================
//  model/Chassis.h  --  the robot's body, wheels and odometer as the host world moves them. Only
//  the simulator reads these; the firmware's are in include/spec/. Tags: include/spec/README.md.
// ============================================================================

#pragma once

namespace model
{

// ---- Body ------------------------------------------------- <<TBD HARDWARE>> --
// From the wheel axle, with a ruler (#54).
constexpr float NOSE_MM = 89.0f;            // to the front of the body
constexpr float BODY_HALF_WIDTH_MM = 90.0f; // to a tyre's outer edge

// ---- Wheels ---------------------------------------------------- [MEASURED] --
// A rolling wheel keeps going down to this duty (on the stand, PR #28). To start: MOTOR_MIN_PWM_*.
constexpr double KEEP_ROLLING_DUTY = 110.0;

// ---- Wheel response --------------------------------------- <<TBD HARDWARE>> --
// Duty per mm/s above the keep-rolling duty, left wheel; today the controller's own gain. Measure
// speed against duty at three levels (#54).
constexpr float WHEEL_DUTY_PER_MM_S = 1.15f;
constexpr double RIGHT_WHEEL_GAIN = 0.8; // the right wheel is weaker
constexpr double WHEEL_LAG_S = 0.02;     // time constant of a speed step

// ---- Odometer ---------------------------------------------- <<TBD HARDWARE>> --
// True mm per count over the spec's. Tape-measure 2 m against the counts (#54).
constexpr float ODOMETER_SCALE_ERROR = 1.003f;

} // namespace model
