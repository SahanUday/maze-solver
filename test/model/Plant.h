// ============================================================================
//  model/Plant.h  --  the robot's body and drive as the host world moves it. Only the simulator
//  reads these; what the firmware reads is in include/spec/. Tags: include/spec/README.md.
// ============================================================================

#pragma once

namespace model
{

// ---- Body ------------------------------------------------- <<TBD HARDWARE>> --
// From the wheel axle, with a ruler (#54).
constexpr float NOSE_MM = 89.0f;      // to the front of the body
constexpr float BODY_HALF_MM = 90.0f; // to a tyre's outer edge

// ---- Wheels ---------------------------------------------------- [MEASURED] --
// A rolling wheel keeps going down to this duty (on the stand, PR #28). What a stopped wheel needs
// to start: MOTOR_MIN_PWM_* in spec/measurements/Robot.h.
constexpr double KEEP_ROLLING_DUTY = 110.0;

// ---- Wheel response --------------------------------------- <<TBD HARDWARE>> --
// Speed per duty above the keep-rolling duty, as the left wheel gives it; today the controller's
// own feedforward gain. Measure speed against duty at three levels (#54).
constexpr float WHEEL_DUTY_PER_MM_S = 1.15f;
constexpr double RIGHT_WHEEL_GAIN = 0.8; // the right wheel is weaker
constexpr double WHEEL_LAG_S = 0.02;     // time constant of a speed step

// ---- Odometer ---------------------------------------------- <<TBD HARDWARE>> --
// True mm per count over the spec's. Tape-measure 2 m against the counts (#54).
constexpr float ODOMETER_SCALE = 1.003f;

} // namespace model
