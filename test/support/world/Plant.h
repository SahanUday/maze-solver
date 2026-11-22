// The robot's body in the arena: two wheels with stiction, a pose, and the wall in front that
// stops the nose.
#pragma once

#include <math.h>

#include "../../model/Chassis.h"
#include "Arena.h"
#include "Angles.h"
#include "motion/DriveCommand.h"
#include "spec/fixed/Hardware.h"
#include "spec/measurements/Robot.h"

namespace world
{

constexpr double STEP_S = 0.001; // one physics step, a control tick takes CONTROL_LOOP_PERIOD_MS

struct Plant {
    // Wheel speeds (mm/s) and positions (mm), pose in the arena (mm, deg).
    double v[2] = {0, 0};
    double pos[2] = {0, 0};
    double x = 0;
    double y = 0;
    double yawDeg = 0;
    double yawRateDps = 0;
    // A stopped wheel needs MOTOR_MIN_PWM_* x `sticking` duty to start, a rolling one `sustain` x
    // (`sustainRight` for the right) to go on.
    float sticking = 1.0f;
    double sustain = model::KEEP_ROLLING_DUTY;
    double sustainRight = 1.0;
    ClearanceCache clearance;

    float headingRad() const
    {
        return static_cast<float>(yawDeg) * DEG_TO_RAD_F;
    }

    // The IR array's centre, ahead of the axle.
    void arrayCentre(float &cx, float &cy) const
    {
        const float a = headingRad();
        cx = static_cast<float>(x) + IR_ARRAY_FORWARD_OFFSET_MM * cosf(a);
        cy = static_cast<float>(y) + IR_ARRAY_FORWARD_OFFSET_MM * sinf(a);
    }

    // Distance from the axle along `angleDeg` to the nearest wall.
    float ray(const Arena &arena, float angleDeg) const
    {
        const float a = angleDeg * DEG_TO_RAD_F;
        return arena.ray(static_cast<float>(x), static_cast<float>(y), cosf(a), sinf(a));
    }

    // The wall in front is within the nose.
    bool blocked(const Arena &arena)
    {
        const float nose = model::NOSE_MM;
        return !clearance.noWallWithin(arena, static_cast<float>(x), static_cast<float>(y), nose) &&
               ray(arena, static_cast<float>(yawDeg)) <= nose;
    }

    // One control tick of `cmd`.
    void step(const DriveCommand &cmd, const Arena &arena)
    {
        const double duty[2] = {cmd.enable ? static_cast<double>(cmd.left) : 0.0,
                                cmd.enable ? static_cast<double>(cmd.right) : 0.0};
        const double kL = model::WHEEL_DUTY_PER_MM_S;
        const double k[2] = {kL, kL / model::RIGHT_WHEEL_GAIN};
        const double start[2] = {MOTOR_MIN_PWM_L * sticking, MOTOR_MIN_PWM_R * sticking};
        for (int ms = 0; ms < CONTROL_LOOP_PERIOD_MS; ++ms) {
            for (int w = 0; w < 2; ++w) {
                const double a = fabs(duty[w]);
                const double sgn = duty[w] >= 0 ? 1.0 : -1.0;
                double ss = 0.0;
                const double keep = sustain * (w == 1 ? sustainRight : 1.0);
                if (a >= keep && (fabs(v[w]) > 2.0 || a >= start[w])) {
                    ss = sgn * (a - keep) / k[w];
                }
                v[w] += (ss - v[w]) * (STEP_S / model::WHEEL_LAG_S);
                // The body is stopped by the wall in front of it.
                if (v[0] + v[1] > 0 && blocked(arena)) {
                    v[w] = 0.0;
                }
                pos[w] += v[w] * STEP_S;
            }
            yawRateDps = (v[1] - v[0]) / TRACK_WIDTH_MM * RAD_TO_DEG;
            yawDeg += yawRateDps * STEP_S;
            const double mid = 0.5 * (v[0] + v[1]);
            const double a = yawDeg * DEG_TO_RAD;
            x += mid * cos(a) * STEP_S;
            y += mid * sin(a) * STEP_S;
        }
    }
};

} // namespace world
