// ============================================================================
//  mission/MissionParams.h  --  what the mission manager runs with. Pure logic, no hardware.
//  docs/architecture/modules/mission_manager.md
// ============================================================================

#pragma once

#include "mission/CellExplorer.h"
#include "mission/MarkerExit.h"
#include "mission/Parts.h"
#include "motion/LineFollow.h"
#include "motion/MotionControl.h"
#include "motion/MotionPrimitives.h"
#include "spec/fixed/Hardware.h"
#include "spec/tuning/FloorPattern.h"
#include "spec/tuning/Imu.h"
#include "spec/tuning/LineFollow.h"
#include "spec/tuning/Motion.h"

struct MissionParams : CellExplorerParams, MarkerExitParams {
    PartRange parts; // which parts to run: the whole course by default
    MotionControlParams control;
    LineParams line;
};

// Every parameter at its spec/ value: what the robot runs with.
inline MissionParams missionParams()
{
    MissionParams p;
    PlanParams &plan = p.step.plan;
    plan.accelMmS2 = ACCEL_MM_S2;
    plan.turnRateMaxDps = TURN_RATE_MAX_DPS;
    plan.turnAccelDps2 = TURN_ACCEL_DPS2;
    plan.stepToleranceMm = STEP_TOLERANCE_MM;
    plan.turnToleranceDeg = TURN_TOLERANCE_DEG;
    plan.turnSettleMs = TURN_SETTLE_MS;
    plan.progressMinMm = PROGRESS_MIN_MM;
    plan.progressMinDeg = PROGRESS_MIN_DEG;
    plan.patienceMs = PATIENCE_MS;

    MotionControlParams &c = p.control;
    c.kpDist = KP_DISTANCE;
    c.kiDist = KI_DISTANCE;
    c.kdDist = KD_DISTANCE;
    c.kpTurn = KP_TURN;
    c.kiTurn = KI_TURN;
    c.kdTurn = KD_TURN;
    c.kiHold = KI_TURN_HOLD;
    c.ffScaleR = FEEDFORWARD_RIGHT_SCALE;
    c.ffPwmPerMmS = FEEDFORWARD_PWM_PER_MM_S;
    c.ffPwmPerDps = FEEDFORWARD_PWM_PER_DPS;
    c.ffPwmFriction = FEEDFORWARD_FRICTION_DUTY;
    c.minPwmL = CONTROL_MIN_PWM_L;
    c.minPwmR = CONTROL_MIN_PWM_R;
    c.pwmTop = MOTOR_PWM_TOP;
    c.stallDuty = STALL_DUTY;
    c.stallTimeMs = STALL_TIME_MS;
    c.stallMinSpeedMmS = STALL_MIN_SPEED_MM_S;
    c.stallMinYawDps = STALL_MIN_YAW_DPS;

    p.pattern.frameMinMm = MARKER_A_FRAME_MIN_MM;
    p.pattern.frameMaxMm = MARKER_A_FRAME_MAX_MM;
    p.pattern.farFrameMinMm = MARKER_A_FAR_FRAME_MIN_MM;
    p.pattern.centreMinMm = MARKER_A_CENTRE_MIN_MM;
    p.pattern.centreMaxMm = MARKER_A_CENTRE_MAX_MM;
    p.pattern.gapMinMm = GAP_MIN_MM;
    p.pattern.glitchMm = FLOOR_GLITCH_MM;

    LineParams &l = p.line;
    l.kp = KP_LINE;
    l.kd = KD_LINE;
    l.steerMaxDeg = LINE_STEER_MAX_DEG;
    l.speedMmS = LINE_SPEED_MM_S;
    l.accelMmS2 = LINE_ACCEL_MM_S2;
    l.slowdownErrMm = LINE_SLOWDOWN_ERR_MM;
    l.slowdownPct = LINE_SLOWDOWN_PCT;
    l.searchRateDps = LINE_SEARCH_RATE_DPS;
    l.searchSpeedMmS = LINE_SEARCH_SPEED_MM_S;
    l.lostTimeoutMs = LINE_LOST_TIMEOUT_MS;
    l.endLostMm = LINE_END_LOST_MM;
    l.rampInclineCounts = RAMP_INCLINE_COUNTS;
    return p;
}
