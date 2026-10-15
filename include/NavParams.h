// ============================================================================
//  NavParams.h  --  RobotSpec constants gathered into the parameter structs the
//  control modules take. No hardware, no Arduino.h.
//
//  The modules take structs, not the constants, so the host tests can run them
//  with values of their own (the spec's are 0 until calibrated). This is the one
//  place that reads the spec for them.
// ============================================================================

#pragma once

#include "FloorPattern.h"
#include "LineFollow.h"
#include "Mission.h"
#include "MotionControl.h"
#include "Perception.h"
#include "RobotSpec.h"
#include "StepCell.h"

inline MotionControlParams specMotionControl()
{
    MotionControlParams p;
    p.kpDist = KP_DISTANCE;
    p.kiDist = KI_DISTANCE;
    p.kdDist = KD_DISTANCE;
    p.kpTurn = KP_TURN;
    p.kiTurn = KI_TURN;
    p.kiHold = KI_TURN_HOLD;
    p.kdTurn = KD_TURN;
    p.ffPwmPerMmS = FEEDFORWARD_PWM_PER_MM_S;
    p.ffPwmPerDps = FEEDFORWARD_PWM_PER_DPS;
    p.ffScaleR = FEEDFORWARD_RIGHT_SCALE;
    p.minPwmL = CONTROL_MIN_PWM_L;
    p.minPwmR = CONTROL_MIN_PWM_R;
    p.pwmTop = MOTOR_PWM_TOP;
    p.stallDuty = STALL_DUTY;
    p.stallTimeMs = STALL_TIME_MS;
    p.stallMinSpeedMmS = STALL_MIN_SPEED_MM_S;
    p.stallMinYawDps = STALL_MIN_YAW_DPS;
    return p;
}

inline WallParams specWall()
{
    WallParams p;
    p.sideMm = WALL_PRESENT_SIDE_MM;
    p.frontMm = WALL_PRESENT_FRONT_MM;
    p.centerDistMm = WALL_CENTER_DIST_MM;
    p.kp = KP_WALL;
    p.kd = KD_WALL;
    p.steerMaxDeg = STEER_MAX_DEG;
    return p;
}

inline PlanParams specPlan()
{
    PlanParams p;
    p.accelMmS2 = ACCEL_MM_S2;
    p.turnRateMaxDps = TURN_RATE_MAX_DPS;
    p.turnAccelDps2 = TURN_ACCEL_DPS2;
    p.stepToleranceMm = STEP_TOLERANCE_MM;
    p.turnToleranceDeg = TURN_TOLERANCE_DEG;
    p.turnSettleMs = TURN_SETTLE_MS;
    p.moveTimeoutMs = MOVE_TIMEOUT_MS;
    return p;
}

inline StepCellParams specStepCell()
{
    StepCellParams p;
    p.plan = specPlan();
    p.wall = specWall();
    p.pitchMm = static_cast<float>(TILE_PITCH_MM);
    p.cruiseMmS = SEARCH_SPEED_MM_S;
    p.sampleLeadMm = static_cast<float>(WALL_SAMPLE_LEAD_MM);
    p.sampleWindowMm = static_cast<float>(WALL_SAMPLE_WINDOW_MM);
    p.frontStopMm = static_cast<float>(FRONT_STOP_MM);
    return p;
}

inline LineParams specLine()
{
    LineParams p;
    p.kp = KP_LINE;
    p.kd = KD_LINE;
    p.steerMaxDeg = LINE_STEER_MAX_DEG;
    p.speedMmS = LINE_SPEED_MM_S;
    p.accelMmS2 = LINE_ACCEL_MM_S2;
    p.slowdownErrMm = LINE_SLOWDOWN_ERR_MM;
    p.slowdownPct = LINE_SLOWDOWN_PCT;
    p.searchRateDps = LINE_SEARCH_RATE_DPS;
    p.lostTimeoutMs = LINE_LOST_TIMEOUT_MS;
    p.searchSpeedMmS = LINE_SEARCH_SPEED_MM_S;
    p.endLostMm = LINE_END_LOST_MM;
    p.rampTiltDeg = RAMP_TILT_DEG;
    return p;
}

inline PatternParams specPattern()
{
    PatternParams p;
    p.frameMinMm = MARKER_FRAME_MIN_MM;
    p.frameMaxMm = MARKER_FRAME_MAX_MM;
    p.reportMm = MARKER_FRAME_REPORT_MM;
    p.centreMinMm = MARKER_CENTRE_MIN_MM;
    p.centreMaxMm = MARKER_CENTRE_MAX_MM;
    p.gapMinMm = GAP_MIN_MM;
    p.glitchMm = FLOOR_GLITCH_MM;
    return p;
}

inline FloorParams specFloor()
{
    FloorParams p;
    p.markerMinMm = MARKER_RUN_MIN_MM;
    p.markerMaxMm = MARKER_RUN_MAX_MM;
    p.lineEnterMm = LINE_ENTER_RUN_MM;
    p.gapMm = FLOOR_RUN_GAP_MM;
    return p;
}

inline PerceiveParams specPerceive()
{
    PerceiveParams p;
    p.mmPerCount = MM_PER_ENCODER_COUNT;
    p.dtMs = CONTROL_LOOP_PERIOD_MS;
    p.wall = specWall();
    p.irPitchMm = IR_SENSOR_PITCH_MM;
    p.irD1OnRight = IR_D1_ON_RIGHT;
    p.irAirCounts = IR_AIR_COUNTS;
    p.irAirChannels = IR_AIR_CHANNELS;
    return p;
}

// Time to stand and read the start cell's walls: every ultrasonic gets a fresh reading, plus slack.
constexpr uint16_t START_SENSE_MS = US_SENSOR_COUNT * US_SLOT_MS + 40;

// `hand` = +1 prefer left turns on a tie, -1 right (RUN_HAND, or a DIP switch if
// RUN_USE_DIP_SWITCHES).
inline MissionParams specMission(int8_t hand)
{
    MissionParams p;
    p.step = specStepCell();
    p.line = specLine();
    p.pattern = specPattern();
    p.bridgeMinLineMm = BRIDGE_HANDOVER_MIN_MM;
    p.bridgeMaxMm = BRIDGE_MAX_MM;
    p.control = specMotionControl();
    p.senseMs = START_SENSE_MS;
    p.hand = hand;
    p.wallsOnly = !RUN_USE_FLOOR_SENSING;
    p.dtMs = CONTROL_LOOP_PERIOD_MS;
    return p;
}
