// ============================================================================
//  test_nav_params.cpp  --  NavParams.h hands each RobotSpec constant to the right field. The
//  spec values are 0 until calibrated, so this checks the wiring, not the numbers. No hardware
//  needed.
// ============================================================================

#include <unity.h>

#include "NavParams.h"

static void test_motion_control_takes_the_gains_and_limits()
{
    const MotionControlParams p = specMotionControl();
    TEST_ASSERT_EQUAL_FLOAT(KP_DISTANCE, p.kpDist);
    TEST_ASSERT_EQUAL_FLOAT(KI_DISTANCE, p.kiDist);
    TEST_ASSERT_EQUAL_FLOAT(KD_DISTANCE, p.kdDist);
    TEST_ASSERT_EQUAL_FLOAT(KP_TURN, p.kpTurn);
    TEST_ASSERT_EQUAL_FLOAT(KI_TURN, p.kiTurn);
    TEST_ASSERT_EQUAL_FLOAT(KI_TURN_HOLD, p.kiHold);
    TEST_ASSERT_EQUAL_FLOAT(FEEDFORWARD_RIGHT_SCALE, p.ffScaleR);
    TEST_ASSERT_EQUAL_FLOAT(KD_TURN, p.kdTurn);
    TEST_ASSERT_EQUAL_FLOAT(FEEDFORWARD_PWM_PER_MM_S, p.ffPwmPerMmS);
    TEST_ASSERT_EQUAL_FLOAT(FEEDFORWARD_PWM_PER_DPS, p.ffPwmPerDps);
    TEST_ASSERT_EQUAL_UINT16(CONTROL_MIN_PWM_L, p.minPwmL);
    TEST_ASSERT_EQUAL_UINT16(CONTROL_MIN_PWM_R, p.minPwmR);
    TEST_ASSERT_EQUAL_UINT16(MOTOR_PWM_TOP, p.pwmTop);
    TEST_ASSERT_EQUAL_UINT16(STALL_DUTY, p.stallDuty);
    TEST_ASSERT_EQUAL_UINT16(STALL_TIME_MS, p.stallTimeMs);
    TEST_ASSERT_EQUAL_FLOAT(STALL_MIN_SPEED_MM_S, p.stallMinSpeedMmS);
    TEST_ASSERT_EQUAL_FLOAT(STALL_MIN_YAW_DPS, p.stallMinYawDps);
}

static void test_wall_params()
{
    const WallParams p = specWall();
    TEST_ASSERT_EQUAL_UINT16(WALL_PRESENT_SIDE_MM, p.sideMm);
    TEST_ASSERT_EQUAL_UINT16(WALL_PRESENT_FRONT_MM, p.frontMm);
    TEST_ASSERT_EQUAL_UINT16(WALL_CENTER_DIST_MM, p.centerDistMm);
    TEST_ASSERT_EQUAL_FLOAT(KP_WALL, p.kp);
    TEST_ASSERT_EQUAL_FLOAT(KD_WALL, p.kd);
    TEST_ASSERT_EQUAL_FLOAT(STEER_MAX_DEG, p.steerMaxDeg);
}

static void test_plan_and_step_cell_params()
{
    const PlanParams plan = specPlan();
    TEST_ASSERT_EQUAL_FLOAT(ACCEL_MM_S2, plan.accelMmS2);
    TEST_ASSERT_EQUAL_FLOAT(TURN_RATE_MAX_DPS, plan.turnRateMaxDps);
    TEST_ASSERT_EQUAL_FLOAT(TURN_ACCEL_DPS2, plan.turnAccelDps2);
    TEST_ASSERT_EQUAL_FLOAT(STEP_TOLERANCE_MM, plan.stepToleranceMm);
    TEST_ASSERT_EQUAL_FLOAT(TURN_TOLERANCE_DEG, plan.turnToleranceDeg);
    TEST_ASSERT_EQUAL_UINT16(TURN_SETTLE_MS, plan.turnSettleMs);
    TEST_ASSERT_EQUAL_UINT16(MOVE_TIMEOUT_MS, plan.moveTimeoutMs);

    const StepCellParams step = specStepCell();
    TEST_ASSERT_EQUAL_FLOAT(TILE_PITCH_MM, step.pitchMm);
    TEST_ASSERT_EQUAL_FLOAT(SEARCH_SPEED_MM_S, step.cruiseMmS);
    TEST_ASSERT_EQUAL_FLOAT(WALL_SAMPLE_LEAD_MM, step.sampleLeadMm);
    TEST_ASSERT_EQUAL_FLOAT(WALL_SAMPLE_WINDOW_MM, step.sampleWindowMm);
    TEST_ASSERT_EQUAL_FLOAT(FRONT_STOP_MM, step.frontStopMm);
}

static void test_line_and_floor_params()
{
    const LineParams l = specLine();
    TEST_ASSERT_EQUAL_FLOAT(KP_LINE, l.kp);
    TEST_ASSERT_EQUAL_FLOAT(KD_LINE, l.kd);
    TEST_ASSERT_EQUAL_FLOAT(LINE_STEER_MAX_DEG, l.steerMaxDeg);
    TEST_ASSERT_EQUAL_FLOAT(LINE_SPEED_MM_S, l.speedMmS);
    TEST_ASSERT_EQUAL_FLOAT(LINE_ACCEL_MM_S2, l.accelMmS2);
    TEST_ASSERT_EQUAL_UINT8(LINE_SLOWDOWN_PCT, l.slowdownPct);
    TEST_ASSERT_EQUAL_FLOAT(LINE_SLOWDOWN_ERR_MM, l.slowdownErrMm);
    TEST_ASSERT_EQUAL_FLOAT(LINE_SEARCH_RATE_DPS, l.searchRateDps);
    TEST_ASSERT_EQUAL_UINT16(LINE_LOST_TIMEOUT_MS, l.lostTimeoutMs);
    TEST_ASSERT_EQUAL_FLOAT(LINE_SEARCH_SPEED_MM_S, l.searchSpeedMmS);
    TEST_ASSERT_EQUAL_FLOAT(LINE_END_LOST_MM, l.endLostMm);
    TEST_ASSERT_EQUAL_FLOAT(RAMP_TILT_DEG, l.rampTiltDeg);

    const PatternParams pat = specPattern();
    TEST_ASSERT_EQUAL_FLOAT(MARKER_FRAME_MIN_MM, pat.frameMinMm);
    TEST_ASSERT_EQUAL_FLOAT(MARKER_FRAME_MAX_MM, pat.frameMaxMm);
    TEST_ASSERT_EQUAL_FLOAT(MARKER_CENTRE_MIN_MM, pat.centreMinMm);
    TEST_ASSERT_EQUAL_FLOAT(MARKER_CENTRE_MAX_MM, pat.centreMaxMm);
    TEST_ASSERT_EQUAL_FLOAT(GAP_MIN_MM, pat.gapMinMm);
    TEST_ASSERT_EQUAL_FLOAT(FLOOR_GLITCH_MM, pat.glitchMm);

    const FloorParams f = specFloor();
    TEST_ASSERT_EQUAL_FLOAT(MARKER_RUN_MIN_MM, f.markerMinMm);
    TEST_ASSERT_EQUAL_FLOAT(MARKER_RUN_MAX_MM, f.markerMaxMm);
    TEST_ASSERT_EQUAL_FLOAT(LINE_ENTER_RUN_MM, f.lineEnterMm);
    TEST_ASSERT_EQUAL_FLOAT(FLOOR_RUN_GAP_MM, f.gapMm);
}

static void test_perceive_params()
{
    const PerceiveParams p = specPerceive();
    TEST_ASSERT_EQUAL_FLOAT(MM_PER_ENCODER_COUNT, p.mmPerCount);
    TEST_ASSERT_EQUAL_UINT16(CONTROL_LOOP_PERIOD_MS, p.dtMs);
    TEST_ASSERT_EQUAL_FLOAT(IR_SENSOR_PITCH_MM, p.irPitchMm);
    TEST_ASSERT_EQUAL(IR_D1_ON_RIGHT, p.irD1OnRight);
    TEST_ASSERT_EQUAL_UINT16(IR_AIR_COUNTS, p.irAirCounts);
    TEST_ASSERT_EQUAL_UINT8(IR_AIR_CHANNELS, p.irAirChannels);
}

static void test_mission_params_and_hand()
{
    const MissionParams left = specMission(1);
    const MissionParams right = specMission(-1);
    TEST_ASSERT_EQUAL_INT8(1, left.hand);
    TEST_ASSERT_EQUAL_INT8(-1, right.hand);
    TEST_ASSERT_EQUAL(!RUN_USE_FLOOR_SENSING, left.wallsOnly);
    TEST_ASSERT_EQUAL_FLOAT(IR_ARRAY_FORWARD_OFFSET_MM, left.irAheadMm);
    TEST_ASSERT_EQUAL_FLOAT(LINE_LEAVE_MARGIN_MM, left.leaveMarginMm);
    TEST_ASSERT_EQUAL_UINT16(CONTROL_LOOP_PERIOD_MS, left.dtMs);
}

static void test_sense_window_covers_a_full_ultrasonic_round()
{
    // Every sensor must have been read at least once inside the window the start cell is read in.
    TEST_ASSERT_TRUE(START_SENSE_MS > US_SENSOR_COUNT * US_SLOT_MS);
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_motion_control_takes_the_gains_and_limits);
    RUN_TEST(test_wall_params);
    RUN_TEST(test_plan_and_step_cell_params);
    RUN_TEST(test_line_and_floor_params);
    RUN_TEST(test_perceive_params);
    RUN_TEST(test_mission_params_and_hand);
    RUN_TEST(test_sense_window_covers_a_full_ultrasonic_round);
    return UNITY_END();
}
