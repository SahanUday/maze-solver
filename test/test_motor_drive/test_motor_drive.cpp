// ============================================================================
//  test_motor_drive.cpp  --  motorDutyFromCommand() tests. No hardware needed.
// ============================================================================

#include <unity.h>

#include "MotorDrive.h"
#include "RobotSpec.h"

static void test_zero_is_both_legs_off()
{
    const MotorDuty d = motorDutyFromCommand(0, MOTOR_PWM_TOP);
    TEST_ASSERT_EQUAL_UINT16(0, d.rpwm);
    TEST_ASSERT_EQUAL_UINT16(0, d.lpwm);
}

static void test_positive_drives_rpwm_only()
{
    const MotorDuty d = motorDutyFromCommand(300, MOTOR_PWM_TOP);
    TEST_ASSERT_EQUAL_UINT16(300, d.rpwm);
    TEST_ASSERT_EQUAL_UINT16(0, d.lpwm);
}

static void test_negative_drives_lpwm_only()
{
    const MotorDuty d = motorDutyFromCommand(-300, MOTOR_PWM_TOP);
    TEST_ASSERT_EQUAL_UINT16(0, d.rpwm);
    TEST_ASSERT_EQUAL_UINT16(300, d.lpwm);
}

static void test_clamps_to_top_both_directions()
{
    TEST_ASSERT_EQUAL_UINT16(MOTOR_PWM_TOP, motorDutyFromCommand(32767, MOTOR_PWM_TOP).rpwm);
    TEST_ASSERT_EQUAL_UINT16(MOTOR_PWM_TOP, motorDutyFromCommand(-32767, MOTOR_PWM_TOP).lpwm);
}

static void test_int16_min_does_not_overflow()
{
    const MotorDuty d = motorDutyFromCommand(INT16_MIN, MOTOR_PWM_TOP);
    TEST_ASSERT_EQUAL_UINT16(0, d.rpwm);
    TEST_ASSERT_EQUAL_UINT16(MOTOR_PWM_TOP, d.lpwm);
}

static void test_pwm_top_is_20khz()
{
    TEST_ASSERT_EQUAL_UINT16(799, MOTOR_PWM_TOP);
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_zero_is_both_legs_off);
    RUN_TEST(test_positive_drives_rpwm_only);
    RUN_TEST(test_negative_drives_lpwm_only);
    RUN_TEST(test_clamps_to_top_both_directions);
    RUN_TEST(test_int16_min_does_not_overflow);
    RUN_TEST(test_pwm_top_is_20khz);
    return UNITY_END();
}
