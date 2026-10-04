// ============================================================================
//  motors.cpp  --  Fast PWM (mode 14, ICRn = TOP) at MOTOR_PWM_FREQ_HZ.
//  Left motor on Timer4 (OC4A/OC4B), right on Timer1 (OC1A/OC1B). The idle leg
//  is disconnected from its timer and held low, so duty 0 is a true constant
//  low rather than OCRnx = 0's one-clock spike. See docs/architecture/modules/motors.md.
// ============================================================================

#include "hal/motors.h"

#include <avr/io.h>

#include "MotorDrive.h"
#include "RobotConfig.h"
#include "RobotSpec.h"

// The register code below is hand-mapped to these pins.
static_assert(PIN_MOTOR_L_RPWM == 6 && PIN_MOTOR_L_LPWM == 7 && PIN_MOTOR_L_EN == 8,
              "left motor must be PH3/PH4 (OC4A/OC4B) + PH5 enable");
static_assert(PIN_MOTOR_R_RPWM == 11 && PIN_MOTOR_R_LPWM == 12 && PIN_MOTOR_R_EN == 10,
              "right motor must be PB5/PB6 (OC1A/OC1B) + PB4 enable");

static void setLeft(const MotorDuty &d)
{
    // Drop the leg going to zero first, then raise the other.
    if (d.rpwm == 0) {
        TCCR4A &= ~(1 << COM4A1);
        OCR4A = 0;
    }
    if (d.lpwm == 0) {
        TCCR4A &= ~(1 << COM4B1);
        OCR4B = 0;
    }
    if (d.rpwm != 0) {
        OCR4A = d.rpwm;
        TCCR4A |= (1 << COM4A1);
    }
    if (d.lpwm != 0) {
        OCR4B = d.lpwm;
        TCCR4A |= (1 << COM4B1);
    }
}

static void setRight(const MotorDuty &d)
{
    if (d.rpwm == 0) {
        TCCR1A &= ~(1 << COM1A1);
        OCR1A = 0;
    }
    if (d.lpwm == 0) {
        TCCR1A &= ~(1 << COM1B1);
        OCR1B = 0;
    }
    if (d.rpwm != 0) {
        OCR1A = d.rpwm;
        TCCR1A |= (1 << COM1A1);
    }
    if (d.lpwm != 0) {
        OCR1B = d.lpwm;
        TCCR1A |= (1 << COM1B1);
    }
}

void motorsInit()
{
    // Outputs low before anything can drive them.
    PORTH &= ~((1 << PH3) | (1 << PH4) | (1 << PH5));
    DDRH |= (1 << PH3) | (1 << PH4) | (1 << PH5);
    PORTB &= ~((1 << PB4) | (1 << PB5) | (1 << PB6));
    DDRB |= (1 << PB4) | (1 << PB5) | (1 << PB6);

    // WGMn3:0 = 1110 (Fast PWM, TOP = ICRn), clk/1, outputs disconnected.
    TCCR4B = 0;
    TCCR4A = (1 << WGM41);
    TCNT4 = 0;
    ICR4 = MOTOR_PWM_TOP;
    OCR4A = 0;
    OCR4B = 0;
    TCCR4B = (1 << WGM43) | (1 << WGM42) | (1 << CS40);

    TCCR1B = 0;
    TCCR1A = (1 << WGM11);
    TCNT1 = 0;
    ICR1 = MOTOR_PWM_TOP;
    OCR1A = 0;
    OCR1B = 0;
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);
}

void motorsEnable(bool enabled)
{
    if (enabled) {
        PORTH |= (1 << PH5);
        PORTB |= (1 << PB4);
    } else {
        PORTH &= ~(1 << PH5);
        PORTB &= ~(1 << PB4);
    }
}

void motorSet(Motor motor, int16_t command)
{
    MotorDuty d = motorDutyFromCommand(command, MOTOR_PWM_TOP);
    const bool invert = (motor == Motor::Left) ? MOTOR_L_INVERT : MOTOR_R_INVERT;
    if (invert) {
        const uint16_t t = d.rpwm;
        d.rpwm = d.lpwm;
        d.lpwm = t;
    }
    if (motor == Motor::Left) {
        setLeft(d);
    } else {
        setRight(d);
    }
}
