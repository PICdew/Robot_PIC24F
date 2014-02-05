//****************************************
// moter_l298n
// for L298N motor driver module
// 2013.11.12
// Regis Hsu
//
//
//
//****************************************
/* Standard includes. */
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"
#include "led.h"

#include "i2c.h"
#include "i2c_func.h"
#include "lcd_i2c_1602.h"

#include "moter_l298n.h"

//PPS Outputs for PIC24FJ256GA110 and PIC24FJ256GB110
#define OC1_IO		18
#define OC2_IO		19
#define OC3_IO		20
#define OC4_IO		21
#define OC5_IO		22
#define OC6_IO          23
#define OC7_IO          24
#define OC8_IO          25
#define OC9_IO          35

static int _motor_dir[2];
static int _motor_rpm;
/*
 * Motor model:
 * HC01-120, 1::120, 3v:50rpm, 5v:83rpm, 1v:16.6rpm
 * HC01-72, 1::72, 3v:83rpm, 1v:27.7rpm
 * Wheel weight: 40g
 * Wheel width: 28mm
 * Wheel diameter: 65mm
 */

void Motor_RPM_Set(int volt)
{
    // HC01-120, 9v: 149.4rpm, 2.49rps, 0.4s/rotate, 9degree/ms=> (150rpm*360)/60/1000
    _motor_rpm = volt * 16.6;
    
}

int Motor_Dir_Get(int motor_id)
{
    return (_motor_dir[motor_id]);
}

void Motor_Dir_Set(int motor_id, int motor_dir)
{
    _motor_dir[motor_id] = motor_dir;
    if (motor_id == MOTOR_L)
    {
        switch (motor_dir)
        {
            case MOTOR_STOP:
                _LATB0 = 0;
                _LATB1 = 0;
                break;
            case MOTOR_CW:
                _LATB0 = 1;
                _LATB1 = 0;
                break;
            case MOTOR_CCW:
                _LATB0 = 0;
                _LATB1 = 1;
                break;
            default:
                LED_ERR(5);
                break;

        }
    }
    else
    //if (motor_id == MOTOR_R)
    {
        switch (motor_dir)
        {
            case MOTOR_STOP:
                _LATB2 = 0;
                _LATB4 = 0;
                break;
            case MOTOR_CW:
                _LATB2 = 1;
                _LATB4 = 0;
                break;
            case MOTOR_CCW:
                _LATB2 = 0;
                _LATB4 = 1;
                break;
            default:
                LED_ERR(5);
                break;

        }
    }
}


void Motor_L298N_Init(void)
{

    // set the port as output
    TRISBbits.TRISB0 = 0;      //output, RP0/RB0
    TRISBbits.TRISB1 = 0;      //output, RP1/RB1
    TRISBbits.TRISB2 = 0;      //output, RP13/RB2
    TRISBbits.TRISB4 = 0;      //output, RP28/RB4

    _motor_dir[MOTOR_L] = MOTOR_STOP;
    _motor_dir[MOTOR_R] = MOTOR_STOP;

}
/*
 *  initial the motor control pins with PWM feature
 *  period, it is "Hz", period = 1000 -> 1kHz
 */
static int _OC_period;

int Motor_L298N_PWN_Period(void)
{
    return _OC_period;
}

void Motor_L298N_PWM_Init(int period)
{
    double cal_period;
    // 62.5ns = 0.0625us
    cal_period = period;
    cal_period = 1000000 / (cal_period*0.0625);

    _OC_period = cal_period - 1;
    //_OC_period = 0x3e7f;

    // set the port as output
    TRISBbits.TRISB0 = 0;      //output, RP0/RB0
    TRISBbits.TRISB1 = 0;      //output, RP1/RB1
    TRISBbits.TRISB2 = 0;      //output, RP13/RB2
    TRISBbits.TRISB4 = 0;      //output, RP28/RB4

    // it doesn't work for this way ...Regis 11/28
    //ODCBbits.ODB0 = 1;
    //ODCBbits.ODB1 = 1;
    //ODCBbits.ODB2 = 1;
    //ODCBbits.ODB4 = 1;

    // Configure Output Functions *********************
    RPOR0bits.RP0R      = OC1_IO;	// RR0	pin#25, J5#01,AN0,
    RPOR0bits.RP1R 	= OC2_IO;	// RP1	pin#24, J5#02,AN1
    RPOR6bits.RP13R	= OC3_IO;	// RP13	pin#23, J5#03,AN2
    RPOR14bits.RP28R 	= OC4_IO;	// RP28	pin#21, J5#05,AN4

    /*
	The following configuration for OC1, OC2, OC3 and OC4 will set the
	Output Compare modules for PWM mode w/o FAULT pin enabled,
	a xxx% duty cycle and PWM frequency of (freq)Hz at Fosc = 32 MHz.
	System clock is selected as the clock for the PWM time base
	and no interrupt is enabled.
	Find the Period register value for a desired PWM frequency of (freq)Hz,
	where Fosc = 8 MHz with PLL (32 MHz device clock rate).
	Tcy = 2/Fosc = 62.5 ns
	PWM Period   =  1/PWM Frequency = 1/(freq)Hz
	PWM Period   = (OC1RS + 1) * Tcy
	ex:
        1000us = 1/1KHz
        1000us = (OC1RS + 1) * 62.5 ns * 1
	OC1RS = 15999 (0x3E7F)
    */

    OC2CON1			= 0; 		// It is a good practice to clear off the control bits initially
    OC2CON2			= 0;
    //OC2R			= 0x1f3f;	// Initialize Compare Register1 with 50% duty cycle
    OC2R			= 0;
    OC2RS			= _OC_period;	//
    //OC2CON2bits.SYNCSEL	= 0x1;		// synchronized by the OC1 module
    OC2CON2bits.SYNCSEL         = 0x2;		// synchronized by the OC2 module
    OC2CON1bits.OCTSEL          = 0x7;		// System Clock (Tcy) is the clock source for output Compare
    OC2CON1bits.OCFLT           = 0;		// No PWM Fault condition has occurred (this bit is only used when OCM<2:0> = 111)
    OC2CON2bits.OCINV           = 0;		// OCx output is not inverted
    OC2CON1bits.OCM		= 0x6;		// Edge-aligned PWM mode on OC

    OC3CON1			= 0; 		// It is a good practice to clear off the control bits initially
    OC3CON2			= 0;
    OC3R			= 0;
    OC3RS			= _OC_period;	//
    //	OC3CON2bits.SYNCSEL	= 0x1;		// synchronized by the OC1 module
    OC3CON2bits.SYNCSEL         = 0x3;		// synchronized by the OC3 module
    OC3CON1bits.OCTSEL          = 0x7;		// System Clock (Tcy) is the clock source for output Compare
    OC3CON1bits.OCFLT           = 0;		// No PWM Fault condition has occurred (this bit is only used when OCM<2:0> = 111)
    OC3CON2bits.OCINV           = 0;		// OCx output is not inverted
    OC3CON1bits.OCM		= 0x6;		// Edge-aligned PWM mode on OC

    OC4CON1			= 0; 		// It is a good practice to clear off the control bits initially
    OC4CON2			= 0;
    OC4R			= 0;
    OC4RS			= _OC_period;	//
    //	OC4CON2bits.SYNCSEL	= 0x1;		// synchronized by the OC1 module
    OC4CON2bits.SYNCSEL         = 0x4;		// synchronized by the OC4 module
    OC4CON1bits.OCTSEL          = 0x7;		// System Clock (Tcy) is the clock source for output Compare
    OC4CON1bits.OCFLT           = 0;		// No PWM Fault condition has occurred (this bit is only used when OCM<2:0> = 111)
    OC4CON2bits.OCINV           = 0;		// OCx output is inverted
    OC4CON1bits.OCM		= 0x6;		// Edge-aligned PWM mode on OC

    OC1CON1			= 0; 		// It is a good practice to clear off the control bits initially
    OC1CON2			= 0;
    OC1R			= 0;
    OC1RS			= _OC_period;	//
    //	OC1CON2bits.SYNCSEL	= 0x1;		// synchronized by the OC1 module
    OC1CON2bits.SYNCSEL         = 0x1;		// synchronized by the OC1 module
    OC1CON1bits.OCTSEL          = 0x7;		// System Clock (Tcy) is the clock source for output Compare
    OC1CON1bits.OCFLT           = 0;		// No PWM Fault condition has occurred (this bit is only used when OCM<2:0> = 111)
    OC1CON2bits.OCINV           = 0;		// OCx output is not inverted
    OC1CON1bits.OCM		= 0x6;		// Edge-aligned PWM mode on OC

    _motor_dir[MOTOR_L] = MOTOR_STOP;
    _motor_dir[MOTOR_R] = MOTOR_STOP;
}

void Motor_PWM_Dir_Set(int motor_id, int motor_dir, int duty)
{
    double tmp;
    int reg_duty;

    tmp = duty;
    tmp = _OC_period * tmp / 100.0;
    reg_duty = tmp;

    //duty = 0x1f3f;
    _motor_dir[motor_id] = motor_dir;
    if (motor_id == MOTOR_L)
    {
        switch (motor_dir)
        {
            case MOTOR_STOP:
                OC1R = 0;
                OC2R = 0;
                break;
            case MOTOR_CW:
                OC1R = reg_duty;
                OC2R = 0;
                break;
            case MOTOR_CCW:
                OC1R = 0;
                OC2R = reg_duty;
                break;
            default:
                LED_ERR(5);
                break;

        }
    }
    else
    //if (motor_id == MOTOR_R)
    {
        switch (motor_dir)
        {
            case MOTOR_STOP:
                OC3R = 0;
                OC4R = 0;
                break;
            case MOTOR_CW:
                OC3R = reg_duty;
                OC4R = 0;
                break;
            case MOTOR_CCW:
                OC3R = 0;
                OC4R = reg_duty;
                break;
            default:
                LED_ERR(5);
                break;

        }
    }
}

void Motor_PWM_Dir_Set_Raw(int motor_id, int motor_dir, int PWM)
{
    int PWM_Operate;

    if (PWM > _OC_period)
        PWM_Operate = _OC_period;
    else
        PWM_Operate = PWM;

    //duty = 0x1f3f;
    _motor_dir[motor_id] = motor_dir;
    if (motor_id == MOTOR_L)
    {
        switch (motor_dir)
        {
            case MOTOR_STOP:
                OC1R = 0;
                OC2R = 0;
                break;
            case MOTOR_CW:
                OC1R = PWM_Operate;
                OC2R = 0;
                break;
            case MOTOR_CCW:
                OC1R = 0;
                OC2R = PWM_Operate;
                break;
            default:
                LED_ERR(5);
                break;

        }
    }
    else
    //if (motor_id == MOTOR_R)
    {
        switch (motor_dir)
        {
            case MOTOR_STOP:
                OC3R = 0;
                OC4R = 0;
                break;
            case MOTOR_CW:
                OC3R = PWM_Operate;
                OC4R = 0;
                break;
            case MOTOR_CCW:
                OC3R = 0;
                OC4R = PWM_Operate;
                break;
            default:
                LED_ERR(5);
                break;

        }
    }
}



