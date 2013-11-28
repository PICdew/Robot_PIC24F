//****************************************
// moter_l298n
// for L298N motor driver module
// 2013.11.12
// Regis Hsu
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

static int _motor_dir[2];

int Motor_Dir_Get(int motor_id)
{
    return (_motor_dir[motor_id]);
}

int Motor_Dir_Set(int motor_id, int motor_dir)
{
    _motor_dir[motor_id] = motor_dir;
    if (motor_id == MOTOR_L)
    {
        switch (motor_dir)
        {
            case MOTOR_STOP:
                _LATA4 = 0;
                _LATA5 = 0;
                break;
            case MOTOR_CW:
                _LATA4 = 1;
                _LATA5 = 0;
                break;
            case MOTOR_CCW:
                _LATA4 = 0;
                _LATA5 = 1;
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
                _LATA14 = 0;
                _LATA15 = 0;
                break;
            case MOTOR_CW:
                _LATA14 = 1;
                _LATA15 = 0;
                break;
            case MOTOR_CCW:
                _LATA14 = 0;
                _LATA15 = 1;
                break;
            default:
                LED_ERR(5);
                break;

        }
    }

}

int Motor_L298N_Init(void)
{
   // set the port as output
    TRISAbits.TRISA4 = 0;       //output
    TRISAbits.TRISA5 = 0;       //output
    TRISAbits.TRISA14 = 0;      //output
    TRISAbits.TRISA15 = 0;      //output

    // it doesn't work for this way ...Regis 11/28
    //ODCAbits.ODA4 = 1;
    //ODCAbits.ODA5 = 1;
    //ODCAbits.ODA14 = 1;
    //ODCAbits.ODA15 = 1;

    _motor_dir[MOTOR_L] = MOTOR_STOP;
    _motor_dir[MOTOR_R] = MOTOR_STOP;

}

/*
 * sw = 1, PWM on
 * sw = 0, PWM off
  */
void Motor_PWM_Set(int sw)
{
   //Make the pin of RP17(RF5) as normal function
    RPOR8bits.RP17R = 0;
    // Drive RF5 low and make it an output
    LATFbits.LATF5 = 0;
    TRISFbits.TRISF5 = 0;

    // Reset PWM
    OC1CON1 = 0x0000;
    OC1CON2 = 0x0000;

    // config timer3
    T3CON = 0x00; //Stops the Timer3 and reset control reg.
    TMR3 = 0x00; //Clear contents of the timer register
    PR3 = 0xFFFF; //Load the Period register with the value 0xFFFF

    /*
     *  bit 5-4 TCKPS<1:0>: Timerx Input Clock Prescale Select bits
     *  11 = 1:256 prescale value, 16us
     *  10 = 1:64 prescale value, 4us
     *  01 = 1:8 prescale value, 0.5us
     *  00 = 1:1 prescale value, 0.0625us,
     *   Fcy=16MHz=0.0625us
     *
     */
    T3CONbits.TCKPS = 0x03; //1:256, 16us
    // configure PWM
    OC1CON2 = 0x001F;   /* Sync with This OC module                               */
    //OC1CON1 = 0x1C08;   /* Clock sourc Fcyc, trigger mode 1, Mode 0 (disable OC1) */
    OC1CON1 = 0x0408;   /* Clock sourc timer3, trigger mode 1, Mode 0 (disable OC1) */
    /* enable the PWM */
    OC1CON1 = OC1CON1 | 0x0006;   /* Mode 6, Edge-aligned PWM Mode */

    // Make the pin of RP17(RF5) as OC1 (PWM output)
    RPOR8bits.RP17R = 18;

    T3CONbits.TON = 1; //Start Timer3
    // set the PWM period, 1250 * 16us = 20ms
    OC1RS   = 1250;
    for (;; )
    {
        BYTE j;
        // 0~100, 100*16us = 1.6ms
        j = POT_Read();

        // set PWM duty cycle
        OC1R    = j;
        delay_ms(200);
    }
      
}