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

    _motor_dir[MOTOR_L] = MOTOR_STOP;
    _motor_dir[MOTOR_R] = MOTOR_STOP;

}

