//****************************************
// Gyro_car
// Test the car balance
// MCU: PIC24F
// 2013.11.26
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
#include "gyro_mpu6050.h"
#include "kalman.h"
#include "math.h"
#include "gyro_car.h"
#include "moter_l298n.h"


// Apply the complementary filter to figure out the change in angle - choice of alpha is
// estimated now.  Alpha depends on the sampling rate...
#define COMP_ALFA       0.93

void vTask_Gyro_Car(void *pvParameters )
{
    int gyro_x, gyro_y, gyro_z;
    int base_x, base_y, base_z;
    int x;
    portTickType xLastWakeTime;

    Motor_L298N_Init();
    Gyro_MPU6050_Init();
    Gyro_MPU6050_Calibration(10);
    Gyro_MPU6050_Kalman_Angle_Start();
    Gyro_MPU6050_Kalman_Angle_Get(&base_x, &base_y, &base_z);
    xLastWakeTime = xTaskGetTickCount();
    for( ;; )
    {
        vTaskDelayUntil(&xLastWakeTime, 500);
        Gyro_MPU6050_Kalman_Angle_Get(&gyro_x, &gyro_y, &gyro_z);

        gyro_y = gyro_y - base_y;

        if (gyro_y > 0 && gyro_y < 30)
        {
            Motor_Dir_Set(MOTOR_L, MOTOR_CCW);
            Motor_Dir_Set(MOTOR_R, MOTOR_CCW);
        }
        else
        {
            Motor_Dir_Set(MOTOR_L, MOTOR_STOP);
            Motor_Dir_Set(MOTOR_R, MOTOR_STOP);
        }


        if (gyro_y < 0 && gyro_y > -30)
        {
            Motor_Dir_Set(MOTOR_L, MOTOR_CW);
            Motor_Dir_Set(MOTOR_R, MOTOR_CW);
        }
        else
        {
            Motor_Dir_Set(MOTOR_L, MOTOR_STOP);
            Motor_Dir_Set(MOTOR_R, MOTOR_STOP);
        }

        x = 0;
        LCD_Show_Value(x, 0, gyro_x, 10, 4);
        x += 5;
        LCD_Show_Value(x, 0, gyro_y, 10, 4);
        x += 5;
        LCD_Show_Value(x, 0, gyro_z, 10, 4);
    }
}



