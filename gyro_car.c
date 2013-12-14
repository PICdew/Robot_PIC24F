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

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

// Apply the complementary filter to figure out the change in angle - choice of alpha is
// estimated now.  Alpha depends on the sampling rate...
#define COMP_ALFA       0.93

static int _PWM_freq;
static portTickType _Task_timer;
static int _Kp;
static int _Min, _Max;
static int _Angle_Max, _Angle_Min;

void Gyro_Car_Kp(int kp)
{
    _Kp = kp;
}

// the unit is ms
void Gyro_Car_Task_Timer(portTickType ms)
{
    _Task_timer = ms;
}

void Gyro_Car_Limit_Min(int min)
{
    _Min = min;
}

void Gyro_Car_Limit_Max(int max)
{
    _Max = max;
}

void Gyro_Car_Angle_Max(int a)
{
    _Angle_Max = a;
}

void Gyro_Car_Angle_Min(int a)
{
    _Angle_Min = a;
}


void vTask_Gyro_Car(void *pvParameters )
{
    (void)pvParameters; // prevent compiler worning/error
    int gyro_x, gyro_y, gyro_z;
    int base_x, base_y, base_z;
    int dir_y, dir_abs_y;
    int k_dir_y;
    int x;
    portTickType xLastWakeTime;


    Motor_L298N_PWM_Init(1500); //1.5kHz
    Gyro_MPU6050_Init();
    Gyro_MPU6050_Calibration(10);
    Gyro_MPU6050_Kalman_Angle_Start();
    // get the balance point
    Gyro_MPU6050_Kalman_Angle_Get(&base_x, &base_y, &base_z);
    // set the timer as 15 ms in initial stage
    Gyro_Car_Task_Timer(20);
    Gyro_Car_Kp(5);
    Gyro_Car_Limit_Min(50);
    Gyro_Car_Limit_Max(100);
    Gyro_Car_Angle_Max(35);
    Gyro_Car_Angle_Min(-35);
    
    xLastWakeTime = xTaskGetTickCount();
    for( ;; )
    {
        vTaskDelayUntil(&xLastWakeTime, _Task_timer);
        Gyro_MPU6050_Kalman_Angle_Get(&gyro_x, &gyro_y, &gyro_z);

        dir_y = gyro_y - base_y;
        dir_abs_y = dir_y * _Kp;
        dir_abs_y = abs(dir_abs_y);
        k_dir_y = constrain(dir_abs_y, _Min, _Max);         // 限定值域

        if (dir_y > _Angle_Max || dir_y < _Angle_Min)
        {
            Motor_PWM_Dir_Set(MOTOR_L, MOTOR_STOP, 0);
            Motor_PWM_Dir_Set(MOTOR_R, MOTOR_STOP, 0);
        }
        else
        if (dir_y > 1)
        {
            Motor_PWM_Dir_Set(MOTOR_L, MOTOR_CW, k_dir_y);
            Motor_PWM_Dir_Set(MOTOR_R, MOTOR_CW, k_dir_y);
        }
        else
        if (dir_y < -1)
        {
            Motor_PWM_Dir_Set(MOTOR_L, MOTOR_CCW, k_dir_y);
            Motor_PWM_Dir_Set(MOTOR_R, MOTOR_CCW, k_dir_y);
        }
        else
        {
            Motor_PWM_Dir_Set(MOTOR_L, MOTOR_STOP, 0);
            Motor_PWM_Dir_Set(MOTOR_R, MOTOR_STOP, 0);
        }

        x = 0;
        LCD_Show_Value(x, 0, dir_y, 10, 4);
/*
        x += 5;
        LCD_Show_Value(x, 0, gyro_y, 10, 4);
        x += 5;
        LCD_Show_Value(x, 0, k_dir_y, 10, 4);
 */
     }
}

static double Kp,Ki,Kd;                    // 比例系数、积分系数、微分系数
static double SampleTime;                  // 采样时间(s)
static double Setpoint;                    // 设定目标值(degree)
static double outMin, outMax;              // 输出上限、输出下限

void PID_SetTunings(double kp, double ki, double kd)
{
   Kp = kp;
   Ki = ki;
   Kd = kd;
}
void PID_SetSampleTime(float t)
{
    SampleTime = t;
}

void PID_SetPoint(float p)
{
    Setpoint = p;
}

void PID_SetOutLimit(float min, float max)
{
    outMin = min;
    outMax = max;
}

#define _PID_TIME_  700

void vTask_Gyro_Car_PID(void *pvParameters )
{
    (void)pvParameters; // prevent compiler worning/error
    int gyro_x, gyro_y, gyro_z;
    int base_x, base_y, base_z;
    int dir_y, dir_abs_y;
    int x;
    double Input, lastInput, error, lastErr, errSum, dErr, Output;
    portTickType xLastWakeTime, now, lastTime;
    double timeChange;

    Motor_L298N_PWM_Init(1500); //1.5kHz
    Gyro_MPU6050_Init();
    Gyro_MPU6050_Calibration(10);

    Gyro_MPU6050_Kalman_Angle_Start();
    Gyro_MPU6050_Kalman_Angle_Get(&base_x, &base_y, &base_z);

    PID_SetTunings(10.0, 0.0, 0.0);
    PID_SetSampleTime(0.1); // 100ms
    PID_SetPoint(base_y);
    PID_SetOutLimit(-100.0, + 100.0);
    xLastWakeTime = xTaskGetTickCount();
    for( ;; )
    {
        vTaskDelayUntil(&xLastWakeTime, _PID_TIME_);
        Gyro_MPU6050_Kalman_Angle_Get(&gyro_x, &gyro_y, &gyro_z);
        now = xTaskGetTickCount();
        timeChange = (now - lastTime) / 1000.0;             // 采样时间(s)

        Input = gyro_y;                                     // 输入赋值
        //Compute all the working error variables
        error = Setpoint - Input;
        errSum += (error * timeChange);
        dErr = (error - lastErr) / timeChange;

        //Compute PID Output
        Output = Kp * error + Ki * errSum + Kd * dErr;
        Output = constrain(Output, outMin, outMax);         // 限定值域

        if (Output > 0)
            dir_y = MOTOR_CW;
        else
            if (Output < 0)
                dir_y = MOTOR_CCW;
            else
                dir_y = MOTOR_STOP;



/*
        double timeChange = (double)(now - lastTime);
        //Compute all the working error variables
        double error = Setpoint - Input;
        errSum += (error * timeChange);
        double dErr = (error - lastErr) / timeChange;

        //Compute PID Output
        Output = kp * error + ki * errSum + kd * dErr;
 */
        dir_abs_y = Output;
        dir_abs_y = abs(dir_abs_y);
        Motor_PWM_Dir_Set(MOTOR_L, dir_y, dir_abs_y);
        Motor_PWM_Dir_Set(MOTOR_R, dir_y, dir_abs_y);
        lastInput = Input;                                  // 记录输入值
        lastTime = now;                                     // 记录本次时间
        x = 0;
        LCD_Show_Value(x, 0, dir_abs_y, 10, 4);
    }
}