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
#include "uart1.h"

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

// Apply the complementary filter to figure out the change in angle - choice of alpha is
// estimated now.  Alpha depends on the sampling rate...
#define COMP_ALFA       0.93

typedef struct
{
    int PWM_freq;
    portTickType Task_timer;
    int Kp, Ki, Kd;
    int PWM_Min, PWM_Max, PWM_step, PWM_static;
    int Angle_Max, Angle_Min;
    int balance_angle;
    int angle_step;
    int angle_target_F, angle_target_B;
    int motor_onoff, motor_dir_L, motor_dir_R;
    int debug;
}Gyro_Car_Str;

Gyro_Car_Str car2 =
{
    1000,
    15,
    70,20,30,
    50,100, 0, 0,
    35, -35,
    185,
    1, 
    0,0,
    0, MOTOR_STOP, MOTOR_STOP,
    0
};

void Gyro_Car_Debug(void)
{
    char m[20];
    car2.debug = 1 - car2.debug;
    sprintf(m,"Debug=%d\r\n", car2.debug);
    UART_PutString(UART_BT, m);
}
void Gyro_Car_Motor(void)
{
    char m[20];
    car2.motor_onoff = 1 - car2.motor_onoff;
    sprintf(m,"Motor=%d\r\n", car2.motor_onoff);
    UART_PutString(UART_BT, m);
}

void Gyro_Car_Kp_Up(void)
{
    char m[20];
    car2.Kp += 1;
    sprintf(m,"Kp=%d\r\n", car2.Kp);
    UART_PutString(UART_BT, m);
}

void Gyro_Car_Kp_Down(void)
{
    char m[20];
    car2.Kp -= 1;
    if (car2.Kp < 0)
        car2.Kp = 0;
    sprintf(m,"Kp=%d\r\n", car2.Kp);
    UART_PutString(UART_BT, m);
}

void Gyro_Car_Ki_Up(void)
{
    char m[20];
    car2.Ki += 1;
    sprintf(m,"Ki=%d\r\n", car2.Ki);
    UART_PutString(UART_BT, m);
}

void Gyro_Car_Ki_Down(void)
{
    char m[20];
    car2.Ki -= 1;
    if (car2.Ki < 0)
        car2.Ki = 0;
    sprintf(m,"Ki=%d\r\n", car2.Ki);
    UART_PutString(UART_BT, m);
}

void Gyro_Car_Kd_Up(void)
{
    char m[20];
    car2.Kd += 1;
    sprintf(m,"Kd=%d\r\n", car2.Kd);
    UART_PutString(UART_BT, m);
}

void Gyro_Car_Kd_Down(void)
{
    char m[20];
    car2.Kd -= 1;
    if (car2.Kd < 0)
        car2.Kd = 0;
    sprintf(m,"Kd=%d\r\n", car2.Kd);
    UART_PutString(UART_BT, m);
}

void Gyro_Car_TaskTimer_Up(void)
{
    char m[20];
    car2.Task_timer += 1;
    sprintf(m,"Timer=%d\r\n", car2.Task_timer);
    UART_PutString(UART_BT, m);
}

void Gyro_Car_TaskTimer_Down(void)
{
    char m[20];
    car2.Task_timer -= 1;
    if (car2.Task_timer < 5)
        car2.Task_timer = 5;
    sprintf(m,"Timer=%d\r\n", car2.Task_timer);
    UART_PutString(UART_BT, m);
}


void Gyro_Car_Balance_Up(void)
{
    char m[20];
    car2.balance_angle += 1;
    sprintf(m,"Balance=%d\r\n", car2.balance_angle);
    UART_PutString(UART_BT, m);
}

void Gyro_Car_Balance_Down(void)
{
    char m[20];
    car2.balance_angle -= 1;
    sprintf(m,"Balance=%d\r\n", car2.balance_angle);
    UART_PutString(UART_BT, m);
}

void Gyro_Car_Angle_Max_Up(void)
{
    char m[20];
    car2.Angle_Max += 1;
    sprintf(m,"Angle_Max=%d\r\n", car2.Angle_Max);
    UART_PutString(UART_BT, m);
}

void Gyro_Car_Angle_Max_Down(void)
{
    char m[20];
    car2.Angle_Max -= 1;
    sprintf(m,"Angle_Max=%d\r\n", car2.Angle_Max);
    UART_PutString(UART_BT, m);
}

void Gyro_Car_Angle_Min_Up(void)
{
    char m[20];
    car2.Angle_Min += 1;
    sprintf(m,"Angle_Min=%d\r\n", car2.Angle_Min);
    UART_PutString(UART_BT, m);
}

void Gyro_Car_Angle_Min_Down(void)
{
    char m[20];
    car2.Angle_Min -= 1;
    sprintf(m,"Angle_Min=%d\r\n", car2.Angle_Min);
    UART_PutString(UART_BT, m);
}

void Gyro_Car_Angle_Step_Up(void)
{
    char m[20];
    car2.angle_step += 1;
    sprintf(m,"Angle_Step=%d\r\n", car2.angle_step);
    UART_PutString(UART_BT, m);
}

void Gyro_Car_Angle_Step_Down(void)
{
    char m[20];
    car2.angle_step -= 1;
    if (car2.angle_step < 0)
        car2.angle_step = 0;
    sprintf(m,"Angle_Step=%d\r\n", car2.angle_step);
    UART_PutString(UART_BT, m);
}

void Gyro_Car_PWM_Step_Up(void)
{
    char m[20];
    car2.PWM_step += 1;
    sprintf(m,"PWM_step=%d\r\n", car2.PWM_step);
    UART_PutString(UART_BT, m);
}

void Gyro_Car_PWM_Step_Down(void)
{
    char m[20];
    car2.PWM_step -= 1;
    if (car2.PWM_step < 0)
        car2.PWM_step = 0;
    sprintf(m,"PWM_step=%d\r\n", car2.PWM_step);
    UART_PutString(UART_BT, m);
}

void Gyro_Car_PWM_Static_Up(void)
{
    char m[20];
    car2.PWM_static += 1;
    sprintf(m,"PWM_static=%d\r\n", car2.PWM_static);
    UART_PutString(UART_BT, m);
}

void Gyro_Car_PWM_Static_Down(void)
{
    char m[20];
    car2.PWM_static -= 1;
    if (car2.PWM_static < 0)
        car2.PWM_static = 0;
    sprintf(m,"PWM_static=%d\r\n", car2.PWM_static);
    UART_PutString(UART_BT, m);
}

void Gyro_Car_PWM_FF(void)
{
    car2.motor_dir_L = MOTOR_CW;
    car2.motor_dir_R = MOTOR_CW;
}

void Gyro_Car_PWM_BK(void)
{
    car2.motor_dir_L = MOTOR_CCW;
    car2.motor_dir_R = MOTOR_CCW;
}

void Gyro_Car_PWM_Right(void)
{
    car2.motor_dir_L = MOTOR_CCW;
    car2.motor_dir_R = MOTOR_CW;
}
void Gyro_Car_PWM_Left(void)
{
    car2.motor_dir_L = MOTOR_CW;
    car2.motor_dir_R = MOTOR_CCW;
}

void Gyro_Car_PWM_Stop(void)
{
    car2.motor_dir_L = MOTOR_STOP;
    car2.motor_dir_R = MOTOR_STOP;
}

void Gyro_Car_PWM_Speed_Up(void)
{
    char m[20];
    car2.PWM_step += 50;
    if (car2.PWM_step > car2.Angle_Max)
        car2.PWM_step = car2.PWM_Max;
    sprintf(m,"PWM_speed=%d %d\r\n", car2.PWM_step, car2.PWM_Max);
    UART_PutString(UART_BT, m);
}

void Gyro_Car_PWM_Speed_Down(void)
{
    char m[20];
    car2.PWM_step -= 50;
    if (car2.PWM_step < car2.Angle_Min)
        car2.PWM_step = car2.PWM_Min;
    sprintf(m,"PWM_step=%d %d\r\n", car2.PWM_step, car2.PWM_Min);
    UART_PutString(UART_BT, m);
}


// balance car, for battery12v to 6v + BT control
void vTask_Gyro_Car_5(void *pvParameters )
{
    (void)pvParameters; // prevent compiler worning/error
    int gyro_x, gyro_y, gyro_z;
    int base_x, base_y, base_z;
    //double Ki, Kp, Kd, P, P_last, I, I_last, D, D_last, U, E, E_last;
    int Ei, Ui, PWM;

    char m[100];
    portTickType xLastWakeTime;
    Motor_L298N_Init();
    Gyro_MPU6050_Init();
    Gyro_MPU6050_Calibration(10);
    Gyro_MPU6050_Kalman_Angle_Start();
    // get the balance point
    Gyro_MPU6050_Kalman_Angle_Get(&base_x, &base_y, &base_z);
    car2.balance_angle = base_y;
    // set the timer as 15 ms in initial stage
    xLastWakeTime = xTaskGetTickCount();
    //E_last = I_last = P_last = D_last = 0;

    for( ;; )
    {
        //vTaskDelayUntil(&xLastWakeTime, car2.Task_timer);
        Gyro_MPU6050_Kalman_Angle_Get(&gyro_x, &gyro_y, &gyro_z);
        Ei = gyro_y - car2.balance_angle;

        if (car2.motor_onoff)
        {
            if (Ei > car2.Angle_Max || Ei < car2.Angle_Min)
            {
                Motor_Dir_Set(MOTOR_L, MOTOR_STOP);
                Motor_Dir_Set(MOTOR_R, MOTOR_STOP);
            }
            else
            if (Ei > car2.angle_step) //body back down
            {
                Motor_Dir_Set(MOTOR_L, MOTOR_CCW);
                Motor_Dir_Set(MOTOR_R, MOTOR_CCW);
            }
            else
            if (Ei < -car2.angle_step) // body front down
            {
                Motor_Dir_Set(MOTOR_L, MOTOR_CW);
                Motor_Dir_Set(MOTOR_R, MOTOR_CW);
            }
            else
            {
                Motor_Dir_Set(MOTOR_L, MOTOR_STOP);
                Motor_Dir_Set(MOTOR_R, MOTOR_STOP);
            }
        }
        else
        {
            Motor_Dir_Set(MOTOR_L, MOTOR_STOP);
            Motor_Dir_Set(MOTOR_R, MOTOR_STOP);
        }

        // check the command from BT
        UART_BT_Cmd( );

        if (car2.debug)
        {
            sprintf(m, "%d ", Ei);
            UART_PutString(UART_BT, m);
        }

        if (BTN3_Pressed())
        {
            car2.motor_onoff = 1 - car2.motor_onoff;
            sprintf(m, "Motor=%d\r\n", car2.motor_onoff);
            UART_PutString(UART_BT, m);
        }
    }
}


//4 wheels, for battery12v to 6v + BT control
void vTask_Gyro_Car_4(void *pvParameters )
{
    (void)pvParameters; // prevent compiler worning/error
    portTickType xLastWakeTime;

    Motor_L298N_PWM_Init(car2.PWM_freq); //1.5kHz
    car2.PWM_Max = Motor_L298N_PWN_Period();
    car2.PWM_step = car2.PWM_Max / 8;
    xLastWakeTime = xTaskGetTickCount();
    for( ;; )
    {
        vTaskDelayUntil(&xLastWakeTime, 200);
        if (car2.motor_onoff)
        {
            Motor_PWM_Dir_Set_Raw(MOTOR_L, car2.motor_dir_L, car2.PWM_step);
            Motor_PWM_Dir_Set_Raw(MOTOR_R, car2.motor_dir_R, car2.PWM_step);
        }
    }
}


// balance car, for battery12v to 6v + BT control
void vTask_Gyro_Car_3(void *pvParameters )
{
    (void)pvParameters; // prevent compiler worning/error
    int gyro_x, gyro_y, gyro_z;
    int base_x, base_y, base_z;
    int Ei, PWM;
    int sw, motor_dir;
    char m[100];
    portTickType xLastWakeTime;

    Motor_L298N_PWM_Init(car2.PWM_freq); //1.5kHz
    Gyro_MPU6050_Init();
    Gyro_MPU6050_Calibration(10);
    Gyro_MPU6050_Kalman_Angle_Start();
    // get the balance point
    Gyro_MPU6050_Kalman_Angle_Get(&base_x, &base_y, &base_z);
    car2.balance_angle = base_y;
    car2.angle_target_F = car2.balance_angle - 10;
    car2.angle_target_B = car2.balance_angle + 10;

    // set the timer as 15 ms in initial stage
    xLastWakeTime = xTaskGetTickCount();
    sw = 3;
    motor_dir = MOTOR_STOP;
    car2.PWM_step = Motor_L298N_PWN_Period() / 100;
    car2.PWM_static = car2.PWM_step * 70;
    PWM = Motor_L298N_PWN_Period();
    for( ;; )
    {
        vTaskDelayUntil(&xLastWakeTime, car2.Task_timer);
        Gyro_MPU6050_Kalman_Angle_Get(&gyro_x, &gyro_y, &gyro_z);
        Ei = gyro_y - car2.balance_angle;

        if (sw == 3)
        {
            if (Ei > 0)
            {
                sw = 0;
                motor_dir = MOTOR_CW;
            }
            else
            {
                sw = 1;
                motor_dir = MOTOR_CCW;
            }
        }

        if (Ei > car2.Angle_Max || Ei < car2.Angle_Min)
        {
            Motor_PWM_Dir_Set_Raw(MOTOR_L, MOTOR_STOP, 0);
            Motor_PWM_Dir_Set_Raw(MOTOR_R, MOTOR_STOP, 0);
        }
        else
        {
            if (car2.motor_onoff)
            {
                if (gyro_y >= car2.angle_target_B && sw == 0)
                {
                    sw = 1;
                    motor_dir = MOTOR_CW;
                }

                if (gyro_y <= car2.angle_target_F && sw == 1)
                {
                    sw = 0;
                    motor_dir = MOTOR_CCW;
                }
                
                Motor_PWM_Dir_Set_Raw(MOTOR_R, motor_dir, PWM);
                Motor_PWM_Dir_Set_Raw(MOTOR_L, motor_dir, PWM);
            }
        }

        if (car2.debug)
        {
            sprintf(m, "%d %d %d %d\r\n", gyro_y, car2.angle_target_F, car2.angle_target_B, motor_dir);
            UART_PutString(UART_BT, m);
        }

        if (BTN3_Pressed())
        {
            car2.motor_onoff = 1 - car2.motor_onoff;
            sprintf(m, "Motor=%d\r\n", car2.motor_onoff);
            UART_PutString(UART_BT, m);
        }
    }
}

// for battery12v to 6v + BT control
void vTask_Gyro_Car_2(void *pvParameters )
{
    (void)pvParameters; // prevent compiler worning/error
    int gyro_x, gyro_y, gyro_z;
    int base_x, base_y, base_z;
    double Ki, Kp, Kd, P, P_last, I, I_last, D, D_last, U, E, E_last;
    int Ei, Ui, PWM;

    char m[100];
    portTickType xLastWakeTime;

    Motor_L298N_PWM_Init(car2.PWM_freq); //1.5kHz
    Gyro_MPU6050_Init();
    Gyro_MPU6050_Calibration(10);
    Gyro_MPU6050_Kalman_Angle_Start();
    // get the balance point
    Gyro_MPU6050_Kalman_Angle_Get(&base_x, &base_y, &base_z);
    car2.balance_angle = base_y;
    // set the timer as 15 ms in initial stage
    xLastWakeTime = xTaskGetTickCount();
    E_last = I_last = P_last = D_last = 0;
    car2.PWM_step = Motor_L298N_PWN_Period() / 100;
    car2.PWM_static = car2.PWM_step * 70;
    
    for( ;; )
    {
        vTaskDelayUntil(&xLastWakeTime, car2.Task_timer);
        Gyro_MPU6050_Kalman_Angle_Get(&gyro_x, &gyro_y, &gyro_z);
        Ei = gyro_y - car2.balance_angle;

        // type convert, int 2 double
        E = (double)Ei;
        Kp = (double)car2.Kp;
        Ki = (double)car2.Ki;
        Kd = (double)car2.Kd;

        P = Kp/100 * E;
        I = Ki/100 * (I_last + E) ;
        //D = Kd/100 * (E - E_last);
        D = Kd/100 * (E_last - E);

        U = (P + I + D);

        Ui = (int)U;
        PWM = abs(Ui);
        PWM = PWM * car2.PWM_step + car2.PWM_static;

        I_last = I;
        E_last = E;

        if (car2.motor_onoff)
        {
            if (E > car2.Angle_Max || E < car2.Angle_Min)
            {
                Motor_PWM_Dir_Set_Raw(MOTOR_L, MOTOR_STOP, 0);
                Motor_PWM_Dir_Set_Raw(MOTOR_R, MOTOR_STOP, 0);
            }
            else
            if (E > car2.angle_step) //body back down
            {
                Motor_PWM_Dir_Set_Raw(MOTOR_L, MOTOR_CW, PWM);
                Motor_PWM_Dir_Set_Raw(MOTOR_R, MOTOR_CW, PWM);
            }
            else
            if (E < -car2.angle_step) // body front down
            {
                Motor_PWM_Dir_Set_Raw(MOTOR_L, MOTOR_CCW, PWM);
                Motor_PWM_Dir_Set_Raw(MOTOR_R, MOTOR_CCW, PWM);
            }
            else
            {
                Motor_PWM_Dir_Set_Raw(MOTOR_L, MOTOR_STOP, 0);
                Motor_PWM_Dir_Set_Raw(MOTOR_R, MOTOR_STOP, 0);
            }
        }
        else
        {
            Motor_PWM_Dir_Set_Raw(MOTOR_L, MOTOR_STOP, 0);
            Motor_PWM_Dir_Set_Raw(MOTOR_R, MOTOR_STOP, 0);
        }

        if (car2.debug)
        {
            //sprintf(m, "x:%d y:%d z:%d\n", gyro_x, gyro_y, gyro_z);
            //sprintf(m, "%d %d %d %d %d %d %d %d %d %d\r\n", E, U, PWM, P, I, D, gyro_y, car2.Kp, car2.Ki, car2.Kd);
            sprintf(m, "%d %d %5.2f %5.2f %5.2f %5.2f %5.2f\r\n", gyro_y, PWM, E, U, P, I, D);
            UART_PutString(UART_BT, m);
        }

        if (BTN3_Pressed())
        {
            car2.motor_onoff = 1 - car2.motor_onoff;
            sprintf(m, "Motor=%d\r\n", car2.motor_onoff);
            UART_PutString(UART_BT, m);
        }
    }
}

// for 9V input motor driver
void vTask_Gyro_Car_1(void *pvParameters )
{
    (void)pvParameters; // prevent compiler worning/error
    int gyro_x, gyro_y, gyro_z;
    int base_x, base_y, base_z;
//    int dir_y, dir_abs_y;
//    int k_dir_y;
//    int x;
    int y_PWM, y_diff, u_power, u_power_abs;
    char m[32];
    portTickType xLastWakeTime;

    Motor_L298N_PWM_Init(1000); //1kHz
    Gyro_MPU6050_Init();
    Gyro_MPU6050_Calibration(10);
    Gyro_MPU6050_Kalman_Angle_Start();
    // get the balance point
    Gyro_MPU6050_Kalman_Angle_Get(&base_x, &base_y, &base_z);

    UART_PutString(UART_BT, "X Y Z Kp U PWM Diff\r\n");
    xLastWakeTime = xTaskGetTickCount();
    for( ;; )
    {
        vTaskDelayUntil(&xLastWakeTime, car2.Task_timer);
        Gyro_MPU6050_Kalman_Angle_Get(&gyro_x, &gyro_y, &gyro_z);

        y_diff = gyro_y - base_y;
        u_power = y_diff * car2.Kp / 100;
        u_power_abs = abs(u_power);
        y_PWM = constrain(u_power_abs, car2.PWM_Min, car2.PWM_Max);         // 限定值域

        if (car2.motor_onoff)
        {
            if (y_diff > car2.Angle_Max || y_diff < car2.Angle_Min)
            {
                Motor_PWM_Dir_Set(MOTOR_L, MOTOR_STOP, 0);
                Motor_PWM_Dir_Set(MOTOR_R, MOTOR_STOP, 0);
            }
            else
            if (y_diff > 1)
            {
                Motor_PWM_Dir_Set(MOTOR_L, MOTOR_CW, y_PWM);
                Motor_PWM_Dir_Set(MOTOR_R, MOTOR_CW, y_PWM);
            }
            else
            if (y_diff < -1)
            {
                Motor_PWM_Dir_Set(MOTOR_L, MOTOR_CCW, y_PWM);
                Motor_PWM_Dir_Set(MOTOR_R, MOTOR_CCW, y_PWM);
            }
            else
            {
                Motor_PWM_Dir_Set(MOTOR_L, MOTOR_STOP, 0);
                Motor_PWM_Dir_Set(MOTOR_R, MOTOR_STOP, 0);
            }
        }
        if (car2.debug)
        {
            //sprintf(m, "x:%d y:%d z:%d\n", gyro_x, gyro_y, gyro_z);
            sprintf(m, "%d %d %d %d %d\r\n", gyro_y, car2.Kp, u_power, y_PWM, y_diff);
            UART_PutString(UART_BT, m);
       }
/*
        x = 0;
        LCD_Show_Value(x, 0, dir_y, 10, 4);
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