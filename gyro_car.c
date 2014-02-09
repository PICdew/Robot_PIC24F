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

Gyro_Car_Str car2 =
{
    1000,
    15,
    70,20,30,
    3600,15999, 0, 0,
    35, -35,
    185,
    1, 
    0,0,
    0, MOTOR_STOP, MOTOR_STOP,
    0
};

void Gyro_Car_Debug(void)
{
    char m[80];
    car2.debug = 1 - car2.debug;
    sprintf(m,"Debug=%d\r\n", car2.debug);
    UART_PutString(UART_BT, m);
}
void Gyro_Car_Motor(void)
{
    char m[80];
    car2.motor_onoff = 1 - car2.motor_onoff;
    sprintf(m,"Motor=%d\r\n", car2.motor_onoff);
    UART_PutString(UART_BT, m);
}

void Gyro_Car_Kp_Up(void)
{
    char m[80];
    car2.Kp += 1;
    sprintf(m,"Kp=%d\r\n", car2.Kp);
    UART_PutString(UART_BT, m);
}

void Gyro_Car_Kp_Down(void)
{
    char m[80];
    car2.Kp -= 1;
    if (car2.Kp < 0)
        car2.Kp = 0;
    sprintf(m,"Kp=%d\r\n", car2.Kp);
    UART_PutString(UART_BT, m);
}

void Gyro_Car_Ki_Up(void)
{
    char m[80];
    car2.Ki += 1;
    sprintf(m,"Ki=%d\r\n", car2.Ki);
    UART_PutString(UART_BT, m);
}

void Gyro_Car_Ki_Down(void)
{
    char m[80];
    car2.Ki -= 1;
    if (car2.Ki < 0)
        car2.Ki = 0;
    sprintf(m,"Ki=%d\r\n", car2.Ki);
    UART_PutString(UART_BT, m);
}

void Gyro_Car_Kd_Up(void)
{
    char m[80];
    car2.Kd += 1;
    sprintf(m,"Kd=%d\r\n", car2.Kd);
    UART_PutString(UART_BT, m);
}

void Gyro_Car_Kd_Down(void)
{
    char m[80];
    car2.Kd -= 1;
    if (car2.Kd < 0)
        car2.Kd = 0;
    sprintf(m,"Kd=%d\r\n", car2.Kd);
    UART_PutString(UART_BT, m);
}

void Gyro_Car_TaskTimer_Up(void)
{
    char m[80];
    car2.Task_timer += 1;
    sprintf(m,"Timer=%d\r\n", car2.Task_timer);
    UART_PutString(UART_BT, m);
}

void Gyro_Car_TaskTimer_Down(void)
{
    char m[80];
    car2.Task_timer -= 1;
    if (car2.Task_timer < 5)
        car2.Task_timer = 5;
    sprintf(m,"Timer=%d\r\n", car2.Task_timer);
    UART_PutString(UART_BT, m);
}


void Gyro_Car_Balance_Up(void)
{
    char m[80];
    car2.balance_angle += 1;
    sprintf(m,"Balance=%d\r\n", car2.balance_angle);
    UART_PutString(UART_BT, m);
}

void Gyro_Car_Balance_Down(void)
{
    char m[80];
    car2.balance_angle -= 1;
    sprintf(m,"Balance=%d\r\n", car2.balance_angle);
    UART_PutString(UART_BT, m);
}

void Gyro_Car_Angle_Max_Up(void)
{
    char m[80];
    car2.Angle_Max += 1;
    sprintf(m,"Angle_Max=%d\r\n", car2.Angle_Max);
    UART_PutString(UART_BT, m);
}

void Gyro_Car_Angle_Max_Down(void)
{
    char m[80];
    car2.Angle_Max -= 1;
    sprintf(m,"Angle_Max=%d\r\n", car2.Angle_Max);
    UART_PutString(UART_BT, m);
}

void Gyro_Car_Angle_Min_Up(void)
{
    char m[80];
    car2.Angle_Min += 1;
    sprintf(m,"Angle_Min=%d\r\n", car2.Angle_Min);
    UART_PutString(UART_BT, m);
}

void Gyro_Car_Angle_Min_Down(void)
{
    char m[80];
    car2.Angle_Min -= 1;
    sprintf(m,"Angle_Min=%d\r\n", car2.Angle_Min);
    UART_PutString(UART_BT, m);
}

void Gyro_Car_Angle_Step_Up(void)
{
    char m[80];
    car2.angle_step += 1;
    sprintf(m,"Angle_Step=%d\r\n", car2.angle_step);
    UART_PutString(UART_BT, m);
}

void Gyro_Car_Angle_Step_Down(void)
{
    char m[80];
    car2.angle_step -= 1;
    if (car2.angle_step < 0)
        car2.angle_step = 0;
    sprintf(m,"Angle_Step=%d\r\n", car2.angle_step);
    UART_PutString(UART_BT, m);
}

void Gyro_Car_PWM_Step_Up(void)
{
    char m[80];
    car2.PWM_step += 1;
    sprintf(m,"PWM_step=%d\r\n", car2.PWM_step);
    UART_PutString(UART_BT, m);
}

void Gyro_Car_PWM_Step_Down(void)
{
    char m[80];
    car2.PWM_step -= 1;
    if (car2.PWM_step < 0)
        car2.PWM_step = 0;
    sprintf(m,"PWM_step=%d\r\n", car2.PWM_step);
    UART_PutString(UART_BT, m);
}

void Gyro_Car_PWM_Static_Up(void)
{
    char m[80];
    car2.PWM_static += 1;
    sprintf(m,"PWM_static=%d\r\n", car2.PWM_static);
    UART_PutString(UART_BT, m);
}

void Gyro_Car_PWM_Static_Down(void)
{
    char m[80];
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
    char m[80];
    car2.PWM_step += 200;
    if (car2.PWM_step > car2.PWM_Max)
        car2.PWM_step = car2.PWM_Max;
    sprintf(m,"PWM_step=%d %d\r\n", car2.PWM_step, car2.PWM_Max);
    UART_PutString(UART_BT, m);
}

void Gyro_Car_PWM_Speed_Down(void)
{
    char m[80];
    car2.PWM_step -= 200;
    if (car2.PWM_step < car2.PWM_Min)
        car2.PWM_step = car2.PWM_Min;
    sprintf(m,"PWM_step=%d %d\r\n", car2.PWM_step, car2.PWM_Min);
    UART_PutString(UART_BT, m);
}

int UART_BT_Cmd(void)
{
    int i,n,m;
    char cmd[30];

    n = Check_Rx_Buffer(UART_BT);

    if (n >= 2)
    {
        // parser the command
        cmd[0] = UART_PopFIFO(UART_BT);
        cmd[1] = UART_PopFIFO(UART_BT);
        cmd[2] = 0;
        //UART_PutString(UART_BT, cmd);
        m = atoi(cmd);
        sprintf(cmd, "Cmd=%d => ", m);
        UART_PutString(UART_BT, cmd);
        switch (m)
                {
                    case CMD_DEBUG:
                        Gyro_Car_Debug();
                        break;
                    case CMD_TASK_TIME_U:
                        Gyro_Car_TaskTimer_Up();
                        break;
                    case CMD_TASK_TIME_D:
                        Gyro_Car_TaskTimer_Down();
                        break;
                    case CMD_KP_U:
                        Gyro_Car_Kp_Up();
                        break;
                    case CMD_KP_D:
                        Gyro_Car_Kp_Down();
                        break;
                    case CMD_KI_U:
                        Gyro_Car_Ki_Up();
                        break;
                    case CMD_KI_D:
                        Gyro_Car_Ki_Down();
                        break;
                    case CMD_KD_U:
                        Gyro_Car_Kd_Up();
                        break;
                    case CMD_KD_D:
                        Gyro_Car_Kd_Down();
                        break;
                    case CMD_MOTOR:
                        Gyro_Car_Motor();
                        break;
                    case CMD_BL_U:
                        Gyro_Car_Balance_Up();
                        break;
                    case CMD_BL_D:
                        Gyro_Car_Balance_Down();
                        break;
                    case CMD_ANGLE_MAX_U:
                        Gyro_Car_Angle_Max_Up();
                        break;
                    case CMD_ANGLE_MAX_D:
                        Gyro_Car_Angle_Max_Down();
                        break;
                    case CMD_ANGLE_MIN_U:
                        Gyro_Car_Angle_Min_Up();
                        break;
                    case CMD_ANGLE_MIN_D:
                        Gyro_Car_Angle_Min_Down();
                        break;
                    case CMD_ANGLE_STEP_U:
                        Gyro_Car_Angle_Step_Up();
                        break;
                    case CMD_ANGLE_STEP_D:
                        Gyro_Car_Angle_Step_Down();
                        break;
                    case CMD_PWM_STEP_U:
                        Gyro_Car_PWM_Step_Up();
                        break;
                    case CMD_PWM_STEP_D:
                        Gyro_Car_PWM_Step_Down();
                        break;
                    case CMD_PWM_STATIC_U:
                        Gyro_Car_PWM_Static_Up();
                        break;
                    case CMD_PWM_STATIC_D:
                        Gyro_Car_PWM_Static_Down();
                        break;
                    case CMD_CAR_FF:
                        Gyro_Car_PWM_FF();
                        break;
                    case CMD_CAR_BK:
                        Gyro_Car_PWM_BK();
                        break;
                    case CMD_CAR_STOP:
                        Gyro_Car_PWM_Stop();
                        break;
                    case CMD_CAR_RIGHT:
                        Gyro_Car_PWM_Right();
                        break;
                    case CMD_CAR_LEFT:
                        Gyro_Car_PWM_Left();
                        break;
                    case CMD_CAR_SPEED_U:
                        Gyro_Car_PWM_Speed_Up();
                        break;
                    case CMD_CAR_SPEED_D:
                        Gyro_Car_PWM_Speed_Down();
                        break;
                    default:
                        UART_PutString(UART_BT, "no cmd!\r\n");
                        break;
                }
            }
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
    //Gyro_MPU6050_Calibration(10);
    Gyro_MPU6050_Kalman_Angle_Start();
    // get the balance point
    Gyro_MPU6050_Kalman_Angle_Get(&base_x, &base_y, &base_z);
    car2.balance_angle = base_y;
    // set the timer as 15 ms in initial stage
    xLastWakeTime = xTaskGetTickCount();
    //E_last = I_last = P_last = D_last = 0;

    for( ;; )
    {
        vTaskDelayUntil(&xLastWakeTime, car2.Task_timer);
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
            sprintf(m, "%d %d %d\r\n", gyro_x, gyro_y, gyro_z);
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
    int accX, accY, accZ;
    int gyroX, gyroY, gyroZ;

    double accXangle, accYangle, accZangle; // Angle calculate using the accelerometer
    double gyroXrate, gyroYrate, gyroZrate;
    char m[300];

    Motor_L298N_PWM_Init(car2.PWM_freq); //1.5kHz
    car2.PWM_Max = Motor_L298N_PWN_Period();
    car2.PWM_step = car2.PWM_Max / 2;
    Gyro_MPU6050_Init();
    Gyro_MPU6050_Kalman_Angle_Start();
    xLastWakeTime = xTaskGetTickCount();
    for( ;; )
    {
        vTaskDelayUntil(&xLastWakeTime, car2.Task_timer);
        Gyro_MPU6050_Kalman_Angle_Get(&gyroX, &gyroY, &gyroZ);
        if (car2.motor_onoff)
        {
            Motor_PWM_Dir_Set_Raw(MOTOR_L, car2.motor_dir_L, car2.PWM_step);
            Motor_PWM_Dir_Set_Raw(MOTOR_R, car2.motor_dir_R, car2.PWM_step);
        }
        else
        {
            Motor_PWM_Dir_Set_Raw(MOTOR_L, MOTOR_STOP, 0);
            Motor_PWM_Dir_Set_Raw(MOTOR_R, MOTOR_STOP, 0);
        }

        if (car2.debug)
        {
            /*
            gyroX = Gyro_MPU6050_GetData(GYRO_XOUT_H);
            gyroY = Gyro_MPU6050_GetData(GYRO_YOUT_H);
            gyroZ = Gyro_MPU6050_GetData(GYRO_ZOUT_H);
            accX = Gyro_MPU6050_GetData(ACCEL_XOUT_H);
            accY = Gyro_MPU6050_GetData(ACCEL_YOUT_H);
            accZ = Gyro_MPU6050_GetData(ACCEL_ZOUT_H);
            sprintf(m, "%d %d %d %d %d %d\r\n", gyroX, gyroY, gyroZ, accX, accY, accZ);
            */
            sprintf(m, "%d %d %d\r\n", gyroX, gyroY, gyroZ);
            UART_PutString(UART_BT, m);
        }

        // check & exec the command from BT
        UART_BT_Cmd( );

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




#if 0
/*
    zlstone sample code
 */
//MCU：Mega16；晶振：8MHz；
//PWM:4KHz;滤波器频率:100Hz;系统频率：100Hz；10ms;
//赵国霖：10.05.10；
#include <iom16v.h>
#include <macros.h>
#include <math.h>

//#define checkbit(var,bit)     (var&(0x01<<(bit)))     /*定义查询位函数*/
//#define setbit(var,bit)     (var|=(0x01<<(bit)))     /*定义置位函数*/
//#define clrbit(var,bit)     (var&=(~(0x01<<(bit)))) /*定义清零位函数*/


//-------------------------------------------------------
//输出端口初始化
void PORT_initial(void)
{
	DDRA=0B00000000;
	PINA=0X00;
	PORTA=0X00;

	DDRB=0B00000000;
	PINB=0X00;
	PORTB=0X00;

	DDRC=0B00010000;
	PINC=0X00;
	PORTC=0X00;

	DDRD=0B11110010;
	PIND=0X00;
	PORTD=0X00;
}


//-------------------------------------------------------
//定时器1初始化
void T1_initial(void)
{
	TCCR1A|=(1<<COM1A1)|(1<<WGM10)|(1<<COM1B1);				//T1：8位快速PWM模式、匹配时清0，TOP时置位
	TCCR1B|=(1<<WGM12)|(1<<CS11);			//PWM：8分频：8M/8/256=4KHz;
}


//-------------------------------------------------------
//定时器2初始化
void T2_initial(void)			//T2：计数至OCR2时产生中断
{
	OCR2=0X4E;			//T2：计数20ms(0X9C)10ms(0X4E)时产生中断;
	TIMSK|=(1<<OCIE2);
	TCCR2|=(1<<WGM21)|(1<<CS22)|(1<<CS21)|(1<<CS20);			//CTC模式，1024分频
}


//-------------------------------------------------------
//外部中断初始化
void INT_initial(void)
{
	MCUCR|=(1<<ISC01)|(1<<ISC00)|(1<<ISC11)|(1<<ISC10);			//INT0、INT1上升沿有效
	GICR|=(1<<INT0)|(1<<INT1);			//外部中断使能
}


//-------------------------------------------------------
//串口初始化；
void USART_initial( void )
{
	UBRRH = 0X00;
	UBRRL = 51;		//f=8MHz;设置波特率:9600:51;19200:25;
	UCSRB = (1<<RXEN)|(1<<TXEN);		//接收器与发送器使能；
	UCSRC = (1<<URSEL)|(1<<UCSZ0)|(1<<UCSZ1);		//设置帧格式: 8 个数据位, 1 个停止位；

	UCSRB|=(1<<RXCIE);		//USART接收中断使能
}


//-------------------------------------------------------
//串口发送数据；
void USART_Transmit( unsigned char data )
{
	while ( !( UCSRA & (1<<UDRE)));		//等待发送缓冲器为空；
	UDR = data;		//将数据放入缓冲器，发送数据；
}


//-------------------------------------------------------
//串口接收数据中断，确定数据输出的状态；
#pragma interrupt_handler USART_Receive_Int:12
static char USART_State;
void USART_Receive_Int(void)
{
	USART_State=UDR;//USART_Receive();
}


//-------------------------------------------------------
//计算LH侧轮速：INT0中断；
//-------------------------------------------------------
static int speed_real_LH;
//-------------------------------------------------------
#pragma interrupt_handler SPEEDLHINT_fun:2
void SPEEDLHINT_fun(void)
{
	if (0==(PINB&BIT(0)))
	{
		speed_real_LH-=1;
	}
	else
	{
		speed_real_LH+=1;
	}
}


//-------------------------------------------------------
//计算RH侧轮速，：INT1中断；
//同时将轮速信号统一成前进方向了；
//-------------------------------------------------------
static int speed_real_RH;
//-------------------------------------------------------
#pragma interrupt_handler SPEEDRHINT_fun:3
void SPEEDRHINT_fun(void)
{
	if (0==(PINB&BIT(1)))
	{
		speed_real_RH+=1;
	}
	else
	{
		speed_real_RH-=1;
	}
}


//-------------------------------------------------------
//ADport采样：10位，采样基准电压Aref
//-------------------------------------------------------
static int AD_data;
//-------------------------------------------------------
int ADport(unsigned char port)
{
	ADMUX=port;
	ADCSRA|=(1<<ADEN)|(1<<ADSC)|(1<<ADPS1)|(1<<ADPS0);			//采样频率为8分频；
	while(!(ADCSRA&(BIT(ADIF))));
	AD_data=ADCL;
	AD_data+=ADCH*256;
	AD_data-=512;
	return (AD_data);
}


//*
//-------------------------------------------------------
//Kalman滤波，8MHz的处理时间约1.8ms；
//-------------------------------------------------------
static float angle, angle_dot; 		//外部需要引用的变量
//-------------------------------------------------------
static const float Q_angle=0.001, Q_gyro=0.003, R_angle=0.5, dt=0.01;
			//注意：dt的取值为kalman滤波器采样时间;
static float P[2][2] = {
							{ 1, 0 },
							{ 0, 1 }
						};

static float Pdot[4] ={0,0,0,0};

static const char C_0 = 1;

static float q_bias, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
//-------------------------------------------------------
void Kalman_Filter(float angle_m,float gyro_m)			//gyro_m:gyro_measure
{
	angle+=(gyro_m-q_bias) * dt;

	Pdot[0]=Q_angle - P[0][1] - P[1][0];
	Pdot[1]=- P[1][1];
	Pdot[2]=- P[1][1];
	Pdot[3]=Q_gyro;

	P[0][0] += Pdot[0] * dt;
	P[0][1] += Pdot[1] * dt;
	P[1][0] += Pdot[2] * dt;
	P[1][1] += Pdot[3] * dt;


	angle_err = angle_m - angle;


	PCt_0 = C_0 * P[0][0];
	PCt_1 = C_0 * P[1][0];

	E = R_angle + C_0 * PCt_0;

	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;

	t_0 = PCt_0;
	t_1 = C_0 * P[0][1];

	P[0][0] -= K_0 * t_0;
	P[0][1] -= K_0 * t_1;
	P[1][0] -= K_1 * t_0;
	P[1][1] -= K_1 * t_1;


	angle	+= K_0 * angle_err;
	q_bias	+= K_1 * angle_err;
	angle_dot = gyro_m-q_bias;
}
//*/

/*
//-------------------------------------------------------
//互补滤波
//-------------------------------------------------------
static float angle,angle_dot; 		//外部需要引用的变量
//-------------------------------------------------------
static float bias_cf;
static const float dt=0.01;
//-------------------------------------------------------
void complement_filter(float angle_m_cf,float gyro_m_cf)
{
	bias_cf*=0.998;			//陀螺仪零飘低通滤波；500次均值；
	bias_cf+=gyro_m_cf*0.002;
	angle_dot=gyro_m_cf-bias_cf;
	angle=(angle+angle_dot*dt)*0.90+angle_m_cf*0.05;
	//加速度低通滤波；20次均值；按100次每秒计算，低通5Hz；
}
*/




//-------------------------------------------------------
//AD采样；
//以角度表示；
//加速度计：1.2V=1g=90°；满量程：1.3V~3.7V；
//陀螺仪：0.5V~4.5V=-80°~+80°；满量程5V=200°=256=200°；
//-------------------------------------------------------
static float gyro,acceler;
//-------------------------------------------------------
void AD_calculate(void)
{

	acceler=ADport(2)+28;			//角度校正
	gyro=ADport(3);

	acceler*=0.004069;		//系数换算：2.5/(1.2*512);
	acceler=asin(acceler);
	gyro*=0.00341;			//角速度系数：(3.14/180)* 100/512=0.01364；

	Kalman_Filter(acceler,gyro);
	//complement_filter(acceler,gyro);
}



//-------------------------------------------------------
//PWM输出
//-------------------------------------------------------
void PWM_output (int PWM_LH,int PWM_RH)
{
	if (PWM_LH<0)
	{
		PORTD|=BIT(6);
		PWM_LH*=-1;
	}
	else
	{
		PORTD&=~BIT(6);
	}

	if (PWM_LH>252)
	{
		PWM_LH=252;
	}


	if (PWM_RH<0)
	{
		PORTD|=BIT(7);
		PWM_RH*=-1;
	}
	else
	{
		PORTD&=~BIT(7);
	}

	if (PWM_RH>252)
	{
		PWM_RH=252;
	}


	OCR1AH=0;
	OCR1AL=PWM_LH;			//OC1A输出；

	OCR1BH=0;
	OCR1BL=PWM_RH;			//OC1B输出；

}




//-------------------------------------------------------
//计算PWM输出值
//车辆直径：76mm; 12*64pulse/rev; 1m=3216pulses;
//-------------------------------------------------------
//static int speed_diff,speed_diff_all,speed_diff_adjust;
//static float K_speed_P,K_speed_I;
static float K_voltage,K_angle,K_angle_dot,K_position,K_position_dot;
static float K_angle_AD,K_angle_dot_AD,K_position_AD,K_position_dot_AD;
static float position,position_dot;
static float position_dot_filter;
static float PWM;
static int speed_output_LH,speed_output_RH;
static int Turn_Need,Speed_Need;
//-------------------------------------------------------
void PWM_calculate(void)
{
	if ( 0==(~PINA&BIT(1)) )	//左转
	{
		Turn_Need=-40;
	}
	else if ( 0==(~PINB&BIT(2)) ) 	//右转
	{
		Turn_Need=40;
	}
	else	//不转
	{
		Turn_Need=0;
	}

	if ( 0==(~PINC&BIT(0)) )	//前进
	{
		Speed_Need=-2;
	}
	else if ( 0==(~PINC&BIT(1)) )	//后退
	{
		Speed_Need=2;
	}
	else	//不动
	{
		Speed_Need=0;
	}

	K_angle_AD=ADport(4)*0.007;
	K_angle_dot_AD=ADport(5)*0.007;
	K_position_AD=ADport(6)*0.007;
	K_position_dot_AD=ADport(7)*0.007;

	position_dot=(speed_real_LH+speed_real_RH)*0.5;

	position_dot_filter*=0.85;		//车轮速度滤波
	position_dot_filter+=position_dot*0.15;

	position+=position_dot_filter;
	//position+=position_dot;
	position+=Speed_Need;

	if (position<-768)		//防止位置误差过大导致的不稳定
	{
		position=-768;
	}
	else if  (position>768)
	{
		position=768;
	}

	PWM = K_angle*angle *K_angle_AD + K_angle_dot*angle_dot *K_angle_dot_AD +
					K_position*position *K_position_AD + K_position_dot*position_dot_filter *K_position_dot_AD;


	speed_output_RH = PWM - Turn_Need;
	speed_output_LH = - PWM - Turn_Need ;

	/*
	speed_diff=speed_real_RH-speed_real_LH;			//左右轮速差PI控制；
	speed_diff_all+=speed_diff;
	speed_diff_adjust=(K_speed_P*speed_diff+K_speed_I*speed_diff_all)/2;
	*/

	PWM_output (speed_output_LH,speed_output_RH);
}

//-------------------------------------------------------
//定时器2中断处理
//-------------------------------------------------------
static unsigned char temp;
//-------------------------------------------------------
#pragma interrupt_handler T2INT_fun:4
void T2INT_fun(void)
{
	AD_calculate();

	PWM_calculate();

	if(temp>=4)			//10ms即中断；每秒计算：100/4=25次；
	{
		if (USART_State==0X30)		//ASCII码：0X30代表字符'0'
		{
			USART_Transmit(angle*57.3+128);
			USART_Transmit(angle_dot*57.3+128);
			USART_Transmit(128);
		}
		else if(USART_State==0X31)		//ASCII码：0X30代表字符'1'
		{
			USART_Transmit(speed_output_LH+128);
			USART_Transmit(speed_output_RH+128);
			USART_Transmit(128);
		}
		else if(USART_State==0X32)		//ASCII码：0X30代表字符'2'
		{
			USART_Transmit(speed_real_LH+128);
			USART_Transmit(speed_real_RH+128);
			USART_Transmit(128);
		}
		else if(USART_State==0X33)		//ASCII码：0X30代表字符'3'
		{
			USART_Transmit(K_angle+128);
			USART_Transmit(K_angle_dot+128);
			USART_Transmit(K_position_dot+128);
		}
		temp=0;
	}
	speed_real_LH=0;
	speed_real_RH=0;
	temp+=1;
}



//-------------------------------------------------------
int i,j;
//-------------------------------------------------------
void main(void)
{
	PORT_initial();
	T2_initial();
	INT_initial();
	USART_initial ();

	SEI();

	K_position=0.8 * 0.209;		//换算系数：(256/10) * (2*pi/(64*12))=0.20944；//256/10:电压换算至PWM，256对应10V；
	K_angle=34 * 25.6;		//换算系数：256/10 =25.6；
	K_position_dot=1.09 * 20.9;		//换算系数：(256/10) * (25*2*pi/(64*12))=20.944；
	K_angle_dot=2 * 25.6;		//换算系数：256/10 =25.6；

	for (i=1;i<=500;i++)		//延时启动PWM，等待卡尔曼滤波器稳定
	{
		for (j=1;j<=300;j++);;

	}
	T1_initial();

	while(1)
	{
		;
	}
}
#endif