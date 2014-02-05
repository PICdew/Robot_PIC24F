
#ifndef _Gyro_Car_
#define _Gyro_Car_

#define CMD_DEBUG           91
#define CMD_MOTOR           92

#define CMD_KP_U            1
#define CMD_KP_D            2
#define CMD_KI_U            3
#define CMD_KI_D            4
#define CMD_KD_U            5
#define CMD_KD_D            6
#define CMD_BL_U            7
#define CMD_BL_D            8
#define CMD_TASK_TIME_U     11
#define CMD_TASK_TIME_D     12
#define CMD_ANGLE_MAX_U     13
#define CMD_ANGLE_MAX_D     14
#define CMD_ANGLE_MIN_U     15
#define CMD_ANGLE_MIN_D     16
#define CMD_ANGLE_STEP_U    17
#define CMD_ANGLE_STEP_D    18
#define CMD_PWM_STEP_U      19
#define CMD_PWM_STEP_D      20
#define CMD_PWM_STATIC_U    21
#define CMD_PWM_STATIC_D    22

#define CMD_CAR_FF          30
#define CMD_CAR_BK          31
#define CMD_CAR_STOP        32
#define CMD_CAR_RIGHT       33
#define CMD_CAR_LEFT        34
#define CMD_CAR_SPEED_U     35
#define CMD_CAR_SPEED_D     36

void Gyro_Car_PWM_Step_Up(void);
void Gyro_Car_PWM_Step_Down(void);
void Gyro_Car_PWM_Static_Up(void);
void Gyro_Car_PWM_Static_Down(void);

void vTask_Gyro_Car_1(void *pvParameters );
void vTask_Gyro_Car_PID(void *pvParameters );
void vTask_Gyro_Car_2(void *pvParameters );
void vTask_Gyro_Car_3(void *pvParameters );
void vTask_Gyro_Car_4(void *pvParameters );
void vTask_Gyro_Car_5(void *pvParameters );

void Gyro_Car_Debug(void);
void Gyro_Car_TaskTimer_Up(void);
void Gyro_Car_TaskTimer_Down(void);
void Gyro_Car_Kp_Down(void);
void Gyro_Car_Kp_Up(void);
void Gyro_Car_Ki_Down(void);
void Gyro_Car_Ki_Up(void);
void Gyro_Car_Kd_Down(void);
void Gyro_Car_Kd_Up(void);
void Gyro_Car_Motor(void);
void Gyro_Car_Balance_Up(void);
void Gyro_Car_Balance_Down(void);
void Gyro_Car_Angle_Max_Up(void);
void Gyro_Car_Angle_Max_Down(void);
void Gyro_Car_Angle_Min_Up(void);
void Gyro_Car_Angle_Min_Down(void);
void Gyro_Car_Angle_Step_Up(void);
void Gyro_Car_Angle_Step_Down(void);
void Gyro_Car_PWM_Step_Up(void);
void Gyro_Car_PWM_Step_Down(void);
void Gyro_Car_PWM_Static_Up(void);
void Gyro_Car_PWM_Static_Down(void);

void Gyro_Car_PWM_FF(void);
void Gyro_Car_PWM_BK(void);
void Gyro_Car_PWM_Stop(void);
void Gyro_Car_PWM_Right(void);
void Gyro_Car_PWM_Left(void);
void Gyro_Car_PWM_Speed_Up(void);
void Gyro_Car_PWM_Speed_Down(void);

#endif

