
#ifndef _Balance_Car_
#define _Balance_Car_

#define CMD_DEBUG_1         91
#define CMD_DEBUG_2         92
#define CMD_DEBUG_3         93
#define CMD_DEBUG_4         94
#define CMD_DEBUG_5         95

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

#define CMD_MOTOR           30
#define CMD_CAR_FF          31
#define CMD_CAR_BK          32
#define CMD_CAR_STOP        33
#define CMD_CAR_RIGHT       34
#define CMD_CAR_LEFT        35
#define CMD_CAR_SPEED_U     36
#define CMD_CAR_SPEED_D     37

#define CMD_ACCEL_X_OFFSET_U    41
#define CMD_ACCEL_X_OFFSET_D    42
#define CMD_GYRO_Y_OFFSET_U     43
#define CMD_GYRO_Y_OFFSET_D     44

void vTask_Balance_Car_1(void *pvParameters );

#endif

