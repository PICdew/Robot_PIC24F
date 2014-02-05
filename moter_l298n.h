//****************************************
// moter_l298n
// 2013.11.12
// Regis Hsu
//****************************************


#ifndef _Moter_L298N_
#define _Moter_L298N_

#define MOTOR_CCW   2
#define MOTOR_CW    1
#define MOTOR_STOP  0

#define MOTOR_L     0
#define MOTOR_R     1

int Motor_Dir_Get(int motor_id);
void Motor_Dir_Set(int motor_id, int motor_dir);
void Motor_L298N_Init(void);

void Motor_L298N_PWM_Init(int);
int Motor_L298N_PWN_Period(void);
void Motor_PWM_Dir_Set(int motor_id, int motor_dir, int duty);
void Motor_PWM_Dir_Set_Raw(int motor_id, int motor_dir, int PWM);

#endif
