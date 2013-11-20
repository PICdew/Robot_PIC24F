//****************************************
// pwm_func.h
// 2013.11.19
// Regis Hsu
//****************************************


#ifndef _PWM_FUNC_
#define _PWM_FUNC_

#include <GenericTypeDefs.h>

void POT_Init(void);
BYTE POT_Read(void);
void PwmInit(void);
void Servo_SG90_Rotation(int d);
void Servo_SG90_Move(int sw);
void Servo_SG90_Init(void);


#endif
