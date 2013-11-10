//****************************************
// Sonar_HCSR04
// 2013.06.23
// Regis Hsu
//****************************************

#ifndef _Sonar_HCSR04_
#define _Sonar_HCSR04_

#define Sonar_HCSR04_TRIG	15

void Sonar_HCSR04_Init(void);
int Sonar_HCSR04_DetectRange(void);
void Sonar_HCSR04_Test(void);
void vTask_Sonar_HCSR04(void *pvParameters );

extern xQueueHandle xSonar_Queue;
extern int Sonar_HCSR04_Count_up, Sonar_HCSR04_Count_down;
#endif
