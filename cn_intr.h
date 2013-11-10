//****************************************
// CN_INTR
// 2013.10.12
// Regis Hsu
//****************************************

#ifndef _CN_INTR_
#define _CN_INTR_

#define CN_FUNC_IR_REMOTE       01
#define CN_FUNC_SONAR           02
#define CN_FUNC_NONE            00

extern xQueueHandle xCN_INT_Queue;

typedef struct {
    int intr_func;
    unsigned int timer_value;
} t_CN_INT_data;

int CN_Function_Enable(xQueueHandle queue, int func, int timer_prescare);
int CN_Function_Release(int i);
void CN_Function_Init(void);
#endif
