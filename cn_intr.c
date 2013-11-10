//****************************************
// CN_Intr
// MCU: PIC24F
// v0.1 2013.10.12
// v0.2 2013.11.19
// Regis Hsu
//
// For the interrupt routine of CHANGE NOTIFICATION (CN) PINS
// the CN interrupt is boundled with timer2
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
#include "sonar_hcsr04.h"
#include "cn_intr.h"

xQueueHandle xCN_INT_Queue;
static int _cn_func, _cn_ir_stage;
static xQueueHandle _cn_dst_queue;

static t_CN_INT_data intr_data;

/*
 * reset CN intr flag and function flag
 * #define CN_FUNC_IR_REMOTE       01
 * #define CN_FUNC_SONAR           02
 * #define CN_FUNC_NONE            00
 */
void CN_Function_Init(void)
{
    _cn_func = CN_FUNC_NONE;
    _CNIE = 0;         //disable the Change Notification general interrupt

    // config timer2
    T2CON = 0x00; //Stops the Timer2 and reset control reg.
    TMR2 = 0x00; //Clear contents of the timer register

    //xCN_INT_Queue = xQueueCreate(30, sizeof(t_CN_INT_data));

    //if (!xCN_INT_Queue)
    //    while (1);
}

/*
 * enable CN interrupt and start timer2 counter
 */
int CN_Function_Enable(xQueueHandle queue, int func, int timer_prescare)
{
    if (_cn_func != CN_FUNC_NONE)
        return 0;

    // config timer2
    T2CON = 0x00; //Stops the Timer2 and reset control reg.
    TMR2 = 0x00; //Clear contents of the timer register
    PR2 = 0xFFFF; //Load the Period register with the value 0xFFFF

    /*
     *  bit 5-4 TCKPS<1:0>: Timerx Input Clock Prescale Select bits
     *  11 = 1:256 prescale value
     *  10 = 1:64 prescale value
     *  01 = 1:8 prescale value
     *  00 = 1:1 prescale value
     */
    if (timer_prescare <=4 && timer_prescare >= 0)
    {
        T2CONbits.TCKPS = timer_prescare;
    }
    else
    {
         T2CONbits.TCKPS = 0x3; // 1:256 as default
    }
    //T2CONbits.TCKPS =0x01; // 1:8 Prescare value, Fcy=8MHz/2=4MHz=0.25us, 1:8= 0.25*8 = 2us
    //T2CONbits.TCKPS =0x00; // 1:1 Prescare value, Fcy=8MHz/2=4MHz=0.25us,

    _cn_func = func;
    _cn_dst_queue = queue;
    
    T2CONbits.TON = 1; //Start Timer2
    _CNIE = 1;         //enable the Change Notification general interrupt

    return 1;
}

/*
 * release CN interrupt and timer2 counter
 */
int CN_Function_Release(int i)
{
    if (_cn_func != i)
        return 0;

    _CNIE = 0;         //disable the Change Notification general interrupt
    // config timer2
    T2CON = 0x00; //Stops the Timer2 and reset control reg.
    TMR2 = 0x00; //Clear contents of the timer register
    _cn_func = CN_FUNC_NONE;
    return 1;

}

/*  Ensure that the CN pin is configured as a digital input by setting the associated bit in the
TRISx register.
2. Enable interrupts for the selected CN pins by setting the appropriate bits in the CNENx
registers.
3. Turn on the weak pull-up devices (if desired) for the selected CN pins by setting the
appropriate bits in the CNPUx registers.
4. Clear the CNxIF interrupt flag.
5. Select the desired interrupt priority for CN interrupts using the CNxIP<2:0> control bits.
6. Enable CN interrupts using the CNxIE control bit.
*/
//Interrupt Service Routine for Change Notification

int _CN_Timer2;

#if 0
//static void _ISR __attribute__((__auto_psv__)) _CNInterrupt (void)
void _ISRFAST __attribute__((__auto_psv__)) _CNInterrupt (void)
{
     _CNIF = 0;    //clear the change notification interrupt bit

     switch (_cn_func)
     {
         case CN_FUNC_IR_REMOTE:
            TMR2 = 0x00; //Clear contents of the timer register
            //xQueueSendToBackFromISR (xQ_IR_Remote, &_CN_Timer2, 0);
            break;

         case CN_FUNC_SONAR:
            if (_RD9) {
                Sonar_HCSR04_Count_up = TMR2 ;
            }
            else {
                Sonar_HCSR04_Count_down = TMR2;
            }
            break;

           default:
             _cn_func = CN_FUNC_NONE;
             break;
     }
}
#endif

void _ISRFAST __attribute__((__auto_psv__)) _CNInterrupt (void)
{
     _CNIF = 0;    //clear the change notification interrupt bit

     intr_data.intr_func = _cn_func;
     intr_data.timer_value = TMR2;

     if (_cn_dst_queue)
         xQueueSendToBackFromISR (_cn_dst_queue, &intr_data, 0);
}

#if 0
void vTask_CN_INTR(void *pvParameters )
{
    portBASE_TYPE xStatus;
    t_CN_INT_data data;

    (void)pvParameters;

    for( ;; )
    {
        xStatus = xQueueReceive(xCN_INT_Queue, &data, portMAX_DELAY);
	if (xStatus != pdPASS)
	{
            while(1){};
            /* The send operation could not complete because the queue was full -
            this must be an error as the queue should never contain more than
            one item! */
            //printf("xxxxxxxxxxxxx Could not send to the queue.\r\n");
	}

        // set the CN interrupt to serve Sonar and timer2 as 1:8 prescare
        // 1:8 Prescare value, Fcy=8MHz/2=4MHz=0.25us, 1:8= 0.25*8 = 2us
        CN_Function_Enable(CN_FUNC_SONAR, 0x01);

        // Trigger the sonar
        Sonar_HCSR04_Trigger();
        vTaskDelay(100);       //spec: 60ms for each detection

        // release CN interrupt and stop the timer2
        CN_Function_Release(CN_FUNC_SONAR);

        if (Sonar_HCSR04_Count_down >0 && Sonar_HCSR04_Count_up >0) {
            distance = Sonar_HCSR04_Count_down - Sonar_HCSR04_Count_up;
            distance = distance / (116) ;   // 116 = 58*2, timer is 2us
                                            // 為什麼除以58等於厘米，  Y米=（X秒*344）/2
                                            // X秒=（ 2*Y米）/344 ==》X秒=0.0058*Y米 ==》厘米=微秒/58
            LCD_Show_String(0, 0, "D:", 2);
            LCD_Show_Value(2, 0, distance, 10, 5);
        }
        Sonar_HCSR04_Count_up = Sonar_HCSR04_Count_down = 0;
    }
}
#endif
