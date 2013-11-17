//****************************************
// Sonar_HCSR04
// MCU: PIC24F
// 2013.06.23
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
#include "sonar_hcsr04.h"
#include "cn_intr.h"

int Sonar_HCSR04_Count_up, Sonar_HCSR04_Count_down;

#if 0
 //Interrupt Service Routine for Change Notification
 //void _ISRFAST _CNInterrupt (void)
void _ISRFAST __attribute__((__auto_psv__)) _CNInterrupt (void)
{
     _CNIF = 0;    //clear the change notification interrupt bit
     if (_RD9) {
         Sonar_HCSR04_Count_up = TMR2 ;
     }
     else {
         Sonar_HCSR04_Count_down = TMR2;
     }

 }
#endif

xQueueHandle xSonar_Queue;

/*
 * connect RD8 to the trigger pin of HCSR04
 * connect RD9 to the echo pin of HCSR04, set it as interrupt CN54
 * must set open-drain feature because we use 3.3v to drive 5v module.
 * Timer2: 1:8 Prescare value, Fcy=8MHz/2=4MHz=0.25us, 1:8= 0.25*8 = 2us
 * Regis
 */
void Sonar_HCSR04_Init(void)
{

    TRISDbits.TRISD8 = 0;       // output of RD8
    ODCDbits.ODD8 = 1;          // turn-on the Open-Drain feature to control 5v output by pull-high-R
    TRISDbits.TRISD9 = 1;       // input of RD9
    ODCDbits.ODD9 = 1;          // turn-on the Open-Drain feature to control 5v output by pull-high-R

    // cleart trigger pin
    _LATD8 = 0;

    //Sonar_HCSR04_Count_up = Sonar_HCSR04_Count_up = 0;

    //xSonar_Queue = xQueueCreate(4, sizeof(t_CN_INT_data));
    
}


/*
 * send a 10us pulse to the Soner device
 * set timer2 as 1:8 Prescare value, Fcy=8MHz/2=4MHz=0.25us, 1:8= 0.25*8 = 2us
 * enable CN interrupt to get the echo to calculate the distance
 * 為什麼Timer2除以58等於厘米，  Y(m)=（X(s)*344）/2, X(s)=（ 2*Y(m)）/344 ==》X(s)=0.0058*Y(m) ==》Y(cm) = X(us)/58

 */
static void Sonar_HCSR04_Trigger(void)
{
    int i;
    UINT16 count;

#if 0
    // config timer3
    T3CON = 0x00; //Stops the Timer3 and reset control reg.
    TMR3 = 0x00; //Clear contents of the timer register
    PR3 = 0xFFFF; //Load the Period register with the value 0xFFFF
     /*
     *  bit 5-4 TCKPS<1:0>: Timerx Input Clock Prescale Select bits
     *  11 = 1:256 prescale value, 16us
     *  10 = 1:64 prescale value, 4us
     *  01 = 1:8 prescale value, 0.5us
     *  00 = 1:1 prescale value, 0.0625us,
     *   Fcy=16MHz=0.0625us
      *
      */
    T3CONbits.TCKPS = 2; // 1:64, 0.0625 x 64 = 4us
     
     // generate a 10us pulse to trigger the Soner device

    _LATD8 = 1;
    T3CONbits.TON = 1; //Start Timer3
    while (TMR3 < 3)
    {
        Nop();
    }
    _LATD8 = 0;
    T3CONbits.TON = 0; //Stop Timer3

#endif
     // generate a 10us pulse to trigger the Soner device
    _LATD8 = 1;
    delay_us(10);
    _LATD8 = 0;

   
}

/*
 * enable or disable CN54 interrupt feature
 */
static void Sonar_HCSR04_Intr(int sw)
{
    /** Configure Change Notification general interrupt  */
    _CN54IE = sw;        // Enable CN54 interrupt (RP4/DPLN/CN54/RD9)
    _CN54PUE = sw;       // pull-up enable
    _CN54PDE = sw;       // pull-down enable
    _CNIF = 0;         //Clear the interrupt flag
    _CNIP = 2;         //Choose the priority 2
}

/*
 * the detect cycle time should be more then 60ms
 */
int Sonar_HCSR04_Get_Distance(void)
{
        int distance, i, n;
        unsigned int timer_last;
        portBASE_TYPE xStatus;
        t_CN_INT_data intr_data;

        distance = 0;
        timer_last = 0;

        Sonar_HCSR04_Init();

        // Trigger the sonar
        Sonar_HCSR04_Trigger();
        // enable interrupt 
        Sonar_HCSR04_Intr(1);
        // set the CN interrupt to serve Sonar and timer2 as 1:8 prescare
        // 11 = 1:256 prescale value, 16us
        // 10 = 1:64 prescale value, 4us
        // 01 = 1:8 prescale value, 0.5us
        // 00 = 1:1 prescale value, 0.0625us,
        // Fcy=16MHz=0.0625us

        // 1:8 Prescare value, Fcy=8MHz/2=4MHz=0.25us, 1:8= 0.25*8 = 2us
        //CN_Function_Enable(xSonar_Queue, CN_FUNC_SONAR, 0x01); //1:8
        CN_Function_Enable(xSonar_Queue, CN_FUNC_SONAR, 0x02); //1:64

        // for each detection, 400cm is the max, that is 400*58(us/cm) = 23200us, so,
        // I set 60ms to wait the signal should be enough.
        vTaskDelay(60);
        // release CN interrupt and stop the timer2
        CN_Function_Release(CN_FUNC_SONAR);
        Sonar_HCSR04_Intr(0);

        // the CN interrupt mey generate lest then 2 singers, so we need drop it.
        n = uxQueueMessagesWaiting(xSonar_Queue);
        if (n > 2)
            LED_ERR(0x0A);
        for (i =0; i< n; i++)
        {
            xStatus = xQueueReceive(xSonar_Queue, &intr_data, 0);
            if (xStatus != pdPASS)
            {
                LED_ERR(0x05);
                /* The send operation could not complete because the queue was full -
                 this must be an error as the queue should never contain more than
                 one item! */
               //printf("xxxxxxxxxxxxx Could not send to the queue.\r\n");
            }
            distance = intr_data.timer_value - timer_last;
            timer_last =  intr_data.timer_value;
        }
        if (distance > 0)
            distance = distance / 14;
            //Y米=（X秒*340）/2 //音速=340.29m/s 發出+反彈/2
            // 1:64 = 4us, d = n(4us) / 58 = n/14.5;

        return distance;

}


void vTask_Sonar_HCSR04(void *pvParameters )
{
    portTickType xLastWakeTime;
    int distance;
    (void)pvParameters; // prevent compiler worning/error
    
    distance = 0;
    xLastWakeTime = xTaskGetTickCount();
    for( ;; )
    {
        vTaskDelayUntil(&xLastWakeTime,1000);

        distance = Sonar_HCSR04_Get_Distance();
        LCD_Show_String(0, 0, "D:", 2);
        LCD_Show_Value(2, 0, distance, 10, 6);
    }
}
