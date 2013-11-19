/*
    FreeRTOS V7.5.3 - Copyright (C) 2013 Real Time Engineers Ltd. 
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that has become a de facto standard.             *
     *                                                                       *
     *    Help yourself get started quickly and support the FreeRTOS         *
     *    project by purchasing a FreeRTOS tutorial book, reference          *
     *    manual, or both from: http://www.FreeRTOS.org/Documentation        *
     *                                                                       *
     *    Thank you!                                                         *
     *                                                                       *
    ***************************************************************************

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>!AND MODIFIED BY!<< the FreeRTOS exception.

    >>! NOTE: The modification to the GPL is included to allow you to distribute
    >>! a combined work that includes FreeRTOS without being obliged to provide
    >>! the source code for proprietary components outside of the FreeRTOS
    >>! kernel.

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available from the following
    link: http://www.freertos.org/a00114.html

    1 tab == 4 spaces!

    ***************************************************************************
     *                                                                       *
     *    Having a problem?  Start by reading the FAQ "My application does   *
     *    not run, what could be wrong?"                                     *
     *                                                                       *
     *    http://www.FreeRTOS.org/FAQHelp.html                               *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org - Documentation, books, training, latest versions,
    license and Real Time Engineers Ltd. contact details.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.OpenRTOS.com - Real Time Engineers ltd license FreeRTOS to High
    Integrity Systems to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the standard demo application tasks.
 * In addition to the standard demo tasks, the following tasks and tests are
 * defined and/or created within this file:
 *
 * "Fast Interrupt Test" - A high frequency periodic interrupt is generated
 * using a free running timer to demonstrate the use of the 
 * configKERNEL_INTERRUPT_PRIORITY configuration constant.  The interrupt 
 * service routine measures the number of processor clocks that occur between
 * each interrupt - and in so doing measures the jitter in the interrupt 
 * timing.  The maximum measured jitter time is latched in the usMaxJitter 
 * variable, and displayed on the LCD by the 'Check' as described below.  
 * The fast interrupt is configured and handled in the timer_test.c source 
 * file.
 *
 * "LCD" task - the LCD task is a 'gatekeeper' task.  It is the only task that
 * is permitted to access the LCD directly.  Other tasks wishing to write a
 * message to the LCD send the message on a queue to the LCD task instead of 
 * accessing the LCD themselves.  The LCD task just blocks on the queue waiting 
 * for messages - waking and displaying the messages as they arrive.  The LCD
 * task is defined in lcd.c.  
 * 
 * "Check" task -  This only executes every three seconds but has the highest 
 * priority so is guaranteed to get processor time.  Its main function is to 
 * check that all the standard demo tasks are still operational.  Should any
 * unexpected behaviour within a demo task be discovered the 'check' task will
 * write "FAIL #n" to the LCD (via the LCD task).  If all the demo tasks are 
 * executing with their expected behaviour then the check task writes the max
 * jitter time to the LCD (again via the LCD task), as described above.
 */

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
#include "sonar_hcsr04.h"
#include "cn_intr.h"

#include "moter_l298n.h"
#include "pwm_func.h"

// Configuration bits for the device.  Please refer to the device datasheet for each device
//   to determine the correct configuration bit settings
#if defined __C30__ || defined __XC16__
    #if defined(__PIC24FJ256GB110__)
        _CONFIG2(FNOSC_PRIPLL & POSCMOD_HS & PLL_96MHZ_ON & PLLDIV_DIV2) // Primary HS OSC with PLL, USBPLL /2
        _CONFIG1(JTAGEN_OFF & FWDTEN_OFF & ICS_PGx2)   // JTAG off, watchdog timer off
    #elif defined(__PIC24FJ64GB004__)
        _CONFIG1(WDTPS_PS1 & FWPSA_PR32 & WINDIS_OFF & FWDTEN_OFF & ICS_PGx1 & GWRP_OFF & GCP_OFF & JTAGEN_OFF)
        _CONFIG2(POSCMOD_HS & I2C1SEL_PRI & IOL1WAY_OFF & OSCIOFNC_ON & FCKSM_CSDCMD & FNOSC_PRIPLL & PLL96MHZ_ON & PLLDIV_DIV2 & IESO_ON)
        _CONFIG3(WPFP_WPFP0 & SOSCSEL_SOSC & WUTSEL_LEG & WPDIS_WPDIS & WPCFG_WPCFGDIS & WPEND_WPENDMEM)
        _CONFIG4(DSWDTPS_DSWDTPS3 & DSWDTOSC_LPRC & RTCOSC_SOSC & DSBOREN_OFF & DSWDTEN_OFF)
    #elif defined(__PIC24FJ64GB502__)
        _CONFIG1(WDTPS_PS1 & FWPSA_PR32 & WINDIS_OFF & FWDTEN_OFF & ICS_PGx1 & GWRP_OFF & GCP_OFF & JTAGEN_OFF)
        _CONFIG2(I2C1SEL_PRI & IOL1WAY_OFF & FCKSM_CSDCMD & FNOSC_PRIPLL & PLL96MHZ_ON & PLLDIV_DIV2 & IESO_OFF)
        _CONFIG3(WPFP_WPFP0 & SOSCSEL_SOSC & WUTSEL_LEG & WPDIS_WPDIS & WPCFG_WPCFGDIS & WPEND_WPENDMEM)
        _CONFIG4(DSWDTPS_DSWDTPS3 & DSWDTOSC_LPRC & RTCOSC_SOSC & DSBOREN_OFF & DSWDTEN_OFF)
    #elif defined(__PIC24FJ256GB106__)
        _CONFIG1( JTAGEN_OFF & GCP_OFF & GWRP_OFF & COE_OFF & FWDTEN_OFF & ICS_PGx2)
        _CONFIG2( 0xF7FF & IESO_OFF & FCKSM_CSDCMD & OSCIOFNC_OFF & POSCMOD_HS & FNOSC_PRIPLL & PLLDIV_DIV3 & IOL1WAY_ON)
    #elif defined(__PIC24FJ256DA210__) || defined(__PIC24FJ256GB210__)
        _CONFIG1(FWDTEN_OFF & ICS_PGx2 & GWRP_OFF & GCP_OFF & JTAGEN_OFF)
        _CONFIG2(POSCMOD_HS & IOL1WAY_ON & OSCIOFNC_ON & FCKSM_CSDCMD & FNOSC_PRIPLL & PLL96MHZ_ON & PLLDIV_DIV2 & IESO_OFF)
    #elif defined(IOIO)
        _CONFIG1(FWDTEN_OFF & ICS_PGx2 & GWRP_OFF & GCP_OFF & JTAGEN_OFF)
        _CONFIG2(POSCMOD_NONE & IOL1WAY_ON & OSCIOFNC_ON & FCKSM_CSDCMD & FNOSC_FRCPLL & PLL96MHZ_ON & PLLDIV_NODIV & IESO_OFF)
    #elif defined(__dsPIC33EP512MU810__) || defined(PIC24EP512GU810_PIM)
        _FOSCSEL(FNOSC_FRC);
        _FOSC(FCKSM_CSECMD & OSCIOFNC_OFF & POSCMD_XT);
        _FWDT(FWDTEN_OFF);
    #endif
#elif defined( __PIC32MX__ )
    #pragma config UPLLEN   = ON            // USB PLL Enabled
    #pragma config FPLLMUL  = MUL_15        // PLL Multiplier
    #pragma config UPLLIDIV = DIV_2         // USB PLL Input Divider
    #pragma config FPLLIDIV = DIV_2         // PLL Input Divider
    #pragma config FPLLODIV = DIV_1         // PLL Output Divider
    #pragma config FPBDIV   = DIV_1         // Peripheral Clock divisor
    #pragma config FWDTEN   = OFF           // Watchdog Timer
    #pragma config WDTPS    = PS1           // Watchdog Timer Postscale
    //#pragma config FCKSM    = CSDCMD        // Clock Switching & Fail Safe Clock Monitor
    #pragma config OSCIOFNC = OFF           // CLKO Enable
    #pragma config POSCMOD  = HS            // Primary Oscillator
    #pragma config IESO     = OFF           // Internal/External Switch-over
    #pragma config FSOSCEN  = OFF           // Secondary Oscillator Enable (KLO was off)
    #pragma config FNOSC    = PRIPLL        // Oscillator Selection
    #pragma config CP       = OFF           // Code Protect
    #pragma config BWP      = OFF           // Boot Flash Write Protect
    #pragma config PWP      = OFF           // Program Flash Write Protect
    #pragma config ICESEL   = ICS_PGx2      // ICE/ICD Comm Channel Select
    //#pragma config DEBUG    = ON            // Background Debugger Enable

#else
    #error Cannot define configuration bits.
#endif

/*-----------------------------------------------------------*/

/*
 * Setup the processor ready for the demo.
 */
static void prvSetupHardware( void );

/*-----------------------------------------------------------*/

/* The queue used to send messages to the LCD task. */
static xQueueHandle xLCDQueue;

/*-----------------------------------------------------------*/
 
static void vTask_test1 (void *pvParameters );
static void vTask_test2 (void *pvParameters );
static void vTask_test3 (void *pvParameters );
static void vTask_test4 (void *pvParameters );
static void vTask_test5 (void *pvParameters );
static void vTask_test6 (void *pvParameters );
static void vTask_test7 (void *pvParameters );



/****************************************************************************/

xQueueHandle xTest_Queue;

#define vTask_STACK_SIZE    (configMINIMAL_STACK_SIZE * 5)
/*
 * Create the demo tasks then start the scheduler.
 */
int main( void )
{
    InitAllLEDs();
    InitI2C();
    // config CN interrupt and reset the register.
    CN_Function_Init();
    Lcd_1602_init(0x4E, 16, 2, LCD_5x8DOTS);  // set the LCD address to 0x27 for a 16 chars and 2 line display

    PwmInit();
    
    xTest_Queue = xQueueCreate(20, sizeof(int));
    xLCD_Queue = xQueueCreate(50, sizeof(t_LCD_data));
    xSonar_Queue = xQueueCreate(20, sizeof(t_CN_INT_data));

    //PwmInit();
    xTaskCreate( vTask_test1, ( signed char * )"T1", vTask_STACK_SIZE, NULL, 2, NULL );
    xTaskCreate( vTask_test2, ( signed char * )"T2", vTask_STACK_SIZE, NULL, 2, NULL );
    //xTaskCreate( vTask_test3, ( signed char * )"T3", vTask_STACK_SIZE, NULL, 2, NULL );
    //xTaskCreate( vTask_test4, ( signed char * )"T4", vTask_STACK_SIZE, NULL, 4, NULL );
    //xTaskCreate( vTask_test5, ( signed char * )"T5", vTask_STACK_SIZE, NULL, 2, NULL );
    //xTaskCreate( vTask_test6, ( signed char * )"T6", vTask_STACK_SIZE, NULL, 2, NULL );
    xTaskCreate( vTask_test7, ( signed char * )"T6", vTask_STACK_SIZE, NULL, 2, NULL );
    xTaskCreate( vTask_LCD, ( signed char * )"LD", vTask_STACK_SIZE, NULL, 5, NULL );
    xTaskCreate( vTask_Gyro_MPU6050, ( signed char * )"GY", vTask_STACK_SIZE, NULL, 2, NULL );
    //xTaskCreate( vTask_Sonar_HCSR04, ( signed char * )"SO", vTask_STACK_SIZE, NULL, 3, NULL );

    /* Start the high frequency interrupt test. */
    //vSetupTimerTest( mainTEST_INTERRUPT_FREQUENCY );

    /* Finally start the scheduler. */
    vTaskStartScheduler();

    /* Will only reach here if there is insufficient heap available to start
    the scheduler. */
    return 0;
}

static void vTask_test6(void *pvParameters)
{
	unsigned char a;
	portTickType xLastWakeTime;
	(void)pvParameters;

	a = 0;
        Motor_L298N_Init();
	xLastWakeTime = xTaskGetTickCount();
	for (;;)
	{
            vTaskDelayUntil(&xLastWakeTime, 1000);
            switch (a)
            {
                case 0:
                    Motor_Dir_Set(MOTOR_L, MOTOR_CW);
                    Motor_Dir_Set(MOTOR_R, MOTOR_CW);
                    break;
                case 1:
                    Motor_Dir_Set(MOTOR_L, MOTOR_CCW);
                    Motor_Dir_Set(MOTOR_R, MOTOR_CCW);
                    break;
                case 2:
                    Motor_Dir_Set(MOTOR_L, MOTOR_CCW);
                    Motor_Dir_Set(MOTOR_R, MOTOR_CW);
                    break;
                case 3:
                    Motor_Dir_Set(MOTOR_L, MOTOR_CW);
                    Motor_Dir_Set(MOTOR_R, MOTOR_CCW);
                    break;
            }
            a++;
            if (a > 3)
                a = 0;

        }
}

static void vTask_test7(void *pvParameters)
{
	unsigned char a;
	portTickType xLastWakeTime;
        int distance, n, m;
	(void)pvParameters;

	a = 0;
        n = m = 0;
        Motor_L298N_Init();
	xLastWakeTime = xTaskGetTickCount();
	for (;;)
	{
            //vTaskDelayUntil(&xLastWakeTime, 600);
            distance = Sonar_HCSR04_Get_Distance();
            LCD_Show_Value(2, 0, distance, 10, 6);
            if (distance < 8)
            {
                Motor_Dir_Set(MOTOR_L, MOTOR_STOP);
                Motor_Dir_Set(MOTOR_R, MOTOR_STOP);
                vTaskDelay(2000);
                Motor_Dir_Set(MOTOR_L, MOTOR_CCW);
                Motor_Dir_Set(MOTOR_R, MOTOR_CCW);
                vTaskDelay(200);
                Motor_Dir_Set(MOTOR_L, MOTOR_STOP);
                Motor_Dir_Set(MOTOR_R, MOTOR_STOP);
                vTaskDelay(3000);
                if (a)
                {
                    Motor_Dir_Set(MOTOR_L, MOTOR_CCW);
                    Motor_Dir_Set(MOTOR_R, MOTOR_CW);
                }
                else
                {
                    Motor_Dir_Set(MOTOR_L, MOTOR_CW);
                    Motor_Dir_Set(MOTOR_R, MOTOR_CCW);
                }
                vTaskDelay(150);
                Motor_Dir_Set(MOTOR_L, MOTOR_STOP);
                Motor_Dir_Set(MOTOR_R, MOTOR_STOP);
            }
            else
            {
                Motor_Dir_Set(MOTOR_L, MOTOR_CW);
                Motor_Dir_Set(MOTOR_R, MOTOR_CW);
            }
            a = 1 - a;
        }
}

static void vTask_test3(void *pvParameters)
{
	unsigned char a;
	portTickType xLastWakeTime;
	int value[10] = { 100, 101, 102, 103, 104, 105, 106, 107, 108, 109 };
	portBASE_TYPE xStatus;

        t_LCD_data lcd_data;

	(void)pvParameters;

	a = 0;
	xLastWakeTime = xTaskGetTickCount();
	for (;;)
	{
		vTaskDelayUntil(&xLastWakeTime, 500);

                //xStatus = uxQueueMessagesWaiting(xQueue);
		//xStatus = uxQueueMessagesWaiting(xLCD_Queue);
                //Lcd_1602_display_dec(1, 1, (int)a);

                //for (a = 0; a < 10; a++)
		{
                    lcd_data.x = 0;
                    lcd_data.y = 1;
                    lcd_data.value = value[a];
                    lcd_data.base = 10; //dec
                    lcd_data.lengh = 4; //dec

                    xStatus = xQueueSendToBack(xLCD_Queue, &lcd_data, 0);
                    if (xStatus != pdPASS)
                    {
                            while(1){};
				/* The send operation could not complete because the queue was full -
				this must be an error as the queue should never contain more than
				one item! */
				//printf("xxxxxxxxxxxxx Could not send to the queue.\r\n");
                    }
                    //Lcd_1602_display_dec(1, 1, value[a]);
                    //printf("T3 =>Q [%d]\n", value[a]);
		}
                if (a < 9)
                    a++;
                else
                    a = 0;

	}
}

static void vTask_test5(void *pvParameters)
{
	unsigned char a;
	portTickType xLastWakeTime;
	int value[10] = { 10, 11, 12, 13, 14, 15, 16, 17, 18, 19 };

	(void)pvParameters;

	a = 0;
	xLastWakeTime = xTaskGetTickCount();
	for (;;)
	{
		vTaskDelayUntil(&xLastWakeTime, 300);
                //xStatus = uxQueueMessagesWaiting(xLCD_Queue);
		//xStatus = uxQueueMessagesWaiting(xQueue);
		//Lcd_1602_display_dec(5, 1, 33);

                LCD_Show_Value(5, 1, value[a], 10, 4);

                if (a < 9)
                    a++;
                else
                    a = 0;
	}
}


static void vTask_test4(void *pvParameters)
{
	unsigned char a;
	portTickType xLastWakeTime;
	int value;
	portBASE_TYPE xStatus;

	(void)pvParameters;

	a = 0;
	xLastWakeTime = xTaskGetTickCount();
	for (;;)
	{
		xStatus = xQueueReceive(xTest_Queue, &value, portMAX_DELAY);
		if (xStatus != pdPASS)
		{
                    while(1){};
			/* The send operation could not complete because the queue was full -
			this must be an error as the queue should never contain more than
			one item! */
			//printf("xxxxxxxxxxxxx Could not get from the queue.\r\n");
		}
		//printf("T4 <=Q (%d)\n", value);
	}
}

/*-----------------------------------------------------------*/
static void vTask_test1 (void *pvParameters )
{
    unsigned char  a;
    portTickType xLastWakeTime;

    // this is test  git
    a = 0;
    xLastWakeTime = xTaskGetTickCount();
    for( ;; )
    {
        vTaskDelayUntil( &xLastWakeTime, 1000);
        // vTaskDelay( 2000 );
        if (a != 0) {LED0_On();} else {LED0_Off();};
        a = 1-a;
    }
}

static void vTask_test2 (void *pvParameters )
{
    unsigned char  a;
   portTickType xLastWakeTime;

    a = 0;
    xLastWakeTime = xTaskGetTickCount();
    for( ;; )
    {
        vTaskDelayUntil( &xLastWakeTime, 500);
        //vTaskDelay( 500 );

        if (a != 0) {LED1_On();} else {LED1_Off();};
        a = 2-a;
    }
}

static void prvSetupHardware( void )
{
	//vParTestInitialise();
}
/*-----------------------------------------------------------*/
