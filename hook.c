/*
 *  Hook.c
 *  manange extern hook functions
 */

/* Standard includes. */
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"
#include "led.h"

// C30 and C32 Exception Handlers
// If your code gets here, you either tried to read or write
// a NULL pointer, or your application overflowed the stack
// by having too many local variables or parameters declared.
#if defined(__C30__) || defined __XC16__
	void _ISR __attribute__((__no_auto_psv__)) _AddressError(void)
	{
            LED6_On();
            //while(1){}
	}
	void _ISR __attribute__((__no_auto_psv__)) _StackError(void)
	{
            while(1){}
	}

#elif defined(__C32__)
	void _general_exception_handler(unsigned cause, unsigned status)
	{
        #if defined(DEBUG_MODE)
            unsigned long address = _CP0_GET_EPC();
        #endif

        while(1){}
	}
#endif

void vApplicationIdleHook( void )
{
	/* Schedule the co-routines from within the idle task hook. */
	vCoRoutineSchedule();
}

/*-----------------------------------------------------------*/

#if (configUSE_MALLOC_FAILED_HOOK == 1)
void vApplicationMallocFailedHook( void )
{
    LED7_On();
    while(1){};

}
#endif

