//****************************************
// pwm_func.c
// for PWM function
// 2013.11.12
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

#include "moter_l298n.h"



/* PWM example
  ** Assign OC1 to be output on RF5 //RF13
  **
 *  PWM Period = [(PR2) + 1] * 2 * TOSC * (TMR2 Prescale Value)
  */
  #define  PWM_PERIOD 62500
  void
  PwmInit(
      void
     )
  {
      int i;
      //CLKDIV =  0; /* set for default clock operations Fcyc = 4MHz */
      //AD1PCFGL = 0xffff;
      //AD1PCFGH = 0x0003;

      /* Unmap RP31(RF13) */
      //RPOR15bits.RP31R = 0;
      //Unmap RP17(RF5)   
      RPOR8bits.RP17R = 0;

      /* Drive RF13 low and make it an output */
      //LATFbits.LATF13 = 0;
      //TRISFbits.TRISF13 = 0;
      //Nop();
     /* Drive RF5 low and make it an output */
      LATFbits.LATF5 = 0;
      TRISFbits.TRISF5 = 0;

      /* Reset PWM */
      //OC1CON1 = 0x0000;
      //OC1CON2 = 0x0000;

         // config timer2
    T3CON = 0x00; //Stops the Timer3 and reset control reg.
    TMR3 = 0x00; //Clear contents of the timer register
    PR3 = 0xFFFF; //Load the Period register with the value 0xFFFF

    /*
     *  bit 5-4 TCKPS<1:0>: Timerx Input Clock Prescale Select bits
     *  11 = 1:256 prescale value
     *  10 = 1:64 prescale value
     *  01 = 1:8 prescale value
     *  00 = 1:1 prescale value
     * set timer3 as 1:8 Prescare value, Fcy=8MHz/2=4MHz=0.25us, 1:64= 0.25*64 = 16us
     *
     *
     */

     T3CONbits.TCKPS = 0x0;

       /* configure PWM */
      OC1CON2 = 0x001F;   /* Sync with This OC module                               */
      OC1CON1 = 0x1C08;   /* Clock sourc Fcyc, trigger mode 1, Mode 0 (disable OC1) */

      /* enable the PWM */
      OC1CON1 = OC1CON1 | 0x0006;   /* Mode 6, Edge-aligned PWM Mode */

      /* Make pin RP31(RF13) OC1 (PWM output) */
      RPOR8bits.RP17R = 18;

      T3CONbits.TON = 1; //Start Timer3
      for (i=0; i< 1249; i += 2 )
      {
          int j;
          /* set PWM duty cycle to 50% */
          OC1R    = i;
          OC1RS   = 1249;  /* set the period */

          for (j =0; j<30000; j++)
              Nop();

      }
      
      //while(1);
  }

#if 0
  int
  main(
      void
      )
  {

      CLKDIV =  0; /* set for default clock operations Fcyc = 4MHz */
      AD1PCFGL = 0xffff;
      AD1PCFGH = 0x0003;

      PwmInit();
      for( ; ; );     /* hang here forever */
      return 0;
  }
#endif



#if 0

  /*
  What is Pulse Width Modulation (PWM)?
- is a technique for controlling power to intertial electrical devices, made practically by modern electronic power switches.

Setup for Continuous Output Pulse Generation
When the OCM control bits (OCxCON<2:0>) are set to ‘101’, the selected output compare channel initializes the OCx pin to the low state and generates output pulses on each and every compare match event. For the user to configure the module for the generation of a continuous stream of output pulses, the following steps are required (these steps assume the timer source is initially turned off, but this is not a requirement for the module operation):


Determine the instruction clock cycle time. Take into account the frequency of the external clock to the timer source (if one is used) and the timer prescaler settings.
Calculate time to the rising edge of the output pulse relative to the TMRy start value (0000h).
Calculate the time to the falling edge of the pulse, based on the desired pulse width and the time to the rising edge of the pulse.
Write the values computed in step 2 and 3 above into the Compare register, OCxR, and the Secondary Compare register, OCxRS, respectively.
Set Timer Period register, PRy, to value equal to or greater than value in OCxRS, the Secondary Compare register.
Set the OCM bits to ‘101’ and the OCTSEL bit to the desired timer source. The OCx pin state will now be driven low.
Enable the compare time base by setting the TON (TyCON<15>) bit to ‘1’.
Upon the first match between TMRy and OCxR, the OCx pin will be driven high.
When the compare time base, TMRy, matches the Secondary Compare register, OCxRS, the second and trailing edge (high-to-low) of the pulse is driven onto the OCx pin.
As a result of the second compare match event, the OCxIF interrupt flag bit set.
When the compare time base and the value in its respective Period register match, the TMRy register resets to 0x0000 and resumes counting.
Steps 8 through 11 are repeated and a continuous stream of pulses is generated, indefinitely. The OCxIF flag is set on each OCxRS-TMRy compare match event.

NOTE: In changing the duty cycle, all you have to do is to change the value of OCxRS.

*additional Info*
Here are some steps on finding the right value for PRy.
a. PWM Period = [(PRy)+1]*TCY*(TIMER PRESCALE VALUE)
where: PWM Frequency = 1/[PWM Period]
TCY = TOSC * 2
TOSC = 1/FOSC (TABLE 26-14:EXTERNAL CLOCK TIMING REQUIREMENTS)
FOSC = [OSCILLATOR FREQUENCY]

b. PRy = [[PWM Period]/[(Desired PWM Frequency) * (TCY) * (TIMER PRESCALE VALUE)]] - 1

*/
Sample Code
**********************************************************************************  **INITIALIZE**
 /* Reset PWM */
 OC2CON = 0x0000;
 /* set PWM duty cycle to 50% */
 OC2R = 0x63F  /2; /* set the duty cycle to 50% -> Initial*/
 if(initStatus==TRUE)
 OC2RS = 0x63F  /2; /* set the period */
 else
 OC2RS = 0;
 PR2 = 0x63F ;
 /* enable the PWM */
 OC2CON = OC2CON | 0x0005; /* Continous pulse mode*/
 T2CONbits.TON = 1; //Enables Timer

**Changng PWM Duty Cycle**
Changing the value of OC2RS changes the PWM duty cycle.

OC2RS = (OC2RS<(0x63F -2))?OC2RS + 1: 0x63F + 1; //Increment
OC2RS = (OC2RS>=1)?OC2RS - 1:0;  //decrement
#endif
