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
#include "pwm_func.h"


/*
 * init the pin to ADC convert
 * POT 10Kohm at pin of RP9/AN9/CN27/RB9
 */
void POT_Init(void)
{
    AD1PCFGLbits.PCFG9 = 0;
    AD1CON2bits.VCFG = 0x0;
    AD1CON3bits.ADCS = 0xFF;
    AD1CON1bits.SSRC = 0x0;
    AD1CON3bits.SAMC = 0b10000;
    AD1CON1bits.FORM = 0b00;
    AD1CON2bits.SMPI = 0x0;
    AD1CON1bits.ADON = 1;
}

/* PWM example
 ** Assign OC1 to be output on RF5-J7(4)
 **
 *  PWM Period = [(PR2) + 1] * 2 * TOSC * (TMR2 Prescale Value)
 */
void PwmInit(void)
{
    //int i;
    POT_Init();

    //Make the pin of RP17(RF5) as normal function
    RPOR8bits.RP17R = 0;
    // Drive RF5 low and make it an output
    LATFbits.LATF5 = 0;
    TRISFbits.TRISF5 = 0;

    // Reset PWM
    OC1CON1 = 0x0000;
    OC1CON2 = 0x0000;

    // configure PWM
    OC1CON2 = 0x001F;   /* Sync with This OC module                               */
    OC1CON1 = 0x1C08;   /* Clock sourc Fcyc(32Mhz, 0.0625us), trigger mode 1, Mode 0 (disable OC1) */
    /* enable the PWM */
    OC1CON1 = OC1CON1 | 0x0006;   /* Mode 6, Edge-aligned PWM Mode */

    // Make the pin of RP17(RF5) as OC1 (PWM output)
    RPOR8bits.RP17R = 18;

    // set the PWM period, 20ms = 320000 * 0.0625us = 20ms
    OC1RS   = 0xFFFF;
    for (;; )
    {
        unsigned int j;
        j = POT_Read();
        if (j>0)
            j = 0xFFFF / j;
        // set PWM duty cycle
        OC1R = j;
        delay_ms(200);
    }

    //while(1);
}

#if 0
/*
Find the Period register value for a desired PWM frequency of 2 kHz,
 where Fosc = 32 MHz  (32 MHz device clock rate) and
 a Timer3 prescaler setting of 1:1.
 Tcy = 2/Fosc = 62.5 ns
 PWM Period   =  1/PWM Frequency = 1/2 kHz = 500 μs
 PWM Period   = (PR2 + 1) * Tcy * (Timer 3 Prescale Value)  *PWM周期计算公式
 500 μs = (PR2 + 1) * 62.5 ns * 1
 PR2 = 800-1
*/

void BZOUT(int Period,int rate)
{
OC1CON1 = 0;//It is a good practice to clear off the control bits initially
OC1CON2 = 0;

OC1CON2.SYNCSEL = 13;
OC1CON2.OCTRIG = 1;
OC1CON1BITS.OCSIDL = 0;// Output capture will continue to operate in CPU Idle mode
OC1CON1BITS.OCTSEL = 1; //This selects the peripheral clock as the clock input to the OC1
      //Select Timer3 as the clock input to the OC1
         //111 = 系统时钟
      //110 = 保留
      //101 = 保留
      //100 = Timer1
      //011 = Timer5
      //010 = Timer4
      //001 = Timer3
      //000 = Timer2
OC1CON1BITS.OCM = 6 ; //110 = PWM模式：OCFA/B 禁用，当OCxTMR = 0 时输出设置为高电平，当OCxTMR = OCxR 时输出设
      //置为低电平


T3CON = 0x00;    //Stops any 16-bit Timer3 operation
TMR3 = 0x00;    //Clear contents of the timer3 register
T3CONBITS.TCKPS = 0; //11 = 1:256
      //10 = 1:64
      //01 = 1:8
      //00 = 1:1
IFS0BITS.T3IF = 0;   //Clear the Timer3 interrupt status flag
IEC0BITS.T3IE = 1;   //Enable Timer3 interrupts
IEC0BITS.OC1IE = 0;  //Disable Compare 1 interrupts

PR3 = Period;    //Determine the period
OC1R = rate;   //Initial Compare Register1 duty cycle
OC1RS = rate;   //Initial Secondary Compare Register1 duty cycle

T3CONBITS.TON = 1;   //Start Timer3
}


/* PWM example
 ** Assign OC1 to be output on RF5-J7(4)
 **
 *  PWM Period = [(PR2) + 1] * 2 * TOSC * (TMR2 Prescale Value)
 */
void PwmInit(void)
{
    //int i;
    POT_Init();

    //Make the pin of RP17(RF5) as normal function
    RPOR8bits.RP17R = 0;
    // Drive RF5 low and make it an output
    LATFbits.LATF5 = 0;
    TRISFbits.TRISF5 = 0;

    // Reset PWM
    OC1CON1 = 0x0000;
    OC1CON2 = 0x0000;

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
    T3CONbits.TCKPS = 0x03;
    // configure PWM
    OC1CON2 = 0x001F;   /* Sync with This OC module                               */
    OC1CON1 = 0x1C08;   /* Clock sourc Fcyc, trigger mode 1, Mode 0 (disable OC1) */
    /* enable the PWM */
    OC1CON1 = OC1CON1 | 0x0006;   /* Mode 6, Edge-aligned PWM Mode */

    // Make the pin of RP17(RF5) as OC1 (PWM output)
    RPOR8bits.RP17R = 18;

    T3CONbits.TON = 1; //Start Timer3
    // set the PWM period, 125 * 16us = 20ms
    OC1RS   = 125;
    for (;; )
    {
        BYTE j;
        j = POT_Read();
        
        // set PWM duty cycle
        OC1R    = j;
        delay_ms(200);
    }
      
    //while(1);
}
#endif

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

/****************************************************************************
  Function:
    BYTE ReadPOT(void)

  Summary:
    Reads the pot value and returns the percentage of full scale (0-100)

  Description:
    Reads the pot value and returns the percentage of full scale (0-100)

  Precondition:
    A/D is initialized properly

  Parameters:
    None

  Return Values:
    None

  Remarks:
    None
  ***************************************************************************/
unsigned int POT_Read(void)
{
    //WORD_VAL w;
    DWORD temp;

    AD1CHS = 0x9;           //MUXA uses AN9

    // Get an ADC sample
    AD1CON1bits.SAMP = 1;           //Start sampling
    //for(w.Val=0;w.Val<1000;w.Val++){Nop();} //Sample delay, conversion start automatically
    delay_ms(1);
    AD1CON1bits.SAMP = 0;           //Start sampling
    //for(w.Val=0;w.Val<1000;w.Val++){Nop();} //Sample delay, conversion start automatically
    delay_ms(1);
    while(!AD1CON1bits.DONE);       //Wait for conversion to complete
    temp = (DWORD)ADC1BUF0;
    temp = temp * 100;
    temp = temp/1023;

    return (BYTE)temp;
}//end ReadPOT
