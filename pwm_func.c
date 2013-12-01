//****************************************
// pwm_func.c
// for PWM function
// 2013.11.12
// Regis Hsu
// refer to Section 35. Output Compare with Dedicated Timer
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

//PPS Outputs for PIC24FJ256GA110 and PIC24FJ256GB110
#define OC1_IO		18
#define OC2_IO		19
#define OC3_IO		20
#define OC4_IO		21
#define OC5_IO		22
#define OC6_IO          23
#define OC7_IO          24
#define OC8_IO          25
#define OC9_IO          35


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
BYTE POT_Read(void)
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

/* PWM example
 ** Assign OC1 to be output on RF5-J7(4)
 **
 *  PWM Period = [(PR2) + 1] * 2 * TOSC * (TMR2 Prescale Value)
 */
void PWM_Init(void)
{
    //int i;
    POT_Init();

    //Make the pin of RP17(RF5) as normal function
    RPOR8bits.RP17R = 0;
    // Drive RF5 low and make it an output
    //LATFbits.LATF5 = 0;
    _LATF5 = 0;
    //TRISFbits.TRISF5 = 0;
    _TRISF5 = 0;
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
     *   Fcy=32MHz/2=16MHz=0.0625us
     *
     */
    T3CONbits.TCKPS = 0x03; //1:256, 16us
    // configure PWM
    OC1CON2 = 0x001F;   /* Sync with This OC module                               */
    //OC1CON1 = 0x1C08;   /* Clock sourc Fcyc, trigger mode 1, Mode 0 (disable OC1) */
    OC1CON1 = 0x0408;   /* Clock sourc timer3, trigger mode 1, Mode 0 (disable OC1) */
    /* enable the PWM */
    OC1CON1 = OC1CON1 | 0x0006;   /* Mode 6, Edge-aligned PWM Mode */

    // Make the pin of RP17(RF5) as OC1 (PWM output)
    RPOR8bits.RP17R = 18;

    T3CONbits.TON = 1; //Start Timer3
    // set the PWM period, 1250 * 16us = 20ms
    OC1RS   = 1250;
    for (;; )
    {
        BYTE j;
        // 0~100, 100*16us = 1.6ms
        j = POT_Read();
        
        // set PWM duty cycle
        OC1R    = j;
        delay_ms(200);
    }
      
    //while(1);
}
/*
 * Servo SG90 spec:
 * 20ms as base clock, duty cycle as below
 * 0.5ms:0 degree； 1.0ms:45；1.5ms:90；
 * 2.0ms:135；2.5ms:180；

 */
void Servo_SG90_Init(void)
{
    // Drive RF5 low and make it an output
    LATFbits.LATF5 = 0;
    TRISFbits.TRISF5 = 0;
    // Make the pin of RP17(RF5) as OC1 (PWM output)
    RPOR8bits.RP17R = 18;

    // configure PWM
    OC1CON2 = 0x001F;   /* Sync with This OC module                               */
    //OC1CON1 = 0x1C08;   /* Clock sourc Fcyc, trigger mode 1, Mode 0 (disable OC1) */
    OC1CON1 = 0x0408;   /* Clock sourc timer3, trigger mode 1, Mode 0 (disable OC1) */
    // enable the PWM
    OC1CON1 = OC1CON1 | 0x0006;   /* Mode 6, Edge-aligned PWM Mode */
    // set the PWM period, 1250 * 16us = 20ms
    OC1RS = 1250;
    OC1R = 0;


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
    T3CONbits.TCKPS = 0x03; //1:256, 16us
    //T3CONbits.TON = 1; //Start Timer3
}

/*
 *  d as the degree of SG90 rotate 
 *  0.5ms:0 degree； 1.0ms:45；1.5ms:90；
 *  2.0ms:135；2.5ms:180；
 */
void Servo_SG90_Rotation(int d)
{
    int i,j;
    if (d < 0 || d > 180)
        i = 90;
    else
        i = d;
    // 1 degree = 11.1us, timer3 scale is 16us, so, i * 11.1/16us = i*0.7, 500us/16 = 31
    j = i * 0.7 + 31;
    // set PWM duty cycle
    OC1R = j;

}

void Servo_SG90_Move(int sw)
{
    //  Start/Stop Timer3
    T3CONbits.TON = sw;
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

#if 0
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
#endif

/*
 * below are copy from Microchip sample code
 *  Regis 2013.12.01
 */
/*
//PPS Outputs for PIC24FJ256GA110 and PIC24FJ256GB110
#define OC1_IO		18
#define OC2_IO		19
#define OC3_IO		20
#define OC4_IO		21
#define OC5_IO		22
#define OC6_IO      23
#define OC7_IO      24
#define OC8_IO      25
#define OC9_IO      35
*/
int PWM_Test (void)
{
	// Disable Watch Dog Timer
	RCONbits.SWDTEN = 0;

	// Configure RA6 as output
	ODCAbits.ODA6 = 0;
	TRISAbits.TRISA6 = 0;

        /*
         *  original sample code, Regis
	// PPS
	// Unlock Registers
	//__builtin_write_OSCCONL(OSCCON & 0xbf);
	// Configure Output Functions *********************
        RPOR6bits.RP13R 	= OC1_IO;	// RP13	pin#23
	RPOR0bits.RP1R 		= OC2_IO;	// RP1	pin#24
	RPOR4bits.RP8R 		= OC3_IO;	// RP8	pin#32
	RPOR4bits.RP9R 		= OC4_IO;	// RP9	pin#33

	RPOR5bits.RP11R 	= OC5_IO;	// RP11	pin#72
	RPOR6bits.RP12R 	= OC6_IO;	// RP12	pin#71
	RPOR1bits.RP3R 		= OC7_IO;	// RP3	pin#70
	RPOR2bits.RP4R 		= OC8_IO;	// RP4	pin#69

	RPOR12bits.RP24R 	= OC9_IO;	// RP24	pin#76
	// Lock Registers
	//__builtin_write_OSCCONL(OSCCON | 0x40);
        */
	// PPS
	// modify by Regis, 2013.12.01
        TRISBbits.TRISB0 = 0;       //output, RP0/RB0
        TRISBbits.TRISB1 = 0;       //output, RP1/RB1
        TRISBbits.TRISB2 = 0;      //output, RP13/RB2
        TRISBbits.TRISB4 = 0;      //output, RP28/RB4
        // Configure Output Functions *********************
        RPOR0bits.RP0R          = OC5_IO;	// RR0	pin#25, J5#01,AN0
	RPOR0bits.RP1R 		= OC6_IO;	// RP1	pin#24, J5#02,AN1
	RPOR6bits.RP13R		= OC7_IO;	// RP13	pin#23, J5#03,AN2
	RPOR14bits.RP28R 	= OC8_IO;	// RP28	pin#21, J5#05,AN4

       	RPOR12bits.RP24R 	= OC3_IO;	// RP24	pin#76, J14#03, D2
       	RPOR11bits.RP23R 	= OC2_IO;	// RP23	pin#77, J14#04, D3
       	RPOR11bits.RP22R 	= OC1_IO;	// RP22	pin#78, J14#05, D4
       	RPOR12bits.RP25R 	= OC4_IO;	// RP25	pin#81, J14#07, D6

       	RPOR10bits.RP20R 	= OC9_IO;	// RP22	pin#82, J14#08, D7

	/*
	The following configuration for OC1, OC2, OC3 and OC4 will set the
	Output Compare modules for PWM mode w/o FAULT pin enabled,
	a 50%/75% duty cycle and PWM frequency of 1 kHz at Fosc = 32 MHz.
	System clock is selected as the clock for the PWM time base
	and no interrupt is enabled. The four PWM channels are synchronised
	by OC1, therefore OC1 should be initialized at last. OC2 and OC4 are
	complementary to OC1 and OC3 respectively.

	Find the Period register value for a desired PWM frequency of 1 kHz,
	where Fosc = 8 MHz with PLL (32 MHz device clock rate).
	Tcy = 2/Fosc = 62.5 ns
	PWM Period   =  1/PWM Frequency = 1/1 kHz = 1000us
	PWM Period   = (OC1RS + 1) * Tcy
	1000us = (OC1RS + 1) * 62.5 ns * 1
	OC1RS = 15999 (0x3E7F)
	*/

	OC2CON1			= 0; 		// It is a good practice to clear off the control bits initially
	OC2CON2			= 0;
	OC2R			= 0x2edf; //1f3f;	// Initialize Compare Register1 with 50% duty cycle
	OC2RS			= 0x3e7f;	// This period is synchronized by OC1
//	OC2CON2bits.SYNCSEL	= 0x1;		// synchronized by the OC1 module
	OC2CON2bits.SYNCSEL	= 0x02;		// synchronized by the OC1 module
	OC2CON1bits.OCTSEL	= 0x7;		// System Clock (Tcy) is the clock source for output Compare
	OC2CON1bits.OCFLT	= 0;		// No PWM Fault condition has occurred (this bit is only used when OCM<2:0> = 111)
	OC2CON2bits.OCINV	= 1;		// OCx output is inverted
	OC2CON1bits.OCM		= 0x6;		// Edge-aligned PWM mode on OC

	OC3CON1			= 0; 		// It is a good practice to clear off the control bits initially
	OC3CON2			= 0;
	OC3R			= 0x1f3f; //2edf;	// Initialize Compare Register1 with 75% duty cycle
	OC3RS			= 0x3e7f;	// This period is synchronized by OC1
//	OC3CON2bits.SYNCSEL	= 0x1;		// synchronized by the OC1 module
	OC3CON2bits.SYNCSEL	= 0x03;		// synchronized by the OC1 module
	OC3CON1bits.OCTSEL	= 0x7;		// System Clock (Tcy) is the clock source for output Compare
	OC3CON1bits.OCFLT	= 0;		// No PWM Fault condition has occurred (this bit is only used when OCM<2:0> = 111)
	OC3CON2bits.OCINV	= 0;		// OCx output is not inverted
	OC3CON1bits.OCM		= 0x6;		// Edge-aligned PWM mode on OC

	OC4CON1			= 0; 		// It is a good practice to clear off the control bits initially
	OC4CON2			= 0;
	OC4R			= 0x2edf;	// Initialize Compare Register1 with 75% duty cycle
	OC4RS			= 0x3e7f;	// This period is synchronized by OC1
//	OC4CON2bits.SYNCSEL	= 0x1;		// synchronized by the OC1 module
	OC4CON2bits.SYNCSEL	= 0x04;		// synchronized by the OC1 module
	OC4CON1bits.OCTSEL	= 0x7;		// System Clock (Tcy) is the clock source for output Compare
	OC4CON1bits.OCFLT	= 0;		// No PWM Fault condition has occurred (this bit is only used when OCM<2:0> = 111)
	OC4CON2bits.OCINV	= 1;		// OCx output is inverted
	OC4CON1bits.OCM		= 0x6;		// Edge-aligned PWM mode on OC

	OC1CON1			= 0; 		// It is a good practice to clear off the control bits initially
	OC1CON2			= 0;
	OC1R			= 0x1f3f;	// Initialize Compare Register1 with 50% duty cycle
	OC1RS			= 0x3e7f;	// Calculate the desired period for OC1, OC2, OC3 and OC4.
	OC1CON2bits.SYNCSEL	= 0x1;		// synchronized by the OC1 module
//	OC1CON2bits.SYNCSEL	= 0x0;		// synchronized by the OC1 module
	OC1CON1bits.OCTSEL	= 0x7;		// System Clock (Tcy) is the clock source for output Compare
	OC1CON1bits.OCFLT	= 0;		// No PWM Fault condition has occurred (this bit is only used when OCM<2:0> = 111)
	OC1CON2bits.OCINV	= 0;		// OCx output is not inverted
	OC1CON1bits.OCM		= 0x6;		// Edge-aligned PWM mode on OC

	/*
	The following configuration for OC5, OC6, OC7 and OC8 will set the
	Output Compare modules for PWM mode w/o FAULT pin enabled,
	a 50% duty cycle and PWM frequency of 50 kHz at Fosc = 32 MHz.
	Timer 1 is selected as the clock for the PWM time base
	and Timer 1 interrupt is enabled. The four PWM channels are
	synchronised by Timer 1, and the waveforms continue one after
	the other.

	Find the Period register value for a desired PWM frequency of 50 kHz,
	where Fosc = 8 MHz with PLL (32 MHz device clock rate).
	Tcy = 2/Fosc = 62.5 ns
	PWM Period   =  1/PWM Frequency = 1/50 kHz = 20 us
	PWM Period   = (PR1 + 1) * Tcy * (Timer 1 Prescale Value)
	20 us = (PR1 + 1) * 62.5 ns * 1
	PR1 = 319 (0x13f)
	*/
	T1CON			= 0;		// Timer 1 is clock for OC5, OC6, OC7 and OC8.
	PR1			= 0x13f;	// This is the period of OC5, OC6, OC7 and OC8.
	T1CONbits.TCKPS		= 0;		// Timer Input Clock Prescale is 1:1
	IFS0bits.T1IF		= 0;		// Clear Output Compare interrupt flag
	//IEC0bits.T1IE		= 1;		// Enable Output Compare interrupts
        IEC0bits.T1IE		= 0;		// disable Output Compare interrupts

	OC5CON1			= 0; 		// It is a good practice to clear off the control bits initially
	OC5CON2			= 0;
	OC5R			= 0x0;		// Initialize Compare Register1 with desired rising edge of the pulse
	OC5RS			= 0x7f;		// Initialize Compare Register1 with desired falling edge of the pulse
	OC5CON2bits.SYNCSEL	= 0x0b;		// synchronized by the Timer 1
	OC5CON1bits.OCTSEL	= 0x4;		// Timer 1 is the clock source for output Compare
	OC5CON1bits.OCFLT	= 0;		// No PWM Fault condition has occurred (this bit is only used when OCM<2:0> = 111)
	OC5CON2bits.OCINV	= 0;		// OCx output is not inverted
	OC5CON1bits.OCM		= 0x7;		// Center-aligned PWM mode on OC

	OC6CON1			= 0; 		// It is a good practice to clear off the control bits initially
	OC6CON2			= 0;
	OC6R			= 0x3f;		// Initialize Compare Register1 with desired rising edge of the pulse
	OC6RS			= 0xbf;		// Initialize Compare Register1 with desired falling edge of the pulse
	OC6CON2bits.SYNCSEL	= 0x0b;		// synchronized by the Timer 1
	OC6CON1bits.OCTSEL	= 0x4;		// Timer 1 is the clock source for output Compare
	OC6CON1bits.OCFLT	= 0;		// No PWM Fault condition has occurred (this bit is only used when OCM<2:0> = 111)
	OC6CON2bits.OCINV	= 0;		// OCx output is not inverted
	OC6CON1bits.OCM		= 0x7;		// Center-aligned PWM mode on OC

	OC7CON1			= 0; 		// It is a good practice to clear off the control bits initially
	OC7CON2			= 0;
	OC7R			= 0x7f;		// Initialize Compare Register1 with desired rising edge of the pulse
	OC7RS			= 0xff;		// Initialize Compare Register1 with desired falling edge of the pulse
	OC7CON2bits.SYNCSEL	= 0x0b;		// synchronized by the Timer 1
	OC7CON1bits.OCTSEL	= 0x4;		// Timer 1 is the clock source for output Compare
	OC7CON1bits.OCFLT	= 0;		// No PWM Fault condition has occurred (this bit is only used when OCM<2:0> = 111)
	OC7CON2bits.OCINV	= 0;		// OCx output is not inverted
	OC7CON1bits.OCM		= 0x7;		// Center-aligned PWM mode on OC

	OC8CON1			= 0; 		// It is a good practice to clear off the control bits initially
	OC8CON2			= 0;
	OC8R			= 0xbf;		// Initialize Compare Register1 with desired rising edge of the pulse
	OC8RS			= 0x13f;	// Initialize Compare Register1 with desired falling edge of the pulse
	OC8CON2bits.SYNCSEL	= 0x0b;		// synchronized by the Timer 1
	OC8CON1bits.OCTSEL	= 0x4;		// Timer 1 is the clock source for output Compare
	OC8CON1bits.OCFLT	= 0;		// No PWM Fault condition has occurred (this bit is only used when OCM<2:0> = 111)
	OC8CON2bits.OCINV	= 0;		// OCx output is not inverted
	OC8CON1bits.OCM		= 0x7;		// Center-aligned PWM mode on OC

	T1CONbits.TON		= 1;		// Output compare modules do not run until sync source is switched on

	/*
	The following configuration for OC9 will set the Output Compare
	modules for PWM mode w/o FAULT pin enabled, a 33% duty cycle
	and PWM frequency of 20 Hz at Fosc = 32 MHz.
	Timer 2 is selected as the clock for the PWM time base
	and no interrupt is enabled. The PWM channel is	synchronised
	by itself.

	Find the Period register value for a desired PWM frequency of 20 Hz,
	where Fosc = 8 MHz with PLL (32 MHz device clock rate).
	Tcy = 2/Fosc = 62.5 ns
	PWM Period   =  1/PWM Frequency = 1/20 Hz = 50 ms
	PWM Period   = (OC9RS + 1) * Tcy * (Timer 2 Prescale Value)
	50 ms = (OC9RS + 1) * 62.5 ns * 64
	OC9RS = 12499 (0x30d3)
	*/
	T2CON			= 0;		// Timer 2 is clock for OC9.
	T2CONbits.TCKPS		= 2;		// Timer Input Clock Prescale is 1:64

	OC9CON1			= 0; 		// It is a good practice to clear off the control bits initially
	OC9CON2			= 0;
	OC9R			= 0x1046;	// Initialize Compare Register1 with desired duty cycle
	OC9RS			= 0x30d3;	// Initialize Compare Register1 with desired period
	OC9CON2bits.SYNCSEL	= 0x1f;		// synchronized by itself
	OC9CON1bits.OCTSEL	= 0x0;		// Timer 2 is the clock source for output Compare
	OC9CON1bits.OCFLT	= 0;		// No PWM Fault condition has occurred (this bit is only used when OCM<2:0> = 111)
	OC9CON2bits.OCINV	= 0;		// OCx output is not inverted
	OC9CON1bits.OCM		= 0x6;		// Edge-aligned PWM mode on OC
	T2CONbits.TON		= 1;		// Output compare modules do not run until sync source is switched on


	while(1)
	{
		Idle();
	}

	return 0;
}

/* Remark by Regis, 2013.12.01
// Example code for Timer1 ISR can be probed as the start time of every period.
void __attribute__ ((__interrupt__, no_auto_psv)) _T1Interrupt(void)
{
	IFS0bits.T1IF = 0;
	__builtin_btg((unsigned int *)&LATA, 6);
}
*/


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

#endif