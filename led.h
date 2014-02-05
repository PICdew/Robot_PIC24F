

#ifndef INC_LED_H
#define INC_LED_H

/** LED ************************************************************/
    #define InitAllLEDs()  {TRISE = 0; LATE = 0;}

    #define mLED_3              LATEbits.LATE0
    #define mLED_4              LATEbits.LATE1
    #define mLED_5              LATEbits.LATE2
    #define mLED_6              LATEbits.LATE3
    #define mLED_7              LATEbits.LATE4
    #define mLED_8              LATEbits.LATE5
    #define mLED_9              LATEbits.LATE6
    #define mLED_10             LATEbits.LATE7

    #define LED0_On()         mLED_3  = 1;
    #define LED1_On()         mLED_4  = 1;
    #define LED2_On()         mLED_5  = 1;
    #define LED3_On()         mLED_6  = 1;
    #define LED4_On()         mLED_7  = 1;
    #define LED5_On()         mLED_8  = 1;
    #define LED6_On()         mLED_9  = 1;
    #define LED7_On()         mLED_10 = 1;

    #define LED0_Off()        mLED_3  = 0;
    #define LED1_Off()        mLED_4  = 0;
    #define LED2_Off()        mLED_5  = 0;
    #define LED3_Off()        mLED_6  = 0;
    #define LED4_Off()        mLED_7  = 0;
    #define LED5_Off()        mLED_8  = 0;
    #define LED6_Off()        mLED_9  = 0;
    #define LED7_Off()        mLED_10 = 0;
    
    #define LED_On(n) {LATE = n;}

// for debug , Regis 2013/11/12
#define LED_ERR(n) {LATE = n; while(1);}

#define BTN5_Pressed()    ((PORTFbits.RF13  == 0)? TRUE : FALSE)
#define BTN4_Pressed()    ((PORTFbits.RF12  == 0)? TRUE : FALSE)
#define BTN3_Pressed()    ((PORTBbits.RB11  == 0)? TRUE : FALSE)
#define BTN2_Pressed()    ((PORTAbits.RA1 == 0)? TRUE : FALSE)

#endif