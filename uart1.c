//****************************************
// UART1_Test
// MCU: PIC24F
// 2013.10.10
// Regis Hsu
//
// Assign U1RX To Pin RP16 (PIN51, F3)
// Assign U1TX To Pin RP30 (PIN52, F2)
// Link these 2 pins together for test, the LCD will show "LOVE"
//****************************************
/* Standard includes. */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

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
#include "kalman.h"
#include "math.h"
#include "gyro_car.h"
#include "moter_l298n.h"
#include "uart1.h"

//static unsigned char rx[4];
//static int uart_buf_cnt;
//static int uart_bufferfull;


//#define BRATE_SPI     416         // 9600 baud (BREGH=1)
//#define BRATE_BT      416         // 9600 baud (BREGH=1)

#define BRATE_SPI     207         // 19200 baud (BREGH=1)
#define BRATE_BT      207         // 19200 baud (BREGH=1)

//#define BRATE_SPI     103         // 38400 baud (BREGH=1)
//#define BRATE_BT      103         // 38400 baud (BREGH=1)

//#define BRATE_BT    70          // 57600 baud (BREGH=1)
//#define BRATE_SPI   70          // 57600 baud (BREGH=1)

//#define BRATE_SPI     34          // 115200 baud (BREGH=1)
//#define BRATE_BT      34          // 115200 baud (BREGH=1)

#define BUFFER_SIZE 64
#define UART_NUM    2
static unsigned char _buffer_u1[UART_NUM][BUFFER_SIZE];

//===============================================================================
//
//
//             T O   H O S T    F I F O   R O U T I N E S
//
//
//===============================================================================

#define Check_Rx_buffer(uart)  _buffer_u1[uart][BUFFER_SIZE-3]

unsigned char Check_Rx_Buffer(int uart)
{
    return Check_Rx_buffer(uart);
}
//******************************************************************************
void UART_FlushFIFO(int uart)      // clear FIFO structure (without resetting the data)
//******************************************************************************
{
    _buffer_u1[uart][BUFFER_SIZE-3] = 0;   // counter
    _buffer_u1[uart][BUFFER_SIZE-2] = 0;   // head
    _buffer_u1[uart][BUFFER_SIZE-1] = 0;   // tail

}

//******************************************************************************
void UART_PushFIFO(int uart, unsigned char new_data)      // push into FIFO one byte
//******************************************************************************
{
    unsigned char var_head;     // we create temporary variable

    var_head = _buffer_u1[uart][BUFFER_SIZE-2];    // read the head
    _buffer_u1[uart][ var_head ] = new_data;       // save the data in FIFO head

    _buffer_u1[uart][BUFFER_SIZE-3]++;             // increment counter of waiting bytes
    if (_buffer_u1[uart][BUFFER_SIZE-3] > BUFFER_SIZE-5)
    {
        _buffer_u1[uart][BUFFER_SIZE-3] = BUFFER_SIZE-5;   // to avoid overflowing
        // when overflow we just don't increment the head
    }
    else
    {
        // ---- only when there is no overflow we increment the head
        var_head++;
        if (var_head > (BUFFER_SIZE-4))               // uups, overflow of the head, we have to wrap around the head
            var_head = 0;     // we wrap it around
    }
    _buffer_u1[uart][BUFFER_SIZE-2] = var_head;    // we return back the head in the array
} // end of PushFIFO routine


//******************************************************************************
unsigned char UART_PopFIFO(int uart)      // pop from FIFO one byte
//******************************************************************************
{
    unsigned char var_tail, to_return;     // we create temporary variable

    var_tail = _buffer_u1[uart][BUFFER_SIZE-1];    // read the tail
    to_return = _buffer_u1[uart][ var_tail ];      // save the data in FIFO head

    if (_buffer_u1[uart][BUFFER_SIZE-3])
    {
        _buffer_u1[uart][BUFFER_SIZE-3]--;           // decrement by one the counter of waiting bytes

        var_tail++;
        if (var_tail > (BUFFER_SIZE-4))               // uups, overflow of the head, we have to wrap around the head
        var_tail = 0;     // we wrap it around

        _buffer_u1[uart][BUFFER_SIZE-1] = var_tail;   // we return back the tail in the array
    }
    return to_return;
} // end of PopFIFO routine

//void __attribute__((__interrupt__, auto_psv)) _U1TXInterrupt(void)
void _ISR __attribute__((__auto_psv__)) _U1TXInterrupt(void)
{
    IFS0bits.U1TXIF = 0;
    mLED_7 = 1 - mLED_7;
}

//void __attribute__((__interrupt__, auto_psv)) _U1RXInterrupt(void)
void _ISR __attribute__((__auto_psv__)) _U1RXInterrupt(void)
{
    IFS0bits.U1RXIF = 0;
    mLED_8 = 1 - mLED_8;

    while(U1STAbits.URXDA)
    {
        UART_PushFIFO(UART_SPI, U1RXREG);
    }
 }

//void __attribute__((__interrupt__, auto_psv)) _U1TXInterrupt(void)
void _ISR __attribute__((__auto_psv__)) _U2TXInterrupt(void)
{
    IFS1bits.U2TXIF = 0;
    //mLED_9 = 1 - mLED_9;
}

//void __attribute__((__interrupt__, auto_psv)) _U1RXInterrupt(void)
void _ISR __attribute__((__auto_psv__)) _U2RXInterrupt(void)
{
    IFS1bits.U2RXIF = 0;
    //mLED_10 = 1 - mLED_10;

    while(U2STAbits.URXDA)
    {
        UART_PushFIFO(UART_BT, U2RXREG);
    }
}

void UART_SPI_Init(void)
{
    //AD1PCFGL = 0xFFFF; //All Digital pins
    // set pin as input
    TRISFbits.TRISF2 = 1;
    // Assign U1RX To Pin RP30 (PIN52, F2), J7#2
    RPINR18bits.U1RXR = 30;             //'30' represents RP30
    // set the pin as output
    TRISFbits.TRISF8 = 0;
    // Assign U1TX To Pin RP15 (PIN53, F8), J7#1
    RPOR7bits.RP15R = 3;               //'3' represents U1TX

    U1BRG = BRATE_SPI;                        // 9600 baud (BREGH=1) at 16MHz FCY
    U1MODEbits.UARTEN = 1;		// UART1 is Enabled
    U1MODEbits.USIDL = 0;		// Continue operation at Idlestate
    U1MODEbits.IREN = 0;		// IrDA En/Decoder is disabled
    U1MODEbits.RTSMD = 0; 		// flow control mode
    //U1MODEbits.UEN = 0b10;		// UTX, RTX, U2CTS, U2RTS are enable and on use.
    U1MODEbits.UEN = 0;                 // UTX, RTX are enable and on use.
    //U1MODEbits.WAKE = 1;		// Wake-up on start bit is enabled
    U1MODEbits.WAKE = 0;		// Wake-up on start bit is disabled
    U1MODEbits.LPBACK = 0;		// Loop-back is disabled
    U1MODEbits.ABAUD = 0;		// auto baud is disabled
    U1MODEbits.RXINV = 0;		// No RX inversion
    U1MODEbits.BRGH = 1;		// Hi boud rate
    U1MODEbits.PDSEL = 0;               // 8bit no parity
    U1MODEbits.STSEL = 0;		// one stop bit

    //U1STAbits.UTXISEL1 = 1;         //Interrupt when a character is transferred to the Transmit Shift Register (TSR) and as a result, the transmit buffer becomes empty
    U1STAbits.UTXISEL1 = 1;
    U1STAbits.UTXISEL0 = 0;
    U1STAbits.UTXINV = 0;
    U1STAbits.UTXBRK = 0;
    U1STAbits.UTXEN = 1;
    U1STAbits.URXISEL1 = 0;        //Interrupt is set when any character is received and transferred from the RSR to the receive buffer.Receive buffer has one or more characters.
    U1STAbits.URXISEL0 = 0;         // Interrupt flag bit is set when a character is received
    U1STAbits.ADDEN = 0;
    U1STAbits.RIDLE = 0;
    //reset RX TX interrupt flag
    IFS0bits.U1RXIF = 0;
    IFS0bits.U1TXIF = 0;
    //set up interrupt for UART1, receive and transmit
    //IFS0bits.U1RXIF = 0;

    // U1RX interrup priority
    IPC2bits.U1RXIP2=1;
    IPC2bits.U1RXIP1=0;
    IPC2bits.U1RXIP0=1;

    IPC3bits.U1TXIP2 = 1;
    IPC3bits.U1TXIP1 = 0;
    IPC3bits.U1TXIP0 = 1;

    //IEC0bits.U1RXIE = 0;
    _U1RXIE = 1;            // 1: enable U1RX interrupt
    _U1TXIE = 1;            // 1: enable U1TX interrupt

}

void UART_BT_Init(void)
{
    //AD1PCFGL = 0xFFFF; //All Digital pins
    // set pin as input
    TRISFbits.TRISF3 = 1;
    // Assign U1RX To Pin RP16 (PIN51, F3), J7#3
    RPINR19bits.U2RXR = 16;             //'16' represents RP16
    // set the pin as output
    TRISFbits.TRISF5 = 0;
    // Assign U1TX To Pin RP17 (PIN50, F5), J7#4
    RPOR8bits.RP17R = 5;               //'5' represents U2TX

    U2BRG = BRATE_BT;                        // 9600 baud (BREGH=1) at 16MHz FCY
    U2MODEbits.UARTEN = 1;		// UART2 is Enabled
    U2MODEbits.USIDL = 0;		// Continue operation at Idlestate
    U2MODEbits.IREN = 0;		// IrDA En/Decoder is disabled
    U2MODEbits.RTSMD = 0; 		// flow control mode
    //U2MODEbits.UEN = 0b10;		// UTX, RTX, U2CTS, U2RTS are enable and on use.
    U2MODEbits.UEN = 0;                 // UTX, RTX are enable and on use.
    //U2MODEbits.WAKE = 1;		// Wake-up on start bit is enabled
    U2MODEbits.WAKE = 0;		// Wake-up on start bit is disabled
    U2MODEbits.LPBACK = 0;		// Loop-back is disabled
    U2MODEbits.ABAUD = 0;		// auto baud is disabled
    U2MODEbits.RXINV = 0;		// No RX inversion
    U2MODEbits.BRGH = 1;		// Hi boud rate
    U2MODEbits.PDSEL = 0;               // 8bit no parity
    U2MODEbits.STSEL = 0;		// one stop bit

    //U2STAbits.UTXISEL1 = 1;         //Interrupt when a character is transferred to the Transmit Shift Register (TSR) and as a result, the transmit buffer becomes empty
    U2STAbits.UTXISEL1 = 1;
    U2STAbits.UTXISEL0 = 0;
    U2STAbits.UTXINV = 0;
    U2STAbits.UTXBRK = 0;
    U2STAbits.UTXEN = 1;            // 1: enable Tx interrupt, 0: disable Tx interrupt

    U2STAbits.URXISEL1 = 0;        //Interrupt is set when any character is received and transferred from the RSR to the receive buffer.Receive buffer has one or more characters.
    U2STAbits.URXISEL0 = 0;         // Interrupt flag bit is set when a character is received
    U2STAbits.ADDEN = 0;
    U2STAbits.RIDLE = 0;

    // U2RX interrup priority
    IPC7bits.U2RXIP2 = 1;
    IPC7bits.U2RXIP1 = 1;
    IPC7bits.U2RXIP0 = 0;
    IPC7bits.U2TXIP2 = 1;
    IPC7bits.U2TXIP1 = 1;
    IPC7bits.U2TXIP0 = 0;

    //reset RX TX interrupt flag
    IFS1bits.U2RXIF = 0;
    IFS1bits.U2TXIF = 0;
    //set up interrupt for UART2, receive and transmit
    //IFS1bits.U2RXIF = 0;

    _U2RXIE = 1;            // 1: enable U2RX interrupt
    _U2TXIE = 1;            // 1: enable U2TX interrupt

    UART_FlushFIFO(UART_BT);
    UART_PutString(UART_BT, "BT_Ready\r\n");
}

//UART transmit function, parameter Ch is the character to send
void UART_PutChar (int uart, unsigned char Ch) //I have the missing ( in my code, the forum is not letting me post it!.
{
    if (uart == UART_BT)
    {
        //transmit ONLY if TX buffer is empty
        while (U2STAbits.UTXBF == 1);
        U2TXREG = Ch;
    }
    else
    {
        //transmit ONLY if TX buffer is empty
        while (U1STAbits.UTXBF == 1);
        U1TXREG = Ch;
    }
}

 //UART receive function, returns the value received.
unsigned char UART_GetChar(int uart) //I have the missing ( in my code, the forum is not letting me post it!.
{
    if (Check_Rx_buffer(uart))
        return UART_PopFIFO(uart);
}

void UART_PutString(int uart, char *s)
{
    int a, i;
    a = strlen(s);
    for (i =0; i < a; i++)
    {
        UART_PutChar(uart, (unsigned char)*s);
        s++;
    }
}

void UART_Test(void)
{

    char s[30] = "Regis is a smart guy!!!";

    UART_FlushFIFO(UART_SPI);
    UART_FlushFIFO(UART_BT);

    UART_SPI_Init();
    UART_BT_Init();

    UART_PutString(UART_BT, s);
    UART_PutString(UART_SPI, s);
    while(1)
    {
        if (Check_Rx_buffer(UART_SPI))
        {
            UART_PutChar(UART_BT, UART_PopFIFO(UART_SPI));
        }
        if (Check_Rx_buffer(UART_BT))
        {
            UART_PutChar(UART_SPI, UART_PopFIFO(UART_BT));
        }
    }

}
#if 0
int atoi(char *s)
{
    return (int)strtoul(s, NULL, 10);
}
#endif

void vTask_UART_BT(void *pvParameters )
{
    (void)pvParameters; // prevent compiler worning/error
    portTickType xLastWakeTime;
    int i,n,m;
    char cmd[30];

    UART_BT_Init();
    //UART_PutString(UART_BT, "BT_Ready\r\n");

    xLastWakeTime = xTaskGetTickCount();
    for( ;; )
    {
        vTaskDelayUntil(&xLastWakeTime, 200);
        n = Check_Rx_buffer(UART_BT);
        //sprintf(cmd, "n=%d\n", n);
        //UART_PutString(UART_BT, cmd);
        if (n > 0)
        {
            n = n/2;
            // parser the command
            for (i = 0; i < n; i++)
            {
                cmd[0] = UART_PopFIFO(UART_BT);
                cmd[1] = UART_PopFIFO(UART_BT);
                cmd[2] = 0;
                //UART_PutString(UART_BT, cmd);
                m = atoi(cmd);
                sprintf(cmd, "Cmd=%d => ", m);
                UART_PutString(UART_BT, cmd);
/*
#define CMD_KP_U            1
#define CMD_KP_D            2
#define CMD_KI_U            3
#define CMD_KI_D            4
#define CMD_KD_U            5
#define CMD_KD_D            6
#define CMD_BL_U            7
#define CMD_BL_D            8
#define CMD_DEBUG           9
#define CMD_MOTOR           10
#define CMD_TASK_TIME_U     11
#define CMD_TASK_TIME_D     12
#define CMD_ANGLE_MAX_U     13
#define CMD_ANGLE_MAX_D     14
#define CMD_ANGLE_MIN_U     15
#define CMD_ANGLE_MIN_D     16
#define CMD_ANGLE_STEP_U    17
#define CMD_ANGLE_STEP_D    18

 */
                switch (m)
                {
                    case CMD_DEBUG:
                        Gyro_Car_Debug();
                        break;
                    case CMD_TASK_TIME_U:
                        Gyro_Car_TaskTimer_Up();
                        break;
                    case CMD_TASK_TIME_D:
                        Gyro_Car_TaskTimer_Down();
                        break;
                    case CMD_KP_U:
                        Gyro_Car_Kp_Up();
                        break;
                    case CMD_KP_D:
                        Gyro_Car_Kp_Down();
                        break;
                    case CMD_KI_U:
                        Gyro_Car_Ki_Up();
                        break;
                    case CMD_KI_D:
                        Gyro_Car_Ki_Down();
                        break;
                    case CMD_KD_U:
                        Gyro_Car_Kd_Up();
                        break;
                    case CMD_KD_D:
                        Gyro_Car_Kd_Down();
                        break;
                    case CMD_MOTOR:
                        Gyro_Car_Motor();
                        break;
                    case CMD_BL_U:
                        Gyro_Car_Balance_Up();
                        break;
                    case CMD_BL_D:
                        Gyro_Car_Balance_Down();
                        break;
                    case CMD_ANGLE_MAX_U:
                        Gyro_Car_Angle_Max_Up();
                        break;
                    case CMD_ANGLE_MAX_D:
                        Gyro_Car_Angle_Max_Down();
                        break;
                    case CMD_ANGLE_MIN_U:
                        Gyro_Car_Angle_Min_Up();
                        break;
                    case CMD_ANGLE_MIN_D:
                        Gyro_Car_Angle_Min_Down();
                        break;
                    case CMD_ANGLE_STEP_U:
                        Gyro_Car_Angle_Step_Up();
                        break;
                    case CMD_ANGLE_STEP_D:
                        Gyro_Car_Angle_Step_Down();
                        break;
                    case CMD_PWM_STEP_U:
                        Gyro_Car_PWM_Step_Up();
                        break;
                    case CMD_PWM_STEP_D:
                        Gyro_Car_PWM_Step_Down();
                        break;
                    case CMD_PWM_STATIC_U:
                        Gyro_Car_PWM_Static_Up();
                        break;
                    case CMD_PWM_STATIC_D:
                        Gyro_Car_PWM_Static_Down();
                        break;
                    case CMD_CAR_FF:
                        Gyro_Car_PWM_FF();
                        break;
                    case CMD_CAR_BK:
                        Gyro_Car_PWM_BK();
                        break;
                    case CMD_CAR_STOP:
                        Gyro_Car_PWM_Stop();
                        break;
                    case CMD_CAR_RIGHT:
                        Gyro_Car_PWM_Right();
                        break;
                    case CMD_CAR_LEFT:
                        Gyro_Car_PWM_Left();
                        break;
                    case CMD_CAR_SPEED_U:
                        Gyro_Car_PWM_Speed_Up();
                        break;
                    case CMD_CAR_SPEED_D:
                        Gyro_Car_PWM_Speed_Down();
                        break;
                    default:
                        UART_PutString(UART_BT, "no cmd!\r\n");
                        break;
                }
            }
        }
    }
}

#if 0
void vTask_Uart1_Test(void *pvParameters )
{
    portTickType xLastWakeTime;
    unsigned char i, j;

    i = j = 0;

    UART1_Init();
    xLastWakeTime = xTaskGetTickCount();
    for( ;; )
    {
        vTaskDelayUntil(&xLastWakeTime,2000);

        i = 1 - i;
        mLED_6 = i;

        if (i)
        {
            UART1PutChar ('L');
            UART1PutChar ('O');
            UART1PutChar ('V');
            UART1PutChar ('E');
            //vTaskDelay(100);       //100ms
            Lcd_1602_clear();

        }
        else
        {
            mLED_5 = 0;
            mLED_7 = 0;

            LCD_Show_String(2, 0, rx, 4);
            sprintf(rx, "%4d",i);
            rx[0] = rx[1] = rx[2] = rx[3] = 0;
        }
    }
}

#endif

#if 0
void vTask_IR_Remote(void *pvParameters )
{
    portTickType xLastWakeTime;
    unsigned char i, j;

    i = j = 0;

    IR_Remote_Init();
    xLastWakeTime = xTaskGetTickCount();
    for( ;; )
    {
        vTaskDelayUntil(&xLastWakeTime,2000);

        i = 1 - i;
        mLED_6 = i;

        if (j == 0)
        {
            rx[0] = rx[1] = rx[2] = rx[3] = 0;
            UART1PutChar ('L');
            UART1PutChar ('O');
            UART1PutChar ('V');
            UART1PutChar ('E');
            vTaskDelay(100);       //100ms
        }

       //if (uart_bufferfull)
        {
            Lcd_1602_DisplayOneChar(2,1,rx[j]);

            mLED_5 = 0;
            mLED_7 = 0;
            //uart_bufferfull = 0;
            j++;
            if (j == 4)
            {
                j = 0;
                //rx[0] = rx[1] = rx[2] = rx[3] = 0;
            }
        }
    }
}
#endif
