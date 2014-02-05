//****************************************
// UART1_Test
// 2013.10.10
// Regis Hsu
//****************************************

#ifndef _UART_SPI_BT_
#define _UART_SPI_BT_

#define UART_SPI    0
#define UART_BT     1

void UART_SPI_Init(void);
void UART_BT_Init(void);
unsigned char UART_GetChar(int uart);
unsigned char UART_PopFIFO(int uart);
void UART_PutString(int uart, char *s);
void vTask_UART_BT(void *pvParameters );

#endif