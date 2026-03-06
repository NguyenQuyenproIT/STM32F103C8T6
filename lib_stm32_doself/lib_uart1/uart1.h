#ifndef _UART_H_
#define _UART_H_

#include "stm32f10x.h"
/************************/
// khai bao nguyen mau ham

void PIN_MODE_UART1(void);
void USART1_SendChar(char c); // gui 1 ky tu
void USART1_SendString(char *s); // gui 1 chuoi
char USART1_GetChar(void); // nhan ki tu
void delay_ms(unsigned int time);
char UARTx_Getc(USART_TypeDef* USARTx);

/**************************/
#endif
