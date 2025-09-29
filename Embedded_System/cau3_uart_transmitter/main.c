#include "stm32f10x.h"                  // Device header
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_usart.h"            // Keil::Device:StdPeriph Drivers:USART

void delay_ms(uint16_t time){
		uint16_t i;
			for(i = 0; i<time; i++){
					SysTick -> CTRL = 0x00000005;
					SysTick -> LOAD = 72000-1;
					SysTick -> VAL = 0;
			while(!(SysTick -> CTRL & (1 << 16))){}			
			}
}



void pin_uart(){
	
		GPIO_InitTypeDef usart_pin;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
		
		usart_pin.GPIO_Pin = GPIO_Pin_9;
		usart_pin.GPIO_Mode = GPIO_Mode_AF_PP;
		usart_pin.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &usart_pin);
	
		usart_pin.GPIO_Pin = GPIO_Pin_10;
		usart_pin.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		usart_pin.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &usart_pin);
	
}
	
void usart_mode(){

			USART_InitTypeDef usart_init;
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
			
			usart_init.USART_BaudRate = 115200;
			usart_init.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
			usart_init.USART_WordLength = USART_WordLength_8b;
			usart_init.USART_Parity = USART_Parity_No;
			usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
			
			USART_Init(USART1, &usart_init); // configure to register: Baudrate, Word length, Parity, Stop bit, Flow control, Mode… but still not turn on USART.
			USART_Cmd(USART1, ENABLE); // bat bo USART. -> complete
}	
	
	
// send a char
void USART1_SendChar(char c) {
    USART_SendData(USART1, c);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}
 // send string
void USART1_SendString(char *s) {
    while (*s) {
        USART1_SendChar(*s++);
    }
}

	
int main(){
	
	pin_uart();
	usart_mode();

	while(1){
			USART1_SendString("Hello He thong nhung\n");
			delay_ms(1000);   // delay 1000ms = 1s
}
	
	


}