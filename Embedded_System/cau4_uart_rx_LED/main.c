#include "stm32f10x.h"                  // Device header
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_usart.h"            // Keil::Device:StdPeriph Drivers:USART

void PIN_MODE_UART1();
void USART1_SendChar(char c); // gui 1 ky tu
void USART1_SendString(char *s); // gui 1 chuoi
char USART1_GetChar(); // nhan ki tu
void delay_ms(unsigned int time);

void delay_ms(unsigned int time){
		unsigned int i;
			for(i = 0; i<time; i++){
					SysTick -> CTRL = 0x00000005;
					SysTick -> LOAD = 72000-1;
					SysTick -> VAL = 0;
			while(!(SysTick -> CTRL & (1 << 16))){}			
			}
}



void PIN_MODE_UART1(){
	
		GPIO_InitTypeDef gpio_pin;
		USART_InitTypeDef usart_init;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	// pin led	
		gpio_pin.GPIO_Pin = GPIO_Pin_0;
		gpio_pin.GPIO_Mode = GPIO_Mode_Out_PP;
		gpio_pin.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &gpio_pin);
	
	
	// pin USART
		gpio_pin.GPIO_Pin = GPIO_Pin_9;
		gpio_pin.GPIO_Mode = GPIO_Mode_AF_PP;
		gpio_pin.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &gpio_pin);
	
		gpio_pin.GPIO_Pin = GPIO_Pin_10;
		gpio_pin.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		gpio_pin.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &gpio_pin);
	
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
// function receive char
char USART1_GetChar() {
    while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
    return USART_ReceiveData(USART1);
}


	char c;
int main(){
	
	PIN_MODE_UART1();
	
	while(1){
		c = USART1_GetChar();
			if(c == 'B'){
					GPIO_SetBits(GPIOA, GPIO_Pin_0);
			}
			else if(c == 'A'){
					GPIO_ResetBits(GPIOA, GPIO_Pin_0);
}
			}
	

}