#include "uart1.h"
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_usart.h"            // Keil::Device:StdPeriph Drivers:USART
#include "stm32f10x.h"                  // Device header
#include "stdio.h"


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
		NVIC_InitTypeDef NVIC_InitStructure;

	
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
	// pin USART
		gpio_pin.GPIO_Pin = GPIO_Pin_9;
		gpio_pin.GPIO_Mode = GPIO_Mode_AF_PP;
		gpio_pin.GPIO_Speed = GPIO_Speed_50MHz; // TX
		GPIO_Init(GPIOA, &gpio_pin);
	
		gpio_pin.GPIO_Pin = GPIO_Pin_10;
		gpio_pin.GPIO_Mode = GPIO_Mode_IN_FLOATING; // RX
		gpio_pin.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &gpio_pin);
	
		usart_init.USART_BaudRate = 115200;
		usart_init.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
		usart_init.USART_WordLength = USART_WordLength_8b;
		usart_init.USART_StopBits = USART_StopBits_1;
		usart_init.USART_Parity = USART_Parity_No;
		usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
			
		
		USART_Init(USART1, &usart_init); // configure to register: Baudrate, Word length, Parity, Stop bit, Flow control, Mode… but still not turn on USART.
			
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // enable ngat RXNE
		USART_Cmd(USART1, ENABLE); // bat bo USART. -> complete
			
		//	NVIC_EnableIRQ(USART1_IRQn);
		NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
			
			
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

char UARTx_Getc(USART_TypeDef* USARTx) {
    while (USART_GetFlagStatus(USARTx, USART_FLAG_RXNE) == RESET);
    return USART_ReceiveData(USARTx);
}



// function receive char
char USART1_GetChar() {
    while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
    return USART_ReceiveData(USART1);
}

// use function "printf", add 	stdio.h

struct __FILE{
int handle; 
};
FILE __stdout;

int fputc(int ch, FILE *f) {
    USART_SendData(USART1, (uint8_t) ch);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    return ch;
}

///////////////
// ngat uart gui chuoi ki tu

//void UART1_Interrupt_Config() {
//    NVIC_InitTypeDef nvic_init;

//    // Cho phép ngat RXNE (Receive Not Empty)
//    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

//    // Cau hình NVIC cho USART1
//    nvic_init.NVIC_IRQChannel = USART1_IRQn;               
//    nvic_init.NVIC_IRQChannelPreemptionPriority = 0;   
//    nvic_init.NVIC_IRQChannelSubPriority = 0;          
//    nvic_init.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&nvic_init);
//}

//void USART1_IRQHandler() {
//    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
//        vrc_Getc = USART_ReceiveData(USART1);
//        USART1_SendChar(vrc_Getc); // echo test

//        if (vrc_Getc == '\r' || vrc_Getc == '\n') {
//            if (vri_Count > 0) {
//                vrc_Res[vri_Count] = '\0';
//                vri_Stt = 1;
//                vri_Count = 0;
//            }
//        } else if (vri_Count < MAX - 1) {
//            vrc_Res[vri_Count++] = vrc_Getc;
//        }
//        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
//    }
//}
