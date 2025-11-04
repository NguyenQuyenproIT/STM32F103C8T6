#include "stm32f10x.h"                  // Device header
#include "uart1.h"            // Keil::Device:StdPeriph Drivers:USART
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stdio.h"
#include <string.h>
#include <stdlib.h>	
//#include "stm32f10x_usart.h"            // Keil::Device:StdPeriph Drivers:USART

#include "stm32f10x_usart.h"
	
#include "FreeRTOS.h"                   // ARM.FreeRTOS::RTOS:Core
#include "FreeRTOSConfig.h"             // ARM.FreeRTOS::RTOS:Config	
#include "queue.h"                      // ARM.FreeRTOS::RTOS:Core
#include "timers.h"                     // ARM.FreeRTOS::RTOS:Timers

#define MAX 100

char vrc_Getc;
char vrc_Res[MAX]; // M?ng luu chu?i nh?p vào
int vri_Count = 0;
int vri_Stt = 0;

void config_led(){ // config for GPIO - PB0
	GPIO_InitTypeDef gpio_init;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	gpio_init.GPIO_Pin = GPIO_Pin_0;
	gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOA, &gpio_init);
}


void USART1_IRQHandler() {
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        vrc_Getc = USART_ReceiveData(USART1);
     //   USART1_SendChar(vrc_Getc); // echo test

        if (vrc_Getc == '\r' || vrc_Getc == '\n') {
            if (vri_Count > 0) {
                vrc_Res[vri_Count] = '\0';
                vri_Stt = 1;
                vri_Count = 0;
            }
        } else if (vri_Count < MAX - 1) {
            vrc_Res[vri_Count++] = vrc_Getc;
        }
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}

int main(){
	// SystemInit(); // Khoi tao he thong xung nhip
	PIN_MODE_UART1();                    // cau hình UART
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // enable ngat RXNE
//	UART1_Interrupt_Config();            // enable NVIC
	config_led();                        // config PA0
	printf("start");
	while(1) {
    if(vri_Stt) {
        vri_Stt = 0;
        printf(" Received: %s\n", vrc_Res);
        if(strcmp(vrc_Res, "ON") == 0) GPIO_ResetBits(GPIOA, GPIO_Pin_0);
        if(strcmp(vrc_Res, "OFF") == 0) GPIO_SetBits(GPIOA, GPIO_Pin_0);
				if((strcmp(vrc_Res, "TOGGLE") == 0)) GPIOA -> ODR ^= 1;
    }
}
}