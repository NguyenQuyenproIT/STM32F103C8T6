#include "stm32f10x.h"                  // Device header
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_exti.h"             // Keil::Device:StdPeriph Drivers:EXTI

// BUTTON - PIN PA2
// LED - PIN PA0



void config_gpio(){
	GPIO_InitTypeDef gpio_init;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	gpio_init.GPIO_Pin = GPIO_Pin_0; // LED
	gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpio_init);
	
	gpio_init.GPIO_Pin = GPIO_Pin_2; // BUTTON
	gpio_init.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &gpio_init);
	// configure to register
//	GPIOA -> CRL = 0x00000083;

};

void config_NVIC(){
	
		NVIC_InitTypeDef nvic_init;
	
		NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0); 
		
		
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	
		nvic_init.NVIC_IRQChannel = EXTI2_IRQn;
		nvic_init.NVIC_IRQChannelPreemptionPriority = 0;
		nvic_init.NVIC_IRQChannelSubPriority = 0;
		nvic_init.NVIC_IRQChannelCmd = ENABLE;
		
		NVIC_Init(&nvic_init);
	

};


void congfig_exti(){
	EXTI_InitTypeDef exti_init;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);	
		
	exti_init.EXTI_Line = EXTI_Line2; // PA1
	exti_init.EXTI_Mode = EXTI_Mode_Interrupt;
	exti_init.EXTI_Trigger = EXTI_Trigger_Falling; // interrupt when click button(0)
	exti_init.EXTI_LineCmd = ENABLE;
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource2); // tag PA1 <-> EXTI1.
	
	EXTI_Init(&exti_init);
	
	
};

volatile uint8_t led_state = 0;

int i;

void EXTI2_IRQHandler() {
    if (EXTI_GetITStatus(EXTI_Line2) != RESET) {
        for (i = 0; i < 50000; i++); // delay ng?n ~5ms
        if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2) == 0) {
            led_state = !led_state;
            if (led_state)
                GPIO_ResetBits(GPIOA, GPIO_Pin_0);
            else
                GPIO_SetBits(GPIOA, GPIO_Pin_0);
        }
        EXTI_ClearITPendingBit(EXTI_Line2);
    }
}

int main(){
	config_gpio();
	config_NVIC();
	congfig_exti();

	while (1) {

			}
}



