#include "stm32f10x.h"                  // Device header
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_tim.h"              // Keil::Device:StdPeriph Drivers:TIM


void config_gpio();
void config_timer();

void config_gpio(){

	GPIO_InitTypeDef GPIO_init;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
//	GPIO_init.GPIO_Pin = GPIO_Pin_0;
//	GPIO_init.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_init.GPIO_Speed = GPIO_Speed_50MHz;
//	
	
		GPIOA -> CRL = 0x00000003; // PIN PA0
	
	//GPIO_Init(GPIOA, &GPIO_init);
};

void config_timer(){ // delay 1s
	
		TIM_TimeBaseInitTypeDef timer_init;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
		
		
		timer_init.TIM_Prescaler = 35999;
		timer_init.TIM_Period = 1999;
		timer_init.TIM_CounterMode = TIM_CounterMode_Up;
		timer_init.TIM_ClockDivision 		= 0;
		timer_init.TIM_RepetitionCounter = 0;
	
		TIM_TimeBaseInit(TIM1,&timer_init);
	
		TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);   // Enable interrupt update
    NVIC_EnableIRQ(TIM1_UP_IRQn);                // Enable NVIC line
	
		TIM_Cmd(TIM1, ENABLE);
};

//void delay_s(uint16_t time){
//    while (time--){
//        TIM_SetCounter(TIM1, 0);
//        TIM_ClearFlag(TIM1, TIM_FLAG_Update);
//        TIM_Cmd(TIM1, ENABLE);
//				while (TIM_GetFlagStatus(TIM1, TIM_FLAG_Update) == RESET){};
//        TIM_ClearFlag(TIM1, TIM_FLAG_Update);
//        TIM_Cmd(TIM1, DISABLE);
//    }
//}




void TIM1_UP_IRQHandler(){ // (ISR )noi xu ly moi lan timer tràn.
		if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) {		// Bat interrupt flag
			GPIOA -> ODR ^= GPIO_Pin_0;
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update); // clear interrupt flag
}
};

int main(){
	config_gpio();
	config_timer();
	
	while(1){
//			GPIO_SetBits(GPIOA, GPIO_Pin_0);
//				delay_s(3);
//			GPIO_ResetBits(GPIOA, GPIO_Pin_0);
//				delay_s(3);
	}

};