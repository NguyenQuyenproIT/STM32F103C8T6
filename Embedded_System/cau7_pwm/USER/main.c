#include "stm32f10x.h"                  // Device header
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_tim.h"              // Keil::Device:StdPeriph Drivers:TIM
#include "stm32f10x_usart.h"            // Keil::Device:StdPeriph Drivers:USART
#include "stdio.h"

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
}

void config_TIMER_PWM(){
	
			GPIO_InitTypeDef gpio_init;
			TIM_TimeBaseInitTypeDef tim_init;
			TIM_OCInitTypeDef oc_init;
			
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_TIM1, ENABLE);

			gpio_init.GPIO_Pin = GPIO_Pin_8; // channel 1
			gpio_init.GPIO_Mode = GPIO_Mode_AF_PP; // Alternate Function Push-Pull
			gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOA, &gpio_init);

			// 3. Cau hình Time Base (f = 1 kHz)
			tim_init.TIM_Prescaler = 71;          // PSC = 71 -> clk = 72MHz/72 = 1 MHz
			tim_init.TIM_Period = 999;            // ARR = 999 -> T = 1000 tick = 1 ms = 1 kHz
			tim_init.TIM_CounterMode = TIM_CounterMode_Up;
			TIM_TimeBaseInit(TIM1, &tim_init);

			// 4. Cau hình PWM cho CH1
			oc_init.TIM_OCMode = TIM_OCMode_PWM1; // TIM_OCMode_PWM2; duty tang làm LED sáng dan
																						// TIM_OCMode_PWM1; tat dan

			oc_init.TIM_OutputState = TIM_OutputState_Enable;
			oc_init.TIM_Pulse = 300;              // CCR1 = 300 -> Duty = 300/1000 = 30% (if connect common +)
																						// if connect common - then 100% - duty (30%)
			oc_init.TIM_OCPolarity = TIM_OCPolarity_High;
			TIM_OC1Init(TIM1, &oc_init);
			TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

			// 5. Bat output chính (MOE) vì TIM1 là advanced timer
			TIM_CtrlPWMOutputs(TIM1, ENABLE);

			// 6. Start Timer
			TIM_Cmd(TIM1, ENABLE);
}


struct __FILE{
int handle; 
};
FILE __stdout;

// Redirect hàm fputc v? UART
int fputc(int ch, FILE *f) {
    USART_SendData(USART1, (uint8_t) ch);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    return ch;
}



int duty;
int main(void) {
    config_TIMER_PWM();
		PIN_MODE_UART1();
    while (1) {
        for (duty = 0; duty <= 999; duty += 10) {
						
					printf("PWM: F = %d Hz, Duty = %d/1000 = %.1f%% \n", 1000, duty, duty/10.0);
					
					TIM1->CCR1 = duty;   // CCR1: giá tri so sánh cho kênh CH1.
								delay_ms(300);
        }   
			}
}

