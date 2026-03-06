
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