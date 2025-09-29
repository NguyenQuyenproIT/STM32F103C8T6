#include "stm32f10x.h"                  // Device header
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_usart.h"            // Keil::Device:StdPeriph Drivers:USART
#include "stm32f10x_adc.h"              // Keil::Device:StdPeriph Drivers:ADC
#include "stdio.h"

/* UART1: APB2
TX: PA9 - AF_PP
RX: PA10 - IN_FLOATING

ADC: PA4 - AIN;
*/



void delay_ms(uint16_t time);
void config_uart();
void config_ADC();

void delay_ms(uint16_t time){ // 1s
		uint16_t i;
			for(i = 0; i<time; i++){
					SysTick -> CTRL = 0x00000005;
					SysTick -> LOAD = 72000-1;
					SysTick -> VAL = 0;
			while(!(SysTick -> CTRL & (1 << 16))){}			
			}
}

//struct __FILE {
//    int dummy;
//};
//FILE __stdout;

//int fputc(int ch, FILE *f) {
//    USART1_SendChar(ch);
//    return ch;
//}

void config_uart(){
		GPIO_InitTypeDef gpio_pin;
		USART_InitTypeDef usart_init;
	
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);
	
	// pin USART
		gpio_pin.GPIO_Pin = GPIO_Pin_9; // TX
		gpio_pin.GPIO_Mode = GPIO_Mode_AF_PP;
		gpio_pin.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &gpio_pin);
	
		gpio_pin.GPIO_Pin = GPIO_Pin_10; // RX
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
 // send string to hercules
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

void config_ADC(){
	GPIO_InitTypeDef adc_pin;
	ADC_InitTypeDef adc_init;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE);
	
	adc_pin.GPIO_Pin = GPIO_Pin_4;
	adc_pin.GPIO_Mode = GPIO_Mode_AIN; // Analog INput
	adc_pin.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOA, &adc_pin);
///////////////////////////////

	adc_init.ADC_Mode = ADC_Mode_Independent;
	adc_init.ADC_ContinuousConvMode = ENABLE;
	adc_init.ADC_DataAlign = ADC_DataAlign_Right;
	adc_init.ADC_ScanConvMode = DISABLE; // only a channel then not scan
	adc_init.ADC_NbrOfChannel = 1;
	adc_init.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	
	ADC_Init(ADC1, &adc_init);
	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_55Cycles5);
	
	ADC_Cmd(ADC1, ENABLE);
	
	 ADC_ResetCalibration(ADC1);
 while(ADC_GetResetCalibrationStatus(ADC1));
 ADC_StartCalibration(ADC1);
 while(ADC_GetCalibrationStatus(ADC1));
	
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

uint16_t ADC_Read() {
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET); // cho xong convert
    return ADC_GetConversionValue(ADC1);
}


char c;
uint16_t value;
uint32_t voltage;
char buffer[40];

int main(){
	
	config_uart();
  config_ADC();   
	
	while(1){
			value = ADC_Read();
			voltage = (value * 3300) / 4095;		
    sprintf(buffer, "ADC: %u Voltage: %u.%3u V\n", value, voltage/1000, voltage%1000);
    USART1_SendString(buffer);
			delay_ms(1000);
			}
}