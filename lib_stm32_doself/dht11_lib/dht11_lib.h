#ifndef _DHT11_H_
#define _DHT11_H_

#include "stm32f10x.h"                  // Device header

void Timer2_Init(void);
void delay_us(uint16_t us);
void delay_ms(uint16_t ms);
void GPIO_Config(void);
void DHT11_Start(void);
uint8_t DHT11_Read_Byte(void);
uint8_t DHT11_Read_Data(uint8_t *hum_int,uint8_t *hum_dec,uint8_t *temp_int,uint8_t *temp_dec);

#endif
