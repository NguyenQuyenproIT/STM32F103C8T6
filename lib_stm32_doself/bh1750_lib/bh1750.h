#ifndef _BH1750_H_
#define _BH1750_H_

#include "stm32f10x.h"                  // Device header

void delay_ms(uint16_t time);
void Config_I2C();
void I2C_WriteByte(uint8_t address, uint8_t data);
void BH1750_Init(void);
uint16_t BH1750_ReadLight();


#endif