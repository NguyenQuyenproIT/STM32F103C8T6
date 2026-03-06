#ifndef _RFID_RC522_
#define _RFID_RC522_

#include "stm32f10x.h"                  // Device header

void delay_ms(unsigned int time);
void SPI1_Init(void);
uint8_t SPI1_Transfer(uint8_t data);
void RC522_WriteReg(uint8_t addr, uint8_t val);
uint8_t RC522_ReadReg(uint8_t addr);
void RC522_Reset(void);
void RC522_AntennaOn(void);
void RC522_Init(void);
uint8_t RC522_Transceive(uint8_t *sendData, uint8_t sendLen, uint8_t *backData,  uint8_t *backLen);
uint8_t RC522_Request(uint8_t *tagType);
uint8_t RC522_AntiColl(uint8_t *uid);
void RC522_Halt(void);


#endif