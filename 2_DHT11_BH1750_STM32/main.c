#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_i2c.h"
#include <stdio.h>

/* ================= UART ================= */

void UART1_Init(){

	GPIO_InitTypeDef gpio;
	USART_InitTypeDef uart;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);

	gpio.GPIO_Pin = GPIO_Pin_9;
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&gpio);

	gpio.GPIO_Pin = GPIO_Pin_10;
	gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA,&gpio);

	uart.USART_BaudRate = 115200;
	uart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	uart.USART_WordLength = USART_WordLength_8b;
	uart.USART_StopBits = USART_StopBits_1;
	uart.USART_Parity = USART_Parity_No;
	uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

	USART_Init(USART1,&uart);
	USART_Cmd(USART1,ENABLE);
}

/* redirect printf */

struct __FILE { int handle; };
FILE __stdout;

int fputc(int ch, FILE *f){

	USART_SendData(USART1,ch);
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);
	return ch;
}

/* ================= TIMER ================= */

void Timer2_Init(){

	TIM_TimeBaseInitTypeDef timer;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);

	timer.TIM_Prescaler = 72-1;
	timer.TIM_Period = 0xFFFF;
	timer.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM2,&timer);
	TIM_Cmd(TIM2,ENABLE);
}

void delay_us(uint16_t us){

	TIM_SetCounter(TIM2,0);
	while(TIM_GetCounter(TIM2)<us);
}

void delay_ms(uint16_t ms){

	while(ms--) delay_us(1000);
}

/* ================= DHT11 ================= */

void DHT11_Start(){

	GPIO_InitTypeDef gpio;

	gpio.GPIO_Pin = GPIO_Pin_12;
	gpio.GPIO_Mode = GPIO_Mode_Out_OD;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&gpio);

	GPIO_ResetBits(GPIOB,GPIO_Pin_12);
	delay_ms(20);

	GPIO_SetBits(GPIOB,GPIO_Pin_12);
	delay_us(30);

	gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB,&gpio);
}

uint8_t DHT11_Read_Byte(){

	uint8_t i,byte=0;

	for(i=0;i<8;i++)
	{
		while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)==0);

		TIM_SetCounter(TIM2,0);

		while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)==1);

		if(TIM_GetCounter(TIM2)>45)
			byte=(byte<<1)|1;
		else
			byte=(byte<<1);
	}

	return byte;
}

uint8_t DHT11_Read_Data(uint8_t *h1,uint8_t *h2,uint8_t *t1,uint8_t *t2){

	uint8_t data[5];
	uint8_t i;

	while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)==1);
	while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)==0);
	while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)==1);

	for(i=0;i<5;i++)
		data[i]=DHT11_Read_Byte();

	*h1=data[0];
	*h2=data[1];
	*t1=data[2];
	*t2=data[3];

	return data[4];
}

/* ================= I2C ================= */

#define BH1750_ADDR 0x46
#define BH1750_PWR_ON 0x01
#define BH1750_CONT_HRES_MODE 0x10

void I2C_Config(){

	GPIO_InitTypeDef gpio;
	I2C_InitTypeDef i2c;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);

	gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	gpio.GPIO_Mode = GPIO_Mode_AF_OD;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&gpio);

	i2c.I2C_ClockSpeed = 100000;
	i2c.I2C_Mode = I2C_Mode_I2C;
	i2c.I2C_DutyCycle = I2C_DutyCycle_2;
	i2c.I2C_OwnAddress1 = 0;
	i2c.I2C_Ack = I2C_Ack_Enable;
	i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

	I2C_Init(I2C1,&i2c);
	I2C_Cmd(I2C1,ENABLE);
}

void I2C_Write(uint8_t addr,uint8_t data){

	I2C_GenerateSTART(I2C1,ENABLE);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));

	I2C_Send7bitAddress(I2C1,addr,I2C_Direction_Transmitter);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	I2C_SendData(I2C1,data);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	I2C_GenerateSTOP(I2C1,ENABLE);
}

void BH1750_Init(){

	delay_ms(10);
	I2C_Write(BH1750_ADDR,BH1750_PWR_ON);
	delay_ms(10);
	I2C_Write(BH1750_ADDR,BH1750_CONT_HRES_MODE);
	delay_ms(200);
}

uint16_t BH1750_Read(){

	uint8_t msb,lsb;
	uint16_t lux;

	I2C_GenerateSTART(I2C1,ENABLE);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));

	I2C_Send7bitAddress(I2C1,BH1750_ADDR,I2C_Direction_Receiver);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED));
	msb = I2C_ReceiveData(I2C1);

	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED));
	lsb = I2C_ReceiveData(I2C1);

	I2C_AcknowledgeConfig(I2C1,DISABLE);
	I2C_GenerateSTOP(I2C1,ENABLE);
	I2C_AcknowledgeConfig(I2C1,ENABLE);

	lux = ((msb<<8)|lsb)/1.2;

	return lux;
}

/* ================= MAIN ================= */

int main(){

	uint8_t h1,h2,t1,t2,checksum;
	uint16_t lux;

	UART1_Init();
	Timer2_Init();
	I2C_Config();
	BH1750_Init();
	


	while(1){

		/* DHT11 */
		DHT11_Start();
		checksum = DHT11_Read_Data(&h1,&h2,&t1,&t2);

		if((h1+h2+t1+t2)==checksum)
			printf("Temp:%d.%dC  Hum:%d.%d%%  ",t1,t2,h1,h2);
		else
			printf("DHT11 Error  ");

		/* BH1750 */
		lux = BH1750_Read();
		printf("Light:%d lux\r\n",lux);

		delay_ms(1000);
	}
}