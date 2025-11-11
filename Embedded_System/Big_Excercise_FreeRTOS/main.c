#include "stm32f10x.h"                  // Device header
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "FreeRTOS.h"                   // ARM.FreeRTOS::RTOS:Core
#include "task.h"                       // ARM.FreeRTOS::RTOS:Core
#include "queue.h"                      // ARM.FreeRTOS::RTOS:Core
#include <string.h>
#include <stdio.h>

#define MAX 100

volatile char vrc_Getc;
volatile char vrc_Res[MAX]; // Mang luu chuoi nhap vào
volatile int vri_Count = 0;
volatile int vri_Stt = 0;

void Config_LED(){ 
	GPIO_InitTypeDef gpio_init;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	gpio_init.GPIO_Pin = GPIO_Pin_0;
	gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOA, &gpio_init);
}

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
			
		//	NVIC_EnableIRQ(USART1_IRQn);
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);		
}	

void USART1_IRQHandler() {
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        vrc_Getc = USART_ReceiveData(USART1);
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

QueueHandle_t xQueueCmd;
QueueHandle_t xQueueFeedback;  // hàng doi goi phan hoi ve UART
QueueHandle_t xQueueAction;    // hàng doi chia lenh dieu khien GPIO

typedef enum {
    CMD_NONE = 0,
    CMD_ON,
    CMD_OFF,
    CMD_TOGGLE
} Command_t;

// -------------------- Task 1: Nh?n chu?i --------------------
void Task_UART_Receive(void *pvParameters) {
    char buffer[MAX];

    while (1) {
        if (vri_Stt == 1) {
            vri_Stt = 0;
            taskENTER_CRITICAL();
            strcpy(buffer, (const char *)vrc_Res);
            taskEXIT_CRITICAL();
            xQueueSend(xQueueCmd, buffer, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// -------------------- Task 2: Goi phan hoi --------------------
void Task_UART_Feedback(void *pvParameters) {
    char recvBuffer[MAX];
 //   USART1_SendString("Task_UART_Feedback started\r\n");

    while (1) {
        if (xQueueReceive(xQueueCmd, recvBuffer, portMAX_DELAY) == pdTRUE) {
            USART1_SendString("Feedback: ");
            USART1_SendString(recvBuffer);
            USART1_SendString("\n");
            xQueueSend(xQueueFeedback, recvBuffer, 0);
        }
    }
}

// -------------------- Task 3: Ki?m tra l?nh --------------------
void Task_Check_Command(void *pvParameters) {
    char cmdBuffer[MAX];
    Command_t cmd;
  //  USART1_SendString("Task_Check_Command started\r\n");

    while (1) {
        if (xQueueReceive(xQueueFeedback, cmdBuffer, portMAX_DELAY) == pdTRUE) {
            if (strcmp(cmdBuffer, "ON") == 0) cmd = CMD_ON;
            else if (strcmp(cmdBuffer, "OFF") == 0) cmd = CMD_OFF;
            else if (strcmp(cmdBuffer, "TOGGLE") == 0) cmd = CMD_TOGGLE;
            else cmd = CMD_NONE;

            if (cmd != CMD_NONE) {
                xQueueSend(xQueueAction, &cmd, 0);
            }
        }
    }
}

// -------------------- Task 4: Ði?u khi?n GPIO --------------------
void Task_GPIO_Control(void *pvParameters) {
    Command_t cmd;
  //  USART1_SendString("Task_GPIO_Control started\r\n");

    while (1) {
        if (xQueueReceive(xQueueAction, &cmd, portMAX_DELAY) == pdTRUE) {
            switch (cmd) {
                case CMD_ON:    GPIO_ResetBits(GPIOA, GPIO_Pin_0); break;
                case CMD_OFF:   GPIO_SetBits(GPIOA, GPIO_Pin_0); break;
                case CMD_TOGGLE: GPIOA->ODR ^= GPIO_Pin_0; break;
                default: break;
            }
        //    USART1_SendString("GPIO action done\r\n");
        }
    }
}

int main(void) {
    SystemInit();
    PIN_MODE_UART1();
    Config_LED();

    // Tao queue
    xQueueCmd = xQueueCreate(3, MAX);
    xQueueFeedback = xQueueCreate(3, MAX);
    xQueueAction = xQueueCreate(3, sizeof(Command_t));

    USART1_SendString("System started\r\n");

    // Tao task
    xTaskCreate(Task_UART_Receive, "UART_RX", 256, NULL, 2, NULL);
    xTaskCreate(Task_UART_Feedback, "UART_FB", 256, NULL, 2, NULL);
    xTaskCreate(Task_Check_Command, "CMD_Check", 256, NULL, 2, NULL);
    xTaskCreate(Task_GPIO_Control, "GPIO_CTRL", 256, NULL, 2, NULL);

    vTaskStartScheduler();

    while (1);
}

