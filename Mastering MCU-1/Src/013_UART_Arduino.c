/*********************************************** USART 1 **************************************************
 * Alternate Function Mode : 7
 * PA8 -> USART1_CK
 * PA8 -> USART1_TX
 * PA8 -> USART1_RX
 * PA8 -> USART1_CTS
 * PA8 -> USART1_RTS
 **********************************************************************************************************/
#include "stm32f401xx.h"
#include <string.h>
#include <stdio.h>

extern void initialise_monitor_handles(void);

#define RCVLEN  0x51
#define RCVDATA  0x52
#define MY_ADDR  0x69

void USART1_GPIOInit(void);
void USART1_Init(void);
void Button_Init(void);
void delay(uint32_t time);

uint8_t data[32] = "Hi This is STM Nucleo\n\r";

USART_Handle_t USART1Handle;

int main(void)
{
	uint8_t dataLen = strlen((char*)data);
	initialise_monitor_handles();
	printf("Application is running\n");

	// Initialize button
	Button_Init();

	// USART Pin Initialization
	USART1_GPIOInit();

	// USART Peripheral Configuration
	USART1_Init();

	// Enable USART Peripheral
	USART_PeripheralControl(USART1, ENABLE);

	while(1)
	{
		while((GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13)));
		delay(50);
		USART_SendData(&USART1Handle, data, dataLen);
	}

	return 0;
}

void USART1_GPIOInit(void)
{
	GPIO_Handle_t USARTPins;

	USARTPins.pGPIOx = GPIOA;
	USARTPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	USARTPins.GPIO_PinConfig.GPIO_PinAltFunMode = 7;
	USARTPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	USARTPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	USARTPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// USART1_CK
	USARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	GPIO_Init(&USARTPins);

	// USART1_TX
	USARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	GPIO_Init(&USARTPins);

	// USART1_RX
	USARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
	GPIO_Init(&USARTPins);

	// USART1_CTS
	USARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_11;
	GPIO_Init(&USARTPins);

	// USART1_TRTS
	USARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&USARTPins);
}

void USART1_Init(void)
{
	USART1Handle.pUSARTx = USART1;
	USART1Handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	USART1Handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	USART1Handle.USART_Config.USART_Mode = USART_MODE_TXRX;
	USART1Handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	USART1Handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART1Handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;

	USART_Init(&USART1Handle);
}

void Button_Init(void)
{
	// Button Pin Configurations
	GPIO_Handle_t BUTTON;
	BUTTON.pGPIOx = GPIOC;
	BUTTON.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	BUTTON.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
	BUTTON.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	BUTTON.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_Init(&BUTTON);
}

void delay(uint32_t time)
{
	uint32_t j, i;
	for(i=0; i<10000; i++)
		for(j=0; j<time; j++);
}
