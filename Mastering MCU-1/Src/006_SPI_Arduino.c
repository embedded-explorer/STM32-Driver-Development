/******************************************** 006_SPI_Arduino*** *****************************************
 * Alternate Function Mode : 5
 * PB12 -> SPI2_NSS
 * PB13 -> SPI2_SCK
 * PB14 -> SPI2_MISO
 * PB15 -> SPI2_MOSI
 **********************************************************************************************************/
#include "stm32f401xx.h"
#include <string.h>

void SPI2_GPIOInit(void);
void SPI2_Init(void);
void Button_Init(void);
void EXTI15_10_IRQHandler(void);

char user_data[] = "Hello World";

int main(void)
{
	// Initialize GPIO Pins to Behave as SPI2 Pins
	SPI2_GPIOInit();

	// Initialize SPI2 Configuration
	SPI2_Init();

	// Initialize Button Interrupt
	Button_Init();

	while(1);

	return 0;
}

void SPI2_GPIOInit(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;

	// SPI2_NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);

	// SPI2_SCK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	// SPI2_MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);
}

void SPI2_Init(void)
{
	SPI_Handle_t SPI2Handle;
	SPI2Handle.pSPIx = SPI2;

	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_FDUPLEX;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_SAMP1;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BIT;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_MASTER;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_HW;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_CLK_SPEED_DIV8;	// 	Clock Speed = 8MHz
	SPI2Handle.SPIConfig.SPI_SS0E = ENABLE;
	SPI_Init(&SPI2Handle);
}

void Button_Init(void)
{
	// Button Pin Configurations
	GPIO_Handle_t BUTTON;
	BUTTON.pGPIOx = GPIOC;
	BUTTON.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	BUTTON.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	BUTTON.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	BUTTON.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;

	GPIO_Init(&BUTTON);

	// IRQ Configurations
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);
}

void EXTI15_10_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_NO_13);	// Clear Pending Register

	uint8_t DataLen = strlen(user_data);

	// Enable SPI2 Peripheral
	SPI_PeripheralControl(SPI2, ENABLE);

	// Send Length Information
	SPI_SendData(SPI2, &DataLen, 1);
	while(SPI2->SPI_SR & (1 << SPI_SR_BSY ));

	SPI_SendData(SPI2, (uint8_t*)user_data, DataLen);

	// Wait unit BSY flag is reset
	while(SPI2->SPI_SR & (1 << SPI_SR_BSY ));

	// Disable SPI2 Peripheral
	SPI_PeripheralControl(SPI2, DISABLE);
}
