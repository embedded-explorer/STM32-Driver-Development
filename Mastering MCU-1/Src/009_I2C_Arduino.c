/******************************************** 009_I2C_Arduino*********************************************
 * Alternate Function Mode : 4
 * PB6 -> I2C1_SCL
 * PB7 -> I2C1_SDA
 **********************************************************************************************************/
#include "stm32f401xx.h"
#include <string.h>

#define MY_ADDR 0x61
uint8_t SLAVE_ADDR = 0x68;
uint8_t data[] = "We are testing I2C Master\n";

void I2C1_GPIOInit(void);
void I2C1_Init(void);
void Button_Init(void);
void delay(uint32_t time);

I2C_Handle_t I2C1Handle;

int main(void)
{
	uint8_t dataLen = strlen((char*)data);
	// Initialize button
	Button_Init();

	// I2C Pin Initialization
	I2C1_GPIOInit();

	// I2C Peripheral Configuration
	I2C1_Init();

	// Enable I2C Peripheral
	I2C_PeripheralControl(I2C1, ENABLE);
	while(1)
	{
		// wait for button press
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));
		delay(50);

		// Send Data
		I2C_MasterSendData(&I2C1Handle, data, dataLen, SLAVE_ADDR);
	}
	return 0;
}

void I2C1_GPIOInit(void)
{
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// I2C1_SCL
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);

	// I2C1_SDA
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&I2CPins);
}

void I2C1_Init(void)
{
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCLSpeed_SM;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FMDUTY_2;

	I2C_Init(&I2C1Handle);
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
