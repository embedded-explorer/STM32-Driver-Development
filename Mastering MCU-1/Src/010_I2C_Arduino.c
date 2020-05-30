/******************************************** 009_I2C_Arduino*********************************************
 * Alternate Function Mode : 4
 * PB6 -> I2C1_SCL
 * PB7 -> I2C1_SDA
 **********************************************************************************************************/
#include "stm32f401xx.h"
#include <string.h>
#include <stdio.h>

extern void initialise_monitor_handles(void);

uint8_t RCVLEN = 0x51;
uint8_t RCVDATA = 0x52;

uint8_t MY_ADDR = 0x61;
uint8_t SLAVE_ADDR = 0x68;

void I2C1_GPIOInit(void);
void I2C1_Init(void);
void Button_Init(void);
void delay(uint32_t time);

uint8_t dataLen = 0;
uint8_t data[50];

I2C_Handle_t I2C1Handle;

int main(void)
{
	initialise_monitor_handles();
	printf("Application is running\n");

	// Initialize button
	Button_Init();

	// I2C Pin Initialization
	I2C1_GPIOInit();

	// I2C Peripheral Configuration
	I2C1_Init();

	// Enable I2C Peripheral
	I2C_PeripheralControl(I2C1, ENABLE);

	// Enable ACKing after PE = 1
	I2C1Handle.pI2Cx->I2C_CR1 |= (1 << I2C_CR1_ACK);

	while(1)
	{
		// wait for button press
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));
		delay(50);

		// Send Command to receive length of data
		I2C_MasterSendData(&I2C1Handle, &RCVLEN, 1, SLAVE_ADDR, ENABLE);

		// Read one byte of length information
		I2C_MasterReceiveData(&I2C1Handle, &dataLen, 1, SLAVE_ADDR, ENABLE);

		// Send Command to receive data
		I2C_MasterSendData(&I2C1Handle, &RCVDATA, 1, SLAVE_ADDR, ENABLE);

		// Read the data
		I2C_MasterReceiveData(&I2C1Handle, data, dataLen, SLAVE_ADDR, DISABLE);

		data[dataLen+1] = '\0';
		printf("Received Data: %s", data);
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
