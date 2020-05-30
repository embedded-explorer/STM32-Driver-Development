/******************************************** 009_I2C_Arduino*********************************************
 * Alternate Function Mode : 4
 * PB6 -> I2C1_SCL
 * PB7 -> I2C1_SDA
 **********************************************************************************************************/
#include "stm32f401xx.h"
#include <string.h>
#include <stdio.h>

extern void initialise_monitor_handles(void);

#define RCVLEN  0x51
#define RCVDATA  0x52
#define MY_ADDR  0x69

void I2C1_GPIOInit(void);
void I2C1_Init(void);
void Button_Init(void);
void delay(uint32_t time);

uint8_t data[32] = "Hi This is STM Nucleo";

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

	// I2C IRQ Configurations
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE);

	I2C_SlaveEnableDisableCallbackEvents(I2C1, ENABLE);

	// Enable I2C Peripheral
	I2C_PeripheralControl(I2C1, ENABLE);

	// Enable ACKing after PE = 1
	I2C1Handle.pI2Cx->I2C_CR1 |= (1 << I2C_CR1_ACK);

	while(1);

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

void I2C1_EV_IRQHandler(void)
{
	I2C_EV_IRQHandling(&I2C1Handle);
}

void I2C1_ER_IRQHandler(void)
{
	I2C_ER_IRQHandling(&I2C1Handle);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv)
{
	static uint8_t commandCode = 0;
	static uint8_t cnt = 0;

	if(AppEv == I2C_EV_DATA_REQ)
	{
		// Slave must send data
		if(commandCode == RCVLEN)
			I2C_SlaveSendData(I2C1, strlen((char*)data));
		else if(commandCode == RCVDATA)
			I2C_SlaveSendData(I2C1, data[cnt++]);
	}
	else if(AppEv == I2C_EV_DATA_RCV)
	{
		// Slave must read data
		commandCode = I2C_SlaveReceiveData(I2C1);
	}
	else if(AppEv == I2C_ERROR_AF)
	{
		// Happens during slave transmission
		commandCode = 0xFF;
		cnt = 0;
	}
	else if(AppEv == I2C_EV_STOP)
	{
		// Master has ended i2c communication
	}
}

void delay(uint32_t time)
{
	uint32_t j, i;
	for(i=0; i<10000; i++)
		for(j=0; j<time; j++);
}
