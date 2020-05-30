#include "stm32f401xx.h"

void delay(uint32_t time);

int main(void)
{
	GPIO_Handle_t LED;
	LED.pGPIOx = GPIOA;
	LED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	LED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	LED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	LED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	LED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;

	GPIO_Handle_t BUTTON;
	BUTTON.pGPIOx = GPIOC;
	BUTTON.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	BUTTON.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
	BUTTON.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	BUTTON.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&LED);
	GPIO_Init(&BUTTON);

	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == 0)
		{
			delay(5);
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
		}
	}
	return 0;
}

void delay(uint32_t time)
{
	uint32_t j, i;
	for(i=0; i<10000; i++)
		for(j=0; j<time; j++);
}
