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

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&LED);

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
		delay(10);
	}
	return 0;
}

void delay(uint32_t time)
{
	uint32_t j, i;
	for(i=0; i<65000; i++)
		for(j=0; j<time; j++);
}
