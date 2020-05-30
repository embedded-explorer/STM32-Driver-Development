#include "stm32f401xx.h"

int main(void)
{
	// LED Pin Configurations
	GPIO_Handle_t LED;
	LED.pGPIOx = GPIOA;
	LED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	LED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	LED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	LED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	LED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&LED);

	// Button Pin Configurations
	GPIO_Handle_t BUTTON;
	BUTTON.pGPIOx = GPIOC;
	BUTTON.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	BUTTON.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	BUTTON.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	BUTTON.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;

	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&BUTTON);

	// IRQ Configurations
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);

	while(1);
	return 0;
}

void EXTI15_10_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_NO_13);
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
}
