#include "stm32f401xx_gpio.h"

/***********************************************************************************
 * @fn			: GPIO_PeriClockControl
 *
 * @brief		: Enables or Disables Peripheral Clock for Given GPIO Port
 *
 * @param[in]	: Pointer to Base Address of GPIO Peripheral
 * @param[in]	: ENABLE or DISABLE
 *
 * @return		: none
 *
 * @Note		: none
 ************************************************************************************/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
			GPIOA_PCLK_EN();
		else if(pGPIOx == GPIOB)
			GPIOB_PCLK_EN();
		else if(pGPIOx == GPIOC)
			GPIOC_PCLK_EN();
		else if(pGPIOx == GPIOD)
			GPIOD_PCLK_EN();
		else if(pGPIOx == GPIOE)
			GPIOE_PCLK_EN();
		else if(pGPIOx == GPIOH)
			GPIOH_PCLK_EN();
	}
	else
	{
		if(pGPIOx == GPIOA)
			GPIOA_PCLK_DI();
		else if(pGPIOx == GPIOB)
			GPIOB_PCLK_DI();
		else if(pGPIOx == GPIOC)
			GPIOC_PCLK_DI();
		else if(pGPIOx == GPIOD)
			GPIOD_PCLK_DI();
		else if(pGPIOx == GPIOE)
			GPIOE_PCLK_DI();
		else if(pGPIOx == GPIOH)
			GPIOH_PCLK_DI();
	}
}

/***********************************************************************************
 * @fn			: GPIO_Init
 *
 * @brief		: Initializes GPIO Pin
 *
 * @param[in]	: pointer to a GPIO_Handle_t structure that contains GPIO Port Base Address
 * 				  and the configuration information for the specified GPIO peripheral.
 *
 * @return		: none
 *
 * @Note		: none
 ************************************************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	// Enable Clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	uint32_t temp = 0;
	// 1. Configure Mode of GPIO Pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)	// Non Interrupt Modes
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));	// Clearing Bit Fields
		pGPIOHandle->pGPIOx->MODER |= temp;
	}
	else	// Interrupt Modes
	{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			// 1. Configure FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Clear Corresponding RTSR bit
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			// 1. Configure RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Clear Corresponding FTSR bit
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			// 1. Configure both RTSR and FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		// 2. Configure GPIO Port Selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPOIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] |= portcode << (temp2 * 4);

		// 3. Enable Interrupt Delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}
	temp = 0;

	// 2. Configure the Speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	// 3. Configure Pull-up Pull-down Settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	// 4. Configure Output Type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;

	// 5. Configure Alternate Functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint8_t temp1 = 0x00;
		uint8_t temp2 = 0X00;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
	}
}

/***********************************************************************************
 * @fn			: GPIO_DeInit
 *
 * @brief		: DeInitializes GPIO Port to its reset value
 *
 * @param[in]	: pointer to  GPIO Port Base Address
 *
 * @return		: none
 *
 * @Note		: none
 ************************************************************************************/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
		GPIOA_REG_RESET();
	else if(pGPIOx == GPIOB)
		GPIOB_REG_RESET();
	else if(pGPIOx == GPIOC)
		GPIOC_REG_RESET();
	else if(pGPIOx == GPIOD)
		GPIOD_REG_RESET();
	else if(pGPIOx == GPIOE)
		GPIOE_REG_RESET();
	else if(pGPIOx == GPIOH)
		GPIOH_REG_RESET();
}


/***********************************************************************************
 * @fn			: GPIO_ReadFromInputPin
 *
 * @brief		: Read input data on GPIO Pin
 *
 * @param[in]	: pointer to  GPIO Port Base Address
 * @param[in]	: GPIO Pin Number
 *
 * @return		: 0 or 1
 *
 * @Note		: none
 ************************************************************************************/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint16_t PinNumber)
{
	uint8_t Value = 0x00;
	Value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return Value;
}

/***********************************************************************************
 * @fn			: GPIO_ReadFromInputPort
 *
 * @brief		: Read input data on GPIO Port
 *
 * @param[in]	: pointer to  GPIO Port Base Address
 *
 * @return		: Data on GPIO Port
 *
 * @Note		: none
 ************************************************************************************/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t Value = 0x0000;
	Value = (uint16_t)pGPIOx->IDR;
	return Value;
}

/***********************************************************************************
 * @fn			: GPIO_WriteToOutputPin
 *
 * @brief		: Write Data on to GPIO Pin
 *
 * @param[in]	: pointer to  GPIO Port Base Address
 * @param[in]	: GPIO Pin Number
 * @param[in]	: Data to be Written
 *
 * @return		: none
 *
 * @Note		: none
 ************************************************************************************/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint16_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/***********************************************************************************
 * @fn			: GPIO_WriteToOutputPort
 *
 * @brief		: Write Data on to GPIO Port
 *
 * @param[in]	: pointer to  GPIO Port Base Address
 * @param[in]	: Data to be Written
 *
 * @return		: none
 *
 * @Note		: none
 ************************************************************************************/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR |= Value;
}

/***********************************************************************************
 * @fn			: GPIO_ToggleOutputPin
 *
 * @brief		: Toggle value of GPIO Pin
 *
 * @param[in]	: pointer to  GPIO Port Base Address
 * @param[in]	: GPIO Pin Number
 *
 * @return		: none
 *
 * @Note		: none
 ************************************************************************************/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint16_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

/***********************************************************************************
 * @fn			: GPIO_IRQInterruptConfig
 *
 * @brief		: Disable or Enable Interrupt
 *
 * @param[in]	: IRQ Number
 * @param[in]	: Enable or Disable
 *
 * @return		: none
 *
 * @Note		: none
 ************************************************************************************/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)		// Program ISER0
		{
			*NVIC_ISER0  |= (1 << IRQNumber);
		}
		else if(IRQNumber >31 && IRQNumber <64)		// Program ISER1
		{
			*NVIC_ISER1  |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >65 && IRQNumber <96)		// Program ISER2
		{
			*NVIC_ISER2  |= (1 << (IRQNumber % 64));
		}
	}
	else
	{
		if(IRQNumber <= 31)		// Program ICER0
		{
			*NVIC_ICER0  |= (1 << IRQNumber);
		}
		else if(IRQNumber >31 && IRQNumber <64)		// Program ICER1
		{
			*NVIC_ICER1  |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >65 && IRQNumber <96)		// Program ICER2
		{
			*NVIC_ICER2  |= (1 << (IRQNumber % 64));
		}
	}
}

/***********************************************************************************
 * @fn			: GPIO_IRQPriorityConfig
 *
 * @brief		:
 *
 * @param[in]	: IRQ Number
 * @param[in]	: Enable or Disable
 *
 * @return		: none
 *
 * @Note		: none
 ************************************************************************************/
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t temp1 = IRQNumber / 4;
	uint8_t temp2 = IRQNumber % 4;
	uint8_t shift_amount = (8 * temp2) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASEADDR + temp1) |= (IRQPriority << shift_amount);
}

/***********************************************************************************
 * @fn			: GPIO_IRQHandling
 *
 * @brief		: Clear PR Register bit Corresponding to PinNumber
 *
 * @param[in]	: PinNumber
 *
 * @return		: none
 *
 * @Note		: none
 ************************************************************************************/
void GPIO_IRQHandling(uint16_t PinNumber)
{
	// Clear PR Register bit Corresponding to PinNumber
	if(EXTI->PR & (1 << PinNumber))
	{
		EXTI->PR |= (1 << PinNumber);
	}
}
